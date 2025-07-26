#include "tmc2209.h"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "TMC2209";

TMC2209Stepper::TMC2209Stepper(UartBus& b, uint8_t a)
    : _bus(b), _addr(a), _diag_pin(GPIO_NUM_NC),
    _callback(nullptr), _initialized(false),
    _shadow_ihold_irun(0x00081010), _shadow_tpowerdown(0),
    _shadow_tpwmthrs(0), _shadow_vactual(0), _shadow_tcoolthrs(0),
    _shadow_sgthrs(0), _shadow_coolconf(0)
{
    if (a > 3) ESP_LOGE(TAG, "Addr %u invalid", a);
}

esp_err_t TMC2209Stepper::initialize() {
    if (!_bus.is_initialized()) return ESP_ERR_INVALID_STATE;

    // reset to defaults
    return reset_to_defaults();
}

esp_err_t TMC2209Stepper::reset_to_defaults() {
    uint32_t g = 0;
    g |= (1 << 6) | (1 << 7);
    esp_err_t r = _bus.write_register(_addr, REG_GCONF, g); if (r != ESP_OK) return r;

    // Update shadow registers with default values
    _shadow_ihold_irun = 0x00081010;
    r = _bus.write_register(_addr, REG_IHOLD_IRUN, _shadow_ihold_irun); if (r != ESP_OK) return r;

    _shadow_coolconf = 0x00000000;
    r = _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf); if (r != ESP_OK) return r;

    _shadow_tpwmthrs = 0;
    r = _bus.write_register(_addr, REG_TPWMTHRS, _shadow_tpwmthrs); if (r != ESP_OK) return r;

    _shadow_tcoolthrs = 0;
    r = _bus.write_register(_addr, REG_TCOOLTHRS, _shadow_tcoolthrs); if (r != ESP_OK) return r;

    r = _bus.write_register(_addr, REG_THIGH, 0); if (r != ESP_OK) return r;

    vTaskDelay(pdMS_TO_TICKS(10));
    _initialized = true;
    ESP_LOGI(TAG, "Defaults loaded");
    return ESP_OK;
}

uint8_t TMC2209Stepper::calculate_current_scale(uint16_t mA) {
    if (mA == 0) return 0;
    // datasheet formula: CS = (I_rms*32*(Rsense+Rint)*√2)/Vfs -1
    const float Vfs = 0.325f;
    const float Rsense = 0.11f;
    const float Rint = 0.02f;
    float Irms = mA / 1000.0f;
    float csf = (Irms * 32.0f * (Rsense + Rint) * 1.41421356f) / Vfs - 1.0f;
    int cs = int(std::round(csf));
    return uint8_t(std::clamp(cs, 0, 31));
}

esp_err_t TMC2209Stepper::set_motor_current(uint16_t mA) {
    const uint16_t MAXmA = 2000; // datasheet 2A RMS
    if (mA > MAXmA) {
        ESP_LOGW(TAG, "Clamped %u mA→%u mA", mA, MAXmA);
        mA = MAXmA;
    }
    uint8_t cs = calculate_current_scale(mA);
    _shadow_ihold_irun = (_shadow_ihold_irun & ~((0x1F << 8) | 0x1F)) | (uint32_t(cs) << 8) | cs;
    ESP_LOGI(TAG, "I %u mA → CS=%u", mA, cs);
    return _bus.write_register(_addr, REG_IHOLD_IRUN, _shadow_ihold_irun);
}

esp_err_t TMC2209Stepper::set_direction(bool cw) {
    uint32_t g; esp_err_t r = _bus.read_register(_addr, REG_GCONF, &g);
    if (r != ESP_OK) return r;
    if (cw) g &= ~(1 << 4); else g |= (1 << 4);
    return _bus.write_register(_addr, REG_GCONF, g);
}

esp_err_t TMC2209Stepper::set_hold_current_percentage(uint8_t pct) {
    if (pct > 100) return ESP_ERR_INVALID_ARG;
    uint8_t irun = (_shadow_ihold_irun >> 8) & 0x1F;
    uint8_t ihold = (irun * pct) / 100;
    _shadow_ihold_irun = (_shadow_ihold_irun & ~0x1F) | ihold;
    ESP_LOGI(TAG, "Hold %u%% → IHOLD=%u", pct, ihold);
    return _bus.write_register(_addr, REG_IHOLD_IRUN, _shadow_ihold_irun);
}

esp_err_t TMC2209Stepper::set_microstep_resolution(uint8_t mres) {
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    c = (c & ~(0xF << 24)) | ((uint32_t(mres & 0xF)) << 24);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_interpolation_enable(bool en) {
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    if (en) c |= (1 << 28); else c &= ~(1 << 28);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_stealthchop_enable(bool en) {
    uint32_t g; esp_err_t r = _bus.read_register(_addr, REG_GCONF, &g);
    if (r != ESP_OK) g = 0;
    if (en) g &= ~(1 << 2); else g |= (1 << 2);
    return _bus.write_register(_addr, REG_GCONF, g);
}

esp_err_t TMC2209Stepper::set_stealthchop_threshold(uint32_t th) {
    _shadow_tpwmthrs = th;
    return _bus.write_register(_addr, REG_TPWMTHRS, _shadow_tpwmthrs);
}

esp_err_t TMC2209Stepper::set_stealthchop_pwm_gradient(uint8_t g) {
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_PWMCONF, &c);
    if (r != ESP_OK) c = 0xC40C001E;
    c = (c & ~(0xFF << 16)) | (uint32_t(g) << 16);
    return _bus.write_register(_addr, REG_PWMCONF, c);
}

esp_err_t TMC2209Stepper::set_stealthchop_pwm_amplitude(uint8_t a) {
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_PWMCONF, &c);
    if (r != ESP_OK) c = 0xC40C001E;
    c = (c & ~(0xFF << 8)) | (uint32_t(a) << 8);
    return _bus.write_register(_addr, REG_PWMCONF, c);
}

esp_err_t TMC2209Stepper::set_chopper_mode(bool off_time) {
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    if (off_time) c |= (1 << 14); else c &= ~(1 << 14);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_chopper_off_time(uint8_t toff) {
    if (toff < 1 || toff>15) return ESP_ERR_INVALID_ARG;
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    c = (c & ~0xF) | toff;
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_chopper_hysteresis_start(uint8_t h) {
    if (h > 7) return ESP_ERR_INVALID_ARG;
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    c = (c & ~(0x7 << 4)) | (uint32_t(h) << 4);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_chopper_hysteresis_end(int8_t h) {
    if (h < -3 || h>12) return ESP_ERR_INVALID_ARG;
    uint8_t hr = (h + 3) & 0xF;
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    c = (c & ~(0xF << 7)) | (uint32_t(hr) << 7);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_chopper_blank_time(uint8_t tbl) {
    if (tbl > 3) return ESP_ERR_INVALID_ARG;
    uint32_t c; esp_err_t r = _bus.read_register(_addr, REG_CHOPCONF, &c);
    if (r != ESP_OK) c = 0x10000053;
    c = (c & ~(0x3 << 15)) | (uint32_t(tbl) << 15);
    return _bus.write_register(_addr, REG_CHOPCONF, c);
}

esp_err_t TMC2209Stepper::set_stallguard_callback(gpio_num_t diag_pin, StallGuardCallback cb) {
    _diag_pin = diag_pin; _callback = cb;

    if (_diag_pin != GPIO_NUM_NC && cb) {
        gpio_config_t cfg{ 0 };
        cfg.pin_bit_mask = 1ULL << _diag_pin;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_POSEDGE;
        esp_err_t r = gpio_config(&cfg);
        if (r != ESP_OK) return r;
        r = gpio_install_isr_service(0);
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) return r;
        r = gpio_isr_handler_add(_diag_pin, stallguard_isr_handler, this);
        if (r != ESP_OK) return r;
    }
    return ESP_OK;
}

esp_err_t TMC2209Stepper::set_stallguard_enable(bool en) {
    uint32_t g; esp_err_t r = _bus.read_register(_addr, REG_GCONF, &g);
    if (r != ESP_OK) g = 0;
    if (en) g |= (1 << 2); else g &= ~(1 << 2);
    return _bus.write_register(_addr, REG_GCONF, g);
}

esp_err_t TMC2209Stepper::set_stallguard_min_speed(uint32_t min_spd) {
    _shadow_tcoolthrs = min_spd;
    return _bus.write_register(_addr, REG_TCOOLTHRS, _shadow_tcoolthrs);
}

esp_err_t TMC2209Stepper::set_coolstep_enable(bool en) {
    if (en) _shadow_coolconf |= (1 << 0); else _shadow_coolconf &= ~(1 << 0);
    return _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf);
}

esp_err_t TMC2209Stepper::set_coolstep_min_current(uint8_t min_cur) {
    if (min_cur > 1) return ESP_ERR_INVALID_ARG;
    _shadow_coolconf = (_shadow_coolconf & ~(1 << 15)) | (uint32_t(min_cur) << 15);
    return _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf);
}

esp_err_t TMC2209Stepper::set_coolstep_current_increment(uint8_t inc) {
    if (inc > 3) return ESP_ERR_INVALID_ARG;
    _shadow_coolconf = (_shadow_coolconf & ~(0x3 << 13)) | (uint32_t(inc) << 13);
    return _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf);
}

esp_err_t TMC2209Stepper::set_coolstep_upper_threshold(uint8_t th) {
    if (th > 15) return ESP_ERR_INVALID_ARG;
    _shadow_coolconf = (_shadow_coolconf & ~(0xF << 8)) | (uint32_t(th) << 8);
    return _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf);
}

esp_err_t TMC2209Stepper::set_coolstep_lower_threshold(uint8_t th) {
    if (th > 15) return ESP_ERR_INVALID_ARG;
    _shadow_coolconf = (_shadow_coolconf & ~0xF) | th;
    return _bus.write_register(_addr, REG_COOLCONF, _shadow_coolconf);
}

esp_err_t TMC2209Stepper::set_stallguard_threshold(uint8_t th) {
    // Note: Lower values = MORE sensitive (easier stall detection)
    //       Higher values = LESS sensitive (requires more load)
    //       Typical range: 50-100 for normal operation
    //       Values below 30 may cause false stall detection
    if (th < 30) {
        ESP_LOGW(TAG, "SGTHRS %u may be too sensitive, consider 30-100 range", th);
    }
    _shadow_sgthrs = th;
    return _bus.write_register(_addr, REG_SGTHRS, _shadow_sgthrs);
}

esp_err_t TMC2209Stepper::get_driver_status(uint32_t* f) {
    if (!f) return ESP_ERR_INVALID_ARG;
    return _bus.read_register(_addr, REG_DRV_STATUS, f);
}

uint16_t TMC2209Stepper::get_stallguard_result() {
    uint32_t v; if (_bus.read_register(_addr, REG_SG_RESULT, &v) != ESP_OK) return 0;
    return uint16_t(v & 0x3FF);
}

bool TMC2209Stepper::is_stalled() {
    return (get_stallguard_result() == 0);
}

uint16_t TMC2209Stepper::get_actual_current() {
    uint32_t v; if (_bus.read_register(_addr, REG_MSCURACT, &v) != ESP_OK) return 0;
    uint16_t cs = v & 0x1FF;
    // I = (CS+1)*Vfs/(32*(Rs+Rint))*1000
    const float Vfs = 0.325f, Rs = 0.11f, Ri = 0.02f;
    float Irms = (cs + 1) * Vfs / (32.0f * (Rs + Ri)) * 1000.0f;
    return uint16_t(std::round(Irms));
}

bool TMC2209Stepper::is_stealthchop_active() {
    uint32_t s; if (_bus.read_register(_addr, REG_DRV_STATUS, &s) != ESP_OK) return false;
    return (s & (1 << 30)) != 0;
}

esp_err_t TMC2209Stepper::get_current_config(uint8_t* ih, uint8_t* ir, uint8_t* id) {
    if (!ih || !ir || !id) return ESP_ERR_INVALID_ARG;
    *ih = _shadow_ihold_irun & 0x1F;
    *ir = (_shadow_ihold_irun >> 8) & 0x1F;
    *id = (_shadow_ihold_irun >> 16) & 0x0F;
    ESP_LOGD(TAG, "CFG IH=%u IR=%u ID=%u", *ih, *ir, *id);
    return ESP_OK;
}

void IRAM_ATTR TMC2209Stepper::stallguard_isr_handler(void* a) {
    static_cast<TMC2209Stepper*>(a)->handle_stallguard_interrupt();
}

void TMC2209Stepper::handle_stallguard_interrupt() {
    if (_callback) {
        bool s = gpio_get_level(_diag_pin) == 1;
        _callback(_addr, s);
    }
}
