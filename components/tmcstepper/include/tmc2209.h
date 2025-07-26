#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "uartbus.h"

enum MicrostepResolution : uint8_t {
    FULL = 8,           // 1/1 step
    HALF = 7,           // 1/2 step
    QUARTER = 6,        // 1/4 step
    EIGHTH = 5,         // 1/8 step
    SIXTEENTH = 4,      // 1/16 step
    THIRTYSECOND = 3,   // 1/32 step
    SIXTYFOURTH = 2,    // 1/64 step
    ONETWENTYEIGHTH = 1, // 1/128 step
    TWOFIFTYSIXTH = 0, // 1/256 step
};

class TMC2209Stepper {
public:
    using StallGuardCallback = void(*)(uint8_t addr, bool stalled);

    TMC2209Stepper(UartBus& bus, uint8_t addr);
    esp_err_t initialize();
    esp_err_t set_direction(bool cw);
    esp_err_t set_motor_current(uint16_t mA);
    esp_err_t set_hold_current_percentage(uint8_t pct);
    esp_err_t set_microstep_resolution(uint8_t mres);
    esp_err_t set_interpolation_enable(bool en);
    esp_err_t set_stealthchop_enable(bool en);
    esp_err_t set_stealthchop_threshold(uint32_t thrs);
    esp_err_t set_stealthchop_pwm_gradient(uint8_t grad);
    esp_err_t set_stealthchop_pwm_amplitude(uint8_t amp);
    esp_err_t set_chopper_mode(bool constant_off_time);
    esp_err_t set_chopper_off_time(uint8_t toff);
    esp_err_t set_chopper_hysteresis_start(uint8_t hstrt);
    esp_err_t set_chopper_hysteresis_end(int8_t hend);
    esp_err_t set_chopper_blank_time(uint8_t tbl);
    esp_err_t set_stallguard_enable(bool en);
    esp_err_t set_stallguard_callback(gpio_num_t diag_pin, StallGuardCallback cb);
    esp_err_t set_stallguard_threshold(uint8_t th);
    esp_err_t set_stallguard_min_speed(uint32_t min_spd);
    esp_err_t set_coolstep_enable(bool en);
    esp_err_t set_coolstep_min_current(uint8_t min_cur);
    esp_err_t set_coolstep_current_increment(uint8_t inc);
    esp_err_t set_coolstep_upper_threshold(uint8_t th);
    esp_err_t set_coolstep_lower_threshold(uint8_t th);
    esp_err_t get_driver_status(uint32_t* flags);

    bool      is_stalled();
    uint16_t  get_stallguard_result();
    uint16_t  get_actual_current();
    bool      is_stealthchop_active();
    esp_err_t reset_to_defaults();
    esp_err_t get_current_config(uint8_t* ihold, uint8_t* irun, uint8_t* ihdly);

private:
    static void IRAM_ATTR stallguard_isr_handler(void* arg);
    void handle_stallguard_interrupt();
    uint8_t calculate_current_scale(uint16_t mA);

    UartBus& _bus;
    uint8_t     _addr;
    gpio_num_t  _diag_pin;
    StallGuardCallback _callback;
    bool        _initialized;

    // Shadow registers for write-only registers
    uint32_t _shadow_ihold_irun;
    uint32_t _shadow_tpowerdown;
    uint32_t _shadow_tpwmthrs;
    uint32_t _shadow_vactual;
    uint32_t _shadow_tcoolthrs;
    uint32_t _shadow_sgthrs;
    uint32_t _shadow_coolconf;

    // Register addresses
    enum : uint8_t {
        REG_GCONF = 0x00,
        REG_GSTAT = 0x01,
        REG_IFCNT = 0x02,
        REG_NODECONF = 0x03,
        REG_OTPPROG = 0x04,
        REG_OTPREAD = 0x05,
        REG_IOIN = 0x06,
        REG_FACTORYCONF = 0x07,
        REG_IHOLD_IRUN = 0x10,
        REG_TPOWERDOWN = 0x11,
        REG_TSTEP = 0x12,
        REG_TPWMTHRS = 0x13,
        REG_TCOOLTHRS = 0x14,
        REG_THIGH = 0x15,
        REG_VACTUAL = 0x22,
        REG_SGTHRS = 0x40,
        REG_SG_RESULT = 0x41,
        REG_COOLCONF = 0x42,
        REG_MSCNT = 0x6A,
        REG_MSCURACT = 0x6B,
        REG_CHOPCONF = 0x6C,
        REG_DCCTRL = 0x6E,
        REG_DRV_STATUS = 0x6F,
        REG_PWMCONF = 0x70,
        REG_PWM_SCALE = 0x71,
        REG_PWM_AUTO = 0x72
    };
};