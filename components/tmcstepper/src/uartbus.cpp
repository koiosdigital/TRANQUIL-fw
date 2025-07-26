#include "uartbus.h"
#include "esp_log.h"

static const char* TAG = "UartBus";

esp_err_t UartBus::initialize(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud) {
    if (initialized_) return ESP_ERR_INVALID_STATE;

    uart_config_t cfg = {
        .baud_rate = int(baud),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_param_config(port, &cfg);
    uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port, 1024, 1024, 0, nullptr, 0);

    uart_port_ = port;
    initialized_ = true;
    ESP_LOGD(TAG, "UART init on %d at %" PRIu32 " baud", port, baud);

    return ESP_OK;
}

void UartBus::deinitialize() {
    if (!initialized_) return;
    uart_driver_delete(uart_port_);
    initialized_ = false;
    uart_port_ = UART_NUM_MAX;
}

uint8_t UartBus::calculate_crc(const uint8_t* d, size_t l) {
    uint8_t crc = 0;
    for (size_t i = 0; i < l; ++i) {
        uint8_t b = d[i];
        for (int j = 0;j < 8;++j) {
            bool mix = ((crc >> 7) ^ (b & 1)) != 0;
            crc <<= 1;
            if (mix) crc ^= 0x07;
            b >>= 1;
        }
    }
    return crc;
}

esp_err_t UartBus::write_register(uint8_t addr, uint8_t reg, uint32_t val) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    uint8_t pkt[8];
    pkt[0] = 0x05;
    pkt[1] = addr;
    pkt[2] = reg | 0x80;
    pkt[3] = (val >> 24) & 0xFF;
    pkt[4] = (val >> 16) & 0xFF;
    pkt[5] = (val >> 8) & 0xFF;
    pkt[6] = val & 0xFF;
    pkt[7] = calculate_crc(pkt, 7);
    uart_flush_input(uart_port_);
    int w = uart_write_bytes(uart_port_, (const char*)pkt, sizeof(pkt));
    if (w != int(sizeof(pkt))) return ESP_ERR_TIMEOUT;
    esp_err_t r = uart_wait_tx_done(uart_port_, pdMS_TO_TICKS(100));
    if (r != ESP_OK) return r;
    ESP_LOGD(TAG, "W (%d) @%02X<=%08lX", addr, reg, val);
    return ESP_OK;
}

esp_err_t UartBus::read_register(uint8_t addr, uint8_t reg, uint32_t* out) {
    if (!initialized_ || !out) return ESP_ERR_INVALID_ARG;
    uint8_t req[4];
    req[0] = 0x05; req[1] = addr; req[2] = reg; req[3] = calculate_crc(req, 3);
    uart_flush_input(uart_port_);
    int w = uart_write_bytes(uart_port_, (const char*)req, sizeof(req));
    if (w != int(sizeof(req))) return ESP_ERR_TIMEOUT;
    esp_err_t r = uart_wait_tx_done(uart_port_, pdMS_TO_TICKS(100));
    if (r != ESP_OK) return r;
    esp_rom_delay_us(500); // Wait for response
    uint8_t buf[12];
    int rd = uart_read_bytes(uart_port_, buf, sizeof(buf), pdMS_TO_TICKS(200));
    if (rd < 8) return ESP_ERR_TIMEOUT;
    uint8_t* p = nullptr;
    for (int i = 0;i <= rd - 8;++i) {
        if (buf[i] == 0x05 && buf[i + 1] == 0xFF) { p = &buf[i]; break; }
    }
    if (!p) return ESP_ERR_NOT_FOUND;
    if (calculate_crc(p, 7) != p[7]) return ESP_ERR_INVALID_CRC;
    *out = (uint32_t(p[3]) << 24) | (uint32_t(p[4]) << 16) | (uint32_t(p[5]) << 8) | p[6];
    ESP_LOGD(TAG, "R (%d) @%02X=>%08lX", addr, reg, *out);
    return ESP_OK;
}