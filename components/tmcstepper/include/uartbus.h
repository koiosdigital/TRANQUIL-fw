#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <stdint.h>

class UartBus {
public:
    esp_err_t initialize(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud);
    void      deinitialize();
    bool      is_initialized() const { return initialized_; }
    esp_err_t write_register(uint8_t addr, uint8_t reg, uint32_t val);
    esp_err_t read_register(uint8_t addr, uint8_t reg, uint32_t* pval);

private:
    uint8_t  calculate_crc(const uint8_t* data, size_t len);
    uart_port_t uart_port_{ UART_NUM_MAX };
    bool      initialized_{ false };
};