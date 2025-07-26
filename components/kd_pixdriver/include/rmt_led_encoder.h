#pragma once

#include "driver/rmt_encoder.h"
#include <cstdint>

struct RMTLedEncoderConfig {
    uint32_t resolution_hz;
};

// Modern C++ wrapper for RMT LED strip encoder
class RMTLedEncoder {
public:
    explicit RMTLedEncoder(const RMTLedEncoderConfig& config);
    ~RMTLedEncoder();

    esp_err_t initialize();
    rmt_encoder_handle_t getHandle() const { return encoder_handle_; }

    // Non-copyable, movable
    RMTLedEncoder(const RMTLedEncoder&) = delete;
    RMTLedEncoder& operator=(const RMTLedEncoder&) = delete;
    RMTLedEncoder(RMTLedEncoder&& other) noexcept;
    RMTLedEncoder& operator=(RMTLedEncoder&& other) noexcept;

private:
    RMTLedEncoderConfig config_;
    rmt_encoder_handle_t encoder_handle_ = nullptr;
    bool initialized_ = false;
};

// C-style encoder implementation (similar to original but cleaner)
extern "C" {
    esp_err_t kd_rmt_new_led_strip_encoder(const RMTLedEncoderConfig* config, rmt_encoder_handle_t* ret_encoder);
}
