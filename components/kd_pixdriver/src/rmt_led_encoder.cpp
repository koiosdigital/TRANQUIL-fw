#include "rmt_led_encoder.h"
#include "esp_check.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "rmt_encoder";

// C-style encoder implementation for compatibility
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t* bytes_encoder;
    rmt_encoder_t* copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static size_t rmt_encode_led_strip(rmt_encoder_t* encoder, rmt_channel_handle_t channel,
    const void* primary_data, size_t data_size,
    rmt_encode_state_t* ret_state) {
    rmt_led_strip_encoder_t* led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
        // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
            sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state = RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t* encoder) {
    rmt_led_strip_encoder_t* led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t* encoder) {
    rmt_led_strip_encoder_t* led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

extern "C" esp_err_t kd_rmt_new_led_strip_encoder(const RMTLedEncoderConfig* config, rmt_encoder_handle_t* ret_encoder) {
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t* led_encoder = nullptr;

    if (!config || !ret_encoder) {
        return ESP_ERR_INVALID_ARG;
    }

    led_encoder = static_cast<rmt_led_strip_encoder_t*>(rmt_alloc_encoder_mem(sizeof(rmt_led_strip_encoder_t)));

    if (!led_encoder) {
        return ESP_ERR_NO_MEM;
    }

    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;

    // WS2812 timing configuration
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .duration0 = static_cast<uint16_t>(0.3 * (config->resolution_hz / 1000000)), // T0H=0.3us
            .level0 = 1,
            .duration1 = static_cast<uint16_t>(0.9 * (config->resolution_hz / 1000000)), // T0L=0.9us
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = static_cast<uint16_t>(0.9 * (config->resolution_hz / 1000000)), // T1H=0.9us
            .level0 = 1,
            .duration1 = static_cast<uint16_t>(0.3 * (config->resolution_hz / 1000000)), // T1L=0.3us
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
        }
    };

    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(led_encoder);
        return ret;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return ret;
    }

    uint16_t reset_ticks = static_cast<uint16_t>((config->resolution_hz / 1000000) * 50 / 2); // reset code duration defaults to 50us
    led_encoder->reset_code = (rmt_symbol_word_t){
        .duration0 = reset_ticks,
        .level0 = 0,
        .duration1 = reset_ticks,
        .level1 = 0,
    };

    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

// C++ wrapper implementation
RMTLedEncoder::RMTLedEncoder(const RMTLedEncoderConfig& config)
    : config_(config), encoder_handle_(nullptr), initialized_(false) {
}

RMTLedEncoder::~RMTLedEncoder() {
    if (initialized_ && encoder_handle_) {
        rmt_del_encoder(encoder_handle_);
        encoder_handle_ = nullptr;
        initialized_ = false;
    }
}

esp_err_t RMTLedEncoder::initialize() {
    if (initialized_) return ESP_OK;

    esp_err_t ret = kd_rmt_new_led_strip_encoder(&config_, &encoder_handle_);
    if (ret == ESP_OK) {
        initialized_ = true;
        ESP_LOGI(TAG, "RMT LED encoder initialized with %lu Hz resolution", config_.resolution_hz);
    }

    return ret;
}

RMTLedEncoder::RMTLedEncoder(RMTLedEncoder&& other) noexcept
    : config_(other.config_), encoder_handle_(other.encoder_handle_), initialized_(other.initialized_) {
    other.encoder_handle_ = nullptr;
    other.initialized_ = false;
}

RMTLedEncoder& RMTLedEncoder::operator=(RMTLedEncoder&& other) noexcept {
    if (this != &other) {
        // Clean up current resources
        if (initialized_ && encoder_handle_) {
            rmt_del_encoder(encoder_handle_);
        }

        // Move resources
        config_ = other.config_;
        encoder_handle_ = other.encoder_handle_;
        initialized_ = other.initialized_;

        // Reset other object
        other.encoder_handle_ = nullptr;
        other.initialized_ = false;
    }
    return *this;
}