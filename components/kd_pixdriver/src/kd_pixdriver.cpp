#include "kd_pixdriver.h"
#include "pixel_effects.h"
#include "rmt_led_encoder.h"
#include "esp_log.h"
#include "nvs.h"
#include "driver/rmt_tx.h"
#include <algorithm>
#include <cstring>
#include <cmath>

static const char* TAG = "kd_pixdriver";
static const char* NVS_NAMESPACE = "pixdriver";

// PixelColor implementation
PixelColor PixelColor::fromHSV(uint8_t hue, uint8_t saturation, uint8_t value) {
    uint8_t r, g, b;

    if (saturation == 0) {
        return PixelColor(value, value, value, 0);
    }

    uint8_t region = hue / 43;
    uint8_t remainder = (hue - (region * 43)) * 6;

    uint8_t p = (value * (255 - saturation)) >> 8;
    uint8_t q = (value * (255 - ((saturation * remainder) >> 8))) >> 8;
    uint8_t t = (value * (255 - ((saturation * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
    case 0: r = value; g = t; b = p; break;
    case 1: r = q; g = value; b = p; break;
    case 2: r = p; g = value; b = t; break;
    case 3: r = p; g = q; b = value; break;
    case 4: r = t; g = p; b = value; break;
    default: r = value; g = p; b = q; break;
    }

    return PixelColor(r, g, b, 0);
}

PixelColor PixelColor::scale(uint8_t brightness) const {
    if (brightness == 255) return *this;

    return PixelColor(
        (r * brightness) / 255,
        (g * brightness) / 255,
        (b * brightness) / 255,
        (w * brightness) / 255
    );
}

// PixelDriver implementation
PixelDriver::PixelDriver(uint32_t update_rate_hz)
    : update_rate_hz_(update_rate_hz) {
    effect_engine_ = std::make_unique<PixelEffectEngine>(update_rate_hz);
    ESP_LOGI(TAG, "PixelDriver initialized with %lu Hz update rate", update_rate_hz);
}

PixelDriver::~PixelDriver() {
    stop();
    channels_.clear();
}

int32_t PixelDriver::addChannel(const ChannelConfig& config) {
    int32_t id = next_channel_id_++;
    auto channel = std::make_unique<PixelChannel>(id, config);

    if (!channel->initialize()) {
        ESP_LOGE(TAG, "Failed to initialize channel %ld", id);
        return -1;
    }

    // Load persisted configuration
    channel->loadFromNVS();

    channels_.emplace_back(std::move(channel));

    ESP_LOGI(TAG, "Added channel %ld: pin %d, %d pixels, %s",
        id, config.pin, config.pixel_count,
        config.format == PixelFormat::RGBW ? "RGBW" : "RGB");

    return id;
}

bool PixelDriver::removeChannel(int32_t channel_id) {
    auto it = std::find_if(channels_.begin(), channels_.end(),
        [channel_id](const auto& channel) {
            return channel->getId() == channel_id;
        });

    if (it != channels_.end()) {
        channels_.erase(it);
        ESP_LOGI(TAG, "Removed channel %ld", channel_id);
        return true;
    }

    return false;
}

PixelChannel* PixelDriver::getChannel(int32_t channel_id) {
    auto it = std::find_if(channels_.begin(), channels_.end(),
        [channel_id](const auto& channel) {
            return channel->getId() == channel_id;
        });

    return (it != channels_.end()) ? it->get() : nullptr;
}

std::vector<int32_t> PixelDriver::getChannelIds() const {
    std::vector<int32_t> ids;
    ids.reserve(channels_.size());

    for (const auto& channel : channels_) {
        ids.push_back(channel->getId());
    }

    return ids;
}

void PixelDriver::setCurrentLimit(int32_t limit_ma) {
    current_limit_ma_ = limit_ma;
    ESP_LOGI(TAG, "Current limit set to %ld mA", limit_ma);
}

void PixelDriver::setUpdateRate(uint32_t rate_hz) {
    update_rate_hz_ = rate_hz;
    effect_engine_ = std::make_unique<PixelEffectEngine>(rate_hz);
}

void PixelDriver::start() {
    if (running_) return;

    running_ = true;
    xTaskCreate(driverTaskWrapper, "pixdriver", 4096, this, 5, &task_handle_);
    ESP_LOGI(TAG, "PixelDriver started");
}

void PixelDriver::stop() {
    if (!running_) return;

    running_ = false;
    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "PixelDriver stopped");
}

void PixelDriver::setAllChannelsEffect(PixelEffect effect) {
    for (auto& channel : channels_) {
        EffectConfig config = channel->getEffectConfig();
        config.effect = effect;
        channel->setEffect(config);
    }
}

void PixelDriver::setAllChannelsColor(const PixelColor& color) {
    for (auto& channel : channels_) {
        channel->setColor(color);
    }
}

void PixelDriver::setAllChannelsBrightness(uint8_t brightness) {
    for (auto& channel : channels_) {
        channel->setBrightness(brightness);
    }
}

void PixelDriver::setAllChannelsEnabled(bool enabled) {
    for (auto& channel : channels_) {
        channel->setEnabled(enabled);
    }
}

uint32_t PixelDriver::getTotalCurrentConsumption() const {
    uint32_t total = 0;
    for (const auto& channel : channels_) {
        total += channel->getCurrentConsumption();
    }
    return total;
}

uint32_t PixelDriver::getScaledCurrentConsumption() const {
    float scale = getCurrentScaleFactor();
    return static_cast<uint32_t>(getTotalCurrentConsumption() * scale);
}

float PixelDriver::getCurrentScaleFactor() const {
    if (current_limit_ma_ <= 0) return 1.0f;

    uint32_t total_current = getTotalCurrentConsumption();
    uint32_t available_current = (current_limit_ma_ > SYSTEM_RESERVE_MA) ?
        (current_limit_ma_ - SYSTEM_RESERVE_MA) : 0;

    if (total_current <= available_current) return 1.0f;
    if (available_current == 0) return 0.0f;

    return static_cast<float>(available_current) / total_current;
}

void PixelDriver::driverTaskWrapper(void* param) {
    static_cast<PixelDriver*>(param)->driverTask();
}

void PixelDriver::driverTask() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(1000 / update_rate_hz_);
    uint32_t tick = 0;

    while (running_) {
        // Update effects for all channels
        for (auto& channel : channels_) {
            if (channel->getEffectConfig().enabled) {
                effect_engine_->updateEffect(channel.get(), tick);
            }
        }

        // Apply current limiting and transmit
        applyCurrentLimiting();

        for (auto& channel : channels_) {
            channel->transmit();
        }

        tick++;
        vTaskDelayUntil(&last_wake_time, update_period);
    }

    vTaskDelete(nullptr);
}

void PixelDriver::applyCurrentLimiting() {
    float scale_factor = getCurrentScaleFactor();

    for (auto& channel : channels_) {
        channel->applyCurrentScaling(scale_factor);
    }
}

// PixelChannel implementation
PixelChannel::PixelChannel(int32_t id, const ChannelConfig& config)
    : id_(id), config_(config), initialized_(false) {

    pixel_buffer_.resize(config.pixel_count, PixelColor(0, 0, 0, 0));
    scaled_buffer_.resize(config.pixel_count, PixelColor(0, 0, 0, 0));

    // Initialize with default effect
    effect_config_.effect = PixelEffect::SOLID;
    effect_config_.color = PixelColor(100, 100, 100, 0);
    effect_config_.brightness = 255;
    effect_config_.speed = 5;
    effect_config_.enabled = true;
}

PixelChannel::~PixelChannel() {
    cleanup();
}

bool PixelChannel::initialize() {
    if (initialized_) return true;

    setupRMT();
    initialized_ = true;
    ESP_LOGI(TAG, "Channel %ld initialized successfully", id_);
    return true;
}

void PixelChannel::setEffect(const EffectConfig& effect_config) {
    effect_config_ = effect_config;

    // Resize mask if provided
    if (!effect_config.mask.empty() && effect_config.mask.size() == config_.pixel_count) {
        setMask(effect_config.mask);
    }

    saveToNVS();
}

void PixelChannel::setColor(const PixelColor& color) {
    effect_config_.color = color;
    saveToNVS();
}

void PixelChannel::setBrightness(uint8_t brightness) {
    effect_config_.brightness = brightness;
    saveToNVS();
}

void PixelChannel::setSpeed(uint8_t speed) {
    effect_config_.speed = std::clamp(speed, uint8_t(1), uint8_t(10));
    saveToNVS();
}

void PixelChannel::setEnabled(bool enabled) {
    effect_config_.enabled = enabled;
    saveToNVS();
}

void PixelChannel::setMask(const std::vector<uint8_t>& mask) {
    if (mask.size() == config_.pixel_count) {
        effect_config_.mask = mask;
    }
}

void PixelChannel::clearMask() {
    effect_config_.mask.clear();
}

void PixelChannel::setupRMT() {
    // Create RMT TX channel
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = config_.pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config_.resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    rmt_channel_handle_t rmt_handle = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &rmt_handle));
    rmt_channel_ = rmt_handle;

    // Create LED strip encoder
    RMTLedEncoderConfig encoder_config = {
        .resolution_hz = config_.resolution_hz
    };

    rmt_encoder_handle_t encoder_handle = nullptr;
    ESP_ERROR_CHECK(kd_rmt_new_led_strip_encoder(&encoder_config, &encoder_handle));
    rmt_encoder_ = encoder_handle;

    ESP_ERROR_CHECK(rmt_enable(rmt_handle));
}

void PixelChannel::cleanup() {
    if (initialized_) {
        if (rmt_channel_) {
            rmt_disable(*(rmt_channel_handle_t*)&rmt_channel_);
            rmt_del_channel(*(rmt_channel_handle_t*)&rmt_channel_);
            rmt_channel_ = nullptr;
        }

        if (rmt_encoder_) {
            rmt_del_encoder(*(rmt_encoder_handle_t*)&rmt_encoder_);
            rmt_encoder_ = nullptr;
        }

        initialized_ = false;
    }
}

std::vector<uint8_t> PixelChannel::convertToRMTBuffer(const std::vector<PixelColor>& pixels) {
    size_t bytes_per_pixel = static_cast<size_t>(config_.format);
    rmt_buffer_.resize(pixels.size() * bytes_per_pixel);

    for (size_t i = 0; i < pixels.size(); ++i) {
        const auto& pixel = pixels[i];
        size_t base_idx = i * bytes_per_pixel;

        // Apply mask if present
        bool masked = effect_config_.mask.empty() ||
            (i < effect_config_.mask.size() && effect_config_.mask[i]);

        if (config_.format == PixelFormat::RGBW) {
            // GRBW order for WS2812-style LEDs
            rmt_buffer_[base_idx + 0] = masked ? pixel.g : 0;
            rmt_buffer_[base_idx + 1] = masked ? pixel.r : 0;
            rmt_buffer_[base_idx + 2] = masked ? pixel.b : 0;
            rmt_buffer_[base_idx + 3] = masked ? pixel.w : 0;
        }
        else {
            // GRB order for WS2812-style LEDs
            rmt_buffer_[base_idx + 0] = masked ? pixel.g : 0;
            rmt_buffer_[base_idx + 1] = masked ? pixel.r : 0;
            rmt_buffer_[base_idx + 2] = masked ? pixel.b : 0;
        }
    }

    return rmt_buffer_;
}

void PixelChannel::transmit() {
    if (!initialized_ || !effect_config_.enabled) return;

    auto buffer_to_transmit = convertToRMTBuffer(scaled_buffer_);

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(
        *(rmt_channel_handle_t*)&rmt_channel_,
        *(rmt_encoder_handle_t*)&rmt_encoder_,
        buffer_to_transmit.data(),
        buffer_to_transmit.size(),
        &tx_config
    ));

    // Wait for transmission to complete
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(
        *(rmt_channel_handle_t*)&rmt_channel_,
        portMAX_DELAY
    ));
}

uint32_t PixelChannel::getCurrentConsumption() const {
    uint32_t total_ma = 0;

    for (const auto& pixel : pixel_buffer_) {
        // Each color channel at 255 brightness consumes 20mA
        total_ma += (pixel.r * PixelDriver::CURRENT_PER_CHANNEL_MA) / 255;
        total_ma += (pixel.g * PixelDriver::CURRENT_PER_CHANNEL_MA) / 255;
        total_ma += (pixel.b * PixelDriver::CURRENT_PER_CHANNEL_MA) / 255;
        if (config_.format == PixelFormat::RGBW) {
            total_ma += (pixel.w * PixelDriver::CURRENT_PER_CHANNEL_MA) / 255;
        }
    }

    return total_ma;
}

void PixelChannel::applyCurrentScaling(float scale_factor) {
    if (scale_factor >= 1.0f) {
        // No scaling needed
        scaled_buffer_ = pixel_buffer_;
    }
    else {
        // Scale all pixels
        for (size_t i = 0; i < pixel_buffer_.size(); ++i) {
            const auto& original = pixel_buffer_[i];
            scaled_buffer_[i] = PixelColor(
                static_cast<uint8_t>(original.r * scale_factor),
                static_cast<uint8_t>(original.g * scale_factor),
                static_cast<uint8_t>(original.b * scale_factor),
                static_cast<uint8_t>(original.w * scale_factor)
            );
        }
    }
}

void PixelChannel::saveToNVS() const {
    nvs_handle_t nvs_handle;
    char key[32];
    snprintf(key, sizeof(key), "ch_%ld", id_);

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for channel %ld: %s", id_, esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(nvs_handle, key, &effect_config_, sizeof(EffectConfig));
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
    }
    else {
        ESP_LOGW(TAG, "Failed to save channel %ld config: %s", id_, esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
}

void PixelChannel::loadFromNVS() {
    nvs_handle_t nvs_handle;
    char key[32];
    snprintf(key, sizeof(key), "ch_%ld", id_);

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved config for channel %ld, using defaults", id_);
        return;
    }

    size_t required_size = sizeof(EffectConfig);
    err = nvs_get_blob(nvs_handle, key, &effect_config_, &required_size);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Failed to load config for channel %ld, using defaults", id_);
    }
    else {
        ESP_LOGI(TAG, "Loaded config for channel %ld", id_);
    }

    nvs_close(nvs_handle);
}
