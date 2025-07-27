#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <functional>
#include <string>
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Forward declarations
class PixelChannel;
class PixelEffectEngine;

// I2S callback function
extern "C" bool i2s_tx_callback(i2s_chan_handle_t handle, i2s_event_data_t* event, void* user_ctx);

enum class PixelFormat : uint8_t {
    RGB = 3,
    RGBW = 4
};

enum class PixelEffect : uint8_t {
    SOLID,
    BLINK,
    BREATHE,
    CYCLIC,
    RAINBOW,
    COLOR_WIPE,
    THEATER_CHASE,
    SPARKLE,
    CUSTOM
};

struct PixelColor {
    uint8_t r = 0, g = 0, b = 0, w = 0;

    PixelColor() = default;
    PixelColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t white = 0)
        : r(red), g(green), b(blue), w(white) {
    }

    // HSV constructor
    static PixelColor fromHSV(uint8_t hue, uint8_t saturation, uint8_t value);

    // Scale color by brightness (0-255)
    PixelColor scale(uint8_t brightness) const;
};

struct ChannelConfig {
    gpio_num_t pin;
    uint16_t pixel_count;
    PixelFormat format = PixelFormat::RGB;
    uint32_t resolution_hz = 10000000;  // 10MHz default
    std::string name;

    ChannelConfig(gpio_num_t gpio_pin, uint16_t count, PixelFormat fmt = PixelFormat::RGB)
        : pin(gpio_pin), pixel_count(count), format(fmt) {
    }
};

struct EffectConfig {
    PixelEffect effect = PixelEffect::SOLID;
    PixelColor color{ 100, 100, 100, 0 };
    uint8_t brightness = 255;
    uint8_t speed = 10;  // 1-10 scale
    bool enabled = true;
    std::vector<uint8_t> mask;  // Optional pixel mask

    // Custom effect callback
    std::function<void(std::vector<PixelColor>&, uint32_t)> custom_effect;
};

// Forward declarations
class PixelChannel;
class PixelEffectEngine;

class PixelDriver {
public:
    static constexpr uint8_t CURRENT_PER_CHANNEL_MA = 20;  // mA per color channel at full brightness
    static constexpr uint32_t SYSTEM_RESERVE_MA = 400;     // Reserve for other components

    explicit PixelDriver(uint32_t update_rate_hz = 60);
    ~PixelDriver();

    // Channel management
    int32_t addChannel(const ChannelConfig& config);
    bool removeChannel(int32_t channel_id);
    PixelChannel* getChannel(int32_t channel_id);
    std::vector<int32_t> getChannelIds() const;

    // Global settings
    void setCurrentLimit(int32_t limit_ma);  // -1 for unlimited
    int32_t getCurrentLimit() const { return current_limit_ma_; }

    void setUpdateRate(uint32_t rate_hz);
    uint32_t getUpdateRate() const { return update_rate_hz_; }

    // Control
    void start();
    void stop();
    bool isRunning() const { return running_; }

    // Global effects
    void setAllChannelsEffect(PixelEffect effect);
    void setAllChannelsColor(const PixelColor& color);
    void setAllChannelsBrightness(uint8_t brightness);
    void setAllChannelsEnabled(bool enabled);

    // Statistics
    uint32_t getTotalCurrentConsumption() const;
    uint32_t getScaledCurrentConsumption() const;
    float getCurrentScaleFactor() const;

private:
    void driverTask();
    static void driverTaskWrapper(void* param);
    void updateChannels();
    void applyCurrentLimiting();

    std::vector<std::unique_ptr<PixelChannel>> channels_;
    std::unique_ptr<PixelEffectEngine> effect_engine_;

    TaskHandle_t task_handle_ = nullptr;
    int32_t current_limit_ma_ = -1;
    uint32_t update_rate_hz_;
    bool running_ = false;
    int32_t next_channel_id_ = 0;
};

class PixelChannel {
public:
    PixelChannel(int32_t id, const ChannelConfig& config);
    ~PixelChannel();

    // Friend function for I2S callback
    friend bool i2s_tx_callback(i2s_chan_handle_t handle, i2s_event_data_t* event, void* user_ctx);

    // Configuration
    int32_t getId() const { return id_; }
    const ChannelConfig& getConfig() const { return config_; }

    // Effect control
    void setEffect(const EffectConfig& effect_config);
    const EffectConfig& getEffectConfig() const { return effect_config_; }

    void setColor(const PixelColor& color);
    void setBrightness(uint8_t brightness);
    void setSpeed(uint8_t speed);
    void setEnabled(bool enabled);
    void setMask(const std::vector<uint8_t>& mask);
    void clearMask();

    // Buffer access
    const std::vector<PixelColor>& getPixelBuffer() const { return pixel_buffer_; }
    std::vector<PixelColor>& getPixelBuffer() { return pixel_buffer_; }

    // Hardware interface
    bool initialize();
    void transmit();
    uint32_t getCurrentConsumption() const;
    void applyCurrentScaling(float scale_factor);

    // Persistence
    void saveToNVS() const;
    void loadFromNVS();

private:
    void setupI2S();
    void cleanup();
    std::vector<uint8_t> convertToI2SBuffer(const std::vector<PixelColor>& pixels);
    static void i2sTaskWrapper(void* param);
    void i2sTask();
    static bool i2sTxCallback(i2s_chan_handle_t handle, i2s_event_data_t* event, void* user_ctx);

    int32_t id_;
    ChannelConfig config_;
    EffectConfig effect_config_;

    std::vector<PixelColor> pixel_buffer_;
    std::vector<PixelColor> scaled_buffer_;
    std::vector<uint8_t> i2s_buffer_;

    i2s_chan_handle_t i2s_channel_ = nullptr;
    SemaphoreHandle_t transmit_semaphore_ = nullptr;
    SemaphoreHandle_t complete_semaphore_ = nullptr;
    TaskHandle_t i2s_task_handle_ = nullptr;
    bool initialized_ = false;
    bool terminate_task_ = false;
    size_t bytes_sent_ = 0;
};