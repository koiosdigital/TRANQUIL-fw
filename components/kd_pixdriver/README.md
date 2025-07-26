# KD PixDriver

A modern, efficient C++ component for driving WS2812/WS2813 LED strips with multiple channel support, current limiting, and advanced effects.

## Features

### Core Features
- **Multi-channel support**: Drive multiple independent LED strips
- **Current limiting**: Automatic power management with configurable limits
- **Modern C++ design**: Type-safe, RAII-compliant implementation
- **C compatibility layer**: Easy integration with existing C code
- **Effect engine**: Built-in effects with customizable parameters
- **NVS persistence**: Automatic configuration saving/loading

### Effects
- Solid color
- Blink
- Breathe (smooth fade in/out)
- Cyclic (rotating trail)
- Rainbow
- Color wipe
- Theater chase
- Sparkle
- Custom effects (callback-based)

### Advanced Features
- **Pixel masking**: Enable/disable individual pixels per channel
- **Current limiting**: Reserve power for other system components
- **Dual buffering**: Separate display and processing buffers
- **Configurable timing**: Adjustable update rates and effect speeds

## Usage

### Basic C++ Example

```cpp
#include "kd_pixdriver.h"

using namespace kd;

void app_main() {
    // Create driver instance
    PixelDriver driver(60); // 60Hz update rate
    
    // Configure LED strip channel
    ChannelConfig config(GPIO_NUM_5, 30, PixelFormat::RGB);
    int32_t channel_id = driver.addChannel(config);
    
    // Set current limit to 2A (2000mA)
    driver.setCurrentLimit(2000);
    
    // Get channel and set effect
    auto* channel = driver.getChannel(channel_id);
    if (channel) {
        EffectConfig effect;
        effect.effect = PixelEffect::RAINBOW;
        effect.brightness = 128;
        effect.speed = 5;
        channel->setEffect(effect);
    }
    
    // Start the driver
    driver.start();
}
```

### C Compatibility Example

```c
#include "kd_pixdriver_c.h"

void app_main() {
    // Create driver
    kd_pixdriver_handle_t driver = kd_pixdriver_create(60);
    
    // Add channel
    kd_channel_config_t config = {
        .pin = GPIO_NUM_5,
        .pixel_count = 30,
        .format = KD_PIXEL_FORMAT_RGB,
        .resolution_hz = 10000000
    };
    int32_t channel_id = kd_pixdriver_add_channel(driver, &config);
    
    // Set current limit
    kd_pixdriver_set_current_limit(driver, 2000);
    
    // Configure effect
    kd_effect_config_t effect = {
        .effect = KD_PIXEL_EFFECT_RAINBOW,
        .color = {255, 0, 0, 0},
        .brightness = 128,
        .speed = 5,
        .enabled = true
    };
    kd_pixdriver_set_channel_effect(driver, channel_id, &effect);
    
    // Start driver
    kd_pixdriver_start(driver);
}
```

## Current Limiting

The driver supports automatic current limiting to prevent power supply overload:

```cpp
// Set 3A limit, reserving 400mA for system components
driver.setCurrentLimit(3000);

// Monitor actual current consumption
uint32_t total_current = driver.getTotalCurrentConsumption();
uint32_t scaled_current = driver.getScaledCurrentConsumption();
float scale_factor = driver.getCurrentScaleFactor();
```

Current calculations:
- Each color channel at full brightness (255) consumes 20mA
- RGB pixel at full white: 60mA
- RGBW pixel at full white: 80mA
- System reserves 400mA for other components
- Automatic scaling when current exceeds limit

## Multi-Channel Configuration

```cpp
PixelDriver driver;

// Add multiple channels
auto channel1 = driver.addChannel(ChannelConfig(GPIO_NUM_5, 30, PixelFormat::RGB));
auto channel2 = driver.addChannel(ChannelConfig(GPIO_NUM_18, 60, PixelFormat::RGBW));
auto channel3 = driver.addChannel(ChannelConfig(GPIO_NUM_19, 12, PixelFormat::RGB));

// Configure each channel independently
driver.getChannel(channel1)->setEffect({PixelEffect::RAINBOW, {}, 255, 7, true});
driver.getChannel(channel2)->setEffect({PixelEffect::BREATHE, {0, 255, 0}, 128, 3, true});
driver.getChannel(channel3)->setEffect({PixelEffect::SPARKLE, {255, 255, 255}, 200, 8, true});
```

## Custom Effects

```cpp
EffectConfig config;
config.effect = PixelEffect::CUSTOM;
config.custom_effect = [](std::vector<PixelColor>& buffer, uint32_t tick) {
    // Custom effect implementation
    for (size_t i = 0; i < buffer.size(); ++i) {
        uint8_t phase = (tick + i * 10) % 256;
        buffer[i] = PixelColor::fromHSV(phase, 255, 128);
    }
};

channel->setEffect(config);
```

## Pixel Masking

```cpp
// Create mask: enable every other pixel
std::vector<uint8_t> mask(30);
for (size_t i = 0; i < mask.size(); ++i) {
    mask[i] = (i % 2 == 0) ? 1 : 0;
}

channel->setMask(mask);
```

## Performance Characteristics

- **Memory usage**: ~100 bytes per channel + 3-4 bytes per pixel
- **CPU usage**: <5% at 60Hz with 4 channels, 120 pixels total
- **Update rate**: Configurable, recommended 30-120Hz
- **Max channels**: Limited by available GPIO pins and memory
- **Max pixels per channel**: Limited by available memory (~1000+ typical)

## Thread Safety

The driver uses FreeRTOS tasks and is designed for:
- **Configuration**: Thread-safe (can be called from any task)
- **Effect updates**: Handled by dedicated driver task
- **NVS operations**: Atomic save/load operations

## Power Considerations

Current limiting helps prevent:
- Power supply overload
- Voltage drop issues  
- System brownouts
- Excessive heat generation

The driver automatically scales pixel brightness to stay within current limits while preserving relative color relationships.

## Integration with Existing Code

The component provides both C++ and C interfaces for easy integration:
- Use C++ interface for new code (recommended)
- Use C interface for integration with existing C codebases
- Drop-in replacement for basic LED strip functionality
- Enhanced features like multi-channel and current limiting

## Configuration Persistence

Channel configurations are automatically saved to NVS and restored on boot:
- Effect type and parameters
- Color and brightness settings
- Enable/disable state
- Speed settings

No manual save/load calls required - configurations persist across reboots automatically.
