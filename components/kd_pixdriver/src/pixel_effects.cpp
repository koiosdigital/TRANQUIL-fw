#include "pixel_effects.h"
#include "esp_log.h"
#include "esp_random.h"
#include <algorithm>
#include <cmath>

PixelEffectEngine::PixelEffectEngine(uint32_t update_rate_hz)
    : update_rate_hz_(update_rate_hz) {
    // Reserve space for a reasonable number of channels
    channel_states_.reserve(8);
}

void PixelEffectEngine::updateEffect(PixelChannel* channel, uint32_t tick) {
    if (!channel) return;

    const auto& config = channel->getEffectConfig();
    ensureChannelState(channel->getId());

    switch (config.effect) {
    case PixelEffect::SOLID:
        applySolid(channel);
        break;
    case PixelEffect::BLINK:
        applyBlink(channel, tick);
        break;
    case PixelEffect::BREATHE:
        applyBreathe(channel, tick);
        break;
    case PixelEffect::CYCLIC:
        applyCyclic(channel, tick);
        break;
    case PixelEffect::RAINBOW:
        applyRainbow(channel, tick);
        break;
    case PixelEffect::COLOR_WIPE:
        applyColorWipe(channel, tick);
        break;
    case PixelEffect::THEATER_CHASE:
        applyTheaterChase(channel, tick);
        break;
    case PixelEffect::SPARKLE:
        applySparkle(channel, tick);
        break;
    case PixelEffect::CUSTOM:
        applyCustom(channel, tick);
        break;
    }
}

void PixelEffectEngine::applySolid(PixelChannel* channel) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();

    PixelColor scaled_color = config.color.scale(config.brightness);

    std::fill(buffer.begin(), buffer.end(), scaled_color);
}

void PixelEffectEngine::applyBlink(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed);

    if (tick - state.last_update_tick >= interval) {
        state.direction = !state.direction;  // Toggle blink state
        state.last_update_tick = tick;
    }

    if (state.direction) {
        PixelColor scaled_color = config.color.scale(config.brightness);
        std::fill(buffer.begin(), buffer.end(), scaled_color);
    }
    else {
        std::fill(buffer.begin(), buffer.end(), PixelColor(0, 0, 0, 0));
    }
}

void PixelEffectEngine::applyBreathe(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed) / 4;  // Smoother breathing

    if (tick - state.last_update_tick >= interval) {
        if (state.breathe.increasing) {
            state.breathe.current_brightness += 5;
            if (state.breathe.current_brightness >= config.brightness) {
                state.breathe.current_brightness = config.brightness;
                state.breathe.increasing = false;
            }
        }
        else {
            state.breathe.current_brightness -= 5;
            if (state.breathe.current_brightness <= 0) {
                state.breathe.current_brightness = 0;
                state.breathe.increasing = true;
            }
        }
        state.last_update_tick = tick;
    }

    PixelColor scaled_color = config.color.scale(state.breathe.current_brightness);
    std::fill(buffer.begin(), buffer.end(), scaled_color);
}

void PixelEffectEngine::applyCyclic(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed);

    if (tick - state.last_update_tick >= interval) {
        state.cyclic.trail_offset = (state.cyclic.trail_offset + 1) % buffer.size();
        state.last_update_tick = tick;
    }

    // Clear buffer
    std::fill(buffer.begin(), buffer.end(), PixelColor(0, 0, 0, 0));

    // Draw trail
    const int trail_length = std::min(5, static_cast<int>(buffer.size()));
    PixelColor base_color = config.color.scale(config.brightness);

    for (int i = 0; i < trail_length; ++i) {
        int pixel_idx = (state.cyclic.trail_offset + i) % buffer.size();
        uint8_t fade = 255 - (i * 255 / trail_length);
        buffer[pixel_idx] = base_color.scale(fade);
    }
}

void PixelEffectEngine::applyRainbow(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed);

    if (tick - state.last_update_tick >= interval) {
        state.rainbow.rainbow_offset = (state.rainbow.rainbow_offset + 1) % 256;
        state.last_update_tick = tick;
    }

    for (size_t i = 0; i < buffer.size(); ++i) {
        uint8_t hue = static_cast<uint8_t>((i * 256 / buffer.size()) + state.rainbow.rainbow_offset) % 256;
        buffer[i] = PixelColor::fromHSV(hue, 255, config.brightness);
    }
}

void PixelEffectEngine::applyColorWipe(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed);

    if (tick - state.last_update_tick >= interval) {
        if (!state.color_wipe.clearing) {
            if (state.color_wipe.current_pixel < buffer.size()) {
                state.color_wipe.current_pixel++;
            }
            else {
                state.color_wipe.clearing = true;
                state.color_wipe.current_pixel = 0;
            }
        }
        else {
            if (state.color_wipe.current_pixel < buffer.size()) {
                state.color_wipe.current_pixel++;
            }
            else {
                state.color_wipe.clearing = false;
                state.color_wipe.current_pixel = 0;
            }
        }
        state.last_update_tick = tick;
    }

    PixelColor fill_color = state.color_wipe.clearing ?
        PixelColor(0, 0, 0, 0) : config.color.scale(config.brightness);

    // Fill pixels up to current position
    for (size_t i = 0; i < state.color_wipe.current_pixel && i < buffer.size(); ++i) {
        buffer[i] = fill_color;
    }

    // Clear remaining pixels if not clearing phase
    if (!state.color_wipe.clearing) {
        for (size_t i = state.color_wipe.current_pixel; i < buffer.size(); ++i) {
            buffer[i] = PixelColor(0, 0, 0, 0);
        }
    }
}

void PixelEffectEngine::applyTheaterChase(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed);

    if (tick - state.last_update_tick >= interval) {
        state.theater_chase.offset = (state.theater_chase.offset + 1) % 3;
        state.last_update_tick = tick;
    }

    PixelColor on_color = config.color.scale(config.brightness);

    for (size_t i = 0; i < buffer.size(); ++i) {
        buffer[i] = ((i + state.theater_chase.offset) % 3 == 0) ?
            on_color : PixelColor(0, 0, 0, 0);
    }
}

void PixelEffectEngine::applySparkle(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();
    auto& buffer = channel->getPixelBuffer();
    auto& state = channel_states_[channel->getId()];

    uint32_t interval = getEffectInterval(config.speed) / 2;  // Faster sparkle

    if (tick - state.last_update_tick >= interval) {
        // Clear all pixels first
        std::fill(buffer.begin(), buffer.end(), PixelColor(0, 0, 0, 0));

        // Randomly light some pixels
        for (size_t i = 0; i < buffer.size(); ++i) {
            if ((esp_random() % 20) == 0) {  // 5% chance per pixel
                buffer[i] = config.color.scale(config.brightness);
            }
        }

        state.last_update_tick = tick;
    }
}

void PixelEffectEngine::applyCustom(PixelChannel* channel, uint32_t tick) {
    const auto& config = channel->getEffectConfig();

    if (config.custom_effect) {
        auto& buffer = channel->getPixelBuffer();
        config.custom_effect(buffer, tick);
    }
    else {
        // Fallback to solid if no custom effect is defined
        applySolid(channel);
    }
}

uint32_t PixelEffectEngine::getEffectInterval(uint8_t speed) const {
    // Convert speed (1-10) to update interval in ticks
    // Higher speed = lower interval = faster updates
    const uint32_t base_interval = update_rate_hz_ / 10;  // 100ms at 60Hz
    return base_interval * (11 - std::clamp(speed, uint8_t(1), uint8_t(10)));
}

void PixelEffectEngine::ensureChannelState(int32_t channel_id) {
    if (channel_id >= static_cast<int32_t>(channel_states_.size())) {
        channel_states_.resize(channel_id + 1);

        // Initialize new state
        auto& state = channel_states_[channel_id];
        state.last_update_tick = 0;
        state.phase = 0;
        state.direction = false;

        // Initialize effect-specific state
        state.breathe.current_brightness = 0;
        state.breathe.increasing = true;
        state.color_wipe.current_pixel = 0;
        state.color_wipe.clearing = false;
        state.theater_chase.offset = 0;
        state.rainbow.rainbow_offset = 0;
        state.cyclic.trail_offset = 0;
    }
}
