#pragma once

#include "kd_pixdriver.h"
#include <cstdint>
#include <vector>

class PixelEffectEngine {
public:
    explicit PixelEffectEngine(uint32_t update_rate_hz);

    void updateEffect(PixelChannel* channel, uint32_t tick);

private:
    uint32_t update_rate_hz_;

    // Effect implementations
    void applySolid(PixelChannel* channel);
    void applyBlink(PixelChannel* channel, uint32_t tick);
    void applyBreathe(PixelChannel* channel, uint32_t tick);
    void applyCyclic(PixelChannel* channel, uint32_t tick);
    void applyRainbow(PixelChannel* channel, uint32_t tick);
    void applyColorWipe(PixelChannel* channel, uint32_t tick);
    void applyTheaterChase(PixelChannel* channel, uint32_t tick);
    void applySparkle(PixelChannel* channel, uint32_t tick);
    void applyCustom(PixelChannel* channel, uint32_t tick);

    // Effect state tracking (per channel)
    struct EffectState {
        uint32_t last_update_tick = 0;
        uint32_t phase = 0;
        bool direction = false;  // For effects that reverse

        // Effect-specific state
        union {
            struct {
                uint8_t current_brightness;
                bool increasing;
            } breathe;

            struct {
                uint16_t current_pixel;
                bool clearing;
            } color_wipe;

            struct {
                uint8_t offset;
            } theater_chase;

            struct {
                uint8_t rainbow_offset;
            } rainbow;

            struct {
                uint8_t trail_offset;
            } cyclic;
        };
    };

    std::vector<EffectState> channel_states_;

    uint32_t getEffectInterval(uint8_t speed) const;
    void ensureChannelState(int32_t channel_id);
};