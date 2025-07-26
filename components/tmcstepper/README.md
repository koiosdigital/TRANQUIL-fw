# TMC2209 Stepper Driver Component for ESP-IDF

A comprehensive ESP-IDF component for controlling TMC2209 stepper motor drivers via UART communication. This component provides human-readable abstractions for all TMC2209 features including StealthChop, SpreadCycle, CoolStep, StallGuard, and more.

## Features

- **Two-part initialization**: UART bus setup + individual stepper configuration
- **Human-readable API**: Clear function names for all parameters
- **Complete TMC2209 support**: All registers and features accessible
- **StallGuard diagnostics**: Interrupt-driven stall detection with callbacks
- **Multiple drivers**: Support for up to 4 drivers on one UART bus (addresses 0-3)
- **Safety features**: Input validation and error handling
- **Modern C++**: Uses C++11 features like lambdas and std::function

## Hardware Connections

### TMC2209 to ESP32 Connections

```
TMC2209          ESP32
-------          -----
VDD         ->   3.3V
GND         ->   GND
UART_TX     ->   GPIO (your choice, e.g., GPIO16)
UART_RX     ->   GPIO (your choice, e.g., GPIO17)
DIAG        ->   GPIO (optional, for StallGuard, e.g., GPIO18)
STEP        ->   GPIO (controlled separately by your step generation)
DIR         ->   GPIO (controlled separately by your direction control)
EN          ->   GPIO (optional, can be controlled via UART)
```

### Multiple Drivers Setup

When using multiple TMC2209 drivers:

- All drivers share the same UART TX/RX lines
- Each driver must have a unique UART address (0-3) set via MS1/MS2 pins
- DIAG pins can be separate for individual StallGuard detection
- STEP/DIR/EN pins are individual for each motor

## Usage

### Basic Example

```cpp
#include "tmcstepper.h"

// Global UART bus instance
TMC::UartBus tmc_uart_bus;

// Individual stepper instances
TMC::Stepper stepper1(tmc_uart_bus, 0);  // UART address 0
TMC::Stepper stepper2(tmc_uart_bus, 1);  // UART address 1

// StallGuard callback function
void on_stallguard_event(uint8_t uart_addr, bool stall_detected) {
    ESP_LOGI("STEPPER", "Motor %d stall %s", uart_addr, stall_detected ? "detected" : "cleared");

    if (stall_detected) {
        // Handle stall - stop movement, reduce current, etc.
    }
}

void app_main() {
    // Step 1: Initialize UART bus
    esp_err_t ret = tmc_uart_bus.initialize(
        UART_NUM_1,     // UART port
        GPIO_NUM_16,    // TX pin
        GPIO_NUM_17,    // RX pin
        115200          // Baud rate
    );
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize TMC UART bus");
        return;
    }

    // Step 2: Initialize individual steppers
    ret = stepper1.initialize(GPIO_NUM_18, on_stallguard_event);  // With StallGuard
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize stepper1");
        return;
    }

    ret = stepper2.initialize();  // Without StallGuard
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize stepper2");
        return;
    }

    // Step 3: Configure motors
    stepper1.set_motor_current(800);        // 800mA run current
    stepper1.set_hold_current_percentage(50); // 50% hold current
    stepper1.set_microstep_resolution(TMC::MicrostepResolution::SIXTEENTH);
    stepper1.set_interpolation_enable(true); // Smooth 256 microsteps

    // Enable StealthChop for quiet operation
    stepper1.set_stealthchop_enable(true);
    stepper1.set_stealthchop_threshold(1000);  // Switch to SpreadCycle above this speed

    // Configure StallGuard
    stepper1.set_stallguard_enable(true);
    stepper1.set_stallguard_threshold(10);     // Adjust for your load
    stepper1.set_stallguard_min_speed(100);    // Minimum speed for stall detection

    // Enable motors
    stepper1.enable_motor();
    stepper2.enable_motor();

    ESP_LOGI("MAIN", "TMC2209 steppers initialized and ready");

    // Your step generation code goes here...
    // Use separate GPIOs to control STEP and DIR pins
}
```

### Advanced Configuration

```cpp
void configure_high_performance_stepper(TMC::Stepper& stepper) {
    // High current for powerful motors
    stepper.set_motor_current(1500);        // 1.5A run current
    stepper.set_hold_current_percentage(30); // 30% hold to reduce heating

    // High resolution microstepping
    stepper.set_microstep_resolution(TMC::MicrostepResolution::TWOFIFTYSIXTH);
    stepper.set_interpolation_enable(true);

    // SpreadCycle for high speed/torque
    stepper.set_stealthchop_enable(false);
    stepper.set_chopper_mode(TMC::ChopperTiming::SPREAD_CYCLE);
    stepper.set_chopper_off_time(5);
    stepper.set_chopper_hysteresis_start(4);
    stepper.set_chopper_hysteresis_end(1);
    stepper.set_chopper_blank_time(2);

    // Enable CoolStep for automatic current reduction
    stepper.set_coolstep_enable(true);
    stepper.set_coolstep_min_current(0);      // 1/2 current minimum
    stepper.set_coolstep_current_increment(1); // Moderate increment
    stepper.set_coolstep_upper_threshold(10);
    stepper.set_coolstep_lower_threshold(2);

    // Aggressive StallGuard settings
    stepper.set_stallguard_enable(true);
    stepper.set_stallguard_threshold(20);     // Less sensitive
    stepper.set_stallguard_min_speed(200);    // Higher minimum speed
}

void configure_quiet_stepper(TMC::Stepper& stepper) {
    // Moderate current for quiet operation
    stepper.set_motor_current(600);
    stepper.set_hold_current_percentage(50);

    // StealthChop for silent operation
    stepper.set_stealthchop_enable(true);
    stepper.set_stealthchop_threshold(0);      // Always StealthChop
    stepper.set_stealthchop_pwm_gradient(128); // Balanced gradient
    stepper.set_stealthchop_pwm_amplitude(200); // Good torque

    // Basic StallGuard for safety
    stepper.set_stallguard_enable(true);
    stepper.set_stallguard_threshold(5);       // Sensitive detection
}
```

### Status Monitoring

```cpp
void monitor_stepper_status(TMC::Stepper& stepper) {
    // Check driver status
    uint32_t status;
    if (stepper.get_driver_status(&status) == ESP_OK) {
        ESP_LOGI("STATUS", "Driver status: 0x%08lX", status);
    }

    // Monitor actual current
    uint16_t current = stepper.get_actual_current();
    ESP_LOGI("STATUS", "Actual current: %d mA", current);

    // Check StallGuard result
    uint16_t sg_result = stepper.get_stallguard_result();
    ESP_LOGI("STATUS", "StallGuard result: %d", sg_result);

    // Check if in StealthChop mode
    bool stealth_active = stepper.is_stealthchop_active();
    ESP_LOGI("STATUS", "StealthChop active: %s", stealth_active ? "yes" : "no");

    // Check for stall
    if (stepper.is_stalled()) {
        ESP_LOGW("STATUS", "Motor stall detected!");
    }
}
```

## API Reference

### UartBus Class

The UART bus manages communication with all TMC2209 drivers.

#### Methods

- `initialize(uart_port, tx_pin, rx_pin, baud_rate)` - Initialize UART bus
- `deinitialize()` - Clean up UART resources
- `is_initialized()` - Check initialization status

### Stepper Class

Each stepper instance represents one TMC2209 driver.

#### Constructor

- `Stepper(uart_bus, uart_addr)` - Create stepper instance

#### Initialization

- `initialize(diag_pin, stallguard_callback)` - Initialize driver

#### Basic Control

- `enable_motor()` / `disable_motor()` - Enable/disable motor
- `set_direction(clockwise)` - Set rotation direction
- `set_motor_current(current_ma)` - Set run current in mA
- `set_hold_current_percentage(percentage)` - Set hold current as % of run current

#### Microstepping

- `set_microstep_resolution(resolution)` - Set microstepping (FULLSTEP to TWOFIFTYSIXTH)
- `set_interpolation_enable(enable)` - Enable 256 microstep interpolation

#### StealthChop (Quiet Mode)

- `set_stealthchop_enable(enable)` - Enable/disable StealthChop
- `set_stealthchop_threshold(threshold)` - Velocity threshold for mode switching
- `set_stealthchop_pwm_gradient(gradient)` - PWM gradient (0-255)
- `set_stealthchop_pwm_amplitude(amplitude)` - PWM amplitude (0-255)

#### SpreadCycle (High Performance)

- `set_chopper_mode(mode)` - Set chopper timing mode
- `set_chopper_off_time(time)` - Off time (1-15)
- `set_chopper_hysteresis_start(value)` - Hysteresis start (0-7)
- `set_chopper_hysteresis_end(value)` - Hysteresis end (-3 to 12)
- `set_chopper_blank_time(time)` - Blank time (0-3)

#### StallGuard (Stall Detection)

- `set_stallguard_enable(enable)` - Enable/disable StallGuard
- `set_stallguard_threshold(threshold)` - Sensitivity (-64 to 63)
- `set_stallguard_min_speed(speed)` - Minimum speed for detection

#### CoolStep (Automatic Current Control)

- `set_coolstep_enable(enable)` - Enable/disable CoolStep
- `set_coolstep_min_current(level)` - Minimum current level (0-1)
- `set_coolstep_current_increment(steps)` - Current increment steps (0-3)
- `set_coolstep_upper_threshold(threshold)` - Upper load threshold (0-15)
- `set_coolstep_lower_threshold(threshold)` - Lower load threshold (0-15)

#### Status and Diagnostics

- `get_driver_status(status_flags)` - Get raw status register
- `is_stalled()` - Check for motor stall
- `get_stallguard_result()` - Get StallGuard measurement
- `get_actual_current()` - Get actual motor current
- `is_stealthchop_active()` - Check current chopper mode
- `reset_to_defaults()` - Reset all settings to defaults

## Register Reference

The component manages all TMC2209 registers internally, but for reference:

- `GCONF` (0x00) - Global configuration
- `GSTAT` (0x01) - Global status flags
- `IHOLD_IRUN` (0x10) - Current settings
- `CHOPCONF` (0x6C) - Chopper configuration
- `COOLCONF` (0x6D) - CoolStep configuration
- `PWMCONF` (0x70) - StealthChop PWM configuration
- `DRV_STATUS` (0x6F) - Driver status flags

## Error Handling

All functions return `esp_err_t` values:

- `ESP_OK` - Success
- `ESP_ERR_INVALID_STATE` - Driver/bus not initialized
- `ESP_ERR_INVALID_ARG` - Invalid parameter value
- `ESP_ERR_TIMEOUT` - Communication timeout
- `ESP_ERR_INVALID_CRC` - CRC error in response

## Performance Considerations

- UART communication adds ~1-2ms delay per register access
- Cache register values if frequent reads are needed
- Use StallGuard callbacks for real-time stall detection
- StealthChop reduces high-frequency noise but may reduce torque at high speeds
- CoolStep can reduce power consumption by 75% at low loads

## Troubleshooting

1. **No communication**: Check UART wiring and baud rate
2. **Motor not moving**: Verify STEP/DIR pins and motor current settings
3. **StallGuard false triggers**: Adjust threshold and minimum speed
4. **Overheating**: Reduce current or increase hold current reduction
5. **Noise issues**: Enable StealthChop or adjust PWM settings
6. **Lost steps**: Increase current or reduce acceleration

## License

This component is provided under the same license as your ESP-IDF project.
