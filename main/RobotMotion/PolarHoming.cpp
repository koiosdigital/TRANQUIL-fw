// ESP-IDF Polar Robot Homing Implementation
// 
// This file contains all homing-related functionality for the PolarRobot class:
// - Endstop and StallGuard configuration
// - Homing state machine logic
// - ISR handlers for endstop and stall detection
// - Step generation during homing sequences

#include "PolarRobot.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"

static const char* TAG = "PolarHoming";

extern PolarRobot* g_robotInstance; // Defined in main PolarRobot.cpp

// =============================================================================
// HOMING CONFIGURATION AND INITIALIZATION
// =============================================================================

esp_err_t PolarRobot::initHomingHardware() {
    // Configure endstop GPIO with interrupt
    gpio_config_t endstop_config = {
        .pin_bit_mask = (1ULL << _config.theta.endstopPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = _config.theta.endstopInvert ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = _config.theta.endstopInvert ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = _config.theta.endstopInvert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&endstop_config), TAG, "Failed to configure endstop GPIO");

    // Install ISR service for endstop (ignore error if already installed)
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(isr_err, TAG, "Failed to install GPIO ISR service");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(_config.theta.endstopPin, endstopPinISR, this), TAG, "Failed to add endstop ISR");

    // Configure StallGuard for rho stepper
    if (_rhoStepper) {
        _rhoStepper->set_stallguard_threshold(_config.rho.stallguardThreshold);
        _rhoStepper->set_stallguard_min_speed(0xFFFFF);
        _rhoStepper->set_stallguard_callback(_config.diagPin, rhoStallCallback);
        ESP_LOGD(TAG, "Configured StallGuard for rho stepper on pin %d (thresh %d)", _config.diagPin, _config.rho.stallguardThreshold);
    }

    return ESP_OK;
}

void PolarRobot::deinitHomingHardware() {
    // Remove GPIO ISR handlers
    if (_config.theta.endstopPin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(_config.theta.endstopPin);
    }
    // Note: diagPin ISR is now handled by TMC library, no need to remove it here
}

// =============================================================================
// HOMING MAIN ENTRY POINTS
// =============================================================================

esp_err_t PolarRobot::homeAll() {
    _homingControl.clear();
    _homingControl.homingActive = true;
    _homingControl.currentAxis = HomingAxis::SEQUENTIAL_THETA;
    _homingControl.state = HomingState::THETA_BACKING_OFF;
    _thetaEndstopTriggered = false;
    _rhoStallguardTriggered = false;

    // Start with theta homing
    ESP_RETURN_ON_ERROR(startThetaHoming(), TAG, "Failed to start theta homing");

    return ESP_OK;
}

esp_err_t PolarRobot::startThetaHoming() {
    ESP_LOGI(TAG, "Starting theta homing sequence");

    setMotorsActive(true);

    // Calculate maximum steps for safety (2 full rotations), accounting for microstepping
    const float microstepMultiplier = 16.0f; // TMC configured for 16x microstepping
    _homingControl.maxSteps = (_config.theta.stepsPerRot * _config.theta.gearRatio * 2) * microstepMultiplier;
    _homingControl.stepCount = 0;

    // Clear endstop trigger flag
    _thetaEndstopTriggered = false;

    // Check if already on endstop - if so, back off first
    if (readThetaEndstop()) {
        _homingControl.state = HomingState::THETA_BACKING_OFF;
        gpio_set_level(_config.theta.dirPin, 0);
    }
    else {
        _homingControl.state = HomingState::THETA_SEEKING;
        gpio_set_level(_config.theta.dirPin, 1);
    }

    uint64_t homingStepInterval = 1500;
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = homingStepInterval,
        .flags = {.auto_reload_on_alarm = true}
    };
    gptimer_set_alarm_action(_stepTimer, &alarm_config);
    gptimer_start(_stepTimer);

    return ESP_OK;
}

esp_err_t PolarRobot::startRhoHoming() {
    ESP_LOGI(TAG, "Starting rho homing sequence");

    const float microstepMultiplier = 16.0f;
    _homingControl.maxSteps = _config.rho.stepsPerMm * _config.rho.maxRadius * 2 * microstepMultiplier;
    _homingControl.stepCount = 0;

    _rhoStallguardTriggered = false;

    _homingControl.state = HomingState::RHO_SEEKING;
    gpio_set_level(_config.rho.dirPin, 0);

    uint64_t homingStepInterval = 1500;
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = homingStepInterval,
        .flags = {.auto_reload_on_alarm = true}
    };
    gptimer_set_alarm_action(_stepTimer, &alarm_config);
    gptimer_start(_stepTimer);

    return ESP_OK;
}

// =============================================================================
// HOMING STATE MACHINE
// =============================================================================

void PolarRobot::processHomingStateMachine() {
    switch (_homingControl.state) {
    case HomingState::THETA_BACKING_OFF:
        if (!readThetaEndstop()) {
            _homingControl.state = HomingState::THETA_SEEKING;
            gpio_set_level(_config.theta.dirPin, 1); // Direction towards endstop
            _homingControl.stepCount = 0; // Reset step count for seeking phase
        }
        break;

    case HomingState::THETA_SEEKING:
        // Check if endstop was triggered by IRQ
        if (_thetaEndstopTriggered) {
            _thetaPosition = 0; // Set home position
            _thetaEndstopTriggered = false; // Clear flag

            gptimer_stop(_stepTimer);

            if (_homingControl.currentAxis == HomingAxis::SEQUENTIAL_THETA) {
                _homingControl.currentAxis = HomingAxis::SEQUENTIAL_RHO;
                startRhoHoming();
            }
            else {
                _homingControl.state = HomingState::COMPLETE;
                _homingControl.homingActive = false;
                const uint64_t INACTIVITY_TIMEOUT_US = 2000000; // 2 seconds
                esp_timer_start_once(_motorInactivityTimer, INACTIVITY_TIMEOUT_US);
            }
        }
        else if (_homingControl.stepCount >= _homingControl.maxSteps) {
            ESP_LOGE(TAG, "Theta homing failed: exceeded maximum steps (%ld)", _homingControl.maxSteps);
            _homingControl.state = HomingState::ERROR;
            gptimer_stop(_stepTimer);
        }
        break;

    case HomingState::RHO_SEEKING:
        // Check if stallguard was triggered by IRQ or TMC callback
        if (_rhoStallguardTriggered) {
            ESP_LOGI(TAG, "Rho stallguard found at step %ld", _homingControl.stepCount);
            _rhoPosition = 0; // Set home position
            _rhoStallguardTriggered = false; // Clear flag
            gptimer_stop(_stepTimer);
            _homingControl.state = HomingState::COMPLETE;
            _homingControl.homingActive = false;

            if (_homingControl.currentAxis == HomingAxis::SEQUENTIAL_RHO) {
                _isHomed = true;
                ESP_LOGI(TAG, "Full homing sequence complete");
            }

            const uint64_t INACTIVITY_TIMEOUT_US = 2000000; // 2 seconds
            esp_timer_start_once(_motorInactivityTimer, INACTIVITY_TIMEOUT_US);
        }
        else if (_homingControl.stepCount >= _homingControl.maxSteps) {
            ESP_LOGE(TAG, "Rho homing failed: exceeded maximum steps (%ld)", _homingControl.maxSteps);
            _homingControl.state = HomingState::ERROR;
            gptimer_stop(_stepTimer);
        }
        break;

    case HomingState::COMPLETE:
        _homingControl.homingActive = false;
        break;

    case HomingState::ERROR:
        ESP_LOGE(TAG, "Homing error detected, stopping motors");
        _homingControl.homingActive = false;
        setMotorsActive(false);
        break;

    default:
        break;
    }
}

// =============================================================================
// HOMING STEP GENERATION
// =============================================================================

void IRAM_ATTR PolarRobot::generateHomingSteps() {
    // Homing step generation - single axis at a time
    bool stepGenerated = false;

    switch (_homingControl.state) {
    case HomingState::THETA_BACKING_OFF:
    case HomingState::THETA_SEEKING:
        // Generate theta step
        gpio_set_level(_config.theta.stepPin, 1);
        esp_rom_delay_us(2); // Minimum pulse width
        gpio_set_level(_config.theta.stepPin, 0);

        // Update position tracking
        if (_homingControl.state == HomingState::THETA_BACKING_OFF) {
            int32_t newPos = _thetaPosition - 1; // Moving away from endstop
            _thetaPosition = newPos;
        }
        else {
            int32_t newPos = _thetaPosition + 1; // Moving towards endstop
            _thetaPosition = newPos;
        }
        stepGenerated = true;
        break;

    case HomingState::RHO_SEEKING: {
        // Generate rho step
        gpio_set_level(_config.rho.stepPin, 1);
        esp_rom_delay_us(2); // Minimum pulse width
        gpio_set_level(_config.rho.stepPin, 0);

        // Update position tracking (moving towards center)
        int32_t newRhoPos = _rhoPosition - 1;
        _rhoPosition = newRhoPos;
        stepGenerated = true;
        break;
    }
    default:
        // No steps in other states
        break;
    }

    if (stepGenerated) {
        _homingControl.stepCount++;
    }
}

// =============================================================================
// SENSOR READING AND ISR HANDLERS
// =============================================================================

bool PolarRobot::readThetaEndstop() {
    int level = gpio_get_level(_config.theta.endstopPin);
    return _config.theta.endstopInvert ? level : !level;
}

// Static ISR for endstop pin
void IRAM_ATTR PolarRobot::endstopPinISR(void* arg) {
    PolarRobot* robot = static_cast<PolarRobot*>(arg);
    robot->_thetaEndstopTriggered = true;
}

// Static callback for rho stallguard detection
void IRAM_ATTR PolarRobot::rhoStallCallback(uint8_t addr, bool stalled) {
    if (g_robotInstance && stalled) {
        g_robotInstance->_rhoStallguardTriggered = true;
    }
}
