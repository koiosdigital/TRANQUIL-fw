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

#include "sdkconfig.h"

static const char* TAG = "PolarHoming";

extern PolarRobot* g_robotInstance; // Defined in main PolarRobot.cpp

// =============================================================================
// HOMING CONFIGURATION AND INITIALIZATION
// =============================================================================

esp_err_t PolarRobot::initHomingHardware() {
    // Configure endstop GPIO with interrupt
    gpio_config_t endstop_config = {
        .pin_bit_mask = (1ULL << CONFIG_ROBOT_THETA_ENDSTOP_PIN),
        .mode = GPIO_MODE_INPUT,
#if defined(CONFIG_ROBOT_THETA_ENDSTOP_PULLUP) && CONFIG_ROBOT_THETA_ENDSTOP_PULLUP
        .pull_up_en = GPIO_PULLUP_ENABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,
#endif
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#if defined(CONFIG_ROBOT_THETA_ENDSTOP_INVERT) && CONFIG_ROBOT_THETA_ENDSTOP_INVERT
        .intr_type = GPIO_INTR_NEGEDGE,
#else
        .intr_type = GPIO_INTR_POSEDGE,
#endif
    };
    ESP_RETURN_ON_ERROR(gpio_config(&endstop_config), TAG, "Failed to configure endstop GPIO");

    // Install ISR service for endstop (ignore error if already installed)
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(isr_err, TAG, "Failed to install GPIO ISR service");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add((gpio_num_t)CONFIG_ROBOT_THETA_ENDSTOP_PIN, endstopPinISR, this), TAG, "Failed to add endstop ISR");

    // Configure StallGuard for rho stepper
    if (_rhoStepper) {
        _rhoStepper->set_stallguard_threshold(CONFIG_ROBOT_RHO_STALLGUARD_THRESHOLD);
        _rhoStepper->set_stallguard_min_speed(0xFFFFF);
        _rhoStepper->set_stallguard_callback((gpio_num_t)CONFIG_ROBOT_RHO_STALLGUARD_DIAG_PIN, rhoStallCallback);
    }

    return ESP_OK;
}

void PolarRobot::deinitHomingHardware() {
    gpio_isr_handler_remove((gpio_num_t)CONFIG_ROBOT_THETA_ENDSTOP_PIN);
}

// =============================================================================
// HOMING MAIN ENTRY POINTS
// =============================================================================

esp_err_t PolarRobot::homeAll() {
    _homingControl.clear();
    _homingControl.homingActive = true;
    _homingControl.currentAxis = HomingAxis::SEQUENTIAL_RHO;
    _homingControl.state = HomingState::RHO_SEEKING_MAX;
    _thetaEndstopTriggered = false;
    _rhoStallguardTriggered = false;

    setMotorsActive(true); // Ensure motors are disabled before homing
    startRhoMaxHomingISR(); // Start rho max calibration

    return ESP_OK;
}

// ISR-safe version of rho max homing start (seeking maximum radius)
void IRAM_ATTR PolarRobot::startRhoMaxHomingISR() {
    _homingControl.maxSteps = CONFIG_ROBOT_RHO_STEPS_PER_ROT * 4000; // Allow up to 50 rotations
    _homingControl.stepCount = 0;

    _rhoStallguardTriggered = false;

    _homingControl.state = HomingState::RHO_SEEKING_MAX;
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 1); // Move outward to max

    uint64_t homingStepInterval = 750;
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = homingStepInterval,
        .flags = {.auto_reload_on_alarm = true}
    };
    gptimer_set_alarm_action(_stepTimer, &alarm_config);
    gptimer_start(_stepTimer);
}

// ISR-safe version of rho min homing start (seeking minimum radius)
void IRAM_ATTR PolarRobot::startRhoMinHomingISR() {
    _homingControl.maxSteps = CONFIG_ROBOT_RHO_STEPS_PER_ROT * 4000; // Allow up to 50 rotations
    _homingControl.stepCount = 0;

    _rhoStallguardTriggered = false;

    _homingControl.state = HomingState::RHO_SEEKING_MIN;
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 0); // Move inward to min

    uint64_t homingStepInterval = 750;
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = homingStepInterval,
        .flags = {.auto_reload_on_alarm = true}
    };
    gptimer_set_alarm_action(_stepTimer, &alarm_config);
    gptimer_start(_stepTimer);
}

// ISR-safe version of theta calibration homing start
void IRAM_ATTR PolarRobot::startThetaCalibrationISR() {
    _homingControl.maxSteps = (CONFIG_ROBOT_THETA_STEPS_PER_ROT * CONFIG_ROBOT_THETA_GEAR_RATIO * 4) * 16; // Allow up to 4 rotations
    _homingControl.stepCount = 0;
    _homingControl.thetaStepCounter = 0; // Reset theta step counter for coupled motion

    _thetaEndstopTriggered = false;
    _homingControl.first_theta_edge_found = false;

    // Always start rotating in direction 1 (consistent calibration direction)
    // If already on endstop, back off in opposite direction first
    if (readThetaEndstop()) {
        _homingControl.state = HomingState::THETA_BACKING_OFF;
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN, 0); // Back off direction
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 0);   // Rho moves in same direction
    }
    else {
        _homingControl.state = HomingState::THETA_SEEKING_FIRST_EDGE;
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN, 1); // Calibration direction
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 1);   // Rho moves in same direction
    }

    uint64_t homingStepInterval = 400;
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = homingStepInterval,
        .flags = {.auto_reload_on_alarm = true}
    };
    gptimer_set_alarm_action(_stepTimer, &alarm_config);
    gptimer_start(_stepTimer);
}

// =============================================================================
// HOMING STEP GENERATION WITH INTEGRATED STATE MACHINE
// =============================================================================

void IRAM_ATTR PolarRobot::generateHomingSteps() {
    // Homing step generation with integrated state machine - calibration sequence
    bool stepGenerated = false;

    // Process state machine logic first
    switch (_homingControl.state) {
    case HomingState::THETA_BACKING_OFF:
        // Check if we've backed off the endstop
        if (!_thetaEndstopTriggered) {
            // Now switch to calibration direction (always direction 1) and start seeking
            _homingControl.state = HomingState::THETA_SEEKING_FIRST_EDGE;
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN, 1); // Calibration direction
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 1);   // Rho moves in same direction
            _homingControl.stepCount = 0; // Reset step count for seeking phase
            _homingControl.thetaStepCounter = 0; // Reset coupling counter
        }
        break;

    case HomingState::THETA_SEEKING_FIRST_EDGE:
        if (_thetaEndstopTriggered) {
            _thetaEndstopTriggered = false; // Clear flag
            _homingControl.first_theta_edge_found = true;
            _homingControl.stepCount = 0; // Reset step count to measure rotation
            _homingControl.thetaStepCounter = 0;
            _homingControl.state = HomingState::THETA_SEEKING_SECOND_EDGE;
        }
        break;

    case HomingState::THETA_SEEKING_SECOND_EDGE:
        if (_thetaEndstopTriggered) {
            _homingControl.steps_per_theta_rot = _homingControl.stepCount;
            _thetaPosition = 0; // Set home position
            _thetaEndstopTriggered = false; // Clear flag
            gptimer_stop(_stepTimer);
            _homingControl.state = HomingState::COMPLETE;
            _homingControl.homingActive = false;
            _isHomed = true;
            _thetaPosition = 0; // Reset position to home
            _rhoPosition = 0; // Reset rho position to home
            return; // Exit early, no step generation needed
        }
        else if (_homingControl.stepCount >= _homingControl.maxSteps) {
            // Homing failed - exceeded maximum steps
            _homingControl.state = HomingState::ERROR;
            _homingControl.homingActive = false;
            gptimer_stop(_stepTimer);
            return;
        }
        break;

    case HomingState::RHO_SEEKING_MAX:
        // Check if stallguard was triggered by IRQ or TMC callback (reached max radius)
        if (_rhoStallguardTriggered) {
            _rhoStallguardTriggered = false; // Clear flag

            // Stop timer using ISR-safe function
            gptimer_stop(_stepTimer);

            // Start seeking minimum radius
            startRhoMinHomingISR();
            return; // Exit early, no step generation needed
        }
        else if (_homingControl.stepCount >= _homingControl.maxSteps) {
            // Homing failed - exceeded maximum steps
            _homingControl.state = HomingState::ERROR;
            _homingControl.homingActive = false;
            gptimer_stop(_stepTimer);
            return;
        }
        break;

    case HomingState::RHO_SEEKING_MIN:
        // Check if stallguard was triggered by IRQ or TMC callback (reached min radius)
        if (_rhoStallguardTriggered) {
            _homingControl.rho_max_steps = _homingControl.stepCount; // Save calibration result
            _rhoPosition = 0; // Set home position at minimum radius
            _rhoStallguardTriggered = false; // Clear flag

            // Stop timer using ISR-safe function
            gptimer_stop(_stepTimer);

            // Start theta calibration
            _homingControl.currentAxis = HomingAxis::SEQUENTIAL_THETA;
            startThetaCalibrationISR();
            return; // Exit early, no step generation needed
        }
        else if (_homingControl.stepCount >= _homingControl.maxSteps) {
            // Homing failed - exceeded maximum steps
            _homingControl.state = HomingState::ERROR;
            _homingControl.homingActive = false;
            gptimer_stop(_stepTimer);
            return;
        }
        break;

    case HomingState::COMPLETE:
    case HomingState::ERROR:
    default:
        // No steps in these states
        return;
    }

    // Generate steps based on current state
    switch (_homingControl.state) {
    case HomingState::THETA_BACKING_OFF:
    case HomingState::THETA_SEEKING_FIRST_EDGE:
    case HomingState::THETA_SEEKING_SECOND_EDGE: {
        // Generate theta step
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN, 1);
        esp_rom_delay_us(2); // Minimum pulse width
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN, 0);

        // Update position tracking
        if (_homingControl.state == HomingState::THETA_BACKING_OFF) {
            int32_t newPos = _thetaPosition - 1; // Moving away from endstop
            _thetaPosition = newPos;
        }
        else {
            int32_t newPos = _thetaPosition + 1; // Moving towards endstop
            _thetaPosition = newPos;
        }

        // Coupled rho motion during theta homing
        // Calculate gear ratio divisor from the config (e.g., 800 -> 8)
        int32_t gearRatioDivisor = CONFIG_ROBOT_THETA_GEAR_RATIO / 100;
        _homingControl.thetaStepCounter++;

        // Step rho every N theta steps based on gear ratio
        if (_homingControl.thetaStepCounter >= gearRatioDivisor) {
            _homingControl.thetaStepCounter = 0; // Reset counter

            // Generate rho step in same direction as theta
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 1);
            esp_rom_delay_us(2); // Minimum pulse width
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 0);

            // Update rho position in same direction as theta
            if (_homingControl.state == HomingState::THETA_BACKING_OFF) {
                _rhoPosition -= 1; // Same direction as theta (backing off)
            }
            else {
                _rhoPosition += 1; // Same direction as theta (seeking)
            }
        }

        stepGenerated = true;
        break;
    }

    case HomingState::RHO_SEEKING_MIN: {
        // Generate rho step (moving towards center/minimum)
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 1);
        esp_rom_delay_us(2); // Minimum pulse width
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 0);

        // Update position tracking (moving towards center)
        int32_t newRhoPos = _rhoPosition - 1;
        _rhoPosition = newRhoPos;
        stepGenerated = true;
        break;
    }

    case HomingState::RHO_SEEKING_MAX: {
        // Generate rho step (moving towards maximum radius)
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 1);
        esp_rom_delay_us(2); // Minimum pulse width
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 0);

        // Update position tracking (moving outward)
        int32_t newRhoPos = _rhoPosition + 1;
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

bool PolarRobot::readThetaEndstop() {
    int level = gpio_get_level((gpio_num_t)CONFIG_ROBOT_THETA_ENDSTOP_PIN);
#if defined(CONFIG_ROBOT_THETA_ENDSTOP_INVERT) && CONFIG_ROBOT_THETA_ENDSTOP_INVERT
    return (level == 0); // Active low endstop
#else
    return (level == 1); // Active high endstop
#endif
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
