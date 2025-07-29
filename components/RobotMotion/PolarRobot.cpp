// ESP-IDF Polar Robot Motion System Implementation
// 
// KINEMATICS OVERVIEW:
// - Theta axis: Motor shaft is belted to drive gear with configurable gear ratio
//   Motor steps -> Motor rotations -> Drive gear rotations -> Theta angle
//   Example: 5:1 gear ratio means 5 motor rotations = 1 drive gear rotation = 360°
// 
// - Rho axis: Motor directly drives rack mechanism (no gearing)
//   Motor steps -> Rack movement in mm
//   If theta and rho rotate at same rate in same direction, rack stays at same radius
// 
// MOTION CONTROL:
// - Hardware step/dir pins for high-speed coordinated motion
// - Common enable pin for power management
// - StallGuard diagnostic pin for sensorless homing
// - TMC2209 UART for configuration and monitoring

#include "PolarRobot.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char* TAG = "PolarRobot";
PolarRobot* g_robotInstance = nullptr;

PolarRobot::PolarRobot()
    : _tmcBus(nullptr)
    , _thetaStepper(nullptr)
    , _rhoStepper(nullptr)
    , _stepTimer(nullptr)
    , _thetaPosition(0)
    , _rhoPosition(0)
    , _isHomed(false)
    , _isPaused(false)
    , _thetaEndstopTriggered(false)
    , _rhoStallguardTriggered(false)
    , _motorsActive(false)
    , _motorInactivityTimer(nullptr)
    , _maxQueueSize(500) // Default fallback size
{
    g_robotInstance = this;
}

PolarRobot::~PolarRobot() {
    deinit();
    g_robotInstance = nullptr;
}

esp_err_t PolarRobot::init() {
    if (!_commandQueue.init(_maxQueueSize)) {
        ESP_LOGE(TAG, "Failed to init command queue with size %zu", _maxQueueSize);
        return ESP_FAIL;
    }

    initTMC();
    initGPIO();
    initTimer();
    initHomingHardware();

    // Create motor inactivity timer
    esp_timer_create_args_t timer_args = {
        .callback = motorInactivityCallback,
        .arg = this,
        .name = "motor_inactivity"
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &_motorInactivityTimer), TAG, "Failed to create inactivity timer");

    setMotorsActive(false);

    return ESP_OK;
}

esp_err_t PolarRobot::initTMC() {
    // Create TMC UART bus
    _tmcBus = new UartBus();
    if (!_tmcBus) {
        return ESP_ERR_NO_MEM;
    }

    _tmcBus->initialize((uart_port_t)CONFIG_ROBOT_TMC_UART_PORT, (gpio_num_t)CONFIG_ROBOT_TMC_UART_TX_PIN, (gpio_num_t)CONFIG_ROBOT_TMC_UART_RX_PIN, CONFIG_ROBOT_TMC_BAUD_RATE);

    // Create theta stepper
    _thetaStepper = new TMC2209Stepper(*_tmcBus, CONFIG_ROBOT_THETA_TMC_ADDR);
    if (!_thetaStepper) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(_thetaStepper->initialize(), TAG, "Failed to init theta stepper");
    _thetaStepper->set_motor_current(CONFIG_ROBOT_THETA_MOTOR_CURRENT);
    _thetaStepper->set_microstep_resolution(MicrostepResolution::SIXTEENTH);
    _thetaStepper->set_stealthchop_enable(true);
    _thetaStepper->set_stealthchop_threshold(0);

    // Create rho stepper with StallGuard
    _rhoStepper = new TMC2209Stepper(*_tmcBus, CONFIG_ROBOT_RHO_TMC_ADDR);
    if (!_rhoStepper) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(_rhoStepper->initialize(), TAG, "Failed to init rho stepper");
    _rhoStepper->set_motor_current(CONFIG_ROBOT_RHO_MOTOR_CURRENT);
    _rhoStepper->set_microstep_resolution(MicrostepResolution::SIXTEENTH);
    _rhoStepper->set_stealthchop_enable(true);
    _rhoStepper->set_stealthchop_threshold(0);

    return ESP_OK;
}

esp_err_t PolarRobot::initTimer() {
    // Use ESP32-S3 general purpose timer for precise step generation
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1μs resolution
    };

    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &_stepTimer), TAG, "Failed to create step timer");

    gptimer_event_callbacks_t cbs = {
        .on_alarm = stepTimerCallback,
    };
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(_stepTimer, &cbs, this), TAG, "Failed to register timer callbacks");

    ESP_RETURN_ON_ERROR(gptimer_enable(_stepTimer), TAG, "Failed to enable step timer");

    return ESP_OK;
}

esp_err_t PolarRobot::initGPIO() {
    if (CONFIG_ROBOT_COMMON_ENABLE_PIN != GPIO_NUM_NC) {
        gpio_config_t enable_config = {
            .pin_bit_mask = (1ULL << CONFIG_ROBOT_COMMON_ENABLE_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_RETURN_ON_ERROR(gpio_config(&enable_config), TAG, "Failed to configure enable pin");

        gpio_set_level((gpio_num_t)CONFIG_ROBOT_COMMON_ENABLE_PIN, 1);
        ESP_LOGD(TAG, "Common enable pin configured on GPIO %d", CONFIG_ROBOT_COMMON_ENABLE_PIN);
    }

    gpio_config_t theta_pins_config = {
        .pin_bit_mask = (1ULL << CONFIG_ROBOT_THETA_STEP_PIN) | (1ULL << CONFIG_ROBOT_THETA_DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&theta_pins_config), TAG, "Failed to configure theta step/dir pins");
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN, 0);
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN, 0);

    gpio_config_t rho_pins_config = {
        .pin_bit_mask = (1ULL << CONFIG_ROBOT_RHO_STEP_PIN) | (1ULL << CONFIG_ROBOT_RHO_DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rho_pins_config), TAG, "Failed to configure rho step/dir pins");
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 0);
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, 0);

    return ESP_OK;
}

void PolarRobot::setMotorsActive(bool active) {
    if (_motorsActive == active) {
        return;
    }

    _motorsActive = active;

    if (CONFIG_ROBOT_COMMON_ENABLE_PIN != GPIO_NUM_NC) {
        if (active) {
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_COMMON_ENABLE_PIN, 0);
            esp_timer_stop(_motorInactivityTimer);
        }
        else {
            gpio_set_level((gpio_num_t)CONFIG_ROBOT_COMMON_ENABLE_PIN, 1);
        }
    }
}

void PolarRobot::motorInactivityCallback(void* arg) {
    PolarRobot* robot = static_cast<PolarRobot*>(arg);
    robot->updateMotorEnableState();
}

void PolarRobot::updateMotorEnableState() {
    bool shouldBeActive = (_currentMotion.active || !_commandQueue.isEmpty());
    if (!shouldBeActive && _motorsActive) {
        setMotorsActive(false);
    }
}

// Coordinate transformation functions (removed - now pure polar)

float PolarRobot::normalizeAngle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle > 360.0f) angle -= 360.0f;
    return angle;
}

float PolarRobot::calculateMinRotation(float target, float current) {
    float diff = target - current;

    // Find shortest rotation
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

bool PolarRobot::isInBounds(const PolarPoint& polar) {
    // Theta should be 0-360 degrees, rho should be 0.0-1.0 (normalized)
    return (polar.theta >= 0 && polar.theta < 360.0f && polar.rho >= 0.0f && polar.rho <= 1.0f);
}

esp_err_t PolarRobot::moveToPolar(const PolarPoint& target, float feedRate) {
    if (!_isHomed || _isPaused) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check bounds
    if (!isInBounds(target)) {
        ESP_LOGW(TAG, "Target out of bounds");
        return ESP_ERR_INVALID_ARG;
    }

    // Create motion command for polar move
    MotionCommand cmd = {
        .target = target,
        .feedRate = feedRate > 0 ? feedRate : CONFIG_ROBOT_RHO_MAX_SPEED,
        .isRelative = false,
        .commandId = esp_random()
    };

    // Add to queue
    if (!_commandQueue.push(cmd)) {
        return ESP_ERR_NO_MEM;
    }

    // Cancel any pending inactivity timer since we have new motion
    esp_timer_stop(_motorInactivityTimer);

    ESP_LOGD(TAG, "Move to polar (%.2f°, %.3f) @ %.1f RPM - queued (queue size: %zu)",
        target.theta, target.rho, cmd.feedRate, _commandQueue.getCount());
    return ESP_OK;
}

void PolarRobot::service() {
    // Check if homing just completed and start inactivity timer
    if (_homingControl.state == HomingState::COMPLETE && !_homingControl.homingActive) {
        // Reset homing state to idle and start inactivity timer
        _homingControl.state = HomingState::IDLE;
        const uint64_t INACTIVITY_TIMEOUT_US = 2000000; // 2 seconds
        esp_timer_start_once(_motorInactivityTimer, INACTIVITY_TIMEOUT_US);
    }

    // Handle motion command loading when not homing
    if (!_currentMotion.active && !_homingControl.homingActive) {
        loadNextCommand();
    }
}

bool PolarRobot::loadNextCommand() {
    MotionCommand cmd;
    if (!_commandQueue.pop(cmd)) {
        return false; // No commands in queue
    }
    handleStepOverflow(); // Handle step overflow after motion completion
    ESP_LOGD(TAG, "Loading next command: target(%.2f°, %.3f) @ %.1f RPM (queue remaining: %zu)",
        cmd.target.theta, cmd.target.rho, cmd.feedRate, _commandQueue.getCount());
    planMotion(cmd);
    return true;
}

void PolarRobot::planMotion(const MotionCommand& cmd) {
    stepsToPolar(_thetaPosition, _rhoPosition, _currentMotion.startPolar);

    // Target position is already in polar coordinates
    _currentMotion.targetPolar = cmd.target;

    ESP_LOGD(TAG, "Motion planning: current pos(%.2f°, %.3f) → target pos(%.2f°, %.3f)",
        _currentMotion.startPolar.theta, _currentMotion.startPolar.rho,
        _currentMotion.targetPolar.theta, _currentMotion.targetPolar.rho);
    ESP_LOGD(TAG, "Current motor positions: theta=%ld steps, rho=%ld steps", _thetaPosition, _rhoPosition);

    // Calculate coupled motor steps accounting for interdependent axes
    int32_t thetaMotorSteps, rhoMotorSteps;
    calculateCoupledMotorSteps(_currentMotion.startPolar, _currentMotion.targetPolar,
        thetaMotorSteps, rhoMotorSteps);

    // Set step counts and directions
    _currentMotion.thetaStepsRemaining = abs(thetaMotorSteps);
    _currentMotion.rhoStepsRemaining = abs(rhoMotorSteps);
    _currentMotion.thetaStepDir = (thetaMotorSteps >= 0) ? 1 : -1;
    _currentMotion.rhoStepDir = (rhoMotorSteps >= 0) ? 1 : -1;

    // Initialize Bresenham scheduler for coordinated motion
    _bresenham.thetaSteps = _currentMotion.thetaStepsRemaining; // Theta is axis A
    _bresenham.rhoSteps = _currentMotion.rhoStepsRemaining;   // Rho is axis B
    _bresenham.thetaDir = _currentMotion.thetaStepDir;
    _bresenham.rhoDir = _currentMotion.rhoStepDir;

    // Initialize Bresenham error term based on which axis has more steps
    if (_bresenham.thetaSteps >= _bresenham.rhoSteps) {
        _bresenham.err = _bresenham.thetaSteps / 2;  // Theta is primary axis
    }
    else {
        _bresenham.err = -_bresenham.rhoSteps / 2; // Rho is primary axis
    }

    // Calculate motion timing based on angular movement
    float thetaDelta = fabs(calculateMinRotation(_currentMotion.targetPolar.theta, _currentMotion.startPolar.theta));
    float rhoDelta = fabs(_currentMotion.targetPolar.rho - _currentMotion.startPolar.rho);
    _currentMotion.totalDistance = fmaxf(thetaDelta / 360.0f, rhoDelta); // Normalized motion distance
    _currentMotion.currentDistance = 0;

    _currentMotion.feedRate = cmd.feedRate;
    _currentMotion.commandId = cmd.commandId;
    _currentMotion.active = true;

    setMotorsActive(true);

    gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN, _currentMotion.thetaStepDir > 0 ? 1 : 0);
    gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN, _currentMotion.rhoStepDir > 0 ? 1 : 0);

    ESP_LOGD(TAG, "Direction pins set: theta=%d (dir=%ld), rho=%d (dir=%ld)",
        _currentMotion.thetaStepDir > 0 ? 1 : 0, _currentMotion.thetaStepDir,
        _currentMotion.rhoStepDir > 0 ? 1 : 0, _currentMotion.rhoStepDir);

    ESP_LOGD(TAG, "Coupled motion planned: start(%.2f°,%.3f) → target(%.2f°,%.3f)",
        _currentMotion.startPolar.theta, _currentMotion.startPolar.rho,
        _currentMotion.targetPolar.theta, _currentMotion.targetPolar.rho);
    ESP_LOGD(TAG, "Bresenham motion steps: theta=%ld, rho=%ld (dirs: %ld, %ld), err=%ld",
        _bresenham.thetaSteps, _bresenham.rhoSteps, _bresenham.thetaDir, _bresenham.rhoDir, _bresenham.err);

    // Start step generation timer if there's motion to execute
    if (_bresenham.thetaSteps > 0 || _bresenham.rhoSteps > 0) {
        // Calculate step interval based on RPM feed rate
        float feedRateRPM = _currentMotion.feedRate;
        if (feedRateRPM <= 0) {
            feedRateRPM = CONFIG_ROBOT_RHO_MAX_SPEED; // Use max speed if no feedrate specified
        }

        uint32_t totalSteps = _bresenham.thetaSteps + _bresenham.rhoSteps;
        uint64_t stepInterval;

        if (_currentMotion.totalDistance > 0 && totalSteps > 0) {
            // Calculate interval based on RPM and motion distance
            float motionTimeSeconds = _currentMotion.totalDistance / (feedRateRPM / 60.0f); // Normalize motion time
            stepInterval = (uint64_t)((motionTimeSeconds * 1000000.0f) / totalSteps);
        }
        else {
            // Fallback calculation
            stepInterval = calculateStepInterval();
        }

        // Clamp to reasonable bounds (10kHz max, 1Hz min)
        stepInterval = fmaxf(100, fminf(1000000, stepInterval)); // 100us to 1s

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = stepInterval,
            .flags = {.auto_reload_on_alarm = true}
        };
        gptimer_set_alarm_action(_stepTimer, &alarm_config);
        gptimer_start(_stepTimer);
        ESP_LOGD(TAG, "Motion started - step interval: %llu us (%.1f steps/sec)", stepInterval, 1000000.0f / stepInterval);
    }
}

bool IRAM_ATTR PolarRobot::stepTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    PolarRobot* robot = static_cast<PolarRobot*>(user_ctx);
    robot->generateSteps();
    return true;
}

void IRAM_ATTR PolarRobot::generateSteps() {
    if (_homingControl.homingActive) {
        generateHomingSteps();
        return;
    }

    if (!_currentMotion.active) {
        return;
    }

    bool stepTheta = false;
    bool stepRho = false;
    int32_t thetaS = _bresenham.thetaSteps;
    int32_t rhoS = _bresenham.rhoSteps;

    if (thetaS >= rhoS) {
        _bresenham.err -= rhoS;
        if (_bresenham.err < 0) {
            _bresenham.err += thetaS;
            stepRho = true;
        }
        stepTheta = (thetaS > 0);
    }
    else {
        _bresenham.err += thetaS;
        if (_bresenham.err > 0) {
            _bresenham.err -= rhoS;
            stepTheta = true;
        }
        stepRho = (rhoS > 0);
    }

    if (stepTheta && _bresenham.thetaSteps > 0) {
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN, 1);
        esp_rom_delay_us(2);
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN, 0);

        _bresenham.thetaSteps--;
        _currentMotion.thetaStepsRemaining--;
        _thetaPosition += _bresenham.thetaDir;
    }

    if (stepRho && _bresenham.rhoSteps > 0) {
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 1);
        esp_rom_delay_us(2);
        gpio_set_level((gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN, 0);

        _bresenham.rhoSteps--;
        _currentMotion.rhoStepsRemaining--;
        _rhoPosition += _bresenham.rhoDir;
    }

    if (_bresenham.thetaSteps == 0 && _bresenham.rhoSteps == 0) {
        gptimer_stop(_stepTimer);
        _currentMotion.active = false;
    }
}

void PolarRobot::handleStepOverflow() {
    int32_t thetaRotSteps = _homingControl.steps_per_theta_rot * (CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f);

    while (_thetaPosition > thetaRotSteps) {
        _thetaPosition -= thetaRotSteps;
        _rhoPosition -= CONFIG_ROBOT_RHO_STEPS_PER_ROT;
    }
    while (_thetaPosition <= -thetaRotSteps) {
        _thetaPosition += thetaRotSteps;
        _rhoPosition += CONFIG_ROBOT_RHO_STEPS_PER_ROT;
    }
}

esp_err_t PolarRobot::pause(bool pause) {
    _isPaused = pause;
    if (pause) {
        gptimer_stop(_stepTimer);
        ESP_LOGI(TAG, "Robot paused");
    }
    else {
        ESP_LOGI(TAG, "Robot resumed");
    }
    return ESP_OK;
}

esp_err_t PolarRobot::stop() {
    _isPaused = true;
    gptimer_stop(_stepTimer);
    _currentMotion.clear();
    _bresenham.clear();  // Clear Bresenham scheduler state
    _commandQueue.clear();
    setMotorsActive(false);
    return ESP_OK;
}

esp_err_t PolarRobot::emergencyStop() {
    gptimer_stop(_stepTimer);
    _currentMotion.clear();
    _bresenham.clear();  // Clear Bresenham scheduler state
    setMotorsActive(false);
    _isPaused = true;
    return ESP_OK;
}

RobotStatus PolarRobot::getStatus() {
    RobotStatus status = {};
    if (_isHomed) {
        stepsToPolar(_thetaPosition, _rhoPosition, status.position);
    }

    status.theta.position = _thetaPosition;
    status.theta.isEnabled = _motorsActive; // Use actual motor activity state
    status.theta.isHomed = _isHomed;
    status.theta.isStalled = false;

    status.rho.position = _rhoPosition;
    status.rho.isEnabled = _motorsActive; // Use actual motor activity state
    status.rho.isHomed = _isHomed;
    status.rho.isStalled = _rhoStepper ? _rhoStepper->is_stalled() : false;

    // Overall status
    status.isHomed = _isHomed;
    status.isPaused = _isPaused;
    status.canAcceptCommand = isReady();

    return status;
}

void PolarRobot::deinit() {
    if (_stepTimer) {
        gptimer_stop(_stepTimer);
        gptimer_disable(_stepTimer);
        gptimer_del_timer(_stepTimer);
        _stepTimer = nullptr;
    }

    if (_motorInactivityTimer) {
        esp_timer_stop(_motorInactivityTimer);
        esp_timer_delete(_motorInactivityTimer);
        _motorInactivityTimer = nullptr;
    }

    setMotorsActive(false);
    deinitHomingHardware();

    delete _thetaStepper;
    delete _rhoStepper;
    delete _tmcBus;
}

void PolarRobot::stepsToPolar(int32_t thetaSteps, int32_t rhoSteps, PolarPoint& polar) const {
    float motorRotations = (float)thetaSteps / _homingControl.steps_per_theta_rot; // Convert steps to motor rotations
    polar.theta = motorRotations * 360.0f; // gear rotations -> degrees
    polar.theta = normalizeAngle(polar.theta); // Normalize to 0-360 degrees

    float rhoCounteractSteps = (float)thetaSteps / (CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f); // Counteract theta influence on rho
    polar.rho = stepsToNormalizedRho((float)rhoSteps - rhoCounteractSteps); // Convert to normalized 0-1 range

    ESP_LOGD(TAG, "Steps to polar: theta=%d steps (%.2f°), rho=%d steps (%.3f normalized)",
        thetaSteps, polar.theta, rhoSteps, polar.rho);
    ESP_LOGD(TAG, "Calculated polar point: (%.2f°, %.3f)", polar.theta, polar.rho);
}

void PolarRobot::polarToSteps(PolarPoint& polar, int32_t& thetaSteps, int32_t& rhoSteps) const {
    float theta = polar.theta;
    float rho = polar.rho;

    float driveGearRotations = theta / 360.0f;
    thetaSteps = driveGearRotations * _homingControl.steps_per_theta_rot; // motor rotations -> steps

    rhoSteps = normalizedRhoToSteps(rho); // Convert normalized rho (0-1) to steps

    ESP_LOGD(TAG, "Polar to steps: theta=%.2f° (%.2f rotations) → %d steps, rho=%.3f normalized → %d steps",
        theta, driveGearRotations, thetaSteps, rho, rhoSteps);
}

void PolarRobot::calculateCoupledMotorSteps(const PolarPoint& startPolar, const PolarPoint& targetPolar,
    int32_t& thetaMotorSteps, int32_t& rhoMotorSteps) const {
    PolarPoint deltaPolar;
    deltaPolar.theta = calculateMinRotation(targetPolar.theta, startPolar.theta);
    deltaPolar.rho = targetPolar.rho - startPolar.rho;

    int32_t rhoBase;
    polarToSteps(deltaPolar, thetaMotorSteps, rhoBase);

    float rhoCounteractSteps = thetaMotorSteps / (CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f); // Counteract theta influence on rho
    rhoMotorSteps = rhoBase + rhoCounteractSteps;

    ESP_LOGD(TAG, "Coupled motor steps: theta=%d steps, rho=%d steps (rho base: %d + counteract: %.2f)",
        thetaMotorSteps, rhoMotorSteps, rhoBase, rhoCounteractSteps);
    ESP_LOGD(TAG, "Delta polar: (%.2f°, %.3f)", deltaPolar.theta, deltaPolar.rho);
    ESP_LOGD(TAG, "Target polar: (%.2f°, %.3f)", targetPolar.theta, targetPolar.rho);
    ESP_LOGD(TAG, "Start polar: (%.2f°, %.3f)", startPolar.theta, startPolar.rho);
}

uint64_t PolarRobot::calculateStepInterval() {
    float thetaTimeSeconds = 0;
    float rhoTimeSeconds = 0;

    if (_bresenham.thetaSteps > 0) {
        float driveGearRPM = CONFIG_ROBOT_THETA_MAX_SPEED; // RPM of drive gear
        float motorRPM = driveGearRPM * (CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f); // Convert gear RPM to motor RPM
        float stepsPerSec = (motorRPM * _homingControl.steps_per_theta_rot) / 60.0f;
        thetaTimeSeconds = _bresenham.thetaSteps / stepsPerSec;
    }

    if (_bresenham.rhoSteps > 0) {
        float rhoRPM = _currentMotion.feedRate;
        if (rhoRPM <= 0) {
            rhoRPM = CONFIG_ROBOT_RHO_MAX_SPEED;
        }
        float stepsPerSec = (rhoRPM * 200) / 60.0f;
        rhoTimeSeconds = _bresenham.rhoSteps / stepsPerSec;
    }

    float totalTimeSeconds = fmaxf(thetaTimeSeconds, rhoTimeSeconds);
    uint32_t totalSteps = _bresenham.thetaSteps + _bresenham.rhoSteps;
    uint64_t intervalUs = (uint64_t)((totalTimeSeconds * 1000000.0f) / totalSteps);
    return intervalUs;
}

// Convert normalized rho (0-1) to steps using calibrated maximum
int32_t PolarRobot::normalizedRhoToSteps(float normalizedRho) const {
    if (_homingControl.rho_max_steps <= 0) {
        return 0;
    }
    return normalizedRho * (float)_homingControl.rho_max_steps;
}

float PolarRobot::stepsToNormalizedRho(int32_t steps) const {
    if (_homingControl.rho_max_steps <= 0) {
        return 0.0f;
    }
    return (float)steps / (float)_homingControl.rho_max_steps;
}
