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
static const float microstepMultiplier = 16.0f;
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
    , _debugTimer(nullptr)
    , _maxQueueSize(500) // Default fallback size
{
    g_robotInstance = this;
    loadConfig();
}

PolarRobot::~PolarRobot() {
    deinit();
    g_robotInstance = nullptr;
}

void PolarRobot::loadConfig() {
    // Load configuration from Kconfig
    _config.armLength1 = CONFIG_ROBOT_ARM_LENGTH_1;
    _config.armLength2 = CONFIG_ROBOT_ARM_LENGTH_2;

    _config.uartPort = (uart_port_t)CONFIG_ROBOT_TMC_UART_PORT;
    _config.txPin = (gpio_num_t)CONFIG_ROBOT_TMC_UART_TX_PIN;
    _config.rxPin = (gpio_num_t)CONFIG_ROBOT_TMC_UART_RX_PIN;
#ifdef CONFIG_ROBOT_COMMON_ENABLE_PIN
    _config.enablePin = (gpio_num_t)CONFIG_ROBOT_COMMON_ENABLE_PIN;
#else
    _config.enablePin = GPIO_NUM_NC; // Default if not configured
#endif
#ifdef CONFIG_ROBOT_STALLGUARD_DIAG_PIN
    _config.diagPin = (gpio_num_t)CONFIG_ROBOT_STALLGUARD_DIAG_PIN;
#else
    _config.diagPin = GPIO_NUM_NC;
#endif
    _config.baudRate = CONFIG_ROBOT_TMC_BAUD_RATE;
    _config.thetaAddr = CONFIG_ROBOT_THETA_TMC_ADDR;
    _config.rhoAddr = CONFIG_ROBOT_RHO_TMC_ADDR;

    // Theta axis
    _config.theta.maxSpeed = CONFIG_ROBOT_THETA_MAX_SPEED;
    _config.theta.maxAccel = CONFIG_ROBOT_THETA_MAX_ACCEL;
    _config.theta.stepsPerRot = CONFIG_ROBOT_THETA_STEPS_PER_ROT;
#ifdef CONFIG_ROBOT_THETA_GEAR_RATIO
    _config.theta.gearRatio = CONFIG_ROBOT_THETA_GEAR_RATIO / 100.0f; // Convert from integer to float
#else
    _config.theta.gearRatio = 1.0f; // Default to 1:1 if not configured
#endif
    _config.theta.current = CONFIG_ROBOT_THETA_MOTOR_CURRENT;
    _config.theta.endstopPin = (gpio_num_t)CONFIG_ROBOT_THETA_ENDSTOP_PIN;
    _config.theta.stepPin = (gpio_num_t)CONFIG_ROBOT_THETA_STEP_PIN;
    _config.theta.dirPin = (gpio_num_t)CONFIG_ROBOT_THETA_DIR_PIN;

#if defined(CONFIG_ROBOT_THETA_ENDSTOP_INVERT) && CONFIG_ROBOT_THETA_ENDSTOP_INVERT
    _config.theta.endstopInvert = true;
#else
    _config.theta.endstopInvert = false;
#endif

    // Rho axis
    _config.rho.maxSpeed = CONFIG_ROBOT_RHO_MAX_SPEED;
    _config.rho.maxAccel = CONFIG_ROBOT_RHO_MAX_ACCEL;
    _config.rho.stepsPerMm = CONFIG_ROBOT_RHO_STEPS_PER_MM;
    _config.rho.stepsPerRot = CONFIG_ROBOT_RHO_STEPS_PER_ROT;
    _config.rho.current = CONFIG_ROBOT_RHO_MOTOR_CURRENT;
    _config.rho.stepPin = (gpio_num_t)CONFIG_ROBOT_RHO_STEP_PIN;
    _config.rho.dirPin = (gpio_num_t)CONFIG_ROBOT_RHO_DIR_PIN;
    _config.rho.minRadius = CONFIG_ROBOT_RHO_MIN_RADIUS;
    _config.rho.maxRadius = CONFIG_ROBOT_RHO_MAX_RADIUS;
    _config.rho.stallguardThreshold = CONFIG_ROBOT_RHO_STALLGUARD_THRESHOLD;

    // Motion planning
    _config.pipelineLength = CONFIG_ROBOT_MOTION_PIPELINE_LENGTH;
    _config.junctionDeviation = CONFIG_ROBOT_JUNCTION_DEVIATION / 1000.0f; // Convert from μm
    _config.blockDistance = CONFIG_ROBOT_BLOCK_DISTANCE_MM / 1000.0f; // Convert from μm
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

    // Create debug timer for motion status logging
    esp_timer_create_args_t debug_timer_args = {
        .callback = debugTimerCallback,
        .arg = this,
        .name = "motion_debug"
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&debug_timer_args, &_debugTimer), TAG, "Failed to create debug timer");

    setMotorsActive(false);
    return ESP_OK;
}

esp_err_t PolarRobot::initTMC() {
    // Create TMC UART bus
    _tmcBus = new UartBus();
    if (!_tmcBus) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(_tmcBus->initialize(_config.uartPort, _config.txPin, _config.rxPin, _config.baudRate),
        TAG, "Failed to init TMC UART bus");

    // Create theta stepper
    _thetaStepper = new TMC2209Stepper(*_tmcBus, _config.thetaAddr);
    if (!_thetaStepper) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(_thetaStepper->initialize(), TAG, "Failed to init theta stepper");
    _thetaStepper->set_motor_current(_config.theta.current);
    _thetaStepper->set_microstep_resolution(MicrostepResolution::SIXTEENTH);
    _thetaStepper->set_stealthchop_enable(true);
    _thetaStepper->set_stealthchop_threshold(0);

    // Create rho stepper with StallGuard
    _rhoStepper = new TMC2209Stepper(*_tmcBus, _config.rhoAddr);
    if (!_rhoStepper) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(_rhoStepper->initialize(), TAG, "Failed to init rho stepper");
    _rhoStepper->set_motor_current(_config.rho.current);
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
    if (_config.enablePin != GPIO_NUM_NC) {
        gpio_config_t enable_config = {
            .pin_bit_mask = (1ULL << _config.enablePin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_RETURN_ON_ERROR(gpio_config(&enable_config), TAG, "Failed to configure enable pin");

        gpio_set_level(_config.enablePin, 1);
        ESP_LOGI(TAG, "Common enable pin configured on GPIO %d", _config.enablePin);
    }

    gpio_config_t theta_pins_config = {
        .pin_bit_mask = (1ULL << _config.theta.stepPin) | (1ULL << _config.theta.dirPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&theta_pins_config), TAG, "Failed to configure theta step/dir pins");
    gpio_set_level(_config.theta.stepPin, 0);
    gpio_set_level(_config.theta.dirPin, 0);

    gpio_config_t rho_pins_config = {
        .pin_bit_mask = (1ULL << _config.rho.stepPin) | (1ULL << _config.rho.dirPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rho_pins_config), TAG, "Failed to configure rho step/dir pins");
    gpio_set_level(_config.rho.stepPin, 0);
    gpio_set_level(_config.rho.dirPin, 0);

    return ESP_OK;
}

void PolarRobot::setMotorsActive(bool active) {
    if (_motorsActive == active) {
        return;
    }

    _motorsActive = active;

    if (_config.enablePin != GPIO_NUM_NC) {
        if (active) {
            gpio_set_level(_config.enablePin, 0);
            esp_timer_stop(_motorInactivityTimer);
        }
        else {
            gpio_set_level(_config.enablePin, 1);
        }
    }
}

void PolarRobot::motorInactivityCallback(void* arg) {
    PolarRobot* robot = static_cast<PolarRobot*>(arg);
    robot->updateMotorEnableState();
}

void PolarRobot::debugTimerCallback(void* arg) {
    PolarRobot* robot = static_cast<PolarRobot*>(arg);
    if (robot->_currentMotion.active) {
        ESP_LOGI(TAG, "Bresenham motion status: theta=%ld remaining, rho=%ld remaining, timer active",
            robot->_bresenham.stepsA, robot->_bresenham.stepsB);
    }
}

void PolarRobot::updateMotorEnableState() {
    bool shouldBeActive = (_currentMotion.active || _commandQueue.count > 0);
    if (!shouldBeActive && _motorsActive) {
        setMotorsActive(false);
    }
}

// Coordinate transformation functions
CartesianPoint PolarRobot::polarToCartesian(const PolarPoint& polar) {
    float theta_rad = polar.theta * M_PI / 180.0f;
    return {
        .x = polar.rho * cosf(theta_rad),
        .y = polar.rho * sinf(theta_rad)
    };
}

PolarPoint PolarRobot::cartesianToPolar(const CartesianPoint& cartesian) {
    float rho = sqrtf(cartesian.x * cartesian.x + cartesian.y * cartesian.y);
    float theta = atan2f(cartesian.y, cartesian.x) * 180.0f / M_PI;

    // Normalize theta to 0-360 degrees
    theta = normalizeAngle(theta);

    return { .theta = theta, .rho = rho };
}

float PolarRobot::normalizeAngle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
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

bool PolarRobot::isInBounds(const PolarPoint& polar, float minRadius, float maxRadius) {
    return (polar.rho >= minRadius && polar.rho <= maxRadius);
}

CartesianPoint PolarRobot::getEffectiveStartPosition() const {
    // Check if there are commands in the queue
    MotionCommand lastCmd;
    if (_commandQueue.peekLast(lastCmd)) {
        // Use the target of the last queued command as our starting position
        return lastCmd.target;
    }
    else {
        // No commands in queue, use current physical position
        PolarPoint currentPolar;
        stepsToPolar(_thetaPosition, _rhoPosition, currentPolar);
        return polarToCartesian(currentPolar);
    }
}

esp_err_t PolarRobot::moveTo(const CartesianPoint& target, float feedRate) {
    if (!_isHomed || _isPaused) {
        return ESP_ERR_INVALID_STATE;
    }

    // Convert to polar and check bounds
    PolarPoint targetPolar = cartesianToPolar(target);
    if (!isInBounds(targetPolar, _config.rho.minRadius, _config.rho.maxRadius)) {
        ESP_LOGW(TAG, "Target out of bounds: rho=%.2f (min=%.2f, max=%.2f)",
            targetPolar.rho, _config.rho.minRadius, _config.rho.maxRadius);
        return ESP_ERR_INVALID_ARG;
    }

    // Get effective starting position (either from queue tail or current position)
    CartesianPoint startCartesian = getEffectiveStartPosition();

    // Calculate straight-line distance
    float distance = sqrtf(
        powf(target.x - startCartesian.x, 2) +
        powf(target.y - startCartesian.y, 2)
    );

    // If move is greater than 1mm, segment it into smaller blocks
    if (distance > 1.0f) {
        return segmentLongMove(startCartesian, target, feedRate);
    }

    // Create motion command for short moves
    MotionCommand cmd = {
        .target = target,
        .feedRate = feedRate > 0 ? feedRate : _config.rho.maxSpeed,
        .isRelative = false,
        .commandId = esp_random()
    };

    // Add to queue
    if (!_commandQueue.push(cmd)) {
        return ESP_ERR_NO_MEM;
    }

    // Cancel any pending inactivity timer since we have new motion
    esp_timer_stop(_motorInactivityTimer);

    ESP_LOGV(TAG, "Move to (%.2f, %.2f) @ %.1f mm/min - queued (queue size: %zu)",
        target.x, target.y, cmd.feedRate, _commandQueue.count);
    return ESP_OK;
}

esp_err_t PolarRobot::moveBy(const CartesianPoint& delta, float feedRate) {
    if (!_isHomed || _isPaused) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get effective starting position (either from queue tail or current position)
    CartesianPoint startCartesian = getEffectiveStartPosition();

    CartesianPoint target = {
        .x = startCartesian.x + delta.x,
        .y = startCartesian.y + delta.y
    };

    return moveTo(target, feedRate);
}

esp_err_t PolarRobot::moveToPolar(const PolarPoint& target, float feedRate) {
    if (!_isHomed || _isPaused) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check bounds
    if (!isInBounds(target, _config.rho.minRadius, _config.rho.maxRadius)) {
        ESP_LOGW(TAG, "Target out of bounds: rho=%.2f (min=%.2f, max=%.2f)",
            target.rho, _config.rho.minRadius, _config.rho.maxRadius);
        return ESP_ERR_INVALID_ARG;
    }

    // Convert to cartesian and use existing moveTo (which handles segmentation)
    CartesianPoint cartesianTarget = polarToCartesian(target);
    return moveTo(cartesianTarget, feedRate);
}

bool PolarRobot::MotionQueue::init(size_t queueSize) {
    if (queueSize == 0) {
        return false;
    }

    // Limit maximum queue size to prevent excessive memory usage
    if (queueSize > 1000) {
        queueSize = 1000;
    }

    buffer = (MotionCommand*)malloc(queueSize * sizeof(MotionCommand));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate motion queue buffer (%zu bytes)", queueSize * sizeof(MotionCommand));
        return false;
    }

    mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        free(buffer);
        buffer = nullptr;
        return false;
    }

    size = queueSize;
    head = tail = count = 0;

    return true;
}

void PolarRobot::MotionQueue::deinit() {
    if (buffer) {
        free(buffer);
        buffer = nullptr;
    }
    if (mutex) {
        vSemaphoreDelete(mutex);
        mutex = nullptr;
    }
}

bool PolarRobot::MotionQueue::push(const MotionCommand& cmd) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) return false;

    bool result = false;
    if (count < size) {
        buffer[tail] = cmd;
        tail = (tail + 1) % size;
        count++;
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

bool PolarRobot::MotionQueue::pop(MotionCommand& cmd) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) return false;

    bool result = false;
    if (count > 0) {
        cmd = buffer[head];
        head = (head + 1) % size;
        count--;
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

bool PolarRobot::MotionQueue::peekLast(MotionCommand& cmd) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) return false;

    bool result = false;
    if (count > 0) {
        // Calculate index of last item (one before tail)
        size_t lastIndex = (tail == 0) ? (size - 1) : (tail - 1);
        cmd = buffer[lastIndex];
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

void PolarRobot::service() {
    if (_homingControl.homingActive) {
        processHomingStateMachine();
        return;
    }

    if (!_currentMotion.active) {
        loadNextCommand();
    }
}

bool PolarRobot::loadNextCommand() {
    MotionCommand cmd;
    if (!_commandQueue.pop(cmd)) {
        return false; // No commands in queue
    }
    handleStepOverflow(); // Handle step overflow after motion completion
    ESP_LOGV(TAG, "Loading next command: target(%.2f, %.2f) @ %.1f mm/min (queue remaining: %zu)",
        cmd.target.x, cmd.target.y, cmd.feedRate, _commandQueue.count);
    planMotion(cmd);
    return true;
}

void PolarRobot::planMotion(const MotionCommand& cmd) {
    stepsToPolar(_thetaPosition, _rhoPosition, _currentMotion.startPolar);

    // Target position in polar coordinates
    _currentMotion.targetPolar = cartesianToPolar(cmd.target);

    ESP_LOGV(TAG, "Motion planning: current pos(%.2f°, %.2fmm) → target pos(%.2f°, %.2fmm)",
        _currentMotion.startPolar.theta, _currentMotion.startPolar.rho,
        _currentMotion.targetPolar.theta, _currentMotion.targetPolar.rho);
    ESP_LOGV(TAG, "Current motor positions: theta=%ld steps, rho=%ld steps", _thetaPosition, _rhoPosition);

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
    _bresenham.stepsA = _currentMotion.thetaStepsRemaining; // Theta is axis A
    _bresenham.stepsB = _currentMotion.rhoStepsRemaining;   // Rho is axis B
    _bresenham.dirA = _currentMotion.thetaStepDir;
    _bresenham.dirB = _currentMotion.rhoStepDir;

    // Initialize Bresenham error term based on which axis has more steps
    if (_bresenham.stepsA >= _bresenham.stepsB) {
        _bresenham.err = _bresenham.stepsA / 2;  // Theta is primary axis
    }
    else {
        _bresenham.err = -_bresenham.stepsB / 2; // Rho is primary axis
    }

    // Calculate motion timing and speed profiles
    CartesianPoint startCartesian = polarToCartesian(_currentMotion.startPolar);
    CartesianPoint targetCartesian = polarToCartesian(_currentMotion.targetPolar);
    float totalDistance = sqrtf(
        powf(targetCartesian.x - startCartesian.x, 2) +
        powf(targetCartesian.y - startCartesian.y, 2)
    );
    _currentMotion.totalDistance = totalDistance;
    _currentMotion.currentDistance = 0;

    _currentMotion.feedRate = cmd.feedRate;
    _currentMotion.commandId = cmd.commandId;
    _currentMotion.active = true;

    setMotorsActive(true);

    gpio_set_level(_config.theta.dirPin, _currentMotion.thetaStepDir > 0 ? 1 : 0);
    gpio_set_level(_config.rho.dirPin, _currentMotion.rhoStepDir > 0 ? 1 : 0);

    ESP_LOGV(TAG, "Direction pins set: theta=%d (dir=%ld), rho=%d (dir=%ld)",
        _currentMotion.thetaStepDir > 0 ? 1 : 0, _currentMotion.thetaStepDir,
        _currentMotion.rhoStepDir > 0 ? 1 : 0, _currentMotion.rhoStepDir);

    ESP_LOGV(TAG, "Coupled motion planned: start(%.2f°,%.2fmm) → target(%.2f°,%.2fmm)",
        _currentMotion.startPolar.theta, _currentMotion.startPolar.rho,
        _currentMotion.targetPolar.theta, _currentMotion.targetPolar.rho);
    ESP_LOGV(TAG, "Bresenham motion steps: theta=%ld, rho=%ld (dirs: %ld, %ld), err=%ld",
        _bresenham.stepsA, _bresenham.stepsB, _bresenham.dirA, _bresenham.dirB, _bresenham.err);

    // Start step generation timer if there's motion to execute
    if (_bresenham.stepsA > 0 || _bresenham.stepsB > 0) {
        // Calculate fixed step interval based on path length and feed rate
        float totalDistance = _currentMotion.totalDistance; // in mm
        float feedRateMMPerSec = _currentMotion.feedRate / 60.0f; // Convert mm/min to mm/sec
        if (feedRateMMPerSec <= 0) {
            feedRateMMPerSec = _config.rho.maxSpeed / 60.0f; // Use max speed if no feedrate specified
        }

        uint32_t totalSteps = _bresenham.stepsA + _bresenham.stepsB;
        uint64_t stepInterval;

        if (totalDistance > 0 && totalSteps > 0) {
            // Calculate interval based on total path time distributed across all steps
            stepInterval = (uint64_t)((totalDistance / feedRateMMPerSec) * 1000000.0f / totalSteps);
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

        // Start debug timer to log motion status every second
        esp_timer_start_periodic(_debugTimer, 1000000); // 1 second
        ESP_LOGV(TAG, "Motion started - step interval: %llu us (%.1f steps/sec)", stepInterval, 1000000.0f / stepInterval);
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

    bool stepA = false, stepB = false;
    int32_t Sa = _bresenham.stepsA, Sb = _bresenham.stepsB;

    // Bresenham-style step scheduling for coordinated motion
    if (Sa >= Sb) {
        // Theta (A) is primary axis
        _bresenham.err -= Sb;
        if (_bresenham.err < 0) {
            _bresenham.err += Sa;
            stepB = true;  // step rho
        }
        stepA = (Sa > 0);
    }
    else {
        // Rho (B) is primary axis  
        _bresenham.err += Sa;
        if (_bresenham.err > 0) {
            _bresenham.err -= Sb;
            stepA = true;  // step theta
        }
        stepB = (Sb > 0);
    }

    // Generate theta steps using hardware GPIO
    if (stepA && _bresenham.stepsA > 0) {
        // Direction pin already set in planMotion() - just generate step pulse
        gpio_set_level(_config.theta.stepPin, 1);
        esp_rom_delay_us(2); // Minimum pulse width for stepper drivers (2µs)
        gpio_set_level(_config.theta.stepPin, 0);

        _bresenham.stepsA--;
        _currentMotion.thetaStepsRemaining--;
        _thetaPosition += _bresenham.dirA;
    }

    // Generate rho steps using hardware GPIO
    if (stepB && _bresenham.stepsB > 0) {
        // Direction pin already set in planMotion() - just generate step pulse
        gpio_set_level(_config.rho.stepPin, 1);
        esp_rom_delay_us(2); // Minimum pulse width for stepper drivers (2µs)
        gpio_set_level(_config.rho.stepPin, 0);

        _bresenham.stepsB--;
        _currentMotion.rhoStepsRemaining--;
        _rhoPosition += _bresenham.dirB;
    }

    if (_bresenham.stepsA == 0 && _bresenham.stepsB == 0) {
        gptimer_stop(_stepTimer);
        _currentMotion.active = false;
    }
}

void PolarRobot::handleStepOverflow() {
    int32_t thetaRotSteps = _config.theta.stepsPerRot * microstepMultiplier * _config.theta.gearRatio;
    int32_t rhoRotSteps = _config.rho.stepsPerRot * microstepMultiplier;

    while (_thetaPosition > thetaRotSteps) {
        _thetaPosition -= thetaRotSteps;
        _rhoPosition -= rhoRotSteps;
    }
    while (_thetaPosition <= -thetaRotSteps) {
        _thetaPosition += thetaRotSteps;
        _rhoPosition += rhoRotSteps;
    }
}

esp_err_t PolarRobot::pause(bool pause) {
    _isPaused = pause;
    if (pause) {
        gptimer_stop(_stepTimer);
        esp_timer_stop(_debugTimer);
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
    esp_timer_stop(_debugTimer);
    _currentMotion.clear();
    _bresenham.clear();  // Clear Bresenham scheduler state
    MotionCommand dummy;
    while (_commandQueue.pop(dummy)) { /* empty queue */ }
    setMotorsActive(false);
    return ESP_OK;
}

esp_err_t PolarRobot::emergencyStop() {
    gptimer_stop(_stepTimer);
    esp_timer_stop(_debugTimer);
    _currentMotion.clear();
    _bresenham.clear();  // Clear Bresenham scheduler state
    setMotorsActive(false);
    _isPaused = true;
    return ESP_OK;
}

RobotStatus PolarRobot::getStatus() {
    RobotStatus status = {};
    stepsToPolar(_thetaPosition, _rhoPosition, status.polarPosition);

    // Convert to cartesian
    status.position = polarToCartesian(status.polarPosition);

    // Motor status
    status.theta.position = _thetaPosition;
    status.theta.isEnabled = _motorsActive; // Use actual motor activity state
    status.theta.isHomed = _isHomed;
    status.theta.isStalled = false;
    status.theta.current = _config.theta.current;

    status.rho.position = _rhoPosition;
    status.rho.isEnabled = _motorsActive; // Use actual motor activity state
    status.rho.isHomed = _isHomed;
    status.rho.isStalled = _rhoStepper ? _rhoStepper->is_stalled() : false;
    status.rho.current = _config.rho.current;

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

    if (_debugTimer) {
        esp_timer_stop(_debugTimer);
        esp_timer_delete(_debugTimer);
        _debugTimer = nullptr;
    }

    // Disable motors via enable pin
    setMotorsActive(false);

    // Deinitialize homing hardware
    deinitHomingHardware();

    delete _thetaStepper;
    delete _rhoStepper;
    delete _tmcBus;

    _commandQueue.deinit();
}

void PolarRobot::stepsToPolar(int32_t thetaSteps, int32_t rhoSteps, PolarPoint& polar) const {
    float fullThetaSteps = (float)thetaSteps / microstepMultiplier;
    float motorRotations = fullThetaSteps / _config.theta.stepsPerRot;
    float driveGearRotations = motorRotations / _config.theta.gearRatio; // motor -> gear
    polar.theta = driveGearRotations * 360.0f; // gear rotations -> degrees
    polar.theta = normalizeAngle(polar.theta); // Normalize to 0-360 degrees

    float fullRhoSteps = (float)rhoSteps / microstepMultiplier;
    float rhoCounteractSteps = fullThetaSteps / _config.theta.gearRatio;
    polar.rho = (fullRhoSteps - rhoCounteractSteps) / _config.rho.stepsPerMm;
}

void PolarRobot::polarToSteps(PolarPoint& polar, int32_t& thetaSteps, int32_t& rhoSteps) const {
    float theta = polar.theta;
    float rho = polar.rho;

    float driveGearRotations = theta / 360.0f;
    float motorRotations = driveGearRotations * _config.theta.gearRatio;
    float thetaFullSteps = motorRotations * _config.theta.stepsPerRot;
    thetaSteps = thetaFullSteps * microstepMultiplier;

    float rhoFullSteps = rho * _config.rho.stepsPerMm;
    rhoSteps = rhoFullSteps * microstepMultiplier;
}

void PolarRobot::calculateCoupledMotorSteps(const PolarPoint& startPolar, const PolarPoint& targetPolar,
    int32_t& thetaMotorSteps, int32_t& rhoMotorSteps) const {
    PolarPoint deltaPolar;
    deltaPolar.theta = calculateMinRotation(targetPolar.theta, startPolar.theta);
    deltaPolar.rho = targetPolar.rho - startPolar.rho;

    int32_t rhoBase;
    polarToSteps(deltaPolar, thetaMotorSteps, rhoBase);

    float rhoCounteractSteps = thetaMotorSteps / _config.theta.gearRatio;
    rhoMotorSteps = rhoBase + rhoCounteractSteps;
}

esp_err_t PolarRobot::segmentLongMove(const CartesianPoint& start, const CartesianPoint& target, float feedRate) {
    // Calculate total distance
    float totalDistance = sqrtf(
        powf(target.x - start.x, 2) +
        powf(target.y - start.y, 2)
    );

    // Calculate number of segments needed (1mm max per segment)
    const float maxSegmentLength = 1.0f; // mm
    int numSegments = (int)ceilf(totalDistance / maxSegmentLength);

    // Ensure we have at least 2 segments for moves > 1mm
    if (numSegments < 2) {
        numSegments = 2;
    }

    // Safety check: limit maximum segments to prevent queue overflow
    const int maxSegments = _maxQueueSize / 2; // Use half of queue capacity
    if (numSegments > maxSegments) {
        ESP_LOGW(TAG, "Move of %.2fmm would require %d segments, limiting to %d",
            totalDistance, numSegments, maxSegments);
        numSegments = maxSegments;
    }

    // Check if we have enough queue space
    if (numSegments > (int)_commandQueue.available()) {
        ESP_LOGW(TAG, "Not enough queue space for %d segments (available: %zu)",
            numSegments, _commandQueue.available());
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Segmenting %.2fmm move into %d segments of %.2fmm each",
        totalDistance, numSegments, totalDistance / numSegments);

    // Calculate step size for each segment
    float deltaX = (target.x - start.x) / numSegments;
    float deltaY = (target.y - start.y) / numSegments;

    // Create intermediate waypoints along the straight line
    for (int i = 1; i <= numSegments; i++) {
        CartesianPoint waypoint = {
            .x = start.x + (deltaX * i),
            .y = start.y + (deltaY * i)
        };

        // Check bounds for this waypoint
        PolarPoint waypointPolar = cartesianToPolar(waypoint);
        if (!isInBounds(waypointPolar, _config.rho.minRadius, _config.rho.maxRadius)) {
            ESP_LOGW(TAG, "Waypoint %d out of bounds: rho=%.2f (min=%.2f, max=%.2f)",
                i, waypointPolar.rho, _config.rho.minRadius, _config.rho.maxRadius);
            return ESP_ERR_INVALID_ARG;
        }

        // Create motion command for this segment
        MotionCommand cmd = {
            .target = waypoint,
            .feedRate = feedRate > 0 ? feedRate : _config.rho.maxSpeed,
            .isRelative = false,
            .commandId = esp_random()
        };

        // Add to queue
        if (!_commandQueue.push(cmd)) {
            ESP_LOGW(TAG, "Failed to queue segment %d - motion queue full", i);
            return ESP_ERR_NO_MEM;
        }

        ESP_LOGV(TAG, "Segment %d: (%.2f, %.2f) @ %.1f mm/min - queued",
            i, waypoint.x, waypoint.y, cmd.feedRate);
    }

    // Cancel any pending inactivity timer since we have new motion
    esp_timer_stop(_motorInactivityTimer);

    ESP_LOGI(TAG, "Segmented move completed: %d segments queued (queue size: %zu)",
        numSegments, _commandQueue.count);

    return ESP_OK;
}

uint64_t PolarRobot::calculateStepInterval() {
    float thetaTimeSeconds = 0;
    float rhoTimeSeconds = 0;

    if (_bresenham.stepsA > 0) {
        const float microstepMultiplier = 16.0f;
        float driveGearRPM = _config.theta.maxSpeed;
        float motorRPM = driveGearRPM * _config.theta.gearRatio;
        float fullMotorStepsPerSecond = (motorRPM * _config.theta.stepsPerRot) / 60.0f;
        float microstepsPerSecond = fullMotorStepsPerSecond * microstepMultiplier;
        thetaTimeSeconds = _bresenham.stepsA / microstepsPerSecond;
    }

    if (_bresenham.stepsB > 0) {
        const float microstepMultiplier = 16.0f;
        float feedRateMMPerSec = _currentMotion.feedRate / 60.0f;
        if (feedRateMMPerSec <= 0) {
            feedRateMMPerSec = _config.rho.maxSpeed / 60.0f;
        }
        float fullStepsPerSecond = feedRateMMPerSec * _config.rho.stepsPerMm;
        float microstepsPerSecond = fullStepsPerSecond * microstepMultiplier;
        rhoTimeSeconds = _bresenham.stepsB / microstepsPerSecond;
    }

    float totalTimeSeconds = fmaxf(thetaTimeSeconds, rhoTimeSeconds);
    uint32_t totalSteps = _bresenham.stepsA + _bresenham.stepsB;
    uint64_t intervalUs = (uint64_t)((totalTimeSeconds * 1000000.0f) / totalSteps);
    return intervalUs;
}
