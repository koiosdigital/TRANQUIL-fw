// ESP-IDF Polar Robot Motion System
// Integrated SandTable Rotary Robot - No Abstraction
// Optimized for ESP32-S3 peripherals

#pragma once

#include "tmc2209.h"
#include "uartbus.h"
#include "MotionQueue.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <cmath>

// Motor status
struct MotorStatus {
    int32_t position;     // Steps from home
    bool isEnabled;
    bool isHomed;
    bool isStalled;
};

// Robot status
struct RobotStatus {
    PolarPoint position;
    MotorStatus theta;
    MotorStatus rho;
    bool isHomed;
    bool isPaused;
    bool canAcceptCommand;
};

class PolarRobot
{
public:
    static constexpr int THETA_AXIS = 0;
    static constexpr int RHO_AXIS = 1;

    // Homing state machine enums (public for status reporting)
    enum class HomingState {
        IDLE,
        // Rho calibration phases
        RHO_SEEKING_MAX,
        RHO_SEEKING_MIN,
        // Theta calibration phases
        THETA_BACKING_OFF,
        THETA_SEEKING_FIRST_EDGE,
        THETA_SEEKING_SECOND_EDGE,
        COMPLETE,
        ERROR
    };

    enum class HomingAxis {
        NONE,
        SEQUENTIAL_THETA,  // First phase of sequential homing
        SEQUENTIAL_RHO     // Second phase of sequential homing
    };

    // Constructor
    PolarRobot();
    ~PolarRobot();

    // Initialization
    esp_err_t init();
    void deinit();

    // Motion control
    esp_err_t moveToPolar(const PolarPoint& target, float feedRate = 0);

    // Homing
    esp_err_t homeAll();

    // Control
    esp_err_t pause(bool pause);
    esp_err_t stop();
    esp_err_t emergencyStop();

    // Status
    RobotStatus getStatus();
    bool isReady() { return _isHomed && !_isPaused && _commandQueue.getCount() < _commandQueue.getSize(); }
    bool areMotorsActive() const { return _motorsActive; }
    bool isHoming() const { return _homingControl.homingActive; }
    HomingState getHomingState() const { return _homingControl.state; }
    HomingAxis getHomingAxis() const { return _homingControl.currentAxis; }

    // Calibration results (available after homing completes)
    int32_t getRhoMaxSteps() const { return _homingControl.rho_max_steps; }
    int32_t getStepsPerThetaRotation() const { return _homingControl.steps_per_theta_rot; }

    // Polar coordinate utilities
    static bool isInBounds(const PolarPoint& polar);

    // Service - call regularly from main task
    void service();

    void setCommandNotifyTask(TaskHandle_t task) { _commandNotifyTask = task; }

private:
    // Hardware interfaces
    UartBus* _tmcBus;
    TMC2209Stepper* _thetaStepper;
    TMC2209Stepper* _rhoStepper;

    // ESP32-S3 Hardware Timer for step generation
    gptimer_handle_t _stepTimer;

    // Current state
    volatile int32_t _thetaPosition;  // Steps from home
    volatile int32_t _rhoPosition;    // Steps from home
    volatile bool _isHomed;
    volatile bool _isPaused;

    // IRQ-based sensor state
    volatile bool _thetaEndstopTriggered;  // Set by endstop IRQ
    volatile bool _rhoStallguardTriggered; // Set by stallguard IRQ

    // Motor activity tracking for enable pin management
    volatile bool _motorsActive;
    gptimer_handle_t _motorInactivityTimer;

    // Motion queue and execution
    MotionQueue _commandQueue;

    size_t _maxQueueSize;

    // Current motion execution
    struct CurrentMotion {
        bool active;
        PolarPoint startPolar;
        PolarPoint targetPolar;
        PolarPoint currentPolar;
        float totalDistance;
        float currentDistance;
        float feedRate;
        uint32_t commandId;

        // Step generation state
        int32_t thetaStepsRemaining;
        int32_t rhoStepsRemaining;
        int32_t thetaStepDir;
        int32_t rhoStepDir;

        CurrentMotion() { clear(); }
        void clear() {
            active = false;
            totalDistance = currentDistance = feedRate = 0;
            thetaStepsRemaining = rhoStepsRemaining = 0;
            thetaStepDir = rhoStepDir = 0;
            commandId = 0;
        }
    } _currentMotion;

    // Bresenham-style step scheduler for coordinated motion
    struct BresenhamScheduler {
        int32_t thetaSteps, rhoSteps;  // Remaining steps for theta (A) and rho (B)
        int32_t err;             // Error accumulator
        int32_t thetaDir, rhoDir;      // Step directions for theta and rho

        BresenhamScheduler() { clear(); }
        void clear() {
            thetaSteps = rhoSteps = err = 0;
            thetaDir = rhoDir = 0;
        }
    } _bresenham;

    // Homing state machine
    struct HomingControl {
        HomingState state;
        HomingAxis currentAxis;
        int32_t stepCount;
        int32_t maxSteps;
        int32_t thetaStepCounter; // For coupled motion: counts theta steps to determine when rho should step
        bool homingActive;

        // Calibration results
        int32_t rho_max_steps;     // Steps from max to min rho position
        int32_t steps_per_theta_rot; // Steps for one complete theta rotation
        bool first_theta_edge_found; // Track if we found the first rising edge

        HomingControl() { clear(); }
        void clear() {
            state = HomingState::IDLE;
            currentAxis = HomingAxis::NONE;
            stepCount = 0;
            maxSteps = 0;
            thetaStepCounter = 0;
            homingActive = false;
            rho_max_steps = 0;
            steps_per_theta_rot = 0;
            first_theta_edge_found = false;
        }
    } _homingControl;

    esp_err_t initTMC();
    esp_err_t initTimer();
    esp_err_t initGPIO();
    esp_err_t initHomingHardware();
    void deinitHomingHardware();

    // Motor enable management
    void setMotorsActive(bool active);
    static void motorInactivityCallback(void* arg);
    static void debugTimerCallback(void* arg);

    // Motion planning
    void planMotion(const MotionCommand& cmd);
    bool loadNextCommand();
    uint64_t calculateStepInterval();

    // Step generation (using ESP32-S3 timer)
    static bool stepTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);
    void generateSteps();
    void generateHomingSteps();

    void startRhoMaxHomingISR(); // ISR-safe version for rho max calibration
    void startRhoMinHomingISR(); // ISR-safe version for rho min calibration
    void startThetaCalibrationISR(); // ISR-safe version for theta calibration

    static void rhoStallCallback(uint8_t addr, bool stalled);
    static void endstopPinISR(void* arg);

    // Endstop reading
    bool readThetaEndstop();

    // Utilities
    static double normalizeAngle(double angle);
    static double calculateMinRotation(double target, double current);
    void updatePosition();
    void handleStepOverflow();

    void stepsToPolar(int32_t thetaSteps, int32_t rhoSteps, PolarPoint& polar) const;
    void polarToSteps(PolarPoint& polar, int32_t& thetaSteps, int32_t& rhoSteps) const;

    // Coupled kinematics for coordinated motion
    void calculateCoupledMotorSteps(const PolarPoint& startPolar, const PolarPoint& targetPolar,
        int32_t& thetaMotorSteps, int32_t& rhoMotorSteps) const;

    // Convert normalized rho (0-1) to steps using calibrated maximum
    int32_t normalizedRhoToSteps(double normalizedRho) const;
    double stepsToNormalizedRho(int32_t steps) const;

    // Kinematic validation functions
    esp_err_t validateKinematics() const;

    TaskHandle_t _commandNotifyTask = nullptr;
    void notifyCommandReadyFromISR();

    void resetMotorInactivityTimer();
    static bool inactivityTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);
};
