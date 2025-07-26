// ESP-IDF Polar Robot Motion System
// Integrated SandTable Rotary Robot - No Abstraction
// Optimized for ESP32-S3 peripherals

#pragma once

#include "RobotCommon.h"
#include "tmc2209.h"
#include "uartbus.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <cmath>

// Polar coordinates
struct PolarPoint {
    float theta;  // Angle in degrees
    float rho;    // Radius in mm
};

// Cartesian coordinates  
struct CartesianPoint {
    float x, y;   // Position in mm
};

// Motion command
struct MotionCommand {
    CartesianPoint target;
    float feedRate;     // mm/min
    bool isRelative;
    uint32_t commandId;
};

// Motor status
struct MotorStatus {
    int32_t position;     // Steps from home
    bool isEnabled;
    bool isHomed;
    bool isStalled;
    uint16_t current;     // mA
};

// Robot status
struct RobotStatus {
    CartesianPoint position;
    PolarPoint polarPosition;
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
        THETA_BACKING_OFF,
        THETA_SEEKING,
        RHO_SEEKING,
        COMPLETE,
        ERROR
    };

    enum class HomingAxis {
        NONE,
        THETA_ONLY,
        RHO_ONLY,
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
    esp_err_t moveTo(const CartesianPoint& target, float feedRate = 0);
    esp_err_t moveBy(const CartesianPoint& delta, float feedRate = 0);
    esp_err_t moveToPolar(const PolarPoint& target, float feedRate = 0);

    // Homing
    esp_err_t homeAll();

    // Control
    esp_err_t pause(bool pause);
    esp_err_t stop();
    esp_err_t emergencyStop();

    // Status
    RobotStatus getStatus();
    bool isReady() { return _isHomed && !_isPaused && _commandQueue.count < _maxQueueSize; }
    bool areMotorsActive() const { return _motorsActive; }
    bool isHoming() const { return _homingControl.homingActive; }
    HomingState getHomingState() const { return _homingControl.state; }
    HomingAxis getHomingAxis() const { return _homingControl.currentAxis; }

    // Coordinate transformations
    static CartesianPoint polarToCartesian(const PolarPoint& polar);
    static PolarPoint cartesianToPolar(const CartesianPoint& cartesian);
    static bool isInBounds(const PolarPoint& polar, float minRadius, float maxRadius);

    // Service - call regularly from main task
    void service();

private:
    // Configuration from Kconfig
    struct Config {
        // Geometry
        float armLength1, armLength2;

        // TMC Configuration  
        uart_port_t uartPort;
        gpio_num_t txPin, rxPin;
        gpio_num_t enablePin;     // Common enable pin (active low)
        gpio_num_t diagPin;       // StallGuard diagnostic pin
        uint32_t baudRate;
        uint8_t thetaAddr, rhoAddr;

        // Motor parameters
        struct {
            float maxSpeed, maxAccel;     // RPM, RPM/s
            int32_t stepsPerRot;          // Motor steps per motor revolution
            float gearRatio;              // Motor:drive gear ratio (e.g., 5:1 = 5.0)
            uint16_t current;
            gpio_num_t endstopPin;
            gpio_num_t stepPin, dirPin;   // Step and direction pins
            bool endstopInvert;
        } theta;

        struct {
            float maxSpeed, maxAccel;     // mm/min, mm/min²
            int32_t stepsPerMm;           // Motor steps per mm of rack movement
            int32_t stepsPerRot;          // Motor steps per motor revolution
            uint16_t current;
            gpio_num_t stepPin, dirPin;   // Step and direction pins
            float minRadius, maxRadius;
            int8_t stallguardThreshold;
        } rho;

        // Motion planning
        int pipelineLength;
        float junctionDeviation;
        float blockDistance;
    } _config;

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
    esp_timer_handle_t _motorInactivityTimer;
    esp_timer_handle_t _debugTimer;

    // Motion queue and execution
    struct MotionQueue {
        MotionCommand* buffer;
        size_t head, tail, count, size;
        SemaphoreHandle_t mutex;

        MotionQueue() : buffer(nullptr), head(0), tail(0), count(0), size(0), mutex(nullptr) {}
        bool init(size_t queueSize);
        void deinit();
        bool push(const MotionCommand& cmd);
        bool pop(MotionCommand& cmd);
        bool peekLast(MotionCommand& cmd) const;
        size_t available() const { return size - count; }
    } _commandQueue;

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
        int32_t stepsA, stepsB;  // Remaining steps for theta (A) and rho (B)
        int32_t err;             // Error accumulator
        int32_t dirA, dirB;      // Step directions for theta and rho

        BresenhamScheduler() { clear(); }
        void clear() {
            stepsA = stepsB = err = 0;
            dirA = dirB = 0;
        }
    } _bresenham;

    // Homing state machine
    struct HomingControl {
        HomingState state;
        HomingAxis currentAxis;
        int32_t stepCount;
        int32_t maxSteps;
        bool homingActive;

        HomingControl() { clear(); }
        void clear() {
            state = HomingState::IDLE;
            currentAxis = HomingAxis::NONE;
            stepCount = 0;
            maxSteps = 0;
            homingActive = false;
        }
    } _homingControl;

    // Private methods
    void loadConfig();
    esp_err_t initTMC();
    esp_err_t initTimer();
    esp_err_t initGPIO();
    esp_err_t initHomingHardware();
    void deinitHomingHardware();

    // Motor enable management
    void setMotorsActive(bool active);
    static void motorInactivityCallback(void* arg);
    static void debugTimerCallback(void* arg);
    void updateMotorEnableState();

    // Motion planning
    void planMotion(const MotionCommand& cmd);
    bool loadNextCommand();
    uint64_t calculateStepInterval();

    // Step generation (using ESP32-S3 timer)
    static bool IRAM_ATTR stepTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);
    void IRAM_ATTR generateSteps();
    void IRAM_ATTR generateHomingSteps();

    // Homing state machine
    void processHomingStateMachine();
    esp_err_t startThetaHoming();
    esp_err_t startRhoHoming();

    static void IRAM_ATTR rhoStallCallback(uint8_t addr, bool stalled);
    static void IRAM_ATTR endstopPinISR(void* arg);

    // Endstop reading
    bool readThetaEndstop();

    // Queue management helpers
    CartesianPoint getEffectiveStartPosition() const;

    // Utilities
    static float normalizeAngle(float angle);
    static float calculateMinRotation(float target, float current);
    void updatePosition();
    void handleStepOverflow();

    void stepsToPolar(int32_t thetaSteps, int32_t rhoSteps, PolarPoint& polar) const;
    void polarToSteps(PolarPoint& polar, int32_t& thetaSteps, int32_t& rhoSteps) const;

    // Coupled kinematics for coordinated motion
    void calculateCoupledMotorSteps(const PolarPoint& startPolar, const PolarPoint& targetPolar,
        int32_t& thetaMotorSteps, int32_t& rhoMotorSteps) const;

    // Path segmentation for long moves
    esp_err_t segmentLongMove(const CartesianPoint& start, const CartesianPoint& target, float feedRate);

    // Kinematic validation functions
    esp_err_t validateKinematics() const;
};
