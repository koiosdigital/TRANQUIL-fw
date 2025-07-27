#pragma once

#include "PolarRobot.h"
#include "esp_err.h"

// Simplified Robot Motion System API using integrated PolarRobot with FreeRTOS task
class RobotMotionSystem
{
public:
    static esp_err_t init();
    static void deinit();
    static PolarRobot* getRobot();

    // Motion commands
    static esp_err_t moveTo(double x, double y, double feedRate = 0);
    static esp_err_t moveToPolar(double theta, double rho, double feedRate = 0);
    static esp_err_t homeAllAxes();
    static esp_err_t stopMotion();
    static esp_err_t pauseMotion(bool pause);
    static esp_err_t emergencyStop();

    // Status
    static RobotStatus getStatus();
    static bool isHomed();
    static bool canAcceptCommand();
    static bool isPaused();

private:
    static PolarRobot* _robot;
    static bool _initialized;

    // FreeRTOS task function
    static void motionTask(void* pvParameters);
};