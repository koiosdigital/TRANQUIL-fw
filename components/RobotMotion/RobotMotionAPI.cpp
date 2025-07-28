#include "RobotMotionAPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

PolarRobot* RobotMotionSystem::_robot = nullptr;
bool RobotMotionSystem::_initialized = false;

static const char* LOG_TAG = "RobotMotionSystem";

esp_err_t RobotMotionSystem::init()
{
    if (_initialized) {
        ESP_LOGW(LOG_TAG, "Robot Motion System already initialized");
        return ESP_OK;
    }

    ESP_LOGI(LOG_TAG, "Initializing Polar Robot Motion System");

    _robot = new PolarRobot();
    if (!_robot) {
        ESP_LOGE(LOG_TAG, "Failed to create polar robot");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = _robot->init();
    if (result != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Failed to initialize polar robot: %s", esp_err_to_name(result));
        delete _robot;
        _robot = nullptr;
        return result;
    }

    // Create the motion control task
    BaseType_t taskResult = xTaskCreate(
        motionTask,
        "robot_motion",
        8192,  // Configurable stack size
        nullptr,                              // Task parameters
        4,    // Configurable priority
        nullptr                               // Task handle
    );

    if (taskResult != pdPASS) {
        ESP_LOGE(LOG_TAG, "Failed to create motion task");
        delete _robot;
        _robot = nullptr;
        return ESP_ERR_NO_MEM;
    }

    _initialized = true;
    ESP_LOGI(LOG_TAG, "Polar Robot Motion System initialized successfully with FreeRTOS task");

    return ESP_OK;
}

void RobotMotionSystem::deinit() {
    if (_robot) {
        _robot->deinit();
        delete _robot;
        _robot = nullptr;
    }
    _initialized = false;
}

// FreeRTOS task for motion control
void RobotMotionSystem::motionTask(void* pvParameters)
{
    ESP_LOGI(LOG_TAG, "Robot motion task started");

    const TickType_t taskDelay = pdMS_TO_TICKS(10);

    while (true) {
        if (_robot && _initialized) {
            _robot->service();
        }

        vTaskDelay(taskDelay);
    }
}

PolarRobot* RobotMotionSystem::getRobot()
{
    return _robot;
}

esp_err_t RobotMotionSystem::moveToPolar(double theta, double rho, double feedRate)
{
    if (!_robot) {
        return ESP_ERR_INVALID_STATE;
    }

    PolarPoint target = { .theta = (float)theta, .rho = (float)rho };
    return _robot->moveToPolar(target, feedRate);
}

esp_err_t RobotMotionSystem::homeAllAxes()
{
    if (!_robot) {
        return ESP_ERR_INVALID_STATE;
    }

    return _robot->homeAll();
}

esp_err_t RobotMotionSystem::stopMotion()
{
    if (!_robot) {
        return ESP_ERR_INVALID_STATE;
    }

    return _robot->stop();
}

esp_err_t RobotMotionSystem::pauseMotion(bool pause)
{
    if (!_robot) {
        return ESP_ERR_INVALID_STATE;
    }

    return _robot->pause(pause);
}

esp_err_t RobotMotionSystem::emergencyStop()
{
    if (!_robot) {
        return ESP_ERR_INVALID_STATE;
    }

    return _robot->emergencyStop();
}

RobotStatus RobotMotionSystem::getStatus()
{
    if (!_robot) {
        return RobotStatus{}; // Return empty status
    }

    return _robot->getStatus();
}

bool RobotMotionSystem::isHomed()
{
    if (!_robot) {
        return false;
    }

    return _robot->getStatus().isHomed;
}

bool RobotMotionSystem::canAcceptCommand()
{
    if (!_robot) {
        return false;
    }

    return _robot->isReady();
}

bool RobotMotionSystem::isPaused()
{
    if (!_robot) {
        return false;
    }

    return _robot->getStatus().isPaused;
}
