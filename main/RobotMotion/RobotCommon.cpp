#include "RobotCommon.h"
#include "sdkconfig.h"
#include <cstring>

// Implementation of RobotConfig methods
String RobotConfig::getString(const char* key, const char* defaultValue) {
    // Return defaults - no model differentiation needed
    return String(defaultValue);
}

double RobotConfig::getDouble(const char* key, double defaultValue) {
    if (strcmp(key, "junctionDeviation") == 0) {
        return CONFIG_ROBOT_JUNCTION_DEVIATION / 1000.0; // Convert from micrometers
    }
    return defaultValue;
}

long RobotConfig::getLong(const char* key, long defaultValue) {
    if (strcmp(key, "pipelineLen") == 0) {
        return CONFIG_ROBOT_MOTION_PIPELINE_LENGTH;
    }
    return defaultValue;
}

bool RobotConfig::getBool(const char* key, bool defaultValue) {
    return defaultValue;
}

bool RobotConfig::hasAxisConfig(int axisIdx) {
    return (axisIdx == 0 || axisIdx == 1); // Only support 2 axes
}

double RobotConfig::getAxisDouble(int axisIdx, const char* param, double defaultValue) {
    if (axisIdx == 0) { // Theta axis
        if (strcmp(param, "maxSpeed") == 0) {
            return CONFIG_ROBOT_THETA_MAX_SPEED; // RPM
        }
        else if (strcmp(param, "maxAcc") == 0) {
            return CONFIG_ROBOT_THETA_MAX_ACCEL; // RPM/s
        }
        else if (strcmp(param, "stepsPerRot") == 0) {
            return CONFIG_ROBOT_THETA_STEPS_PER_ROT;
        }
        else if (strcmp(param, "unitsPerRot") == 0) {
            return 360.0; // degrees per rotation
        }
        else if (strcmp(param, "maxRPM") == 0) {
            return CONFIG_ROBOT_THETA_MAX_SPEED;
        }
    }
    else if (axisIdx == 1) { // Rho axis
        if (strcmp(param, "maxSpeed") == 0) {
            return CONFIG_ROBOT_RHO_MAX_SPEED / 60.0; // Convert mm/min to mm/s
        }
        else if (strcmp(param, "maxAcc") == 0) {
            return CONFIG_ROBOT_RHO_MAX_ACCEL / 3600.0; // Convert mm/min² to mm/s²
        }
        else if (strcmp(param, "stepsPerRot") == 0) {
            return CONFIG_ROBOT_RHO_STEPS_PER_MM * 2.0 * M_PI; // Approximate for lead screw
        }
        else if (strcmp(param, "unitsPerRot") == 0) {
            return 2.0 * M_PI; // mm per rotation (for lead screw)
        }
        else if (strcmp(param, "minVal") == 0) {
            return CONFIG_ROBOT_RHO_MIN_RADIUS;
        }
        else if (strcmp(param, "maxVal") == 0) {
            return CONFIG_ROBOT_RHO_MAX_RADIUS;
        }
    }
    return defaultValue;
}

long RobotConfig::getAxisLong(int axisIdx, const char* param, long defaultValue) {
    if (axisIdx == 0) { // Theta axis
        if (strcmp(param, "isDominantAxis") == 0) {
            return 0; // Theta is not dominant
        }
        else if (strcmp(param, "isPrimaryAxis") == 0) {
            return 1; // Theta is primary
        }
    }
    else if (axisIdx == 1) { // Rho axis
        if (strcmp(param, "isDominantAxis") == 0) {
            return 1; // Rho is dominant (controls feed rate)
        }
        else if (strcmp(param, "isPrimaryAxis") == 0) {
            return 1; // Rho is also primary
        }
    }
    return defaultValue;
}

bool RobotConfig::getAxisBool(int axisIdx, const char* param, bool defaultValue) {
    return defaultValue;
}
