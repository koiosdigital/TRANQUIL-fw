#pragma once

// ESP-IDF includes
#include "esp_log.h"
#include "esp_err.h"
#include <string>
#include <cmath>

// Compatibility defines and types
typedef std::string String;

// Configuration helper class to replace ConfigBase and JSON parsing
class RobotConfig {
public:
    // Get string value from Kconfig
    static String getString(const char* key, const char* defaultValue);

    // Get double value from Kconfig  
    static double getDouble(const char* key, double defaultValue);

    // Get long value from Kconfig
    static long getLong(const char* key, long defaultValue);

    // Get boolean value from Kconfig
    static bool getBool(const char* key, bool defaultValue);

    // Check if axis configuration exists
    static bool hasAxisConfig(int axisIdx);

    // Get axis-specific configuration
    static double getAxisDouble(int axisIdx, const char* param, double defaultValue);
    static long getAxisLong(int axisIdx, const char* param, long defaultValue);
    static bool getAxisBool(int axisIdx, const char* param, bool defaultValue);
};

// Robot command arguments structure
class RobotCommandArgs {
public:
    double x, y, z;
    double feedRate;
    bool isRelative;
    bool hasX, hasY, hasZ;

    RobotCommandArgs() : x(0), y(0), z(0), feedRate(0), isRelative(false),
        hasX(false), hasY(false), hasZ(false) {
    }

    void clear() {
        x = y = z = feedRate = 0;
        isRelative = hasX = hasY = hasZ = false;
    }
};

// Compatibility macros
#define LOG_TAG "RobotMotion"

// Math constants if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
