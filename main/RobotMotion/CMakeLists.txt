idf_component_register(SRCS "RobotCommon.cpp"
                           "RobotMotionAPI.cpp"
                           "PolarRobot.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES tmcstepper driver esp_timer freertos esp_system esp_log)
