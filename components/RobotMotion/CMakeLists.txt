idf_component_register(SRCS "RobotMotionAPI.cpp"
                           "PolarRobot.cpp"
                           "PolarHoming.cpp"
                           "MotionQueue.cpp"
                     
                    INCLUDE_DIRS "."
                    REQUIRES tmcstepper driver esp_timer freertos esp_system)
