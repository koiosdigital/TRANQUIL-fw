#include <stdio.h>
#include <string.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include <esp_wifi.h>
#include "wifi_provisioning/manager.h"
#include "protocomm_ble.h"

#include "kd_common.h"
#include "cJSON.h"
#include "internet_time.h"
#include "api.h"
#include "sd.h"
#include "sd/ManifestManager.h"
#include "sd/ManifestExample.h"

#include "RobotMotion/RobotMotionAPI.h"
#include "stusb4500/stusb4500.h"
#include "kd_pixdriver.h"

static const char* TAG = "main";

ManifestManager manifestManager;
PixelDriver pixelDriver(60); // 60 Hz update rate
STUSB4500 stusb;

void init_stusb() {
    esp_err_t ret = stusb.begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STUSB4500 at 0x28: %s", esp_err_to_name(ret));
    }

    // Check existing NVM PDOs against desired set (5V/3A, 15V/3A, 20V/5A)
    const float desired_voltages[3] = { 5.0f, 15.0f, 20.0f };
    const float desired_currents[3] = { 3.0f, 3.0f, 5.0f };
    bool need_nvm_init = false;

    // Read NVM to check current configuration
    ret = stusb.readNVM();
    if (ret == ESP_OK) {
        for (int i = 1; i <= 3; ++i) {
            float voltage = stusb.getVoltage(i);
            float current = stusb.getCurrent(i);
            if (fabs(voltage - desired_voltages[i - 1]) > 0.1f || fabs(current - desired_currents[i - 1]) > 0.1f) {
                need_nvm_init = true;
                ESP_LOGI(TAG, "PDO %d mismatch: %.1fV/%.1fA (expected %.1fV/%.1fA)", i,
                    voltage, current, desired_voltages[i - 1], desired_currents[i - 1]);
                break;
            }
        }
    }
    else {
        need_nvm_init = true;
        ESP_LOGW(TAG, "Failed to read NVM, will reprogram");
    }

    if (need_nvm_init) {
        // Program custom PDOs
        ESP_LOGI(TAG, "Programming custom PDOs...");

        // Set PDO1: 5V/3A (fallback)
        stusb.setVoltage(1, 5.0f);
        stusb.setCurrent(1, 3.0f);

        // Set PDO2: 15V/3A
        stusb.setVoltage(2, 15.0f);
        stusb.setCurrent(2, 3.0f);

        // Set PDO3: 20V/5A
        stusb.setVoltage(3, 20.0f);
        stusb.setCurrent(3, 5.0f);

        // Set PDO count to 3
        stusb.setPdoNumber(3);

        // Configure USB communication capability
        stusb.setUsbCommCapable(true);
        stusb.setExternalPower(true);

        // Write configuration to NVM
        ret = stusb.writeNVM();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Successfully programmed PDOs to NVM");

            // Verify by reading back
            ret = stusb.readNVM();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Verification - PDO configuration after programming:");
                for (int i = 1; i <= 3; i++) {
                    float voltage = stusb.getVoltage(i);
                    float current = stusb.getCurrent(i);
                    ESP_LOGI(TAG, "PDO %d: %.1fV/%.1fA (%.1fW)", i, voltage, current, voltage * current);
                }
            }

            ESP_LOGW(TAG, "NOTE: STUSB4500 may require a power cycle to fully load new PDO configuration");
        }
        else {
            ESP_LOGE(TAG, "Failed to write PDOs to NVM: %s", esp_err_to_name(ret));
        }
    }
}

extern "C" void app_main(void)
{
    //event loop
    esp_event_loop_create_default();

    vTaskDelay(pdMS_TO_TICKS(1000)); // Allow time for system to stabilize

    init_sd();
    manifestManager.init();

    init_stusb();

    //use protocomm security version 0
    kd_common_set_provisioning_pop_token_format(ProvisioningPOPTokenFormat_t::NONE);
    kd_common_init();

    api_init();
    time_init();

    pixelDriver.addChannel(ChannelConfig((gpio_num_t)18, 144, PixelFormat::RGBW));
    pixelDriver.setCurrentLimit(2000); // Set current limit to 2A
    pixelDriver.start();

    pixelDriver.setAllChannelsEffect(PixelEffect::RAINBOW);
    pixelDriver.setAllChannelsBrightness(255); // Full brightness
    pixelDriver.setAllChannelsEnabled(true);

    RobotMotionSystem::init();
    RobotMotionSystem::homeAllAxes();

    // Calculate 50% of max radius (155mm from config)
    const double half_radius = 40; // 50% of 155mm max radius
    const double feed_rate = 1000.0; // mm/min - moderate speed for test

    while (!RobotMotionSystem::canAcceptCommand()) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait until robot is ready
    }

    RobotMotionSystem::moveTo(half_radius, half_radius, feed_rate);
    RobotMotionSystem::moveTo(-half_radius, half_radius, feed_rate);
    RobotMotionSystem::moveTo(-half_radius, -half_radius, feed_rate);
    RobotMotionSystem::moveTo(half_radius, -half_radius, feed_rate);
    RobotMotionSystem::moveTo(0, 0, feed_rate);

    ESP_LOGI(TAG, "Square movement test completed");


}