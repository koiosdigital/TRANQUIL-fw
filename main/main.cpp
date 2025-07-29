#include <stdio.h>
#include <string.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_event.h"
#include "esp_log.h"

#include "kd_common.h"
#include "kd_ntp.h"
#include "api.h"

#include "stusb4500.h"
#include "kd_pixdriver.h"
#include "RobotMotionAPI.h"
#include "ManifestManager.h"

static const char* TAG = "main";

STUSB4500 stusb;

void init_stusb() {
    esp_err_t ret = stusb.begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STUSB4500: %s", esp_err_to_name(ret));
        esp_restart(); // Restart if initialization fails
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
                break;
            }
        }
    }
    else {
        need_nvm_init = true;
    }

    if (need_nvm_init) {
        stusb.setVoltage(1, 5.0f);
        stusb.setCurrent(1, 3.0f);
        stusb.setVoltage(2, 15.0f);
        stusb.setCurrent(2, 3.0f);
        stusb.setVoltage(3, 20.0f);
        stusb.setCurrent(3, 5.0f);
        stusb.setPdoNumber(3);
        stusb.setUsbCommCapable(true);
        stusb.setExternalPower(true);
        stusb.writeNVM();
    }
}

extern "C" void app_main(void)
{
    //event loop
    esp_event_loop_create_default();

    init_stusb();

    //use protocomm security version 0
    kd_common_set_provisioning_pop_token_format(ProvisioningPOPTokenFormat_t::NONE);
    kd_common_init();

    //api_init();
    KdNTP::init();

    ManifestManager::initialize();

    PixelDriver::initialize(60); // 60Hz update rate
    PixelDriver::addChannel(ChannelConfig((gpio_num_t)18, 144, PixelFormat::RGBW));
    PixelDriver::setCurrentLimit(2000); // Set current limit to 2A
    PixelDriver::start();

    PixelDriver::getMainChannel()->setEffect(EffectConfig{
        .effect = PixelEffect::RAINBOW,
        .color = {0, 255, 0, 0}, // Green
        .brightness = 255,
        .speed = 50,
        .enabled = true,
        });

    RobotMotionSystem::init();
    RobotMotionSystem::homeAllAxes();
}