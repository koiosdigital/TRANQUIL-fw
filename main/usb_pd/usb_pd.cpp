#include "usb_pd.h"

#include "stusb4500.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#include "math.h"

static const char* TAG = "usb_pd";

STUSB4500 stusb;

void stusb_init() {
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