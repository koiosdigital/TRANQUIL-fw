#include "kd_ntp.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cJSON.h"
#include "embedded_tz_db.h"

static const char* TAG = "kd_ntp";

static char itime_response_data[512] = { 0 };

// Static member definitions
bool KdNTP::synced = false;
esp_event_handler_instance_t KdNTP::wifi_handler_instance = nullptr;

// Default time configuration
time_config_t KdNTP::time_config = {
    .auto_timezone = true,                    // Default to API fetch
    .timezone = "UTC",                        // Fallback timezone
    .ntp_server = "pool.ntp.org"             // Default NTP server
};
esp_err_t KdNTP::http_event_handler(esp_http_client_event_t* evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        memcpy(itime_response_data, evt->data, evt->data_len);
        itime_response_data[evt->data_len] = '\0';
    }
    return ESP_OK;
}

bool KdNTP::is_synced() {
    return synced;
}

void KdNTP::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGD(TAG, "WiFi got IP, starting time setup task");
        xTaskCreate(setup_time_task, "setup_time_task", 4096, NULL, 8, NULL);
    }
}

void KdNTP::setup_time_task(void* pvParameter) {
    ESP_LOGI(TAG, "setting up time");

    // Apply current time configuration
    apply_config();

    vTaskDelete(NULL);
}

void KdNTP::init() {
    load_config_from_nvs();

    // Register for WiFi events
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &wifi_handler_instance);
}

// Time configuration functions
time_config_t KdNTP::get_config() {
    return time_config;
}

void KdNTP::set_config(const time_config_t* config) {
    if (config == NULL) return;

    time_config = *config;
    save_config_to_nvs();
    apply_config();
}

void KdNTP::load_config_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found, using defaults");
        return;
    }

    size_t required_size = sizeof(time_config_t);
    err = nvs_get_blob(nvs_handle, "config", &time_config, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Config not found in NVS, using defaults");
    }

    ESP_LOGD(TAG, "Loaded time config from NVS: auto_tz=%d, tz=%s, ntp=%s",
        time_config.auto_timezone, time_config.timezone, time_config.ntp_server);

    nvs_close(nvs_handle);
}

void KdNTP::save_config_to_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(nvs_handle, "config", &time_config, sizeof(time_config_t));
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
}

void KdNTP::apply_config() {
    const char* tzname = time_config.timezone;
    const char* posixTZ = nullptr;

    if (time_config.auto_timezone) {
        esp_http_client_config_t config = {
            .url = TIME_INFO_URL,
            .event_handler = http_event_handler,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t http_client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_perform(http_client);

        if (err == ESP_OK) {
            ESP_LOGD(TAG, "tzapi response: %s", itime_response_data);

            cJSON* root = cJSON_Parse(itime_response_data);
            if (root != NULL) {
                cJSON* tz_json = cJSON_GetObjectItem(root, "tzname");
                if (tz_json != NULL && cJSON_IsString(tz_json)) {
                    tzname = cJSON_GetStringValue(tz_json);

                    strncpy(time_config.timezone, tzname, sizeof(time_config.timezone) - 1);
                    time_config.timezone[sizeof(time_config.timezone) - 1] = '\0';

                    save_config_to_nvs();

                    posixTZ = tz_db_get_posix_str(tzname);
                }
                cJSON_Delete(root);
            }
        }

        esp_http_client_cleanup(http_client);
        memset(itime_response_data, 0, sizeof(itime_response_data));
    }

    if (posixTZ == NULL) {
        ESP_LOGW(TAG, "Timezone %s not found in database, using UTC", tzname);
    }
    else {
        ESP_LOGD(TAG, "Setting POSIX timezone: %s", posixTZ);
        setenv("TZ", posixTZ, 1);
        tzset();
    }

    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(time_config.ntp_server);
    esp_netif_sntp_init(&sntp_config);

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        esp_restart();
    }

    synced = true;
}