#include "internet_time.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#include "cJSON.h"
#include "kd_common.h"
#include "embedded_tz_db.h"

#define TIME_NVS_NAMESPACE "time_cfg"

static const char* TAG = "internet_time";

static bool synced = false;

// Default time configuration
static time_config_t time_config = {
    .auto_timezone = true,                    // Default to API fetch
    .timezone = "UTC",                        // Fallback timezone
    .ntp_server = "pool.ntp.org"             // Default NTP server
};

char itime_response_data[512] = { 0 };
esp_err_t itime_http_event_handler(esp_http_client_event_t* evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        memcpy(itime_response_data, evt->data, evt->data_len);
        itime_response_data[evt->data_len] = '\0';
    }
    return ESP_OK;
}

bool is_time_synced() {
    return synced;
}

void setup_time_task(void* pvParameter) {
    while (true) {
        if (kd_common_is_wifi_connected() == false) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else {
            break;
        }
    }

    ESP_LOGI(TAG, "setting up time");

    // Apply current time configuration
    time_apply_config();

    vTaskDelete(NULL);
}

void time_init() {
    ESP_LOGI(TAG, "initializing time");

    // Load configuration from NVS
    time_load_config_from_nvs();

    // Set up the task to fetch and set the time
    xTaskCreate(setup_time_task, "setup_time_task", 4096, NULL, 5, NULL);
}

// Time configuration functions
time_config_t time_get_config(void) {
    return time_config;
}

void time_set_config(const time_config_t* config) {
    if (config == NULL) return;

    time_config = *config;
    time_save_config_to_nvs();
    time_apply_config();
}

void time_load_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS namespace not found, using defaults");
        return;
    }

    size_t required_size = sizeof(time_config_t);
    err = nvs_get_blob(nvs_handle, "config", &time_config, &required_size);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Config not found in NVS, using defaults");
    }
    else {
        ESP_LOGI(TAG, "Loaded time config from NVS: auto_tz=%d, tz=%s, ntp=%s",
            time_config.auto_timezone, time_config.timezone, time_config.ntp_server);
    }

    nvs_close(nvs_handle);
}

void time_save_config_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(nvs_handle, "config", &time_config, sizeof(time_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config to NVS: %s", esp_err_to_name(err));
    }
    else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(TAG, "Time config saved to NVS");
        }
    }

    nvs_close(nvs_handle);
}

void time_apply_config(void) {
    const char* tzname = time_config.timezone;
    const char* posixTZ = nullptr;

    if (time_config.auto_timezone) {
        ESP_LOGI(TAG, "Fetching timezone from API");

        esp_http_client_config_t config = {
            .url = TIME_INFO_URL,
            .event_handler = itime_http_event_handler,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t http_client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_perform(http_client);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "API response: %s", itime_response_data);

            cJSON* root = cJSON_Parse(itime_response_data);
            if (root != NULL) {
                cJSON* tz_json = cJSON_GetObjectItem(root, "tzname");
                if (tz_json != NULL && cJSON_IsString(tz_json)) {
                    tzname = cJSON_GetStringValue(tz_json);
                    ESP_LOGI(TAG, "API timezone: %s", tzname);

                    // Update the stored timezone with the auto-fetched value
                    strncpy(time_config.timezone, tzname, sizeof(time_config.timezone) - 1);
                    time_config.timezone[sizeof(time_config.timezone) - 1] = '\0';

                    // Save the updated config to NVS so API returns the actual timezone
                    time_save_config_to_nvs();
                    ESP_LOGI(TAG, "Saved auto-fetched timezone to NVS: %s", tzname);

                    posixTZ = tz_db_get_posix_str(tzname);
                }
                cJSON_Delete(root);
            }
        }
        else {
            ESP_LOGW(TAG, "Failed to fetch timezone from API, using manual setting: %s", time_config.timezone);
        }

        esp_http_client_cleanup(http_client);
        memset(itime_response_data, 0, sizeof(itime_response_data));
    }
    else {
        ESP_LOGI(TAG, "Using manual timezone: %s", tzname);
    }

    if (posixTZ == NULL) {
        ESP_LOGW(TAG, "Timezone %s not found in database, using UTC", tzname);
        posixTZ = tz_db_get_posix_str("UTC");
    }

    ESP_LOGI(TAG, "Setting POSIX timezone: %s", posixTZ);
    setenv("TZ", posixTZ, 1);
    tzset();

    // Configure and start SNTP
    ESP_LOGI(TAG, "Starting SNTP with server: %s", time_config.ntp_server);
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(time_config.ntp_server);
    esp_netif_sntp_init(&sntp_config);

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to sync time");
        esp_restart();
    }

    ESP_LOGI(TAG, "Time synchronization complete");
    synced = true;
}