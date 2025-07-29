#include "ota.h"

#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_app_format.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

#include "cJSON.h"
#include "kd_common.h"

static const char* TAG = "kd_ota";

#define OTA_RESPONSE_BUF_SIZE 512

static esp_timer_handle_t ota_timer = NULL;
static SemaphoreHandle_t ota_mutex = NULL;
static bool has_done_boot_check = false;

static esp_err_t _http_event_handler(esp_http_client_event_t* evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->user_data) {
        char* response_buf = (char*)evt->user_data;
        size_t copy_len = evt->data_len < OTA_RESPONSE_BUF_SIZE - 1 ? evt->data_len : OTA_RESPONSE_BUF_SIZE - 1;
        memcpy(response_buf, evt->data, copy_len);
        response_buf[copy_len] = '\0';
    }
    return ESP_OK;
}

static void do_ota_check(void* arg) {
    if (xSemaphoreTake(ota_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "OTA check already running");
        vTaskSuspend(NULL);
        return;
    }
    char* http_response_data = (char*)malloc(OTA_RESPONSE_BUF_SIZE);
    if (!http_response_data) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    memset(http_response_data, 0, OTA_RESPONSE_BUF_SIZE);

    ESP_LOGI(TAG, "checking for updates");
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.url = FIRMWARE_ENDPOINT_URL;
    config.event_handler = _http_event_handler;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.user_data = http_response_data;
    esp_http_client_handle_t http_client = esp_http_client_init(&config);
    const esp_app_desc_t* app_desc = esp_app_get_description();
    esp_http_client_set_header(http_client, "x-firmware-project", app_desc->project_name);
    esp_http_client_set_header(http_client, "x-firmware-version", app_desc->version);
#ifdef FIRMWARE_VARIANT
    esp_http_client_set_header(http_client, "x-firmware-variant", FIRMWARE_VARIANT);
#endif
    if (esp_http_client_perform(http_client) != ESP_OK) {
        ESP_LOGE(TAG, "http request failed");
        esp_http_client_cleanup(http_client);
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    if (esp_http_client_get_status_code(http_client) != 200) {
        ESP_LOGE(TAG, "http request failed: status %d", esp_http_client_get_status_code(http_client));
        esp_http_client_cleanup(http_client);
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    ESP_LOGI(TAG, "response: %s", http_response_data);
    esp_http_client_cleanup(http_client);
    cJSON* root = cJSON_Parse(http_response_data);
    if (root == NULL) {
        ESP_LOGE(TAG, "failed to parse json");
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    if (!cJSON_HasObjectItem(root, "update_available")) {
        ESP_LOGE(TAG, "failed to get update_available");
        cJSON_Delete(root);
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    has_done_boot_check = true;
    if (cJSON_IsFalse(cJSON_GetObjectItem(root, "update_available"))) {
        ESP_LOGI(TAG, "no update available");
        cJSON_Delete(root);
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    if (!cJSON_HasObjectItem(root, "ota_url")) {
        ESP_LOGE(TAG, "failed to get ota_url");
        cJSON_Delete(root);
        free(http_response_data);
        xSemaphoreGive(ota_mutex);
        vTaskSuspend(NULL);
        return;
    }
    const char* ota_url_tmp = cJSON_GetObjectItem(root, "ota_url")->valuestring;
    char* ota_url = strdup(ota_url_tmp);
    cJSON_Delete(root);
    esp_http_client_config_t config2 = {
        .url = ota_url,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &config2,
    };
    esp_err_t err = esp_https_ota(&ota_config);
    free(ota_url);
    free(http_response_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "update failed: %s", esp_err_to_name(err));
        xSemaphoreGive(ota_mutex);
        return;
    }
    ESP_LOGI(TAG, "update successful");
    esp_restart();
}

static void ota_timer_callback(void* arg) {
    if (kd_common_is_wifi_connected()) {
        xTaskCreate(do_ota_check, "do_ota_check", 4096, NULL, 5, NULL);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        if (!has_done_boot_check) {
            xTaskCreate(do_ota_check, "do_ota_check", 4096, NULL, 5, NULL);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xSemaphoreGive(ota_mutex); // Release mutex if OTA was running
    }
}

void ota_init() {
    ota_mutex = xSemaphoreCreateMutex();
    const esp_timer_create_args_t timer_args = {
        .callback = &ota_timer_callback,
        .name = "ota_timer"
    };
    esp_timer_create(&timer_args, &ota_timer);
    esp_timer_start_periodic(ota_timer, 6 * 60 * 60 * 1000000ULL); // 6 hours in microseconds
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
}