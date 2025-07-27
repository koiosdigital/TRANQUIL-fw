#pragma once

#include "esp_event.h"
#include "esp_http_client.h"

#define TIME_INFO_URL "https://firmware.api.koiosdigital.net/tz"
#define TIME_NVS_NAMESPACE "time_cfg"

// Time configuration structure
typedef struct {
    bool auto_timezone;      // true to fetch TZ from API, false to use manual TZ
    char timezone[64];       // IANA timezone name (e.g. "America/New_York")
    char ntp_server[128];    // NTP server URL
} time_config_t;

class KdNTP {
public:
    static void init();
    static bool is_synced();
    static time_config_t get_config();
    static void set_config(const time_config_t* config);

private:
    static bool synced;
    static time_config_t time_config;
    static esp_event_handler_instance_t wifi_handler_instance;

    static void load_config_from_nvs();
    static void save_config_to_nvs();
    static void apply_config();
    static void setup_time_task(void* pvParameter);
    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static esp_err_t http_event_handler(esp_http_client_event_t* evt);
};