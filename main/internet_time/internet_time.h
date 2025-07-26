#pragma once

#define TIME_INFO_URL "https://firmware.api.koiosdigital.net/tz"

// Time configuration structure
typedef struct {
    bool auto_timezone;      // true to fetch TZ from API, false to use manual TZ
    char timezone[64];       // IANA timezone name (e.g. "America/New_York")
    char ntp_server[128];    // NTP server URL
} time_config_t;

void time_init();
bool is_time_synced();

// Time configuration functions
time_config_t time_get_config(void);
void time_set_config(const time_config_t* config);
void time_load_config_from_nvs(void);
void time_save_config_to_nvs(void);
void time_apply_config(void);