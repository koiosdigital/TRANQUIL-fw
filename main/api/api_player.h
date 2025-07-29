#pragma once

#include "esp_err.h"
#include "cJSON.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

    // Register all /api/player endpoints with the given httpd server
    void api_player_register_endpoints(httpd_handle_t server);

    // For internal use: get state as cJSON
    cJSON* api_player_get_state_json();

#ifdef __cplusplus
}
#endif
