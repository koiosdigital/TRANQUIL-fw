#pragma once

#include "esp_http_server.h"

// Registers the cloud API login handler
void cloud_api_register_handlers(httpd_handle_t server);

// Handler to get the stored cloud token
esp_err_t cloud_token_get_handler(httpd_req_t* req);
