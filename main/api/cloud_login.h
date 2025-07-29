#include <stdbool.h>
#include "esp_http_server.h"

// Result struct for login
typedef struct {
    bool success;
    char* token; // malloc'd if present, must be freed by caller
    char* error; // malloc'd if present, must be freed by caller
} cloud_login_result_t;

void cloud_login_clear_response_buffer();
char* cloud_login_get_response_buffer();
cloud_login_result_t cloud_login_perform(const char* email, const char* password);

// HTTP handler for login
esp_err_t cloud_login_handler(httpd_req_t* req);
