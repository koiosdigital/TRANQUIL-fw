#include "cloud_api.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "cloud_login.h"
#include "cloud_download_pattern.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE "cloud"
#define NVS_KEY_TOKEN "token"

// Structure for passing response data to the HTTPD work queue
struct cloud_login_response {
    httpd_req_t* req;
    char* response; // JSON string or error message
    bool success;   // true = 200, false = error
};

static void cloud_login_response_sender(void* arg) {
    cloud_login_response* resp = (cloud_login_response*)arg;
    if (resp->success) {
        httpd_resp_set_type(resp->req, "application/json");
        httpd_resp_send(resp->req, resp->response, strlen(resp->response));
    }
    else {
        httpd_resp_send_err(resp->req, HTTPD_500_INTERNAL_SERVER_ERROR, resp->response);
    }
    free(resp->response);
    free(resp);
}

esp_err_t cloud_token_get_handler(httpd_req_t* req) {
    nvs_handle_t nvs;
    char token[512] = { 0 };
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        size_t required_size = sizeof(token);
        err = nvs_get_str(nvs, NVS_KEY_TOKEN, token, &required_size);
        nvs_close(nvs);
        if (err == ESP_OK) {
            cJSON* resp = cJSON_CreateObject();
            cJSON_AddStringToObject(resp, "token", token);
            char* resp_str = cJSON_PrintUnformatted(resp);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, resp_str, strlen(resp_str));
            cJSON_Delete(resp);
            free(resp_str);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No token found");
    return ESP_FAIL;
}

void cloud_api_register_handlers(httpd_handle_t server) {
    httpd_uri_t login_uri = {
        .uri = "/api/cloud/login",
        .method = HTTP_POST,
        .handler = cloud_login_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &login_uri);

    httpd_uri_t token_uri = {
        .uri = "/api/cloud/token",
        .method = HTTP_GET,
        .handler = cloud_token_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &token_uri);

    httpd_uri_t download_pattern_uri = {
        .uri = "/api/cloud/download_pattern",
        .method = HTTP_POST,
        .handler = cloud_download_pattern_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &download_pattern_uri);
}
