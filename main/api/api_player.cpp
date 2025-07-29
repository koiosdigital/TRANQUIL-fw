#include "api_player.h"
#include "PatternPlayer.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_server.h"

// Helper: get state as cJSON
cJSON* api_player_get_state_json() {
    return PatternPlayer::getStateJSON();
}

// GET /api/player - get state
static esp_err_t handle_get_state(httpd_req_t* req) {
    cJSON* state = api_player_get_state_json();
    char* resp = cJSON_PrintUnformatted(state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    cJSON_Delete(state);
    free(resp);
    return ESP_OK;
}

// POST /api/player/play - play pattern { "uuid": ... }
static esp_err_t handle_play(httpd_req_t* req) {
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';
    cJSON* root = cJSON_Parse(buf);
    if (!root) return ESP_FAIL;
    cJSON* uuid = cJSON_GetObjectItem(root, "uuid");
    if (!uuid || !cJSON_IsString(uuid)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    esp_err_t ret = PatternPlayer::playPattern(uuid->valuestring);
    cJSON_Delete(root);
    if (ret != ESP_OK) return ESP_FAIL;
    return handle_get_state(req);
}

// POST /api/player/pause - pause
static esp_err_t handle_pause(httpd_req_t* req) {
    PatternPlayer::pause();
    return handle_get_state(req);
}

// POST /api/player/stop - stop
static esp_err_t handle_stop(httpd_req_t* req) {
    PatternPlayer::stop();
    return handle_get_state(req);
}

// POST /api/player/speed - set feed rate { "speed": ... }
static esp_err_t handle_speed(httpd_req_t* req) {
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';
    cJSON* root = cJSON_Parse(buf);
    if (!root) return ESP_FAIL;
    cJSON* speed = cJSON_GetObjectItem(root, "speed");
    if (!speed || !cJSON_IsNumber(speed)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    // If setFeedRate is not implemented, stub it here
    // PatternPlayer::setFeedRate(speed->valuedouble);
    cJSON_Delete(root);
    return handle_get_state(req);
}

// URI handler definitions
static httpd_uri_t player_state_uri = {
    .uri = "/api/player",
    .method = HTTP_GET,
    .handler = handle_get_state,
    .user_ctx = NULL
};
static httpd_uri_t player_play_uri = {
    .uri = "/api/player/play",
    .method = HTTP_POST,
    .handler = handle_play,
    .user_ctx = NULL
};
static httpd_uri_t player_pause_uri = {
    .uri = "/api/player/pause",
    .method = HTTP_POST,
    .handler = handle_pause,
    .user_ctx = NULL
};
static httpd_uri_t player_stop_uri = {
    .uri = "/api/player/stop",
    .method = HTTP_POST,
    .handler = handle_stop,
    .user_ctx = NULL
};
static httpd_uri_t player_speed_uri = {
    .uri = "/api/player/speed",
    .method = HTTP_POST,
    .handler = handle_speed,
    .user_ctx = NULL
};

void api_player_register_endpoints(httpd_handle_t server) {
    httpd_register_uri_handler(server, &player_state_uri);
    httpd_register_uri_handler(server, &player_play_uri);
    httpd_register_uri_handler(server, &player_pause_uri);
    httpd_register_uri_handler(server, &player_stop_uri);
    httpd_register_uri_handler(server, &player_speed_uri);
}
