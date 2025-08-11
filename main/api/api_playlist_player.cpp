#include "api_playlist_player.h"
#include "PlaylistPlayer.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_server.h"

// Helper: get state as cJSON
cJSON* api_playlist_player_get_state_json() {
    return PlaylistPlayer::getStateJSON();
}

// GET /api/playlist_player - get state
static esp_err_t handle_get_state(httpd_req_t* req) {
    cJSON* state = api_playlist_player_get_state_json();
    char* resp = cJSON_PrintUnformatted(state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    cJSON_Delete(state);
    free(resp);
    return ESP_OK;
}

// POST /api/playlist_player/play - play playlist { "uuid": "...", "shuffle": true/false }
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
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid uuid");
        return ESP_FAIL;
    }

    bool shuffle = false;
    cJSON* shuffle_item = cJSON_GetObjectItem(root, "shuffle");
    if (shuffle_item && cJSON_IsBool(shuffle_item)) {
        shuffle = cJSON_IsTrue(shuffle_item);
    }

    esp_err_t ret = PlaylistPlayer::playPlaylist(uuid->valuestring, shuffle);
    cJSON_Delete(root);

    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return handle_get_state(req);
}

// POST /api/playlist_player/stop - stop playlist playback
static esp_err_t handle_stop(httpd_req_t* req) {
    PlaylistPlayer::stop();
    return handle_get_state(req);
}

// POST /api/playlist_player/skip - skip to next pattern
static esp_err_t handle_skip(httpd_req_t* req) {
    esp_err_t ret = PlaylistPlayer::skip();
    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    return handle_get_state(req);
}

// POST /api/playlist_player/shuffle - set shuffle mode { "shuffle": true/false }
static esp_err_t handle_shuffle(httpd_req_t* req) {
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) return ESP_FAIL;

    cJSON* shuffle = cJSON_GetObjectItem(root, "shuffle");
    if (!shuffle || !cJSON_IsBool(shuffle)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid shuffle");
        return ESP_FAIL;
    }

    esp_err_t ret = PlaylistPlayer::setShuffle(cJSON_IsTrue(shuffle));
    cJSON_Delete(root);

    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return handle_get_state(req);
}

// URI handler definitions
static httpd_uri_t playlist_player_state_uri = {
    .uri = "/api/playlist_player",
    .method = HTTP_GET,
    .handler = handle_get_state,
    .user_ctx = NULL
};

static httpd_uri_t playlist_player_play_uri = {
    .uri = "/api/playlist_player/play",
    .method = HTTP_POST,
    .handler = handle_play,
    .user_ctx = NULL
};

static httpd_uri_t playlist_player_stop_uri = {
    .uri = "/api/playlist_player/stop",
    .method = HTTP_POST,
    .handler = handle_stop,
    .user_ctx = NULL
};

static httpd_uri_t playlist_player_skip_uri = {
    .uri = "/api/playlist_player/skip",
    .method = HTTP_POST,
    .handler = handle_skip,
    .user_ctx = NULL
};

static httpd_uri_t playlist_player_shuffle_uri = {
    .uri = "/api/playlist_player/shuffle",
    .method = HTTP_POST,
    .handler = handle_shuffle,
    .user_ctx = NULL
};

void api_playlist_player_register_endpoints(httpd_handle_t server) {
    httpd_register_uri_handler(server, &playlist_player_state_uri);
    httpd_register_uri_handler(server, &playlist_player_play_uri);
    httpd_register_uri_handler(server, &playlist_player_stop_uri);
    httpd_register_uri_handler(server, &playlist_player_skip_uri);
    httpd_register_uri_handler(server, &playlist_player_shuffle_uri);
}
