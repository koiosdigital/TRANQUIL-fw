#include "esp_http_server.h"
#include "cJSON.h"
#include "ManifestManager.h"
#include <string.h>
#include <stdlib.h>
#include <vector>

// GET /api/playlists - paginated list
static esp_err_t playlists_list_handler(httpd_req_t* req) {
    char page_str[8] = { 0 };
    char page_size_str[8] = { 0 };
    int page = 0, page_size = 20;
    if (httpd_req_get_url_query_str(req, NULL, 0) > 0) {
        char query[128];
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            httpd_query_key_value(query, "page", page_str, sizeof(page_str));
            httpd_query_key_value(query, "page_size", page_size_str, sizeof(page_size_str));
            if (page_str[0]) page = atoi(page_str);
            if (page_size_str[0]) page_size = atoi(page_size_str);
        }
    }
    std::vector<Playlist> playlists = ManifestManager::getAllPlaylists();
    int total = playlists.size();
    int start = page * page_size;
    int end = start + page_size;
    if (end > total) end = total;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Transfer-Encoding", "chunked");
    httpd_resp_send_chunk(req, "{\"total\":", 9);
    char total_str[16];
    int total_len = snprintf(total_str, sizeof(total_str), "%d", total);
    httpd_resp_send_chunk(req, total_str, total_len);
    httpd_resp_send_chunk(req, ",\"playlists\":[", 14);
    for (int i = start; i < end; ++i) {
        cJSON* json = ManifestManager::playlistToJson(playlists[i]);
        char* json_str = cJSON_PrintUnformatted(json);
        if (i > start) httpd_resp_send_chunk(req, ",", 1);
        httpd_resp_send_chunk(req, json_str, strlen(json_str));
        free(json_str);
        cJSON_Delete(json);
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// GET /api/playlists/{uuid}
static esp_err_t playlists_detail_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/playlists/";
    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    Playlist* playlist = ManifestManager::getPlaylist(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    cJSON* json = ManifestManager::playlistToJson(*playlist);
    char* json_str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /api/playlists
static esp_err_t playlists_create_handler(httpd_req_t* req) {
    int len = req->content_len;
    char* buf = (char*)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    int ret = httpd_req_recv(req, buf, len);
    if (ret <= 0) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON* json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    Playlist playlist = ManifestManager::jsonToPlaylist(json);
    cJSON_Delete(json);
    if (ManifestManager::addPlaylist(playlist) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON* resp = ManifestManager::playlistToJson(playlist);
    char* resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

// POST /api/playlists/{uuid} - add/delete pattern
static esp_err_t playlists_modify_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/playlists/";
    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    // Extract uuid (may have trailing /order)
    char uuid[64];
    const char* uuid_start = uri + strlen(base);
    const char* slash = strchr(uuid_start, '/');
    size_t uuid_len = slash ? (size_t)(slash - uuid_start) : strlen(uuid_start);
    if (uuid_len == 0 || uuid_len >= sizeof(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    strncpy(uuid, uuid_start, uuid_len);
    uuid[uuid_len] = '\0';
    Playlist* playlist = ManifestManager::getPlaylist(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    int len = req->content_len;
    char* buf = (char*)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    int ret = httpd_req_recv(req, buf, len);
    if (ret <= 0) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON* json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }
    cJSON* pattern = cJSON_GetObjectItem(json, "pattern");
    cJSON* action = cJSON_GetObjectItem(json, "action");
    if (!pattern || !cJSON_IsString(pattern) || !action || !cJSON_IsString(action)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }
    bool ok = false;
    if (strcmp(action->valuestring, "add") == 0) {
        ok = ManifestManager::addPatternToPlaylist(uuid, pattern->valuestring);
    }
    else if (strcmp(action->valuestring, "delete") == 0) {
        ok = ManifestManager::removePatternFromPlaylist(uuid, pattern->valuestring);
    }
    cJSON_Delete(json);
    if (ok != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    Playlist* updated = ManifestManager::getPlaylist(uuid);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON* resp = ManifestManager::playlistToJson(*updated);
    char* resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

// POST /api/playlists/{uuid}/order - reorder playlist
static esp_err_t playlists_order_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/playlists/";
    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    // Extract uuid
    const char* uuid_start = uri + strlen(base);
    const char* order_str = strstr(uuid_start, "/order");
    size_t uuid_len = order_str ? (size_t)(order_str - uuid_start) : strlen(uuid_start);
    if (uuid_len == 0 || uuid_len >= 64) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    char uuid[64];
    strncpy(uuid, uuid_start, uuid_len);
    uuid[uuid_len] = '\0';
    Playlist* playlist = ManifestManager::getPlaylist(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    int len = req->content_len;
    char* buf = (char*)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    int ret = httpd_req_recv(req, buf, len);
    if (ret <= 0) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON* json = cJSON_Parse(buf);
    free(buf);
    if (!json || !cJSON_IsArray(json)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        if (json) cJSON_Delete(json);
        return ESP_FAIL;
    }
    std::vector<std::string> new_order;
    cJSON* item = NULL;
    cJSON_ArrayForEach(item, json) {
        if (cJSON_IsString(item)) {
            new_order.emplace_back(item->valuestring);
        }
    }
    cJSON_Delete(json);
    if (ManifestManager::reorderPlaylist(uuid, new_order) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    Playlist* updated = ManifestManager::getPlaylist(uuid);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON* resp = ManifestManager::playlistToJson(*updated);
    char* resp_str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

void playlists_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t playlists_list_uri = {
        .uri = "/api/playlists",
        .method = HTTP_GET,
        .handler = playlists_list_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &playlists_list_uri);

    static httpd_uri_t playlists_create_uri = {
        .uri = "/api/playlists",
        .method = HTTP_POST,
        .handler = playlists_create_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &playlists_create_uri);

    static httpd_uri_t playlists_detail_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_GET,
        .handler = playlists_detail_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &playlists_detail_uri);

    static httpd_uri_t playlists_modify_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_POST,
        .handler = playlists_modify_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &playlists_modify_uri);

    static httpd_uri_t playlists_order_uri = {
        .uri = "/api/playlists/*/order",
        .method = HTTP_POST,
        .handler = playlists_order_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &playlists_order_uri);
}
