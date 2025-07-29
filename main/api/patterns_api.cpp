#include "patterns_api.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "ManifestManager.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <vector>
#include <unistd.h>

// Helper: get pattern file size on disk
static size_t get_pattern_size_on_disk(const std::string& uuid) {
    char path[128];
    snprintf(path, sizeof(path), "/sd/patterns/%s.thr", uuid.c_str());
    struct stat st;
    if (stat(path, &st) == 0) {
        return st.st_size;
    }
    return 0;
}

// GET /api/patterns - paginated, streaming
static esp_err_t patterns_list_handler(httpd_req_t* req) {
    // Parse query params for pagination
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
    std::vector<Pattern> patterns = ManifestManager::getAllPatterns();
    int total = patterns.size();
    int start = page * page_size;
    int end = start + page_size;
    if (end > total) end = total;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Transfer-Encoding", "chunked");

    // Send '{"total":'
    httpd_resp_send_chunk(req, "{\"total\":", 9);

    // Send total as string
    char total_str[16];
    int total_len = snprintf(total_str, sizeof(total_str), "%d", total);
    httpd_resp_send_chunk(req, total_str, total_len);

    // Send ',"patterns":['
    httpd_resp_send_chunk(req, ",\"patterns\":[", 13);

    for (int i = start; i < end; ++i) {
        cJSON* json = ManifestManager::patternToJson(patterns[i]);
        size_t size = get_pattern_size_on_disk(patterns[i].uuid);
        cJSON_AddNumberToObject(json, "size", (double)size);
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

// GET /api/patterns/* - detail by uuid
static esp_err_t patterns_detail_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";
    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    Pattern* pattern = ManifestManager::getPattern(uuid);
    if (!pattern) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    cJSON* json = ManifestManager::patternToJson(*pattern);
    size_t size = get_pattern_size_on_disk(uuid);
    cJSON_AddNumberToObject(json, "size", (double)size);
    char* json_str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// DELETE /api/patterns/* - delete by uuid
static esp_err_t patterns_delete_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";
    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    if (!ManifestManager::patternExists(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    esp_err_t res = ManifestManager::deletePattern(uuid);
    if (res != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    // Optionally, delete the pattern file from disk
    char path[128];
    snprintf(path, sizeof(path), "/sd/patterns/%s.thr", uuid);
    unlink(path);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"deleted\"}", 21);
    return ESP_OK;
}

void patterns_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t patterns_list_uri = {
        .uri = "/api/patterns",
        .method = HTTP_GET,
        .handler = patterns_list_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &patterns_list_uri);

    static httpd_uri_t patterns_detail_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_GET,
        .handler = patterns_detail_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &patterns_detail_uri);

    static httpd_uri_t patterns_delete_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_DELETE,
        .handler = patterns_delete_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &patterns_delete_uri);
}
