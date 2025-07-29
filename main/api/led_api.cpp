#include "led_api.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "kd_pixdriver.h"
#include "pixel_effects.h"

#include <string.h>

// Handler to list available effects
static esp_err_t led_effects_list_handler(httpd_req_t* req) {
    cJSON* root = cJSON_CreateArray();
    PixelEffectEngine* effect_engine = PixelDriver::getEffectEngine();
    std::vector<PixelEffectEngine::EffectInfo> effects = effect_engine->getAllEffects();

    for (size_t i = 0; i < effects.size(); ++i) {
        const PixelEffectEngine::EffectInfo* eff = &effects[i];
        if (eff) {
            cJSON* obj = cJSON_CreateObject();
            cJSON_AddStringToObject(obj, "name", eff->display_name.c_str());
            cJSON_AddStringToObject(obj, "id", eff->id.c_str());
            cJSON_AddItemToArray(root, obj);
        }
    }
    char* json = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// Handler to get LED configuration
static esp_err_t led_config_get_handler(httpd_req_t* req) {
    cJSON* root = cJSON_CreateObject();
    cJSON* channels = cJSON_CreateArray();
    std::vector<int32_t> channel_ids = PixelDriver::getChannelIds();
    for (size_t i = 0; i < channel_ids.size(); ++i) {
        const PixelChannel* ch = PixelDriver::getChannel(channel_ids[i]);
        if (ch) {
            ChannelConfig config = ch->getConfig();
            cJSON* ch_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(ch_obj, "index", i);
            cJSON_AddNumberToObject(ch_obj, "num_leds", config.pixel_count);
            cJSON_AddStringToObject(ch_obj, "type", config.format == PixelFormat::RGB ? "RGB" : "RGBW");
            cJSON_AddItemToArray(channels, ch_obj);
        }
    }
    cJSON_AddItemToObject(root, "channels", channels);
    char* json = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// Handler to configure a channel (POST /api/led/channel)
static esp_err_t led_channel_config_handler(httpd_req_t* req) {
    char channel_str[8];
    int channel_idx = -1;
    // Extract channel index from URI wildcard
    if (httpd_req_get_url_query_str(req, channel_str, sizeof(channel_str)) == ESP_OK) {
        // Not used, fallback to parsing from URI
    }
    else {
        // Parse from URI path
        const char* uri = req->uri;
        const char* base = "/api/led/channel/";
        if (strncmp(uri, base, strlen(base)) == 0) {
            channel_idx = atoi(uri + strlen(base));
        }
    }
    if (channel_idx < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel index");
        return ESP_FAIL;
    }
    PixelChannel* ch = PixelDriver::getChannel(channel_idx);
    if (!ch) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }

    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    cJSON* json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON* color = cJSON_GetObjectItem(json, "color");
    cJSON* brightness = cJSON_GetObjectItem(json, "brightness");
    cJSON* speed = cJSON_GetObjectItem(json, "speed");
    cJSON* on = cJSON_GetObjectItem(json, "on");
    cJSON* effect_id = cJSON_GetObjectItem(json, "effect_id");

    // Set effect config
    EffectConfig eff_cfg = ch->getEffectConfig();
    eff_cfg.effect = effect_id->valuestring;
    if (brightness && cJSON_IsNumber(brightness)) eff_cfg.brightness = brightness->valueint;
    if (speed && cJSON_IsNumber(speed)) eff_cfg.speed = speed->valueint;
    if (on && cJSON_IsBool(on)) eff_cfg.enabled = cJSON_IsTrue(on);
    if (color && cJSON_IsObject(color)) {
        cJSON* r = cJSON_GetObjectItem(color, "r");
        cJSON* g = cJSON_GetObjectItem(color, "g");
        cJSON* b = cJSON_GetObjectItem(color, "b");
        cJSON* w = cJSON_GetObjectItem(color, "w");
        if (cJSON_IsNumber(r)) eff_cfg.color.r = r->valueint;
        if (cJSON_IsNumber(g)) eff_cfg.color.g = g->valueint;
        if (cJSON_IsNumber(b)) eff_cfg.color.b = b->valueint;
        if (w && cJSON_IsNumber(w)) eff_cfg.color.w = w->valueint;
    }

    ch->setEffect(eff_cfg);

    cJSON_Delete(json);
    cJSON* resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "ok");
    char* resp_str = cJSON_Print(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free(resp_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

// Handler to get a single channel configuration (GET /api/led/channel/*)
static esp_err_t led_channel_get_handler(httpd_req_t* req) {
    char channel_str[8];
    int channel_idx = -1;
    // Extract channel index from URI wildcard
    if (httpd_req_get_url_query_str(req, channel_str, sizeof(channel_str)) == ESP_OK) {
        // Not used, fallback to parsing from URI
    }
    else {
        // Parse from URI path
        const char* uri = req->uri;
        const char* base = "/api/led/channel/";
        if (strncmp(uri, base, strlen(base)) == 0) {
            channel_idx = atoi(uri + strlen(base));
        }
    }
    if (channel_idx < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel index");
        return ESP_FAIL;
    }
    const PixelChannel* ch = PixelDriver::getChannel(channel_idx);
    if (!ch) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }
    const ChannelConfig& cfg = ch->getConfig();
    const EffectConfig& eff = ch->getEffectConfig();
    cJSON* ch_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(ch_obj, "effect_id", eff.effect.c_str());
    cJSON_AddNumberToObject(ch_obj, "brightness", eff.brightness);
    cJSON_AddNumberToObject(ch_obj, "speed", eff.speed);
    cJSON_AddBoolToObject(ch_obj, "on", eff.enabled);
    cJSON* color_obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(color_obj, "r", eff.color.r);
    cJSON_AddNumberToObject(color_obj, "g", eff.color.g);
    cJSON_AddNumberToObject(color_obj, "b", eff.color.b);
    if (cfg.format == PixelFormat::RGBW) {
        cJSON_AddNumberToObject(color_obj, "w", eff.color.w);
    }
    cJSON_AddItemToObject(ch_obj, "color", color_obj);
    char* json = cJSON_Print(ch_obj);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    free(json);
    cJSON_Delete(ch_obj);
    return ESP_OK;
}

void led_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t effects_uri = {
        .uri = "/api/led/effects",
        .method = HTTP_GET,
        .handler = led_effects_list_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &effects_uri);

    static httpd_uri_t config_uri = {
        .uri = "/api/led/config",
        .method = HTTP_GET,
        .handler = led_config_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &config_uri);

    static httpd_uri_t channel_get_uri = {
        .uri = "/api/led/channel/*",
        .method = HTTP_GET,
        .handler = led_channel_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &channel_get_uri);

    static httpd_uri_t channel_post_uri = {
        .uri = "/api/led/channel/*",
        .method = HTTP_POST,
        .handler = led_channel_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &channel_post_uri);
}
