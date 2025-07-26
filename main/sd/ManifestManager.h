#pragma once

#include <string>
#include <vector>
#include "esp_err.h"
#include "cJSON.h"

// Pattern structure matching the TypeScript interface
struct Pattern {
    std::string uuid;
    std::string name;
    std::string date;
    int popularity;
    std::string creator;

    Pattern() : popularity(0) {}
    Pattern(const std::string& id, const std::string& n, const std::string& d,
        int pop, const std::string& c)
        : uuid(id), name(n), date(d), popularity(pop), creator(c) {
    }
};

// Playlist structure matching the TypeScript interface
struct Playlist {
    std::string uuid;
    std::string name;
    std::string description;
    std::vector<std::string> patterns;
    std::string featured_pattern;
    std::string date;

    Playlist() {}
    Playlist(const std::string& id, const std::string& n, const std::string& desc,
        const std::vector<std::string>& pats, const std::string& featured,
        const std::string& d)
        : uuid(id), name(n), description(desc), patterns(pats), featured_pattern(featured), date(d) {
    }
};

class ManifestManager {
private:
    static const char* MANIFEST_PATH;
    static const char* TAG;

    cJSON* manifest_json;
    cJSON* patterns_array;
    cJSON* playlists_array;

    // Helper functions
    esp_err_t loadManifest();
    esp_err_t saveManifest();
    esp_err_t createEmptyManifest();
    cJSON* patternToJson(const Pattern& pattern);
    cJSON* playlistToJson(const Playlist& playlist);
    Pattern jsonToPattern(const cJSON* json);
    Playlist jsonToPlaylist(const cJSON* json);
    std::string generateUUID();
    std::string getCurrentDateString();

public:
    ManifestManager();
    ~ManifestManager();

    // Initialization
    esp_err_t init();

    // Pattern CRUD operations
    esp_err_t addPattern(const Pattern& pattern);
    esp_err_t updatePattern(const std::string& uuid, const Pattern& pattern);
    esp_err_t deletePattern(const std::string& uuid);
    std::vector<Pattern> getAllPatterns();
    Pattern* getPattern(const std::string& uuid);
    bool patternExists(const std::string& uuid);

    // Playlist CRUD operations
    esp_err_t addPlaylist(const Playlist& playlist);
    esp_err_t updatePlaylist(const std::string& uuid, const Playlist& playlist);
    esp_err_t deletePlaylist(const std::string& uuid);
    std::vector<Playlist> getAllPlaylists();
    Playlist* getPlaylist(const std::string& uuid);
    bool playlistExists(const std::string& uuid);

    // Utility functions
    esp_err_t addPatternToPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    esp_err_t removePatternFromPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    esp_err_t setFeaturedPattern(const std::string& playlistUuid, const std::string& patternUuid);

    // Statistics
    size_t getPatternCount();
    size_t getPlaylistCount();
};
