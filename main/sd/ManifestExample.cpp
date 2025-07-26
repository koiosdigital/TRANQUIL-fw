#include "ManifestExample.h"
#include "esp_log.h"

static const char* TAG = "ManifestExample";

esp_err_t ManifestExample::runExample() {
    ESP_LOGI(TAG, "Starting ManifestManager example");

    ManifestManager manager;

    // Initialize the manager
    esp_err_t ret = manager.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ManifestManager");
        return ret;
    }

    // Create example data
    ret = createExamplePatterns(manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create example patterns");
        return ret;
    }

    ret = createExamplePlaylists(manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create example playlists");
        return ret;
    }

    // Demonstrate CRUD operations
    ret = demonstratePatternCRUD(manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Pattern CRUD demonstration failed");
        return ret;
    }

    ret = demonstratePlaylistCRUD(manager);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Playlist CRUD demonstration failed");
        return ret;
    }

    // Print final state
    printAllData(manager);

    ESP_LOGI(TAG, "ManifestManager example completed successfully");
    return ESP_OK;
}

esp_err_t ManifestExample::createExamplePatterns(ManifestManager& manager) {
    ESP_LOGI(TAG, "Creating example patterns");

    // Create some example patterns
    Pattern pattern1("pattern-001", "Spiral Galaxy", "2024-01-15T10:30:00Z", 85, "Alice");
    Pattern pattern2("pattern-002", "Mandala Bloom", "2024-01-16T14:20:00Z", 92, "Bob");
    Pattern pattern3("pattern-003", "Zen Garden", "2024-01-17T09:15:00Z", 78, "Charlie");
    Pattern pattern4("pattern-004", "Fibonacci Spiral", "2024-01-18T16:45:00Z", 96, "Diana");

    esp_err_t ret = manager.addPattern(pattern1);
    if (ret != ESP_OK) return ret;

    ret = manager.addPattern(pattern2);
    if (ret != ESP_OK) return ret;

    ret = manager.addPattern(pattern3);
    if (ret != ESP_OK) return ret;

    ret = manager.addPattern(pattern4);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Created %zu patterns", manager.getPatternCount());
    return ESP_OK;
}

esp_err_t ManifestExample::createExamplePlaylists(ManifestManager& manager) {
    ESP_LOGI(TAG, "Creating example playlists");

    // Create playlists
    std::vector<std::string> relaxingPatterns = { "pattern-001", "pattern-003" };
    Playlist playlist1("playlist-001", "Relaxing Patterns",
        "A collection of calming and meditative patterns",
        relaxingPatterns, "pattern-003", "2024-01-20T12:00:00Z");

    std::vector<std::string> popularPatterns = { "pattern-002", "pattern-004" };
    Playlist playlist2("playlist-002", "Most Popular",
        "The highest rated patterns from our community",
        popularPatterns, "pattern-004", "2024-01-21T15:30:00Z");

    esp_err_t ret = manager.addPlaylist(playlist1);
    if (ret != ESP_OK) return ret;

    ret = manager.addPlaylist(playlist2);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Created %zu playlists", manager.getPlaylistCount());
    return ESP_OK;
}

esp_err_t ManifestExample::demonstratePatternCRUD(ManifestManager& manager) {
    ESP_LOGI(TAG, "Demonstrating Pattern CRUD operations");

    // Read - Get a specific pattern
    Pattern* pattern = manager.getPattern("pattern-001");
    if (pattern) {
        ESP_LOGI(TAG, "Found pattern: %s by %s (popularity: %d)",
            pattern->name.c_str(), pattern->creator.c_str(), pattern->popularity);
    }

    // Update - Modify popularity
    if (pattern) {
        Pattern updated = *pattern;
        updated.popularity = 90;
        esp_err_t ret = manager.updatePattern("pattern-001", updated);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update pattern");
            return ret;
        }
        ESP_LOGI(TAG, "Updated pattern popularity to %d", updated.popularity);
    }

    // Create a new pattern
    Pattern newPattern("pattern-005", "Ocean Waves", "2024-01-22T11:00:00Z", 88, "Eve");
    esp_err_t ret = manager.addPattern(newPattern);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add new pattern");
        return ret;
    }
    ESP_LOGI(TAG, "Added new pattern: %s", newPattern.name.c_str());

    // List all patterns
    std::vector<Pattern> allPatterns = manager.getAllPatterns();
    ESP_LOGI(TAG, "Total patterns: %zu", allPatterns.size());

    return ESP_OK;
}

esp_err_t ManifestExample::demonstratePlaylistCRUD(ManifestManager& manager) {
    ESP_LOGI(TAG, "Demonstrating Playlist CRUD operations");

    // Read - Get a specific playlist
    Playlist* playlist = manager.getPlaylist("playlist-001");
    if (playlist) {
        ESP_LOGI(TAG, "Found playlist: %s with %zu patterns",
            playlist->name.c_str(), playlist->patterns.size());
    }

    // Add pattern to playlist
    esp_err_t ret = manager.addPatternToPlaylist("playlist-001", "pattern-005");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to playlist");
        return ret;
    }
    ESP_LOGI(TAG, "Added pattern-005 to playlist-001");

    // Set featured pattern
    ret = manager.setFeaturedPattern("playlist-001", "pattern-005");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set featured pattern");
        return ret;
    }
    ESP_LOGI(TAG, "Set pattern-005 as featured in playlist-001");

    // Create a new playlist
    std::vector<std::string> newPatterns = { "pattern-002", "pattern-005" };
    Playlist newPlaylist("playlist-003", "Dynamic Patterns",
        "Patterns with movement and energy",
        newPatterns, "pattern-002", "2024-01-23T13:45:00Z");

    ret = manager.addPlaylist(newPlaylist);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add new playlist");
        return ret;
    }
    ESP_LOGI(TAG, "Added new playlist: %s", newPlaylist.name.c_str());

    // List all playlists
    std::vector<Playlist> allPlaylists = manager.getAllPlaylists();
    ESP_LOGI(TAG, "Total playlists: %zu", allPlaylists.size());

    return ESP_OK;
}

void ManifestExample::printAllData(ManifestManager& manager) {
    ESP_LOGI(TAG, "=== MANIFEST SUMMARY ===");

    // Print all patterns
    std::vector<Pattern> patterns = manager.getAllPatterns();
    ESP_LOGI(TAG, "Patterns (%zu):", patterns.size());
    for (const auto& pattern : patterns) {
        ESP_LOGI(TAG, "  - %s: '%s' by %s (popularity: %d)",
            pattern.uuid.c_str(), pattern.name.c_str(),
            pattern.creator.c_str(), pattern.popularity);
    }

    // Print all playlists
    std::vector<Playlist> playlists = manager.getAllPlaylists();
    ESP_LOGI(TAG, "Playlists (%zu):", playlists.size());
    for (const auto& playlist : playlists) {
        ESP_LOGI(TAG, "  - %s: '%s' (%zu patterns, featured: %s)",
            playlist.uuid.c_str(), playlist.name.c_str(),
            playlist.patterns.size(),
            playlist.featured_pattern.empty() ? "none" : playlist.featured_pattern.c_str());
        ESP_LOGI(TAG, "    Description: %s", playlist.description.c_str());
        for (const auto& patternUuid : playlist.patterns) {
            ESP_LOGI(TAG, "    Contains: %s", patternUuid.c_str());
        }
    }

    ESP_LOGI(TAG, "=== END SUMMARY ===");
}
