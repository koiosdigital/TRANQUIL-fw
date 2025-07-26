#pragma once

#include "ManifestManager.h"

/**
 * Example usage and test functions for ManifestManager
 * These functions demonstrate how to use the ManifestManager class
 * for managing patterns and playlists stored on SD card.
 */

class ManifestExample {
public:
    /**
     * Initialize and test the ManifestManager
     * @return ESP_OK on success
     */
    static esp_err_t runExample();

    /**
     * Create some example patterns
     * @param manager Reference to ManifestManager instance
     * @return ESP_OK on success
     */
    static esp_err_t createExamplePatterns(ManifestManager& manager);

    /**
     * Create example playlists with patterns
     * @param manager Reference to ManifestManager instance
     * @return ESP_OK on success
     */
    static esp_err_t createExamplePlaylists(ManifestManager& manager);

    /**
     * Demonstrate pattern CRUD operations
     * @param manager Reference to ManifestManager instance
     * @return ESP_OK on success
     */
    static esp_err_t demonstratePatternCRUD(ManifestManager& manager);

    /**
     * Demonstrate playlist CRUD operations
     * @param manager Reference to ManifestManager instance
     * @return ESP_OK on success
     */
    static esp_err_t demonstratePlaylistCRUD(ManifestManager& manager);

    /**
     * Print all patterns and playlists for debugging
     * @param manager Reference to ManifestManager instance
     */
    static void printAllData(ManifestManager& manager);
};
