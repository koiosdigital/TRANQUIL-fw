# ManifestManager

The ManifestManager class provides a comprehensive solution for managing pattern and playlist metadata stored on the SD card in JSON format.

## Overview

ManifestManager stores a `manifest.json` file on the SD card that contains two arrays:

- `patterns`: Array of Pattern objects
- `playlists`: Array of Playlist objects

## Data Structures

### Pattern

```cpp
struct Pattern {
    std::string uuid;        // Unique identifier
    std::string name;        // Display name
    std::string date;        // ISO 8601 date string
    int popularity;          // Popularity score (0-100)
    std::string creator;     // Creator name
};
```

### Playlist

```cpp
struct Playlist {
    std::string uuid;                     // Unique identifier
    std::string name;                     // Display name
    std::string description;              // Description text
    std::vector<std::string> patterns;    // Array of pattern UUIDs
    std::string featured_pattern;         // UUID of featured pattern
    std::string date;                     // ISO 8601 date string
};
```

## JSON Schema

The manifest.json file follows this structure:

```json
{
  "patterns": [
    {
      "uuid": "pattern-001",
      "name": "Spiral Galaxy",
      "date": "2024-01-15T10:30:00Z",
      "popularity": 85,
      "creator": "Alice"
    }
  ],
  "playlists": [
    {
      "uuid": "playlist-001",
      "name": "Relaxing Patterns",
      "description": "A collection of calming patterns",
      "patterns": ["pattern-001", "pattern-002"],
      "featured_pattern": "pattern-001",
      "date": "2024-01-20T12:00:00Z"
    }
  ]
}
```

## Usage

### Basic Setup

```cpp
#include "sd/ManifestManager.h"

ManifestManager manager;
esp_err_t ret = manager.init();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ManifestManager");
    return;
}
```

### Pattern Operations

#### Create Pattern

```cpp
Pattern pattern("pattern-123", "My Pattern", "2024-01-15T10:30:00Z", 75, "John");
esp_err_t ret = manager.addPattern(pattern);
```

#### Read Pattern

```cpp
Pattern* pattern = manager.getPattern("pattern-123");
if (pattern) {
    ESP_LOGI(TAG, "Found: %s by %s", pattern->name.c_str(), pattern->creator.c_str());
}
```

#### Update Pattern

```cpp
Pattern* existing = manager.getPattern("pattern-123");
if (existing) {
    existing->popularity = 90;
    manager.updatePattern("pattern-123", *existing);
}
```

#### Delete Pattern

```cpp
esp_err_t ret = manager.deletePattern("pattern-123");
```

#### List All Patterns

```cpp
std::vector<Pattern> patterns = manager.getAllPatterns();
for (const auto& pattern : patterns) {
    ESP_LOGI(TAG, "Pattern: %s", pattern.name.c_str());
}
```

### Playlist Operations

#### Create Playlist

```cpp
std::vector<std::string> patternIds = {"pattern-001", "pattern-002"};
Playlist playlist("playlist-123", "My Playlist", "Description",
                 patternIds, "pattern-001", "2024-01-20T12:00:00Z");
esp_err_t ret = manager.addPlaylist(playlist);
```

#### Read Playlist

```cpp
Playlist* playlist = manager.getPlaylist("playlist-123");
if (playlist) {
    ESP_LOGI(TAG, "Playlist: %s with %zu patterns",
            playlist->name.c_str(), playlist->patterns.size());
}
```

#### Update Playlist

```cpp
Playlist* existing = manager.getPlaylist("playlist-123");
if (existing) {
    existing->description = "Updated description";
    manager.updatePlaylist("playlist-123", *existing);
}
```

#### Delete Playlist

```cpp
esp_err_t ret = manager.deletePlaylist("playlist-123");
```

### Playlist Management

#### Add Pattern to Playlist

```cpp
esp_err_t ret = manager.addPatternToPlaylist("playlist-123", "pattern-456");
```

#### Remove Pattern from Playlist

```cpp
esp_err_t ret = manager.removePatternFromPlaylist("playlist-123", "pattern-456");
```

#### Set Featured Pattern

```cpp
esp_err_t ret = manager.setFeaturedPattern("playlist-123", "pattern-001");
```

### Statistics

```cpp
size_t patternCount = manager.getPatternCount();
size_t playlistCount = manager.getPlaylistCount();
ESP_LOGI(TAG, "Total: %zu patterns, %zu playlists", patternCount, playlistCount);
```

## Features

- **Automatic JSON Management**: Handles loading, parsing, and saving JSON data
- **CRUD Operations**: Complete Create, Read, Update, Delete support for both patterns and playlists
- **Relationship Management**: Add/remove patterns from playlists, set featured patterns
- **UUID Generation**: Automatic generation of unique identifiers
- **Date Handling**: ISO 8601 date string generation
- **Error Handling**: Comprehensive ESP-IDF error code returns
- **Memory Management**: Proper cleanup of cJSON objects
- **Thread Safety**: Uses file-based persistence for data consistency

## Error Handling

All functions return `esp_err_t` values:

- `ESP_OK`: Success
- `ESP_ERR_NOT_FOUND`: Item not found
- `ESP_ERR_DUPLICATE_ELEMENT`: Item already exists
- `ESP_ERR_INVALID_STATE`: Manager not initialized
- `ESP_ERR_NO_MEM`: Memory allocation failed
- `ESP_ERR_INVALID_ARG`: Invalid JSON structure

## Dependencies

- **cJSON**: For JSON parsing and generation
- **ESP-IDF VFS/FAT**: For file system access
- **C++ STL**: For std::string and std::vector

## Example

See `ManifestExample.cpp` for a complete working example that demonstrates all functionality including pattern creation, playlist management, and CRUD operations.
