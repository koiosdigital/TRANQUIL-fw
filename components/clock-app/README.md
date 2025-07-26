# Clock App Component

This ESP-IDF component automatically downloads the latest web frontend assets from the `koiosdigital/clock-app` repository during build time.

## Features

- **Automatic Download**: Fetches the latest `static_files.h` from GitHub releases
- **Fallback Support**: Provides empty static files if download fails
- **Build Integration**: Seamlessly integrates with ESP-IDF build system
- **Version Tracking**: Uses latest release tags for stable versions

## How It Works

1. **Build Time Download**: During CMake configuration, the component:

   - Queries GitHub API for the latest release of `koiosdigital/clock-app`
   - Downloads the `static_files.h` file from that release
   - Makes it available as an include file for the main application

2. **Fallback Mechanism**: If the download fails (no internet, API limits, etc.):

   - Creates a fallback `static_files.h` with empty file arrays
   - Allows the build to continue without errors
   - Web interface will show basic API welcome message

3. **Include Integration**: The downloaded file is available as:
   ```cpp
   #include "static_files.h"
   ```

## Usage in Main Application

```cpp
#include "static_files.h"

// Access the static files
for (int i = 0; i < static_files::num_of_files; i++) {
    const auto& file = static_files::files[i];
    printf("File: %s, Size: %zu bytes\n", file.path, file.size);
}
```

## Configuration

Use `idf.py menuconfig` to configure the component:

- **Component Config → Clock App Component**
  - Enable/disable automatic downloads
  - Set cache timeout for downloaded files

## Build Requirements

- **curl**: Must be available in the build environment
- **Internet access**: Required during build for downloading files
- **GitHub API access**: Component queries GitHub's public API

## Troubleshooting

### Download Failures

- Check internet connectivity during build
- Verify curl is installed and accessible
- GitHub API rate limits may cause temporary failures

### Empty Web Interface

- If you see only "Welcome to the KD Clock API!" the download likely failed
- Check build logs for download error messages
- Manually verify the repository exists and has releases

### Build Errors

- Ensure curl is available in your build environment
- Check that the component is properly included in your project's dependencies

## Development

For development with local static files:

1. Disable auto-download in menuconfig
2. Manually place your `static_files.h` in the component's include directory
3. The build system will use your local file instead

## Repository Structure Expected

The component expects the target repository to have:

- GitHub releases with version tags
- A `static_files.h` file in the repository root
- Proper C++ namespace structure in the static files header
