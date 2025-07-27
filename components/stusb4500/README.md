# STUSB4500 USB-C Power Delivery Controller

This component provides an ESP-IDF driver for the STUSB4500 USB-C Power Delivery controller. The STUSB4500 is a standalone USB-C PD controller that handles USB-C negotiation and power delivery without requiring host CPU intervention.

## Features

- **No Reset Operations**: The driver deliberately avoids resetting the chip to prevent power interruption (except when explicitly called)
- **Configurable I2C Interface**: Supports configurable SDA/SCL pins and I2C address
- **Status Monitoring**: Read current negotiation status, connection state, and power delivery information
- **Power Measurements**: Read voltage, current, and power measurements
- **PDO Configuration**: Read and write Power Delivery Objects (PDOs) to/from NVM
- **Reset Function**: Software reset capability (use with caution)
- **Non-blocking Operations**: All operations are designed to be fast and non-blocking
- **Debug Support**: Optional debug logging for detailed status information

## Configuration

Configure the STUSB4500 through menuconfig:

```
Component config → STUSB4500 USB-C PD Controller
```

### Key Configuration Options

- **I2C Port**: Which I2C peripheral to use (0 or 1)
- **SDA Pin**: GPIO pin for I2C data line
- **SCL Pin**: GPIO pin for I2C clock line
- **I2C Address**: Device address (default 0x28)

## Usage

### Basic Initialization

```c
#include "stusb4500.h"

void app_main(void)
{
    // Initialize the STUSB4500
    esp_err_t ret = stusb4500_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STUSB4500: %s", esp_err_to_name(ret));
        return;
    }

    // Read device ID to verify communication
    uint8_t device_id;
    ret = stusb4500_read_device_id(&device_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "STUSB4500 Device ID: 0x%02X", device_id);
    }
}
```

### Reading Negotiation Status

```c
void check_pd_status(void)
{
    stusb4500_negotiation_status_t status;
    esp_err_t ret = stusb4500_read_negotiation_status(&status);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Connected: %s", status.is_connected ? "Yes" : "No");
        ESP_LOGI(TAG, "PD Contract: %s", status.pd_negotiation_complete ? "Active" : "None");

        // Print detailed status
        stusb4500_print_negotiation_status(&status);
    }
}
```

### Reading Power Measurements

```c
void read_power_data(void)
{
    stusb4500_power_status_t power_status;
    esp_err_t ret = stusb4500_read_power_status(&power_status);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Voltage: %u mV", power_status.voltage_mv);
        ESP_LOGI(TAG, "Current: %u mA", power_status.current_ma);
        ESP_LOGI(TAG, "Power: %lu mW", power_status.power_mw);

        // Print formatted status
        stusb4500_print_power_status(&power_status);
    }

    // Or read individual measurements
    uint16_t voltage_mv;
    if (stusb4500_read_voltage_mv(&voltage_mv) == ESP_OK) {
        ESP_LOGI(TAG, "Voltage: %u mV", voltage_mv);
    }
}
```

### Configuring PDOs in NVM

```c
void configure_custom_pdos(void)
{
    stusb4500_pdo_t pdos[3];

    // Create custom PDOs
    stusb4500_create_fixed_pdo(&pdos[0], 5000, 3000);   // 5V @ 3A
    stusb4500_create_fixed_pdo(&pdos[1], 9000, 3000);   // 9V @ 3A
    stusb4500_create_fixed_pdo(&pdos[2], 15000, 3000);  // 15V @ 3A

    // Write to NVM (WARNING: This permanently modifies the device!)
    esp_err_t ret = stusb4500_write_all_pdos_to_nvm(pdos);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PDOs written successfully");

        // Reset to load new configuration
        stusb4500_reset();
    }
}
```

### Reading PDO Configuration

````c
void read_pdo_config(void)
{
    stusb4500_pdo_t pdos[3];
    esp_err_t ret = stusb4500_read_all_pdos_from_nvm(pdos);

    if (ret == ESP_OK) {
        stusb4500_print_all_pdos(pdos);

        // Access individual PDO data
        ESP_LOGI(TAG, "PDO 1: %.1fV @ %.1fA",
                 pdos[0].voltage_mv / 1000.0f,
                 pdos[0].max_current_ma / 1000.0f);
    }
}### Monitoring Example

The component includes several example functions:

```c
#include "stusb4500_example.h"

void app_main(void)
{
    // Start continuous monitoring
    stusb4500_example_start();

    // Or run individual examples
    stusb4500_example_read_power_measurements();
    stusb4500_example_configure_pdos(); // WARNING: Modifies NVM!
}
````

## API Reference

### Initialization Functions

- `stusb4500_init()` - Initialize the STUSB4500 driver
- `stusb4500_deinit()` - Deinitialize and cleanup resources

### Status Reading Functions

- `stusb4500_read_device_id()` - Read device identification
- `stusb4500_read_negotiation_status()` - Read complete negotiation status
- `stusb4500_read_cc_status()` - Read CC line status
- `stusb4500_read_pd_typec_status()` - Read PD/Type-C state machine status
- `stusb4500_read_prt_status()` - Read Protocol Layer status

### Power Measurement Functions

- `stusb4500_read_power_status()` - Read complete power status (voltage, current, power)
- `stusb4500_read_voltage_mv()` - Read VBUS voltage in millivolts
- `stusb4500_read_current_ma()` - Read estimated current in milliamps
- `stusb4500_read_power_mw()` - Read calculated power in milliwatts

### NVM PDO Configuration Functions

- `stusb4500_nvm_unlock()` - Unlock NVM for read/write access
- `stusb4500_nvm_lock()` - Lock NVM to prevent accidental writes
- `stusb4500_read_pdo_from_nvm()` - Read a single PDO from NVM
- `stusb4500_write_pdo_to_nvm()` - Write a single PDO to NVM
- `stusb4500_read_all_pdos_from_nvm()` - Read all three PDOs from NVM
- `stusb4500_write_all_pdos_to_nvm()` - Write all three PDOs to NVM

### Reset Function

- `stusb4500_reset()` - Perform software reset (WARNING: Interrupts power!)

### Utility Functions

- `stusb4500_clear_alert_status()` - Clear alert interrupt flags
- `stusb4500_print_negotiation_status()` - Print formatted status information
- `stusb4500_print_power_status()` - Print formatted power measurements
- `stusb4500_print_pdo()` - Print formatted PDO information
- `stusb4500_print_all_pdos()` - Print all PDO configurations
- `stusb4500_cc_state_to_string()` - Convert CC state enum to string
- `stusb4500_typec_fsm_state_to_string()` - Convert FSM state enum to string

### PDO Utility Functions

- `stusb4500_create_fixed_pdo()` - Create a fixed voltage PDO
- `stusb4500_pdo_to_raw()` - Convert PDO structure to raw 32-bit value
- `stusb4500_raw_to_pdo()` - Convert raw 32-bit value to PDO structure

## Status Information

The driver provides detailed information about:

### Connection Status

- USB-C connection state (connected/disconnected)
- CC1/CC2 line states and current capabilities
- Connection orientation detection

### Power Delivery Status

- PD contract negotiation status
- Power role (sink/source)
- Data role (sink/source)
- Current power delivery state

### Type-C State Machine

- Current FSM state (Attached.SNK, Unattached, etc.)
- Handshake completion status
- Protocol engine state

### Power Measurements

- VBUS voltage in millivolts (100mV resolution)
- Estimated current capability in milliamps
- Calculated power in milliwatts
- VSAFE0V detection status

### PDO Configuration

- Up to 3 Power Delivery Objects stored in NVM
- Voltage and current specifications for each PDO
- USB-C capability flags (dual role, communications, etc.)
- Support for reading and writing custom PDO configurations

## Important Notes

1. **No Reset Operations**: This driver intentionally does not perform any reset operations on the STUSB4500 chip during normal operation, as this could interrupt power delivery to the system. The reset function is available but should be used with caution.

2. **I2C Configuration**: Ensure your I2C pins have appropriate pull-up resistors (typically 4.7kΩ).

3. **Power Supply**: The STUSB4500 should be powered before initializing this driver.

4. **Thread Safety**: The driver is not inherently thread-safe. Use appropriate synchronization if accessing from multiple tasks.

5. **NVM Operations**: Writing to NVM permanently modifies the device configuration. Always read and backup existing configuration before making changes.

6. **Power Measurements**: Voltage readings are hardware-measured, but current readings are estimated based on PDO capabilities. For accurate current measurement, use external hardware.

7. **Device Variants**: The driver supports both evaluation (0x21) and production (0x25) device IDs.

## Troubleshooting

### Device Not Found

- Check I2C wiring (SDA/SCL pins, pull-ups)
- Verify I2C address configuration
- Ensure STUSB4500 is powered

### Communication Errors

- Check I2C bus speed (try lower frequencies)
- Verify pin assignments match your hardware
- Check for I2C bus conflicts with other devices

### Status Reading Issues

- Allow time for USB-C negotiation to complete
- Check if USB-C cable is properly connected
- Verify power source supports PD negotiation

### NVM Programming Issues

- Ensure device is not in a power delivery contract during NVM operations
- Verify unlock sequence is successful before attempting writes
- Allow adequate delay between erase and write operations
- Always lock NVM after completing operations

### Power Measurement Issues

- Voltage readings require stable VBUS connection
- Current readings are estimated, not measured
- Power calculations are based on estimated current values
