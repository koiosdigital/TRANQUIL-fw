#include "stusb4500.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <cmath>

static const char* TAG = "STUSB4500";

// Default NVM values from SparkFun library (working configuration)
const uint8_t STUSB4500::default_nvm_sectors[5][8] = {
    {0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00},
    {0x10, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF},
    {0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1},
    {0x00, 0x19, 0x56, 0xAF, 0xF5, 0x35, 0x5F, 0x00},
    {0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB}
};

STUSB4500::STUSB4500()
    : i2c_bus_handle(nullptr)
    , device_handle(nullptr)
    , device_address(0x28)
    , is_initialized(false)
    , nvm_sectors_read(false)
{
    memset(nvm_sectors, 0, sizeof(nvm_sectors));
}

STUSB4500::~STUSB4500()
{
    end();
}

esp_err_t STUSB4500::begin(uint8_t device_address)
{
    if (is_initialized) {
        ESP_LOGW(TAG, "STUSB4500 already initialized");
        return ESP_OK;
    }

#if !CONFIG_STUSB4500_ENABLE
    ESP_LOGE(TAG, "STUSB4500 is disabled in configuration");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    this->device_address = device_address;
    esp_err_t ret = ESP_OK;

    // Configure I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_STUSB4500_I2C_PORT,
        .sda_io_num = (gpio_num_t)CONFIG_STUSB4500_SDA_PIN,
        .scl_io_num = (gpio_num_t)CONFIG_STUSB4500_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
    };

    // Configure GPIO pins with pull-ups (important for I2C)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_STUSB4500_SDA_PIN) | (1ULL << CONFIG_STUSB4500_SCL_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure STUSB4500 device
    i2c_device_config_t device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = this->device_address,
        .scl_speed_hz = CONFIG_STUSB4500_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(i2c_bus_handle, &device_cfg, &device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add STUSB4500 device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus configured: port=%d, SDA=%d, SCL=%d, freq=%dHz, addr=0x%02X",
        CONFIG_STUSB4500_I2C_PORT, CONFIG_STUSB4500_SDA_PIN, CONFIG_STUSB4500_SCL_PIN,
        CONFIG_STUSB4500_I2C_FREQ_HZ, this->device_address);

    // Verify device ID with multiple attempts and different approaches
    uint8_t device_id = 0;
    ret = ESP_ERR_INVALID_RESPONSE;

    // Try reading device ID multiple times with different methods
    for (int attempt = 0; attempt < 3; attempt++) {
        ESP_LOGI(TAG, "Attempting to read device ID (attempt %d/3)", attempt + 1);

        // Try standard register read
        ret = readDeviceId(&device_id);
        if (ret == ESP_OK && device_id != 0) {
            ESP_LOGI(TAG, "Device ID read successful: 0x%02X", device_id);
            break;
        }

        // Try with a delay
        vTaskDelay(pdMS_TO_TICKS(10));

        // Try reading a simple register first (like alert status)
        uint8_t alert_status;
        esp_err_t alert_ret = i2cReadRegister(STUSB4500_REG_ALERT_STATUS_1, &alert_status);
        ESP_LOGD(TAG, "Alert status read: ret=%s, value=0x%02X", esp_err_to_name(alert_ret), alert_status);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (ret != ESP_OK || device_id == 0) {
        ESP_LOGE(TAG, "Failed to read device ID after 3 attempts: %s (read value: 0x%02X)",
            esp_err_to_name(ret), device_id);
        end();
        return ret != ESP_OK ? ret : ESP_ERR_NOT_FOUND;
    }

    if (device_id != STUSB4500_EVAL_DEVICE_ID && device_id != STUSB4500_PROD_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X or 0x%02X)",
            device_id, STUSB4500_EVAL_DEVICE_ID, STUSB4500_PROD_DEVICE_ID);
        end();
        return ESP_ERR_NOT_FOUND;
    }

    // Clear any pending alert status
    ret = clearAlertStatus();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear alert status: %s", esp_err_to_name(ret));
        // Continue anyway, this is not critical
    }

    is_initialized = true;
    ESP_LOGI(TAG, "STUSB4500 initialized successfully (Device ID: 0x%02X)", device_id);

    // Read NVM on first initialization
    if (!nvm_sectors_read) {
        ret = readNVM();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read NVM on initialization: %s", esp_err_to_name(ret));
        }
    }

    return ESP_OK;
}

esp_err_t STUSB4500::end()
{
    if (!is_initialized) {
        return ESP_OK;
    }

    if (device_handle != nullptr) {
        i2c_master_bus_rm_device(device_handle);
        device_handle = nullptr;
    }

    if (i2c_bus_handle != nullptr) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = nullptr;
    }

    is_initialized = false;
    nvm_sectors_read = false;
    ESP_LOGI(TAG, "STUSB4500 deinitialized");
    return ESP_OK;
}

esp_err_t STUSB4500::readDeviceId(uint8_t* device_id)
{
    if (device_id == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2cReadRegister(STUSB4500_REG_DEVICE_ID, device_id);
}

esp_err_t STUSB4500::readNVM()
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = nvmEnterReadMode();
    if (ret != ESP_OK) {
        return ret;
    }

    // Read all 5 sectors (same as SparkFun library)
    for (uint8_t i = 0; i < 5; i++) {
        ret = nvmReadSector(i, nvm_sectors[i]);
        if (ret != ESP_OK) {
            nvmExitTestMode();
            return ret;
        }
    }

    ret = nvmExitTestMode();
    if (ret != ESP_OK) {
        return ret;
    }

    nvm_sectors_read = true;

    // Load PDO settings from NVM to volatile registers (SparkFun approach)
    loadPdoSettingsFromNVM();

    ESP_LOGI(TAG, "Successfully read NVM configuration");
    return ESP_OK;
}

esp_err_t STUSB4500::writeNVM(bool use_defaults)
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (use_defaults) {
        // Copy default values to our buffer
        memcpy(nvm_sectors, default_nvm_sectors, sizeof(nvm_sectors));
    }
    else if (!nvm_sectors_read) {
        // Must read NVM first if we don't have current values
        ret = readNVM();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    // Save current PDO settings to NVM buffer
    if (!use_defaults) {
        savePdoSettingsToNVM();
    }

    // Enter write mode for all sectors
    ret = nvmEnterWriteMode(STUSB4500_NVM_SECTOR_0 | STUSB4500_NVM_SECTOR_1 |
        STUSB4500_NVM_SECTOR_2 | STUSB4500_NVM_SECTOR_3 |
        STUSB4500_NVM_SECTOR_4);
    if (ret != ESP_OK) {
        return ret;
    }

    // Write all 5 sectors
    for (uint8_t i = 0; i < 5; i++) {
        ret = nvmWriteSector(i, nvm_sectors[i]);
        if (ret != ESP_OK) {
            nvmExitTestMode();
            return ret;
        }
    }

    ret = nvmExitTestMode();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Successfully wrote NVM configuration");
    return ESP_OK;
}

// PDO Configuration Functions
esp_err_t STUSB4500::getPdoNumber(uint8_t* pdo_count)
{
    if (!is_initialized || pdo_count == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2cReadRegister(STUSB4500_REG_DPM_PDO_NUMB, pdo_count);
    if (ret == ESP_OK) {
        *pdo_count = *pdo_count & 0x07; // Mask to get only the PDO count bits
    }
    return ret;
}

esp_err_t STUSB4500::setPdoNumber(uint8_t pdo_count)
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pdo_count > 3) {
        pdo_count = 3;
    }

    return i2cWriteRegister(STUSB4500_REG_DPM_PDO_NUMB, pdo_count);
}

float STUSB4500::getVoltage(uint8_t pdo_num)
{
    if (!is_initialized || pdo_num < 1 || pdo_num > 3) {
        return 0.0f;
    }

    uint32_t pdo_data = readPdoFromRegisters(pdo_num);
    uint32_t voltage_raw = (pdo_data >> 10) & 0x3FF;
    return voltage_raw / 20.0f; // 50mV resolution
}

esp_err_t STUSB4500::setVoltage(uint8_t pdo_num, float voltage)
{
    if (!is_initialized || pdo_num < 1 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    // Constrain voltage to 5-20V
    if (voltage < 5.0f) voltage = 5.0f;
    else if (voltage > 20.0f) voltage = 20.0f;

    // PDO1 is fixed at 5V
    if (pdo_num == 1) voltage = 5.0f;

    uint32_t voltage_raw = (uint32_t)(voltage * 20); // Convert to 50mV units
    uint32_t pdo_data = readPdoFromRegisters(pdo_num);

    // Clear voltage bits (10:19) and set new voltage
    pdo_data &= ~(0x3FF << 10);
    pdo_data |= (voltage_raw << 10);

    return writePdoToRegisters(pdo_num, pdo_data);
}

float STUSB4500::getCurrent(uint8_t pdo_num)
{
    if (!is_initialized || pdo_num < 1 || pdo_num > 3) {
        return 0.0f;
    }

    uint32_t pdo_data = readPdoFromRegisters(pdo_num);
    uint32_t current_raw = pdo_data & 0x3FF;
    return current_raw * 0.01f; // 10mA resolution
}

esp_err_t STUSB4500::setCurrent(uint8_t pdo_num, float current)
{
    if (!is_initialized || pdo_num < 1 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current > 5.0f) current = 5.0f; // Max 5A

    uint32_t current_raw = (uint32_t)(current / 0.01f); // Convert to 10mA units
    current_raw &= 0x3FF; // Limit to 10 bits

    uint32_t pdo_data = readPdoFromRegisters(pdo_num);

    // Clear current bits (0:9) and set new current
    pdo_data &= ~0x3FF;
    pdo_data |= current_raw;

    return writePdoToRegisters(pdo_num, pdo_data);
}

// Status and Monitoring Functions
esp_err_t STUSB4500::readNegotiationStatus(stusb4500_negotiation_status_t* status)
{
    if (!is_initialized || status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    memset(status, 0, sizeof(*status));

    // Read device ID
    ret = readDeviceId(&status->device_id);
    if (ret != ESP_OK) return ret;

    // Read CC status
    ret = readCcStatus(&status->cc_status);
    if (ret != ESP_OK) return ret;

    // Read PD/Type-C status
    ret = readPdTypecStatus(&status->pd_typec_status);
    if (ret != ESP_OK) return ret;

    // Read PRT status
    ret = readPrtStatus(&status->prt_status);
    if (ret != ESP_OK) return ret;

    // Read PE FSM state
    ret = i2cReadRegister(STUSB4500_REG_PE_FSM, &status->pe_fsm_state);
    if (ret != ESP_OK) return ret;

    // Determine connection status
    status->is_connected = status->cc_status.connection_result &&
        (status->cc_status.cc1_connected || status->cc_status.cc2_connected);

    // Determine if PD negotiation is complete
    status->pd_negotiation_complete = status->prt_status.pd_contract_active;

    return ESP_OK;
}

esp_err_t STUSB4500::readCcStatus(stusb4500_cc_status_t* status)
{
    if (!is_initialized || status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cc_status_reg;
    esp_err_t ret = i2cReadRegister(STUSB4500_REG_CC_STATUS, &cc_status_reg);
    if (ret != ESP_OK) return ret;

    memset(status, 0, sizeof(*status));

    // Parse CC1 and CC2 states
    status->cc1_state = (stusb4500_cc_state_t)(cc_status_reg & STUSB4500_CC_STATUS_CC1_STATE_MASK);
    status->cc2_state = (stusb4500_cc_state_t)((cc_status_reg & STUSB4500_CC_STATUS_CC2_STATE_MASK) >> STUSB4500_CC_STATUS_CC2_STATE_SHIFT);

    // Determine if CC lines are connected
    status->cc1_connected = (status->cc1_state != STUSB4500_CC_STATE_NOT_IN_UFP);
    status->cc2_connected = (status->cc2_state != STUSB4500_CC_STATE_NOT_IN_UFP);

    // Parse other status bits
    status->connection_result = (cc_status_reg & STUSB4500_CC_STATUS_CONNECT_RESULT) != 0;
    status->looking_for_connection = (cc_status_reg & STUSB4500_CC_STATUS_LOOKING4CONNECTION) != 0;

    return ESP_OK;
}

esp_err_t STUSB4500::readPdTypecStatus(stusb4500_pd_typec_status_t* status)
{
    if (!is_initialized || status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pd_typec_status_reg;
    esp_err_t ret = i2cReadRegister(STUSB4500_REG_PD_TYPEC_STATUS, &pd_typec_status_reg);
    if (ret != ESP_OK) return ret;

    memset(status, 0, sizeof(*status));

    status->pd_typec_handshake_check = (pd_typec_status_reg & STUSB4500_PD_TYPEC_STATUS_PD_TYPEC_HAND_CHECK) != 0;
    status->fsm_state = (stusb4500_typec_fsm_state_t)(pd_typec_status_reg & STUSB4500_PD_TYPEC_STATUS_TYPEC_FSM_STATE_MASK);

    return ESP_OK;
}

esp_err_t STUSB4500::readPrtStatus(stusb4500_prt_status_t* status)
{
    if (!is_initialized || status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t prt_status_reg;
    esp_err_t ret = i2cReadRegister(STUSB4500_REG_PRT_STATUS, &prt_status_reg);
    if (ret != ESP_OK) return ret;

    memset(status, 0, sizeof(*status));

    status->hw_reset_received = (prt_status_reg & STUSB4500_PRT_STATUS_HWRESET_RECEIVED) != 0;
    status->soft_reset_received = (prt_status_reg & STUSB4500_PRT_STATUS_SOFTRESET_RECEIVED) != 0;
    status->data_role_sink = (prt_status_reg & STUSB4500_PRT_STATUS_DATAROLE) != 0;
    status->power_role_sink = (prt_status_reg & STUSB4500_PRT_STATUS_POWERROLE) != 0;
    status->pd_contract_active = (prt_status_reg & STUSB4500_PRT_STATUS_PD_CONTRACT) != 0;
    status->startup_power = (prt_status_reg & STUSB4500_PRT_STATUS_STARTUP_POWER) != 0;
    status->message_received = (prt_status_reg & STUSB4500_PRT_STATUS_MSG_RECEIVED) != 0;
    status->message_sent = (prt_status_reg & STUSB4500_PRT_STATUS_MSG_SENT) != 0;

    return ESP_OK;
}

esp_err_t STUSB4500::clearAlertStatus()
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read alert status to clear it
    uint8_t alert_status;
    return i2cReadRegister(STUSB4500_REG_ALERT_STATUS_1, &alert_status);
}

// Power Measurement Functions
esp_err_t STUSB4500::readPowerStatus(stusb4500_power_status_t* status)
{
    if (!is_initialized || status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    memset(status, 0, sizeof(*status));

    // Read actual measured voltage
    ret = readVoltage(&status->voltage_mv);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read voltage: %s", esp_err_to_name(ret));
        status->voltage_mv = 0;
    }

    // Read actual negotiated current
    ret = readCurrent(&status->current_ma);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read current: %s", esp_err_to_name(ret));
        status->current_ma = 0;
    }

    // Calculate power from actual measurements
    if (status->voltage_mv > 0 && status->current_ma > 0) {
        status->power_mw = (uint32_t)status->voltage_mv * status->current_ma / 1000;
    }
    else {
        status->power_mw = 0;
    }

    // Check USB connection status
    stusb4500_cc_status_t cc_status;
    ret = readCcStatus(&cc_status);

    // Check PD contract status - use both register flag and voltage detection
    stusb4500_prt_status_t prt_status;
    bool prt_flag = false;
    ret = readPrtStatus(&prt_status);
    if (ret == ESP_OK) {
        prt_flag = prt_status.pd_contract_active;
    }

    // Also consider high voltage as indication of PD contract
    bool high_voltage = status->voltage_mv > 5500; // More than USB 5V + tolerance

    if (high_voltage && !prt_flag) {
        ESP_LOGW(TAG, "PD contract flag not set but high voltage detected (%d mV) - assuming PD active",
            status->voltage_mv);
    }

    // Read VSAFE0V status
    uint8_t vsafe0v_reg;
    ret = i2cReadRegister(STUSB4500_REG_VBUS_VSAFE0V, &vsafe0v_reg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read VSAFE0V status: %s", esp_err_to_name(ret));
        status->vsafe0v = false;
    }
    else {
        status->vsafe0v = (vsafe0v_reg & STUSB4500_VBUS_VSAFE0V_THRESHOLD) != 0;
    }

    return ESP_OK;
}

esp_err_t STUSB4500::readVoltage(uint16_t* voltage_mv)
{
    if (!is_initialized || voltage_mv == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t voltage_regs[2];
    esp_err_t ret = i2cRead(STUSB4500_REG_VBUS_VOLTAGE_LOW, voltage_regs, 2);
    if (ret != ESP_OK) return ret;

    // Combine low and high bytes 
    uint16_t voltage_raw = voltage_regs[0] | (voltage_regs[1] << 8);

    // Convert to millivolts using the STUSB4500 scaling factor
    // VBUS voltage scale is 25mV per LSB (from STUSB4500 datasheet)
    *voltage_mv = voltage_raw * STUSB4500_VBUS_LSB_MV;

    ESP_LOGD(TAG, "Raw voltage: 0x%04X (%d), Converted: %d mV", voltage_raw, voltage_raw, *voltage_mv);

    return ESP_OK;
}

esp_err_t STUSB4500::readCurrent(uint16_t* current_ma)
{
    if (!is_initialized || current_ma == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // First, try to determine current based on measured voltage
    // If we're seeing high voltage (>5.5V), we likely have a PD contract even if the flag is wrong
    uint16_t voltage_mv = 0;
    bool high_voltage_detected = false;

    if (readVoltage(&voltage_mv) == ESP_OK && voltage_mv > 5500) {
        high_voltage_detected = true;
        ESP_LOGI(TAG, "High voltage detected (%d mV), assuming PD contract exists", voltage_mv);
    }

    // Check if PD contract is active according to registers
    stusb4500_prt_status_t prt_status;
    esp_err_t ret = readPrtStatus(&prt_status);
    bool pd_contract_active = (ret == ESP_OK && prt_status.pd_contract_active) || high_voltage_detected;

    if (!pd_contract_active) {
        // No PD contract and no high voltage, assume standard USB current
        *current_ma = 500; // Standard USB 2.0 current
        ESP_LOGD(TAG, "No PD contract active and no high voltage, using default USB current: %d mA", *current_ma);
        return ESP_OK;
    }

    // Try to read which PDO is currently negotiated
    uint8_t active_pdo_reg;
    ret = i2cReadRegister(STUSB4500_REG_PE_PDO, &active_pdo_reg);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to read active PDO register, trying alternative method");

        // Alternative: Based on voltage, guess which PDO is active
        if (high_voltage_detected) {
            if (voltage_mv >= 19000) {
                // Likely 20V PDO (PDO3)
                uint32_t pdo_data = readPdoFromRegisters(3);
                if (pdo_data != 0) {
                    uint32_t current_raw = pdo_data & 0x3FF;
                    *current_ma = current_raw * 10;
                    ESP_LOGI(TAG, "Assuming 20V PDO3 based on voltage, current: %d mA", *current_ma);
                    return ESP_OK;
                }
            }
            else if (voltage_mv >= 14000) {
                // Likely 15V PDO (PDO2)
                uint32_t pdo_data = readPdoFromRegisters(2);
                if (pdo_data != 0) {
                    uint32_t current_raw = pdo_data & 0x3FF;
                    *current_ma = current_raw * 10;
                    ESP_LOGI(TAG, "Assuming 15V PDO2 based on voltage, current: %d mA", *current_ma);
                    return ESP_OK;
                }
            }
        }

        // Alternative: read the PDO count and assume highest PDO is active
        uint8_t pdo_count;
        ret = getPdoNumber(&pdo_count);
        if (ret == ESP_OK && pdo_count > 0) {
            uint32_t pdo_data = readPdoFromRegisters(pdo_count);
            if (pdo_data != 0) {
                uint32_t current_raw = pdo_data & 0x3FF;
                *current_ma = current_raw * 10;
                ESP_LOGD(TAG, "Using PDO %d current: %d mA", pdo_count, *current_ma);
                return ESP_OK;
            }
        }

        // Fallback to reasonable estimate
        ESP_LOGW(TAG, "Could not determine negotiated current, using 3A estimate");
        *current_ma = 3000;
        return ESP_OK;
    }

    // The active PDO number is in the lower 3 bits
    uint8_t active_pdo_num = active_pdo_reg & 0x07;
    if (active_pdo_num == 0 || active_pdo_num > 3) {
        ESP_LOGW(TAG, "Invalid active PDO number: %d, using estimate", active_pdo_num);
        *current_ma = 3000;
        return ESP_OK;
    }

    // Read the current from the active PDO registers
    uint32_t pdo_data = readPdoFromRegisters(active_pdo_num);
    if (pdo_data == 0) {
        ESP_LOGW(TAG, "Failed to read PDO data for PDO %d", active_pdo_num);
        *current_ma = 3000;
        return ESP_OK;
    }

    // Extract current from PDO (bits 0-9, in 10mA units)
    uint32_t current_raw = pdo_data & 0x3FF;
    *current_ma = current_raw * 10; // Convert from 10mA units to mA

    ESP_LOGD(TAG, "Active PDO: %d, Raw current: %lu, Current: %d mA",
        active_pdo_num, current_raw, *current_ma);

    return ESP_OK;
}esp_err_t STUSB4500::readPower(uint32_t* power_mw)
{
    if (!is_initialized || power_mw == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t voltage_mv, current_ma;
    esp_err_t ret;

    // Read actual measured voltage
    ret = readVoltage(&voltage_mv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read voltage for power calculation");
        return ret;
    }

    // Read actual negotiated current
    ret = readCurrent(&current_ma);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current for power calculation");
        return ret;
    }

    // Calculate power in milliwatts
    *power_mw = (uint32_t)voltage_mv * current_ma / 1000;

    ESP_LOGD(TAG, "Power calculation: %d mV × %d mA = %lu mW",
        voltage_mv, current_ma, *power_mw);

    return ESP_OK;
}

// Active PDO Functions
esp_err_t STUSB4500::readActivePdo(uint8_t pdo_num, stusb4500_pdo_t* pdo)
{
    if (!is_initialized || pdo == nullptr || pdo_num < 1 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t raw_pdo = readPdoFromRegisters(pdo_num);
    return rawToPdoStruct(raw_pdo, pdo);
}

esp_err_t STUSB4500::readAllActivePdos(stusb4500_pdo_t pdos[3])
{
    if (pdos == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    for (int i = 0; i < 3; i++) {
        ret = readActivePdo(i + 1, &pdos[i]);
        if (ret != ESP_OK) return ret;
    }

    return ESP_OK;
}

esp_err_t STUSB4500::getActivePdoCount(uint8_t* count)
{
    return getPdoNumber(count);
}

esp_err_t STUSB4500::getNegotiatedPdoNumber(uint8_t* pdo_num)
{
    if (!is_initialized || pdo_num == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if PD contract is active first
    stusb4500_prt_status_t prt_status;
    esp_err_t ret = readPrtStatus(&prt_status);
    if (ret != ESP_OK) return ret;

    if (!prt_status.pd_contract_active) {
        ESP_LOGD(TAG, "No PD contract active, no negotiated PDO");
        *pdo_num = 0;
        return ESP_OK;
    }

    // Read which PDO is currently negotiated
    uint8_t active_pdo_reg;
    ret = i2cReadRegister(STUSB4500_REG_PE_PDO, &active_pdo_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read negotiated PDO register: %s", esp_err_to_name(ret));
        return ret;
    }

    // The active PDO number is in the lower 3 bits
    *pdo_num = active_pdo_reg & 0x07;

    ESP_LOGD(TAG, "Currently negotiated PDO: %d (reg value: 0x%02X)", *pdo_num, active_pdo_reg);

    return ESP_OK;
}

// Control Functions
esp_err_t STUSB4500::softReset()
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Performing soft reset");

    // Send soft reset command (based on SparkFun implementation)
    esp_err_t ret = i2cWriteRegister(STUSB4500_REG_TX_HEADER_LOW, 0x0D);
    if (ret != ESP_OK) return ret;

    ret = i2cWriteRegister(STUSB4500_REG_PD_COMMAND_CTRL, 0x26);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset
    return ESP_OK;
}

// Private Implementation Methods

esp_err_t STUSB4500::i2cRead(uint8_t reg_addr, uint8_t* data, size_t len)
{
    if (device_handle == nullptr || data == nullptr || len == 0) {
        ESP_LOGE(TAG, "I2C read failed: invalid parameters");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "I2C read: reg=0x%02X, len=%d", reg_addr, len);

    esp_err_t ret = i2c_master_transmit_receive(device_handle,
        &reg_addr, 1,
        data, len,
        CONFIG_STUSB4500_I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, error=%s", reg_addr, esp_err_to_name(ret));
    }
    else {
        ESP_LOGD(TAG, "I2C read success: reg=0x%02X, data=0x%02X", reg_addr, data[0]);
    }

    return ret;
}

esp_err_t STUSB4500::i2cWrite(uint8_t reg_addr, const uint8_t* data, size_t len)
{
    if (device_handle == nullptr || data == nullptr || len == 0) {
        ESP_LOGE(TAG, "I2C write failed: invalid parameters");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t* write_buffer = (uint8_t*)malloc(len + 1);
    if (write_buffer == nullptr) {
        ESP_LOGE(TAG, "I2C write failed: no memory");
        return ESP_ERR_NO_MEM;
    }

    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data, len);

    ESP_LOGD(TAG, "I2C write: reg=0x%02X, len=%d, data=0x%02X", reg_addr, len, data[0]);

    esp_err_t ret = i2c_master_transmit(device_handle, write_buffer, len + 1, CONFIG_STUSB4500_I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: reg=0x%02X, error=%s", reg_addr, esp_err_to_name(ret));
    }
    else {
        ESP_LOGD(TAG, "I2C write success: reg=0x%02X", reg_addr);
    }

    free(write_buffer);
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay as in SparkFun library

    return ret;
}

esp_err_t STUSB4500::i2cReadRegister(uint8_t reg_addr, uint8_t* data)
{
    return i2cRead(reg_addr, data, 1);
}

esp_err_t STUSB4500::i2cWriteRegister(uint8_t reg_addr, uint8_t data)
{
    return i2cWrite(reg_addr, &data, 1);
}

// NVM Access Functions (based on working SparkFun logic)
esp_err_t STUSB4500::nvmEnterReadMode()
{
    esp_err_t ret;

    // Set password (0x95 -> 0x47)
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CUST_PASSWORD_REG, STUSB4500_REG_FTP_CUST_PASSWORD);
    if (ret != ESP_OK) return ret;

    // NVM internal controller reset (0x96 -> 0x00)
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, 0x00);
    if (ret != ESP_OK) return ret;

    // Set PWR and RST_N bits (0x96 -> 0xC0)
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t STUSB4500::nvmEnterWriteMode(uint8_t sectors_to_erase)
{
    esp_err_t ret;

    // Set password
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CUST_PASSWORD_REG, STUSB4500_REG_FTP_CUST_PASSWORD);
    if (ret != ESP_OK) return ret;

    // Clear RW buffer
    ret = i2cWriteRegister(STUSB4500_REG_RW_BUFFER, 0x00);
    if (ret != ESP_OK) return ret;

    // Reset
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, 0x00);
    if (ret != ESP_OK) return ret;

    // Set PWR and RST_N
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N);
    if (ret != ESP_OK) return ret;

    // Set write sectors opcode
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1,
        ((sectors_to_erase << 3) & STUSB4500_FTP_CUST_SER) | (STUSB4500_NVM_WRITE_SER & STUSB4500_FTP_CUST_OPCODE));
    if (ret != ESP_OK) return ret;

    // Start write operation
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0,
        STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    // Wait for completion
    ret = nvmWaitForCompletion(500);
    if (ret != ESP_OK) return ret;

    // Soft prog sector
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1, STUSB4500_NVM_SOFT_PROG_SECTOR & STUSB4500_FTP_CUST_OPCODE);
    if (ret != ESP_OK) return ret;

    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0,
        STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    ret = nvmWaitForCompletion();
    if (ret != ESP_OK) return ret;

    // Erase sectors
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1, STUSB4500_NVM_ERASE_SECTOR & STUSB4500_FTP_CUST_OPCODE);
    if (ret != ESP_OK) return ret;

    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0,
        STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    return nvmWaitForCompletion();
}

esp_err_t STUSB4500::nvmExitTestMode()
{
    esp_err_t ret;

    // Reset FTP control
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_RST_N);
    if (ret != ESP_OK) return ret;

    // Clear password
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CUST_PASSWORD_REG, 0x00);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t STUSB4500::nvmReadSector(uint8_t sector_num, uint8_t* sector_data)
{
    esp_err_t ret;

    // Set PWR and RST_N bits
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N);
    if (ret != ESP_OK) return ret;

    // Set Read Sectors Opcode
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1, STUSB4500_NVM_READ & STUSB4500_FTP_CUST_OPCODE);
    if (ret != ESP_OK) return ret;

    // Load Read Sectors Opcode with sector number
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0,
        (sector_num & STUSB4500_FTP_CUST_SECT) | STUSB4500_FTP_CUST_PWR |
        STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    // Wait for completion
    ret = nvmWaitForCompletion();
    if (ret != ESP_OK) return ret;

    // Read the sector data
    return i2cRead(STUSB4500_REG_RW_BUFFER, sector_data, 8);
}

esp_err_t STUSB4500::nvmWriteSector(uint8_t sector_num, const uint8_t* sector_data)
{
    esp_err_t ret;

    // Write data to buffer first
    ret = i2cWrite(STUSB4500_REG_RW_BUFFER, sector_data, 8);
    if (ret != ESP_OK) return ret;

    // Set Write to PL Sectors Opcode
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1, STUSB4500_NVM_WRITE_PL & STUSB4500_FTP_CUST_OPCODE);
    if (ret != ESP_OK) return ret;

    // Load Write to PL Sectors Opcode
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    // Wait for completion
    ret = nvmWaitForCompletion();
    if (ret != ESP_OK) return ret;

    // Set Prog Sectors Opcode
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_1, STUSB4500_NVM_PROG_SECTOR & STUSB4500_FTP_CUST_OPCODE);
    if (ret != ESP_OK) return ret;

    // Load Prog Sectors Opcode with sector number
    ret = i2cWriteRegister(STUSB4500_REG_FTP_CTRL_0,
        (sector_num & STUSB4500_FTP_CUST_SECT) | STUSB4500_FTP_CUST_PWR |
        STUSB4500_FTP_CUST_RST_N | STUSB4500_FTP_CUST_REQ);
    if (ret != ESP_OK) return ret;

    // Wait for completion
    return nvmWaitForCompletion();
}

esp_err_t STUSB4500::nvmWaitForCompletion(uint32_t timeout_ms)
{
    uint8_t status;
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    do {
        esp_err_t ret = i2cReadRegister(STUSB4500_REG_FTP_CTRL_0, &status);
        if (ret != ESP_OK) return ret;

        // Check if operation is complete (REQ bit cleared)
        if ((status & STUSB4500_FTP_CUST_REQ) == 0) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    } while ((xTaskGetTickCount() - start_time) < timeout_ticks);

    ESP_LOGE(TAG, "NVM operation timeout");
    return ESP_ERR_TIMEOUT;
}

uint32_t STUSB4500::readPdoFromRegisters(uint8_t pdo_num)
{
    if (pdo_num < 1 || pdo_num > 3) {
        return 0;
    }

    uint8_t pdo_regs[4];
    uint8_t base_reg = STUSB4500_REG_DPM_SNK_PDO1_0 + ((pdo_num - 1) * 4);

    esp_err_t ret = i2cRead(base_reg, pdo_regs, 4);
    if (ret != ESP_OK) {
        return 0;
    }

    // Combine bytes (little endian format)
    uint32_t pdo_data = 0;
    for (uint8_t i = 0; i < 4; i++) {
        pdo_data += ((uint32_t)pdo_regs[i] << (i * 8));
    }

    return pdo_data;
}

esp_err_t STUSB4500::writePdoToRegisters(uint8_t pdo_num, uint32_t pdo_data)
{
    if (pdo_num < 1 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pdo_regs[4];
    uint8_t base_reg = STUSB4500_REG_DPM_SNK_PDO1_0 + ((pdo_num - 1) * 4);

    // Convert to little endian bytes
    pdo_regs[0] = pdo_data & 0xFF;
    pdo_regs[1] = (pdo_data >> 8) & 0xFF;
    pdo_regs[2] = (pdo_data >> 16) & 0xFF;
    pdo_regs[3] = (pdo_data >> 24) & 0xFF;

    return i2cWrite(base_reg, pdo_regs, 4);
}

uint32_t STUSB4500::pdoStructToRaw(const stusb4500_pdo_t* pdo)
{
    if (pdo == nullptr) {
        return 0;
    }

    uint32_t raw = 0;

    if (pdo->type == STUSB4500_PDO_TYPE_FIXED) {
        // Fixed PDO format (USB PD specification)
        raw |= (STUSB4500_PDO_TYPE_FIXED << 30);

        if (pdo->dual_role_power) raw |= (1 << 29);
        if (pdo->usb_suspend_supported) raw |= (1 << 28);
        if (pdo->unconstrained_power) raw |= (1 << 27);
        if (pdo->usb_comm_capable) raw |= (1 << 26);
        if (pdo->dual_role_data) raw |= (1 << 25);

        raw |= ((pdo->peak_current & 0x03) << 20);
        raw |= (((pdo->voltage_mv + 25) / 50) & 0x3FF) << 10;
        raw |= ((pdo->max_current_ma + 5) / 10) & 0x3FF;
    }

    return raw;
}

esp_err_t STUSB4500::rawToPdoStruct(uint32_t raw_pdo, stusb4500_pdo_t* pdo)
{
    if (pdo == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(pdo, 0, sizeof(*pdo));

    pdo->type = (stusb4500_pdo_type_t)((raw_pdo >> 30) & 0x03);

    if (pdo->type == STUSB4500_PDO_TYPE_FIXED) {
        pdo->dual_role_power = (raw_pdo & (1 << 29)) != 0;
        pdo->usb_suspend_supported = (raw_pdo & (1 << 28)) != 0;
        pdo->unconstrained_power = (raw_pdo & (1 << 27)) != 0;
        pdo->usb_comm_capable = (raw_pdo & (1 << 26)) != 0;
        pdo->dual_role_data = (raw_pdo & (1 << 25)) != 0;
        pdo->peak_current = (raw_pdo >> 20) & 0x03;
        pdo->voltage_mv = ((raw_pdo >> 10) & 0x3FF) * 50;
        pdo->max_current_ma = (raw_pdo & 0x3FF) * 10;
    }

    return ESP_OK;
}

void STUSB4500::loadPdoSettingsFromNVM()
{
    if (!nvm_sectors_read) {
        return;
    }

    // Load PDO number from NVM (sector 3, byte 2, bits 2:3)
    uint8_t pdo_count = (nvm_sectors[3][2] & 0x06) >> 1;
    setPdoNumber(pdo_count);

    // Load PDO1 (fixed at 5V)
    setVoltage(1, 5.0f);
    uint8_t current_value = (nvm_sectors[3][2] & 0xF0) >> 4;
    if (current_value == 0) {
        setCurrent(1, 0);
    }
    else if (current_value < 11) {
        setCurrent(1, current_value * 0.25f + 0.25f);
    }
    else {
        setCurrent(1, current_value * 0.50f - 2.50f);
    }

    // Load PDO2
    float voltage2 = ((nvm_sectors[4][1] << 2) + (nvm_sectors[4][0] >> 6)) / 20.0f;
    setVoltage(2, voltage2);
    current_value = nvm_sectors[3][4] & 0x0F;
    if (current_value == 0) {
        setCurrent(2, 0);
    }
    else if (current_value < 11) {
        setCurrent(2, current_value * 0.25f + 0.25f);
    }
    else {
        setCurrent(2, current_value * 0.50f - 2.50f);
    }

    // Load PDO3
    float voltage3 = (((nvm_sectors[4][3] & 0x03) << 8) + nvm_sectors[4][2]) / 20.0f;
    setVoltage(3, voltage3);
    current_value = (nvm_sectors[3][5] & 0xF0) >> 4;
    if (current_value == 0) {
        setCurrent(3, 0);
    }
    else if (current_value < 11) {
        setCurrent(3, current_value * 0.25f + 0.25f);
    }
    else {
        setCurrent(3, current_value * 0.50f - 2.50f);
    }
}

void STUSB4500::savePdoSettingsToNVM()
{
    if (!nvm_sectors_read) {
        return;
    }

    // Get current PDO count and save to NVM
    uint8_t pdo_count;
    if (getPdoNumber(&pdo_count) == ESP_OK) {
        nvm_sectors[3][2] &= 0xF9; // Clear bits 2:3
        nvm_sectors[3][2] |= (pdo_count << 1); // Set PDO count
    }

    // Convert current PDO settings back to NVM format
    for (uint8_t i = 1; i <= 3; i++) {
        float voltage = getVoltage(i);
        float current = getCurrent(i);

        // Convert current to 4-bit NVM format
        uint8_t nvm_current = 0;
        if (current > 0.0f) {
            if (current <= 3.0f) {
                nvm_current = (uint8_t)(current * 4) - 1; // 0.25A steps
            }
            else {
                nvm_current = (uint8_t)(current * 2) + 5; // 0.5A steps
            }
        }

        // Save current values to NVM sectors
        if (i == 1) {
            // PDO1 current (sector 3, byte 2, bits 4:7)
            nvm_sectors[3][2] &= 0x0F;
            nvm_sectors[3][2] |= (nvm_current << 4);
        }
        else if (i == 2) {
            // PDO2 current (sector 3, byte 4, bits 0:3)
            nvm_sectors[3][4] &= 0xF0;
            nvm_sectors[3][4] |= nvm_current;

            // PDO2 voltage (10-bit value)
            uint16_t digital_voltage = (uint16_t)(voltage * 20);
            nvm_sectors[4][0] &= 0x3F;
            nvm_sectors[4][0] |= ((digital_voltage & 0x03) << 6);
            nvm_sectors[4][1] = (digital_voltage >> 2);
        }
        else if (i == 3) {
            // PDO3 current (sector 3, byte 5, bits 4:7)
            nvm_sectors[3][5] &= 0x0F;
            nvm_sectors[3][5] |= (nvm_current << 4);

            // PDO3 voltage (10-bit value)
            uint16_t digital_voltage = (uint16_t)(voltage * 20);
            nvm_sectors[4][2] = digital_voltage & 0xFF;
            nvm_sectors[4][3] &= 0xFC;
            nvm_sectors[4][3] |= (digital_voltage >> 8);
        }
    }
}

// Additional Configuration Functions (matching SparkFun exactly)
uint8_t STUSB4500::getLowerVoltageLimit(uint8_t pdo_num)
{
    if (!nvm_sectors_read || pdo_num < 1 || pdo_num > 3) {
        return 0;
    }

    if (pdo_num == 1) {
        return 0; // PDO1 has fixed limit
    }
    else if (pdo_num == 2) {
        return (nvm_sectors[3][4] >> 4) + 5;
    }
    else {
        return (nvm_sectors[3][6] & 0x0F) + 5;
    }
}

esp_err_t STUSB4500::setLowerVoltageLimit(uint8_t pdo_num, uint8_t percentage)
{
    if (!nvm_sectors_read || pdo_num < 2 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    if (percentage < 5) percentage = 5;
    else if (percentage > 20) percentage = 20;

    if (pdo_num == 2) {
        nvm_sectors[3][4] &= 0x0F;
        nvm_sectors[3][4] |= ((percentage - 5) << 4);
    }
    else {
        nvm_sectors[3][6] &= 0xF0;
        nvm_sectors[3][6] |= (percentage - 5);
    }

    return ESP_OK;
}

uint8_t STUSB4500::getUpperVoltageLimit(uint8_t pdo_num)
{
    if (!nvm_sectors_read || pdo_num < 1 || pdo_num > 3) {
        return 0;
    }

    if (pdo_num == 1) {
        return (nvm_sectors[3][3] >> 4) + 5;
    }
    else if (pdo_num == 2) {
        return (nvm_sectors[3][5] & 0x0F) + 5;
    }
    else {
        return (nvm_sectors[3][6] >> 4) + 5;
    }
}

esp_err_t STUSB4500::setUpperVoltageLimit(uint8_t pdo_num, uint8_t percentage)
{
    if (!nvm_sectors_read || pdo_num < 1 || pdo_num > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    if (percentage < 5) percentage = 5;
    else if (percentage > 20) percentage = 20;

    if (pdo_num == 1) {
        nvm_sectors[3][3] &= 0x0F;
        nvm_sectors[3][3] |= ((percentage - 5) << 4);
    }
    else if (pdo_num == 2) {
        nvm_sectors[3][5] &= 0xF0;
        nvm_sectors[3][5] |= (percentage - 5);
    }
    else {
        nvm_sectors[3][6] &= 0x0F;
        nvm_sectors[3][6] |= ((percentage - 5) << 4);
    }

    return ESP_OK;
}

float STUSB4500::getFlexCurrent()
{
    if (!nvm_sectors_read) {
        return 0.0f;
    }

    uint16_t digital_value = ((nvm_sectors[4][4] & 0x0F) << 6) + ((nvm_sectors[4][3] & 0xFC) >> 2);
    return digital_value / 100.0f;
}

esp_err_t STUSB4500::setFlexCurrent(float current)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    if (current < 0.0f) current = 0.0f;
    else if (current > 5.0f) current = 5.0f;

    uint16_t digital_value = (uint16_t)(current * 100);

    nvm_sectors[4][4] &= 0xF0;
    nvm_sectors[4][4] |= (digital_value >> 6) & 0x0F;

    nvm_sectors[4][3] &= 0x03;
    nvm_sectors[4][3] |= (digital_value << 2) & 0xFC;

    return ESP_OK;
}

bool STUSB4500::getExternalPower()
{
    if (!nvm_sectors_read) {
        return false;
    }

    return (nvm_sectors[3][2] & 0x08) != 0;
}

esp_err_t STUSB4500::setExternalPower(bool enabled)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    nvm_sectors[3][2] &= 0xF7; // Clear bit 3
    if (enabled) {
        nvm_sectors[3][2] |= 0x08; // Set bit 3
    }

    return ESP_OK;
}

bool STUSB4500::getUsbCommCapable()
{
    if (!nvm_sectors_read) {
        return false;
    }

    return (nvm_sectors[3][2] & 0x01) != 0;
}

esp_err_t STUSB4500::setUsbCommCapable(bool enabled)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    nvm_sectors[3][2] &= 0xFE; // Clear bit 0
    if (enabled) {
        nvm_sectors[3][2] |= 0x01; // Set bit 0
    }

    return ESP_OK;
}

uint8_t STUSB4500::getConfigOkGpio()
{
    if (!nvm_sectors_read) {
        return 0;
    }

    return (nvm_sectors[4][4] & 0x60) >> 5;
}

esp_err_t STUSB4500::setConfigOkGpio(uint8_t config)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config > 3) config = 3;

    nvm_sectors[4][4] &= 0x9F; // Clear bits 5:6
    nvm_sectors[4][4] |= (config << 5);

    return ESP_OK;
}

uint8_t STUSB4500::getGpioCtrl()
{
    if (!nvm_sectors_read) {
        return 0;
    }

    return (nvm_sectors[1][0] & 0x30) >> 4;
}

esp_err_t STUSB4500::setGpioCtrl(uint8_t config)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config > 3) config = 3;

    nvm_sectors[1][0] &= 0xCF; // Clear bits 4:5
    nvm_sectors[1][0] |= (config << 4);

    return ESP_OK;
}

bool STUSB4500::getPowerAbove5vOnly()
{
    if (!nvm_sectors_read) {
        return false;
    }

    return (nvm_sectors[4][6] & 0x08) != 0;
}

esp_err_t STUSB4500::setPowerAbove5vOnly(bool enabled)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    nvm_sectors[4][6] &= 0xF7; // Clear bit 3
    if (enabled) {
        nvm_sectors[4][6] |= 0x08; // Set bit 3
    }

    return ESP_OK;
}

bool STUSB4500::getReqSrcCurrent()
{
    if (!nvm_sectors_read) {
        return false;
    }

    return (nvm_sectors[4][6] & 0x10) != 0;
}

esp_err_t STUSB4500::setReqSrcCurrent(bool enabled)
{
    if (!nvm_sectors_read) {
        return ESP_ERR_INVALID_STATE;
    }

    nvm_sectors[4][6] &= 0xEF; // Clear bit 4
    if (enabled) {
        nvm_sectors[4][6] |= 0x10; // Set bit 4
    }

    return ESP_OK;
}

// Static utility function for I2C bus scanning
esp_err_t STUSB4500::scanI2cBus(i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    bool found_devices = false;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_device_config_t device_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };

        i2c_master_dev_handle_t test_handle;
        esp_err_t ret = i2c_master_bus_add_device(bus_handle, &device_cfg, &test_handle);
        if (ret == ESP_OK) {
            // Try to probe the device
            uint8_t dummy = 0;
            ret = i2c_master_transmit_receive(test_handle, &dummy, 0, &dummy, 0, 50);

            if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
                found_devices = true;
            }

            i2c_master_bus_rm_device(test_handle);
        }
    }

    if (!found_devices) {
        ESP_LOGW(TAG, "No I2C devices found on the bus");
    }

    return ESP_OK;
}

// Utility Functions
const char* STUSB4500::ccStateToString(stusb4500_cc_state_t state)
{
    switch (state) {
    case STUSB4500_CC_STATE_NOT_IN_UFP: return "Not in UFP";
    case STUSB4500_CC_STATE_DEFAULT_USB: return "Default USB";
    case STUSB4500_CC_STATE_POWER_1_5A: return "1.5A";
    case STUSB4500_CC_STATE_POWER_3_0A: return "3.0A";
    default: return "Unknown";
    }
}

const char* STUSB4500::typecFsmStateToString(stusb4500_typec_fsm_state_t state)
{
    switch (state) {
    case STUSB4500_TYPEC_FSM_UNATTACHED_SNK: return "Unattached.SNK";
    case STUSB4500_TYPEC_FSM_ATTACH_WAIT_SNK: return "AttachWait.SNK";
    case STUSB4500_TYPEC_FSM_ATTACHED_SNK: return "Attached.SNK";
    case STUSB4500_TYPEC_FSM_DEBUG_ACCESSORY_SNK: return "DebugAccessory.SNK";
    default: return "Unknown";
    }
}

void STUSB4500::printNegotiationStatus(const stusb4500_negotiation_status_t* status)
{
    if (status == nullptr) return;

    ESP_LOGI(TAG, "=== STUSB4500 Status ===");
    ESP_LOGI(TAG, "Device ID: 0x%02X", status->device_id);
    ESP_LOGI(TAG, "Connected: %s", status->is_connected ? "Yes" : "No");
    ESP_LOGI(TAG, "PD Contract: %s", status->pd_negotiation_complete ? "Active" : "None");
    ESP_LOGI(TAG, "CC1: %s", ccStateToString(status->cc_status.cc1_state));
    ESP_LOGI(TAG, "CC2: %s", ccStateToString(status->cc_status.cc2_state));
    ESP_LOGI(TAG, "FSM: %s", typecFsmStateToString(status->pd_typec_status.fsm_state));
}

void STUSB4500::printPowerStatus(const stusb4500_power_status_t* status)
{
    if (status == nullptr) return;

    ESP_LOGI(TAG, "=== Power Status ===");
    ESP_LOGI(TAG, "Voltage: %u mV", status->voltage_mv);
    ESP_LOGI(TAG, "Current: %u mA", status->current_ma);
    ESP_LOGI(TAG, "Power: %lu mW", status->power_mw);
}

void STUSB4500::printPdo(const stusb4500_pdo_t* pdo, uint8_t pdo_number)
{
    if (pdo == nullptr) return;

    ESP_LOGI(TAG, "PDO %d: %.2fV/%.2fA (%.1fW)",
        pdo_number,
        pdo->voltage_mv / 1000.0f,
        pdo->max_current_ma / 1000.0f,
        (pdo->voltage_mv * pdo->max_current_ma) / 1000000.0f);
}
