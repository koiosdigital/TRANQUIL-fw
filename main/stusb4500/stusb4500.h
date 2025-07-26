#ifndef STUSB4500_H
#define STUSB4500_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    // Device identification constants
#define STUSB4500_REG_DEVICE_ID                 0x2F
#define STUSB4500_EVAL_DEVICE_ID                0x25
#define STUSB4500_PROD_DEVICE_ID                0x21

// Status registers
#define STUSB4500_REG_ALERT_STATUS_1            0x0B
#define STUSB4500_REG_CC_STATUS                 0x11
#define STUSB4500_REG_PD_TYPEC_STATUS           0x14
#define STUSB4500_REG_PRT_STATUS                0x16
#define STUSB4500_REG_PE_FSM                    0x29

// Voltage and power registers
#define STUSB4500_REG_VBUS_VOLTAGE_LOW          0x26
#define STUSB4500_REG_VBUS_VSAFE0V              0x27
#define STUSB4500_VBUS_VSAFE0V_THRESHOLD        0x01
#define STUSB4500_VBUS_LSB_MV                   25  // 25mV per LSB for voltage measurement
#define STUSB4500_VBUS_LSB_MB4500_REG_RESET_CTRL                0x19
#define STUSB4500_RESET_SW_RESET_BIT            0x01

// Negotiated power registers (actual values from negotiation)
#define STUSB4500_REG_RDO_REG_STATUS            0x91
#define STUSB4500_REG_PE_PDO                    0x93  // Current negotiated PDO number

// PDO registers (matching SparkFun exactly)
#define STUSB4500_REG_DPM_SNK_PDO1_0            0x85
#define STUSB4500_REG_DPM_SNK_PDO2_0            0x89
#define STUSB4500_REG_DPM_SNK_PDO3_0            0x8D
#define STUSB4500_REG_DPM_PDO_NUMB              0x70

// NVM Flash Programming registers (matching SparkFun exactly)
#define STUSB4500_REG_FTP_CUST_PASSWORD_REG     0x95
#define STUSB4500_REG_FTP_CUST_PASSWORD         0x47
#define STUSB4500_REG_FTP_CTRL_0                0x96
#define STUSB4500_REG_FTP_CTRL_1                0x97
#define STUSB4500_REG_RW_BUFFER                 0x53

// Soft reset registers (matching SparkFun exactly)
#define STUSB4500_REG_TX_HEADER_LOW             0x51
#define STUSB4500_REG_PD_COMMAND_CTRL           0x1A

// NVM Control bits (matching SparkFun exactly)
#define STUSB4500_FTP_CUST_PWR                  0x80
#define STUSB4500_FTP_CUST_RST_N                0x40
#define STUSB4500_FTP_CUST_REQ                  0x10
#define STUSB4500_FTP_CUST_SECT                 0x07
#define STUSB4500_FTP_CUST_SER                  0xF8
#define STUSB4500_FTP_CUST_OPCODE               0x07

// NVM Operation codes (matching SparkFun exactly)
#define STUSB4500_NVM_READ                      0x00
#define STUSB4500_NVM_WRITE_PL                  0x01
#define STUSB4500_NVM_WRITE_SER                 0x02
#define STUSB4500_NVM_ERASE_SECTOR              0x05
#define STUSB4500_NVM_PROG_SECTOR               0x06
#define STUSB4500_NVM_SOFT_PROG_SECTOR          0x07

// NVM Sector masks (matching SparkFun exactly)
#define STUSB4500_NVM_SECTOR_0                  0x01
#define STUSB4500_NVM_SECTOR_1                  0x02
#define STUSB4500_NVM_SECTOR_2                  0x04
#define STUSB4500_NVM_SECTOR_3                  0x08
#define STUSB4500_NVM_SECTOR_4                  0x10

// CC Status register bits
#define STUSB4500_CC_STATUS_CC1_STATE_MASK      0x03
#define STUSB4500_CC_STATUS_CC2_STATE_MASK      0x0C
#define STUSB4500_CC_STATUS_CC2_STATE_SHIFT     2
#define STUSB4500_CC_STATUS_CONNECT_RESULT      0x10
#define STUSB4500_CC_STATUS_LOOKING4CONNECTION  0x20

// PD TypeC Status register bits
#define STUSB4500_PD_TYPEC_STATUS_PD_TYPEC_HAND_CHECK    0x80
#define STUSB4500_PD_TYPEC_STATUS_TYPEC_FSM_STATE_MASK   0x1F

// PRT Status register bits
#define STUSB4500_PRT_STATUS_HWRESET_RECEIVED   0x01
#define STUSB4500_PRT_STATUS_SOFTRESET_RECEIVED 0x02
#define STUSB4500_PRT_STATUS_DATAROLE           0x04
#define STUSB4500_PRT_STATUS_POWERROLE          0x08
#define STUSB4500_PRT_STATUS_PD_CONTRACT        0x10
#define STUSB4500_PRT_STATUS_STARTUP_POWER      0x20
#define STUSB4500_PRT_STATUS_MSG_RECEIVED       0x40
#define STUSB4500_PRT_STATUS_MSG_SENT           0x80

// CC pin states
    typedef enum {
        STUSB4500_CC_STATE_NOT_IN_UFP = 0,
        STUSB4500_CC_STATE_DEFAULT_USB = 1,
        STUSB4500_CC_STATE_POWER_1_5A = 2,
        STUSB4500_CC_STATE_POWER_3_0A = 3
    } stusb4500_cc_state_t;

    // TypeC FSM states
    typedef enum {
        STUSB4500_TYPEC_FSM_UNATTACHED_SNK = 0,
        STUSB4500_TYPEC_FSM_ATTACH_WAIT_SNK = 1,
        STUSB4500_TYPEC_FSM_ATTACHED_SNK = 2,
        STUSB4500_TYPEC_FSM_DEBUG_ACCESSORY_SNK = 3
    } stusb4500_typec_fsm_state_t;

    // PDO types
    typedef enum {
        STUSB4500_PDO_TYPE_FIXED = 0,
        STUSB4500_PDO_TYPE_BATTERY = 1,
        STUSB4500_PDO_TYPE_VARIABLE = 2,
        STUSB4500_PDO_TYPE_APDO = 3
    } stusb4500_pdo_type_t;

    // Status structures
    typedef struct {
        stusb4500_cc_state_t cc1_state;
        stusb4500_cc_state_t cc2_state;
        bool cc1_connected;
        bool cc2_connected;
        bool connection_result;
        bool looking_for_connection;
    } stusb4500_cc_status_t;

    typedef struct {
        bool pd_typec_handshake_check;
        stusb4500_typec_fsm_state_t fsm_state;
    } stusb4500_pd_typec_status_t;

    typedef struct {
        bool hw_reset_received;
        bool soft_reset_received;
        bool data_role_sink;
        bool power_role_sink;
        bool pd_contract_active;
        bool startup_power;
        bool message_received;
        bool message_sent;
    } stusb4500_prt_status_t;

    typedef struct {
        uint8_t device_id;
        stusb4500_cc_status_t cc_status;
        stusb4500_pd_typec_status_t pd_typec_status;
        stusb4500_prt_status_t prt_status;
        uint8_t pe_fsm_state;
        bool is_connected;
        bool pd_negotiation_complete;
    } stusb4500_negotiation_status_t;

    typedef struct {
        uint16_t voltage_mv;
        uint16_t current_ma;
        uint32_t power_mw;
        bool vsafe0v;
    } stusb4500_power_status_t;

    typedef struct {
        stusb4500_pdo_type_t type;
        uint16_t voltage_mv;
        uint16_t max_current_ma;
        uint8_t peak_current;
        bool dual_role_power;
        bool usb_suspend_supported;
        bool unconstrained_power;
        bool usb_comm_capable;
        bool dual_role_data;
    } stusb4500_pdo_t;

    // Main class
    class STUSB4500 {
    public:
        STUSB4500();
        ~STUSB4500();

        // Initialization
        esp_err_t begin(uint8_t device_address = 0x28);
        esp_err_t end();

        // Device identification
        esp_err_t readDeviceId(uint8_t* device_id);

        // NVM operations
        esp_err_t readNVM();
        esp_err_t writeNVM(bool use_defaults = false);

        // PDO configuration
        esp_err_t getPdoNumber(uint8_t* pdo_count);
        esp_err_t setPdoNumber(uint8_t pdo_count);
        float getVoltage(uint8_t pdo_num);
        esp_err_t setVoltage(uint8_t pdo_num, float voltage);
        float getCurrent(uint8_t pdo_num);
        esp_err_t setCurrent(uint8_t pdo_num, float current);

        // Status and monitoring
        esp_err_t readNegotiationStatus(stusb4500_negotiation_status_t* status);
        esp_err_t readCcStatus(stusb4500_cc_status_t* status);
        esp_err_t readPdTypecStatus(stusb4500_pd_typec_status_t* status);
        esp_err_t readPrtStatus(stusb4500_prt_status_t* status);
        esp_err_t clearAlertStatus();

        // Power measurement
        esp_err_t readPowerStatus(stusb4500_power_status_t* status);
        esp_err_t readVoltage(uint16_t* voltage_mv);
        esp_err_t readCurrent(uint16_t* current_ma);
        esp_err_t readPower(uint32_t* power_mw);

        // Active PDO functions
        esp_err_t readActivePdo(uint8_t pdo_num, stusb4500_pdo_t* pdo);
        esp_err_t readAllActivePdos(stusb4500_pdo_t pdos[3]);
        esp_err_t getActivePdoCount(uint8_t* count);
        esp_err_t getNegotiatedPdoNumber(uint8_t* pdo_num); // Get currently negotiated PDO

        // Control functions
        esp_err_t softReset();

        // Configuration functions (matching SparkFun exactly)
        uint8_t getLowerVoltageLimit(uint8_t pdo_num);
        esp_err_t setLowerVoltageLimit(uint8_t pdo_num, uint8_t percentage);
        uint8_t getUpperVoltageLimit(uint8_t pdo_num);
        esp_err_t setUpperVoltageLimit(uint8_t pdo_num, uint8_t percentage);
        float getFlexCurrent();
        esp_err_t setFlexCurrent(float current);
        bool getExternalPower();
        esp_err_t setExternalPower(bool enabled);
        bool getUsbCommCapable();
        esp_err_t setUsbCommCapable(bool enabled);
        uint8_t getConfigOkGpio();
        esp_err_t setConfigOkGpio(uint8_t config);
        uint8_t getGpioCtrl();
        esp_err_t setGpioCtrl(uint8_t config);
        bool getPowerAbove5vOnly();
        esp_err_t setPowerAbove5vOnly(bool enabled);
        bool getReqSrcCurrent();
        esp_err_t setReqSrcCurrent(bool enabled);

        // Utility functions
        static const char* ccStateToString(stusb4500_cc_state_t state);
        static const char* typecFsmStateToString(stusb4500_typec_fsm_state_t state);
        void printNegotiationStatus(const stusb4500_negotiation_status_t* status);
        void printPowerStatus(const stusb4500_power_status_t* status);
        void printPdo(const stusb4500_pdo_t* pdo, uint8_t pdo_number);
        static esp_err_t scanI2cBus(i2c_master_bus_handle_t bus_handle);

        // Default NVM values (exactly matching SparkFun)
        static const uint8_t default_nvm_sectors[5][8];

    private:
        // I2C communication
        esp_err_t i2cRead(uint8_t reg_addr, uint8_t* data, size_t len);
        esp_err_t i2cWrite(uint8_t reg_addr, const uint8_t* data, size_t len);
        esp_err_t i2cReadRegister(uint8_t reg_addr, uint8_t* data);
        esp_err_t i2cWriteRegister(uint8_t reg_addr, uint8_t data);

        // NVM access functions
        esp_err_t nvmEnterReadMode();
        esp_err_t nvmEnterWriteMode(uint8_t sectors_to_erase);
        esp_err_t nvmExitTestMode();
        esp_err_t nvmReadSector(uint8_t sector_num, uint8_t* sector_data);
        esp_err_t nvmWriteSector(uint8_t sector_num, const uint8_t* sector_data);
        esp_err_t nvmWaitForCompletion(uint32_t timeout_ms = 100);

        // PDO manipulation
        uint32_t readPdoFromRegisters(uint8_t pdo_num);
        esp_err_t writePdoToRegisters(uint8_t pdo_num, uint32_t pdo_data);
        uint32_t pdoStructToRaw(const stusb4500_pdo_t* pdo);
        esp_err_t rawToPdoStruct(uint32_t raw_pdo, stusb4500_pdo_t* pdo);

        // NVM settings management
        void loadPdoSettingsFromNVM();
        void savePdoSettingsToNVM();

        // Member variables
        i2c_master_bus_handle_t i2c_bus_handle;
        i2c_master_dev_handle_t device_handle;
        uint8_t device_address;
        bool is_initialized;
        bool nvm_sectors_read;
        uint8_t nvm_sectors[5][8];
    };

#ifdef __cplusplus
}
#endif

#endif // STUSB4500_H
