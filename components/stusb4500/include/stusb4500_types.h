#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    // STUSB4500 Constants
#define STUSB4500_EVAL_DEVICE_ID                0x21
#define STUSB4500_PROD_DEVICE_ID                0x25

// STUSB4500 Register Addresses (based on SparkFun library)
#define STUSB4500_REG_DEVICE_ID                 0x2F
#define STUSB4500_REG_CTRL                      0x18
#define STUSB4500_REG_RESET_CTRL                0x23
#define STUSB4500_REG_CC_STATUS                 0x11
#define STUSB4500_REG_PD_TYPEC_STATUS           0x14
#define STUSB4500_REG_PRT_STATUS                0x17
#define STUSB4500_REG_ALERT_STATUS_1            0x0D
#define STUSB4500_REG_PE_FSM                    0x29

// Power Measurement Registers
#define STUSB4500_REG_VBUS_VOLTAGE_LOW          0x26
#define STUSB4500_REG_VBUS_VOLTAGE_HIGH         0x27
#define STUSB4500_REG_VBUS_VSAFE0V              0x28

// Negotiated power registers (actual values from negotiation)
#define STUSB4500_REG_RDO_REG_STATUS            0x91
#define STUSB4500_REG_PE_PDO                    0x93  // Current negotiated PDO number

// Active PDO Registers (live negotiated values)
#define STUSB4500_REG_DPM_PDO_NUMB              0x70
#define STUSB4500_REG_DPM_SNK_PDO1_0            0x85
#define STUSB4500_REG_DPM_SNK_PDO2_0            0x89
#define STUSB4500_REG_DPM_SNK_PDO3_0            0x8D

// NVM Access Registers (based on SparkFun register map)
#define STUSB4500_REG_FTP_CUST_PASSWORD_REG     0x95
#define STUSB4500_REG_FTP_CUST_PASSWORD         0x47
#define STUSB4500_REG_FTP_CTRL_0                0x96
#define STUSB4500_REG_FTP_CTRL_1                0x97
#define STUSB4500_REG_RW_BUFFER                 0x53

// NVM Control Bits (from SparkFun)
#define STUSB4500_FTP_CUST_PWR                  0x80
#define STUSB4500_FTP_CUST_RST_N                0x40
#define STUSB4500_FTP_CUST_REQ                  0x10
#define STUSB4500_FTP_CUST_SECT                 0x07
#define STUSB4500_FTP_CUST_OPCODE               0x07

// NVM Opcodes (from SparkFun)
#define STUSB4500_NVM_READ                      0x00
#define STUSB4500_NVM_WRITE_PL                  0x01
#define STUSB4500_NVM_WRITE_SER                 0x02
#define STUSB4500_NVM_ERASE_SECTOR              0x05
#define STUSB4500_NVM_PROG_SECTOR               0x06
#define STUSB4500_NVM_SOFT_PROG_SECTOR          0x07

// NVM Sectors (from SparkFun)
#define STUSB4500_NVM_SECTOR_0                  0x01
#define STUSB4500_NVM_SECTOR_1                  0x02
#define STUSB4500_NVM_SECTOR_2                  0x04
#define STUSB4500_NVM_SECTOR_3                  0x08
#define STUSB4500_NVM_SECTOR_4                  0x10

// Constants
#define STUSB4500_RESET_SW_RESET_BIT            0x01
#define STUSB4500_NVM_PASSWORD                  0x47
#define STUSB4500_VBUS_LSB_MV                   25  // 25mV per LSB for voltage measurement
#define STUSB4500_VBUS_VSAFE0V_THRESHOLD        0x0F

// Status bit definitions
#define STUSB4500_CC_STATUS_CC1_STATE_MASK      0x07
#define STUSB4500_CC_STATUS_CC2_STATE_MASK      0x38
#define STUSB4500_CC_STATUS_CC2_STATE_SHIFT     3
#define STUSB4500_CC_STATUS_CONNECT_RESULT      0x40
#define STUSB4500_CC_STATUS_LOOKING4CONNECTION  0x80

#define STUSB4500_PD_TYPEC_STATUS_PD_TYPEC_HAND_CHECK 0x40
#define STUSB4500_PD_TYPEC_STATUS_TYPEC_FSM_STATE_MASK 0x1F

#define STUSB4500_PRT_STATUS_HWRESET_RECEIVED   0x01
#define STUSB4500_PRT_STATUS_SOFTRESET_RECEIVED 0x02
#define STUSB4500_PRT_STATUS_DATAROLE           0x04
#define STUSB4500_PRT_STATUS_POWERROLE          0x08
#define STUSB4500_PRT_STATUS_PD_CONTRACT        0x10
#define STUSB4500_PRT_STATUS_STARTUP_POWER      0x20
#define STUSB4500_PRT_STATUS_MSG_RECEIVED       0x40
#define STUSB4500_PRT_STATUS_MSG_SENT           0x80

// Enumerations
    typedef enum {
        STUSB4500_CC_STATE_NOT_IN_UFP = 0,
        STUSB4500_CC_STATE_DEFAULT_USB = 1,
        STUSB4500_CC_STATE_POWER_1_5A = 2,
        STUSB4500_CC_STATE_POWER_3_0A = 3
    } stusb4500_cc_state_t;

    typedef enum {
        STUSB4500_TYPEC_FSM_UNATTACHED_SNK = 0x00,
        STUSB4500_TYPEC_FSM_ATTACH_WAIT_SNK = 0x01,
        STUSB4500_TYPEC_FSM_ATTACHED_SNK = 0x02,
        STUSB4500_TYPEC_FSM_DEBUG_ACCESSORY_SNK = 0x03,
        STUSB4500_TYPEC_FSM_TRY_WAIT_SNK = 0x04,
        STUSB4500_TYPEC_FSM_UNATTACHED_ACCESSORY = 0x05,
        STUSB4500_TYPEC_FSM_ATTACH_WAIT_ACCESSORY = 0x06,
        STUSB4500_TYPEC_FSM_AUDIO_ACCESSORY = 0x07,
        STUSB4500_TYPEC_FSM_DEBUG_ACCESSORY_SRC = 0x08,
        STUSB4500_TYPEC_FSM_ATTACHED_SRC = 0x09,
        STUSB4500_TYPEC_FSM_TRY_SRC = 0x0A,
        STUSB4500_TYPEC_FSM_UNATTACHED_SRC = 0x0B,
        STUSB4500_TYPEC_FSM_ATTACH_WAIT_SRC = 0x0C,
        STUSB4500_TYPEC_FSM_POWERED_ACCESSORY = 0x0D,
        STUSB4500_TYPEC_FSM_UNSUPPORTED_ACCESSORY = 0x0E,
        STUSB4500_TYPEC_FSM_TRY_WAIT_SRC = 0x0F,
        STUSB4500_TYPEC_FSM_UNORIENTED_DEBUG_ACCESSORY_SRC = 0x10
    } stusb4500_typec_fsm_state_t;

    typedef enum {
        STUSB4500_PDO_TYPE_FIXED = 0,
        STUSB4500_PDO_TYPE_VARIABLE = 1,
        STUSB4500_PDO_TYPE_BATTERY = 2,
        STUSB4500_PDO_TYPE_PROGRAMMABLE = 3
    } stusb4500_pdo_type_t;

    // Structures
    typedef struct {
        bool cc1_connected;
        bool cc2_connected;
        stusb4500_cc_state_t cc1_state;
        stusb4500_cc_state_t cc2_state;
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
        bool data_role_sink;        // true = sink, false = source
        bool power_role_sink;       // true = sink, false = source
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
        uint16_t voltage_mv;        // Voltage in millivolts
        uint16_t current_ma;        // Current in milliamps (estimated from PDO)
        uint32_t power_mw;          // Power in milliwatts (calculated)
        bool vsafe0v;               // VBUS is at safe 0V level
    } stusb4500_power_status_t;

    typedef struct {
        stusb4500_pdo_type_t type;  // PDO type (fixed, variable, etc.)
        uint16_t voltage_mv;        // Voltage in millivolts (50mV resolution)
        uint16_t max_current_ma;    // Maximum current in milliamps (10mA resolution)
        bool dual_role_power;       // Dual role power capability
        bool usb_suspend_supported; // USB suspend support
        bool unconstrained_power;   // Unconstrained power
        bool usb_comm_capable;      // USB communication capability
        bool dual_role_data;        // Dual role data capability
        uint8_t peak_current;       // Peak current capability (0-3)
    } stusb4500_pdo_t;

#ifdef __cplusplus
}
#endif
