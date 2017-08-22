#ifndef __CCG_H__
#define __CCG_H__

/* CCG Register definitions */
typedef enum {
    /* Device status registers */
	CCG_DEVICE_MODE_ADDR,
	CCG_BOOT_MODE_REASON,
	CCG_SILICON_ID,
	CCG_INTR_REG = 0x06,
	CCG_JUMP_TO_BOOT,
	CCG_RESET,
	CCG_GET_VERSION = 0x10,

    /* Flash update registers (bootloader-only) */
	CCG_ENTER_FLASH_MODE = 0x0A,
	CCG_VALIDATE_FIRMWARE,
	CCG_FLASH_READ_WRITE,

    /* Register to send VDMs to port partner */
	CCG_VDM_CTRL = 0x20,

    /* PDO registers */
    CCG_EFFECTIVE_SOURCE_PDO_MASK = 0x24,
	CCG_EFFECTIVE_SINK_PDO_MASK,
	CCG_SELECT_SOURCE_PDO,
	CCG_SELECT_SINK_PDO,

    /* Port information registers */
    CCG_PD_CONTROL,
	CCG_PD_STATUS = 0x2C,
	CCG_TYPE_C_STATUS = 0x30,

    /* Contract information registers */
    CCG_CURRENT_PDO = 0x34,
	CCG_CURRENT_RDO = 0x38,
	CCG_CURRENT_CABLE_VDO = 0x3C,
    CCG_SWAP_RESPONSE = 0x4C,
	CCG_EVENT_MASK = 0x48,
	CCG_RESPONSE = 0x7E,

    /* Data registers */
	CCG_RESPONSE_DATA_MEMORY = 0x80,
	CCG_CMD_DATA_MEMORY = 0xC0,
} pd_reg_t;

/* PD_CONTROL Register Data definitions */
typedef enum {
	PD_CTRL_SET_RP_TYPEC_DEFAULT,
	PD_CTRL_SET_RP_TYPEC_1_5A,
	PD_CTRL_SET_RP_TYPEC_3A,
	PD_CTRL_DATA_ROLE_SWAP,
	PD_CTRL_POWER_ROLE_SWAP,
	PD_CTRL_VCONN_ON,
	PD_CTRL_VCONN_OFF,
	PD_CTRL_VCONN_SOURCE_SWAP,
	PD_CTRL_RETRIVE_SOURCE_CAPS,
	PD_CTRL_RETRIVE_SINK_CAPS,
	PD_CTRL_GOTO_MIN,
	PD_CTRL_SEND_HARD_RESET
} pd_ctrl_t;

/* Event response codes */
typedef enum {
	/* Device state related responses */
	RESP_NO_RESPONSE,
	RESP_SUCCESS = 0x02,
	RESP_FLASH_DATA_AVAILABLE,
	RESP_INVALID_COMMAND = 0x05,
	RESP_COLLISION_DETECTED,
	RESP_FLASH_UPDATE_FAILED,
	RESP_INVALID_FIRMWARE,
	RESP_INVALID_ARGUMENTS,
	RESP_NOT_SUPPORTED,
	RESP_TRANSACTION_FAILED = 0x0C,
	RESP_PD_COMMAND_FAILED,
	RESP_UNDEFINED,
	RESP_RESET_COMPLETE = 0x80,
	RESP_MESSAGE_QUEUE_OVERFLOW,

	/* Type C specific events */
	RESP_OVER_CURRENT_DETECTED,
	RESP_OVER_VOLTAGE_DETECTED,
	RESP_TYPC_C_CONNECTED,
	RESP_TYPE_C_DISCONNECTED,

	/* PD Specific events and asynchronous messages */
	RESP_PD_CONTRACT_ESTABLISHED,
	RESP_SWAP_COMPLETE,
	RESP_PS_RDY_RCVD = 0x8A,
	RESP_GOTOMIN_RCVD,
	RESP_ACCEPT_RCVD,
	RESP_REJECT_RCVD,
	RESP_WAIT_RCVD,
	RESP_HARD_RESET_RCVD,
	RESP_VDM_RECEIVED,
	RESP_SRC_CAP_RCVD,
	RESP_SINK_CAP_RCVD,

    /* DisplayPort specific events */
	RESP_DP_ALTERNATE_MODE,
	RESP_DP_DEVICE_CONNECTED,
	RESP_DP_DEVICE_NOT_CONNECTED,
	RESP_DP_SID_NOT_FOUND,
	RESP_MULTIPLE_SVID_DISCOVERED,
	RESP_DP_FUNCTION_NOT_SUPPORTED,
	RESP_DP_PORT_CONFIG_NOT_SUPPORTED,

    /* Other PD/Type-C events */
	RESP_HARD_RESET_SENT,
	RESP_SOFT_RESET_SENT,
	RESP_SOURCE_DISBALED_STATE_ENTERED,
	RESP_SENDER_RESPONSE_TIMER_TIMEOUT,
	RESP_NO_VDM_RESPONSE_RECEIVED,
    RESP_EMARKED_CABLE_DETECTED = 0xA6
} ccg_response_t;

/* Event data */
typedef struct {
    ccg_response_t event_code;
    uint8_t event_length;
    uint8_t event_data[128];
} ccg_event_t;

/* Swap status type */
typedef enum {
    SWAP_ACCEPTED = 0x00,
    SWAP_REJECTED = 0x10,
    SWAP_WAIT_RECEIVED = 0x20,
    SWAP_NO_RESPONSE_FROM_PORT = 0x30,
    SWAP_HARD_RESET_SENT = 0x40,
    SWAP_IN_PROGRESS = 0x01,
    SWAP_COMMAND_FAILED = 0x02
} swap_response_t;

/* Some useful bit masks */

/* PD_STATUS register masks */
#define PD_STATUS_CONTRACT_EXISTS       (1 << 10)
#define PD_STATUS_SOURCE                (1 << 8)

/* CCG Event Bitmasks. Enable any event you like */
#define CCG_TYPE_C_CONNECT              (1 << 3)
#define CCG_TYPE_C_DISCONNECT           (1 << 4)
#define CCG_PD_CONTRACT_ESTABLISHED     (1 << 5)
#define CCG_PD_CTRL_MESSAGE_RECEIVED    (1 << 6)
#define CCG_VDM_RECEIVED                (1 << 7)
#define CCG_SRC_CAP_RCVD                (1 << 8)
#define CCG_SINK_CAP_RCVD               (1 << 9)
#define CCG_DISPLAYPORT_EVENTS          (1 << 10)
#define CCG_ERROR_EVENTS                (1 << 11)

/* CCG role swap bitmasks */
#define ACCEPT_SWAP                     (0)
#define REJECT_SWAP                     (1)
#define SEND_WAIT_ON_SWAP_REQ           (2)
#define DONT_RESPOND_TO_SWAP_REQ        (3)

#define DATA_ROLE_SWAP_POS              (0)
#define POWER_ROLE_SWAP_POS             (2)
#define VCONN_SRC_SWAP_POS              (4)

extern bool gl_swap_in_progress;

extern swap_response_t gl_swap_status;

/* Function Prototypes */
void reg_read(uint16_t reg_addr, uint8_t * reg_data, uint16_t num_bytes_read);
void reg_write(uint16_t reg_addr, uint8_t * reg_data, uint16_t size);
int reg_read_i2c1(uint16_t reg_addr, uint8_t * reg_data, uint16_t num_bytes_read);
void ccg_irq_handler();
ccg_event_t read_event_reg();
void clear_i2c_intr();
void flush_ccg_event_queue();

bool pd_contract_exists();
bool send_power_role_swap();
bool send_data_role_swap();
bool send_vconn_source_swap();
swap_response_t swap_status();

#endif /* __CCG_H__ */
