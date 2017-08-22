#ifndef _CCGX_FW_H
#define _CCGX_FW_H

#define CCG4_SLAVE_I2C_ADDR        0x10

#define CCGX_FLASH_FW_START        0x20000
#define CCGX_FLASH_FW_SIZE          107522

#define CCGX_FLASH_FW1_START      CCGX_FLASH_FW_START
#define CCGX_FLASH_FW1_SIZE        CCGX_FLASH_FW_SIZE

#define CCGX_FW_DEBUG         1

int ccgx_get_fw_version(int idx, unsigned char *v);
void ccgx_dump_fw_info(void);

int ccgx_set_fw_info(int idx, unsigned offset, unsigned size);
int ccgx_upgrade_fw(int fw_idx);
int ccgx_upgrade_fw_bl(int fw_idx,bool second_fw);
#define CCGX_ERROR_NONE                 0x00
#define CCGX_ERROR_HANDLE          		0x11
#define CCGX_ERROR_FIRM_INDEX           0x12
#define CCGX_ERROR_FIRM_INFO            0x13
#define CCGX_ERROR_RESPONSE             0x14
#define CCGX_ERROR_BL_MODE         		0x15
#define CCGX_ERROR_FW_OFFSET            0x16
#define CCGX_ERROR_FW_LOC               0x17
#define CCGX_ERROR_FLASH_MODE           0x18
#define CCGX_ERROR_IMAGE_LINE           0x19
#define CCGX_ERROR_IMAGE1_ROW           0x1A
#define CCGX_ERROR_IMAGE2_ROW           0x1B
#define CCGX_ERROR_IMAGE_ROW            0x1C
#define CCGX_ERROR_FLASH_ROW            0x1D
#define CCGX_ERROR_VERIFY_ROW           0x1E
#define CCGX_ERROR_READ_ROW             0x1F
#define CCGX_ERROR_VERIFY_FW            0x20
#define CCGX_ERROR_RESET_ERROR          0x21
#define CCGX_SAME_VERSION			    0x22

#endif
