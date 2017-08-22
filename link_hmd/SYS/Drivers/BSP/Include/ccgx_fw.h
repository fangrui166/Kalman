#ifndef _CCGX_FW_H
#define _CCGX_FW_H

#define CCG4_SLAVE_I2C_ADDR        0x10

#define CCGX_FLASH_FW_START        0x20000
#define CCGX_FLASH_FW_SIZE          107522

#define CCGX_FLASH_FW1_START      CCGX_FLASH_FW_START
#define CCGX_FLASH_FW1_SIZE        CCGX_FLASH_FW_SIZE

#define CCGX_FW_DEBUG         0

int ccgx_get_fw_version(int idx, unsigned char *v);
void ccgx_dump_fw_info(void);

int ccgx_set_fw_info(int idx, unsigned offset, unsigned size);
int ccgx_upgrade_fw(int fw_idx);

#endif
