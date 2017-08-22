#ifndef __HLOG_H__
#define __HLOG_H__

#include "htc_memory_define.h"
#include "stdint.h"

#define CONFIG_LOG_PART_ADDR       LOG_PART_START_ADDRESS
#define CONFIG_LOG_HEAD_ADDR       CONFIG_LOG_PART_ADDR
#define CONFIG_LOG_HEAD_SIZE       16
#define CONFIG_LOG_SAVE_ADDR       (CONFIG_LOG_PART_ADDR+CONFIG_LOG_HEAD_SIZE)

#define CONFIG_LOG_BUF_SIZE         (64*1024)

typedef enum{
    PENDING_BY_EXP = 0XABABABAB,
    PENDING_BY_WDG = 0XBABABABA,
}log_pending_t;
typedef struct {
    uint32_t log_size;
    uint8_t  log_exit_flag;
    log_pending_t log_pending_type;
}flash_log_head_t;

int hlog_save_logbuf(void);
int hlog_dump_logbuf(void);

#endif
