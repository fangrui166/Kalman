#ifndef _HLOG_MAIN_H
#define _HLOG_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "htc_usb_cdc_data_service.h"
#include "htc_memory_define.h"

#define LOG_BUF_RUN_TIME_TEST      0
#if 1
#define CONFIG_LOG_PART_ADDR       LOG_PART_START_ADDRESS
#define CONFIG_LOG_HEAD_ADDR       CONFIG_LOG_PART_ADDR
#define CONFIG_LOG_HEAD_SIZE       16
#define CONFIG_LOG_SAVE_ADDR       (CONFIG_LOG_PART_ADDR+CONFIG_LOG_HEAD_SIZE)
typedef enum{
    PENDING_BY_EXP = 0XABABABAB,
    PENDING_BY_WDG = 0XBABABABA,
}log_pending_t;
typedef struct {
    uint32_t log_size;
    uint8_t  log_exit_flag;
    log_pending_t log_pending_type;
}flash_log_head_t;
#else
unsigned hlog_get_nlog_part_offs(void);
unsigned hlog_get_nlog_part_size(void);
unsigned hlog_get_elog_part_offs(void);
unsigned hlog_get_elog_part_size(void);
#define CONFIG_NLOG_PART_OFFS      hlog_get_nlog_part_offs()
#define CONFIG_NLOG_PART_SIZE      hlog_get_nlog_part_size()
#define CONFIG_ELOG_PART_OFFS      hlog_get_elog_part_offs()
#define CONFIG_ELOG_PART_SIZE      hlog_get_elog_part_size()
#endif

#define CONFIG_LOG_BUF_SIZE         (64*1024)

#define CONFIG_LOGMSG_BUF_SIZE      256
#define FLASH_ACCESS_STACK_SIZE     64

#ifdef MVR_DEBUG_BUILD
#define CONFIG_LOG_LEVEL_DEFAULT    HLOG_LVL_DEBUG
#define CONFIG_LOG_TAGS_DEFAULT		0xffffffff
#else
#define CONFIG_LOG_LEVEL_DEFAULT    HLOG_LVL_INFO
#define CONFIG_LOG_TAGS_DEFAULT     ((1 << (HLOG_TAG_OS + 1)) - 1)
#endif

#define CONFIG_HLOG_TS_INTERVAL     256

#define LOG_META_FMT "[%u.%u]<%s>[%s] "

#define DEBUG_HLOG_ENABLE           0

extern unsigned g_log_system_ready;
extern unsigned g_dbg_uart_en;

enum mculog_transfer_type {
	MCULOG_TYPE_UART = 0,
	MCULOG_TYPE_CDC,
};


int hlog_cdc_dump_flash_log(void);
int hlog_cdc_clear_flash_log(void);
//#define hlog_exp_printf(fmt, args...) hlog_printf(HLOG_LVL_MAX, HLOG_TAG_MAX, fmt, ## args)
int hlog_get_flashlog_head(flash_log_head_t * log_head);

#ifdef __cplusplus
}
#endif

#endif
