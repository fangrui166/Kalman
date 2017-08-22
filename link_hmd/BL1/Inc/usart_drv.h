#ifndef __USART_DRV_H__
#define __USART_DRV_H__
#include "boot.h"

#ifdef UART_HLOG_ENABLE
#define uart_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_OS, fmt, ##__VA_ARGS__)
#define uart_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_OS, fmt, ##__VA_ARGS__)
#define uart_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_OS, fmt, ##__VA_ARGS__)
#define uart_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_OS, fmt, ##__VA_ARGS__)
#define uart_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_OS, fmt, ##__VA_ARGS__)
#else /* flash_HLOG_ENABLE */
#define uart_emerg(fmt, ...) \
	printf("[UART][EMR] :" fmt, ##__VA_ARGS__)
#define uart_err(fmt, ...) \
	printf("[UART][ERR] :" fmt, ##__VA_ARGS__)
#define uart_warning(fmt, ...) \
	printf("[UART][WARN]:" fmt, ##__VA_ARGS__)
#define uart_info(fmt, ...) \
	printf("[UART][INFO]:" fmt, ##__VA_ARGS__)
#define uart_debug(fmt, ...) \
	printf("[UART][DBG] :" fmt, ##__VA_ARGS__)
#endif /* misc_HLOG_ENABLE */

void dbg_uart_init(pcbid_t pcb_id);
#endif
