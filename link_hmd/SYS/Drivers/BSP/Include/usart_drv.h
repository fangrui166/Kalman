#ifndef __USART_DRV_H__
#define __USART_DRV_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "misc_data.h"

#define CMD_BUF_LEN 130
#define PARAMETER_SIZE 100
#define MAX_ARGS      88


typedef struct
{
    char cmd_buf[CMD_BUF_LEN];
    char cmd_length;
} cmd_struct;

typedef struct
{
    char CmdQueue;
    cmd_struct rec_buf;
}UsartMsgQueue;
typedef enum {
    MSG_QUEUE_FRAME = 1,
    MSG_QUEUE_RXCHAR_FULL,

}USART_MSG_QUEUE_TYPE;
typedef enum {
    MSG_QUEUE_SUCCESS = 1,
    MSG_QUEUE_ERROR,
    MSG_QUEUE_FULL,

}MSG_QUEUE_RET;
#define UART_HLOG_ENABLE 1

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
void dbg_uart_postinit(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void dbg_uart_set_rxne_flag(UART_HandleTypeDef *huart);
void dbg_uart_rx_handler(UART_HandleTypeDef * huart);
void dbg_uart_wakeup_handler(void);


#ifdef __cplusplus
}
#endif
#endif /*__USART_DRV_H__*/

