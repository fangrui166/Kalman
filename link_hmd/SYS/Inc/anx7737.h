#ifndef __ANX7737_DRIVER_H__
#define __ANX7737_DRIVER_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define ANX7737_HLOG_ENABLE
#ifdef  ANX7737_HLOG_ENABLE
#define anx7737_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define anx7737_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define anx7737_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define anx7737_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define anx7737_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#else /* ANX7737_HLOG_ENABLE */
#define anx7737_emerg(fmt, ...) \
	printf("[AUDIO][EMR] :" fmt, ##__VA_ARGS__)
#define anx7737_err(fmt, ...) \
	printf("[AUDIO][ERR] :" fmt, ##__VA_ARGS__)
#define anx7737_warning(fmt, ...) \
	printf("[AUDIO][WARN]:" fmt, ##__VA_ARGS__)
#define anx7737_info(fmt, ...) \
	printf("[AUDIO][INFO]:" fmt, ##__VA_ARGS__)
#define anx7737_debug(fmt, ...) \
	printf("[AUDIO][DBG] :" fmt, ##__VA_ARGS__)
#endif /* ANX7737_HLOG_ENABLE */

int anx7737_driver_init(void);

int anx7737_i2c_read(uint16_t, uint16_t, uint16_t *);
int anx7737_i2c_write(uint16_t, uint16_t, uint16_t);

#ifdef __cplusplus
}
#endif
#endif /* __ANX7737_DRIVER_H__ */
