#ifndef __AUDIO_ALC4040_DRIVER_H__
#define __AUDIO_ALC4040_DRIVER_H__
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define AUDIO_ALC4040_HLOG_ENABLE
#ifdef AUDIO_ALC4040_HLOG_ENABLE
#define audio_4040_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_4040_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_4040_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_4040_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_4040_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#else /* AUDIO_ALC4040_HLOG_ENABLE */
#define audio_4040_emerg(fmt, ...) \
	printf("[AUDIO][EMR] :" fmt, ##__VA_ARGS__)
#define audio_4040_err(fmt, ...) \
	printf("[AUDIO][ERR] :" fmt, ##__VA_ARGS__)
#define audio_4040_warning(fmt, ...) \
	printf("[AUDIO][WARN]:" fmt, ##__VA_ARGS__)
#define audio_4040_info(fmt, ...) \
	printf("[AUDIO][INFO]:" fmt, ##__VA_ARGS__)
#define audio_4040_debug(fmt, ...) \
	printf("[AUDIO][DBG] :" fmt, ##__VA_ARGS__)
#endif /* AUDIO_ALC4040_HLOG_ENABLE */

enum __audio_4040_path {
	AUDIO_4040_UNSET_PATH = 0,
	AUDIO_4040_HDMI_PATH,
	AUDIO_4040_USB_PATH,
};

int audio_alc4040_driver_init(void);
int audio_alc4040_audio_path_set(enum __audio_4040_path);
#ifdef __cplusplus
}
#endif
#endif /* __AUDIO_ALC4040_DRIVER_H__ */