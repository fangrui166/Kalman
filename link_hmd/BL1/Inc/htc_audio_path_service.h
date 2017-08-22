#ifndef __HTC_AUDIO_PATH_SERVICE_H__
#define __HTC_AUDIO_PATH_SERVICE_H__
#ifdef __cplusplus
 extern "C" {
#endif

//#define AUDIO_PATH_SRV_HLOG_ENABLE
#ifdef AUDIO_PATH_SRV_HLOG_ENABLE
#define audio_srv_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_srv_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_srv_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_srv_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#define audio_srv_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_AUDIO, fmt, ##__VA_ARGS__)
#else /* AUDIO_PATH_SRV_HLOG_ENABLE */
#define audio_srv_emerg(fmt, ...) \
	printf("[AUDIO][EMR] :" fmt, ##__VA_ARGS__)
#define audio_srv_err(fmt, ...) \
	printf("[AUDIO][ERR] :" fmt, ##__VA_ARGS__)
#define audio_srv_warning(fmt, ...) \
	printf("[AUDIO][WARN]:" fmt, ##__VA_ARGS__)
#define audio_srv_info(fmt, ...) \
	printf("[AUDIO][INFO]:" fmt, ##__VA_ARGS__)
#define audio_srv_debug(fmt, ...) \
	printf("[AUDIO][DBG] :" fmt, ##__VA_ARGS__)
#endif /* AUDIO_PATH_SRV_HLOG_ENABLE */

#define AUDIO_SRV_OUTPUT_OFFSET        3

enum __audio_srv_path {
	AUDIO_SRV_UNSET_PATH = 0,
	AUDIO_SRV_HDMI_PATH,
	AUDIO_SRV_USB_PATH,
	AUDIO_SRV_LOOPBACK_PATH,
	AUDIO_SRV_OUTPUT_TO_AJ = 4,
	AUDIO_SRV_OUTPUT_TO_EAR,
	AUDIO_SRV_OUTPUT_TO_BOTH,
};

enum {
	AUDIO_CMD_ID_PATH = 0,
};

enum {
	AMP_POWER_DISABLE,
	AMP_POWER_ENABLE
};

int audio_path_service_init(void);
int audio_path_set(enum __audio_srv_path);
int audio_path_get(enum __audio_srv_path *);

#ifdef __cplusplus
  }
#endif
#endif /* __HTC_AUDIO_PATH_SERVICE_H__ */