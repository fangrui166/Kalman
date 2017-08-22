#ifndef __SYSTEM_COMMAND_H__
#define __SYSTEM_COMMAND_H__

enum {
	USB_SYSCMD_ACTION_GET = 0x00,
	USB_SYSCMD_ACTION_SET = 0x01,
};

enum sys_cmd_service_state {
	SYS_CMD_SERVICE_UNINITIALIZED = 0,
	SYS_CMD_SERVICE_RUNNING,
	SYS_CMD_SERVICE_SUSPEND,
};

#define HTC_SYSCMD_HLOG_ENABLE

#ifdef HTC_SYSCMD_HLOG_ENABLE
#define htc_sys_cmd_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_sys_cmd_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_sys_cmd_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_sys_cmd_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_sys_cmd_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#else /* HTC_SYSCMD_HLOG_ENABLE */
#define htc_sys_cmd_emerg(fmt, ...) \
	printf("[SYS_P][EMR] :" fmt, ##__VA_ARGS__)
#define htc_sys_cmd_err(fmt, ...) \
	printf("[SYS_P][ERR] :" fmt, ##__VA_ARGS__)
#define htc_sys_cmd_warning(fmt, ...) \
	printf("[SYS_P][WARN]:" fmt, ##__VA_ARGS__)
#define htc_sys_cmd_info(fmt, ...) \
	printf("[SYS_P][INFO]:" fmt, ##__VA_ARGS__)
#define htc_sys_cmd_debug(fmt, ...) \
	printf("[SYS_P][DBG] :" fmt, ##__VA_ARGS__)
#endif /* HTC_SYSCMD_HLOG_ENABLE */

enum {
	CMD_ID_SENSOR_FUSION = 0x00,
	CMD_ID_AUDIO_CONTROL,
	CMD_ID_DISPLAY_CONTROL,
	CMD_ID_ECOMPASS_CALIBRATION,
	CMD_ID_LED_CONTROL,
	CMD_ID_POWER_CONTROL,
	CMD_ID_LOG_CONTROL = 0x40,
	CMD_ID_SYSTEM_CONTROL,
};

int32_t system_command(uint8_t *, uint8_t);
void system_command_init(void);
int set_sys_cmd_serv_control(enum sys_cmd_service_state);
enum sys_cmd_service_state get_sys_cmd_serv_control(void);

#endif /* __SYSTEM_COMMAND_H__ */
