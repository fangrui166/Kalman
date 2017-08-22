#ifndef __POWERMANAGER_STATE_MACHINE_H
#define __POWERMANAGER_STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif
typedef enum
{
    PWRMGR_NORMAL = 0,
    PWRMGR_LONG_KEY,
    PWRMGR_SHORT_KEY,
    PWRMGR_BATTERY_LOW,
    PWRMGR_STOP_TIMEOUT,
    PWRMGR_I2C_FAIL,
    PWRMGR_SPI_FAIL,
    PWRMGR_CANTSTOP,
    PWRMGR_SENSOR_FAIL,
    PWRMGR_CHANGE_BOOT_MODE,
    PWRMGR_FW_UPDATE,
    PWRMGR_MFG,
    PWRMGR_DFU,
    PWRMGR_SYSDFU,
    PWRMGR_USB_DATA_OUT,
    PWRMGR_WAIT_USBIN_TIMEOUT,
    PWRMGR_WAIT_VIDEO_TIMEOUT,
    PWRMGR_PSENSOR_AWAY,
    PWRMGR_RTC_UPDATE_SOC,
    PWRMGR_STOP_EXIT,
    PWRMGR_PQM_TEST,
    PWRMGR_MFG_TEST,
    PWRMGR_ENG_TEST,
} PWRMGR_POWEROFF_TYPE;

enum __pwrmgr_stat_machine_mode {
	PWRMGR_STAT_MACHINE_BOOTING_MODE = 0,
	PWRMGR_STAT_MACHINE_IDLE_MODE,
	PWRMGR_STAT_MACHINE_RUNNING_MODE,
	PWRMGR_STAT_MACHINE_SLEEP_MODE,
	PWRMGR_STAT_MACHINE_STOP_MODE,
	PWRMGR_STAT_MACHINE_REBOOT_MODE,
	PWRMGR_STAT_MACHINE_POWEROFF_MODE,
	/* below item should be put at last */
	PWRMGR_STAT_MACHINE_MODE_LAST,
};
extern const char *__pwrmgr_stat_machine_mode_string[];

int pwrmgr_state_machine_init(void);
int pwrmgr_state_machine_set(enum __pwrmgr_stat_machine_mode, int reason);
void pwrmgr_state_machine_get(enum __pwrmgr_stat_machine_mode *);
enum __pwrmgr_stat_machine_mode pwrmgr_state_going_get(void);


#ifdef __cplusplus
}
#endif

#endif /* __POWERMANAGER_STATE_MACHINE_H */
