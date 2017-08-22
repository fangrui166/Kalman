#ifndef __POWERMANAGER_H__
#define __POWERMANAGER_H__
#ifdef __cplusplus
 extern "C" {
#endif

//#include "misc_data.h"
#include "hlog_api.h"
//#include "component.h"
#include "FreeRTOS.h"
#include "timers.h"
/*
#include "PowerManager_power.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_system.h"
#include "PowerManager_battery.h"
#include "PowerManager_peripherals.h"
*/

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

struct pwrmgr_battery_info
{
    uint32_t            battery_voltage;      // mV
    int			     battery_current;
    int                     battery_temperature;  // ¡æ
    int                     battery_raw_soc;     // x%
};

enum charging_status
{
	CHARGER_STATE_DISCHARGING = 0,
	CHARGER_STATE_PRECHARGING,
	CHARGER_STATE_FASTCHARGING,
	CHARGER_STATE_DONE,
};

#define PWRMGR_HLOG_ENABLE 1

#if PWRMGR_HLOG_ENABLE
#define pwrmgr_emerg(fmt, ...) \
    hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pwrmgr_err(fmt, ...) \
    hlog_printf(HLOG_LVL_ERR, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pwrmgr_warning(fmt, ...) \
    hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pwrmgr_info(fmt, ...) \
    hlog_printf(HLOG_LVL_INFO, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pwrmgr_debug(fmt, ...) \
    hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#else /* PWRMGR_HLOG_ENABLE */
#define pwrmgr_emerg(fmt, ...) \
    printf("[PWRMGR][EMR] :" fmt, ##__VA_ARGS__)
#define pwrmgr_err(fmt, ...) \
    printf("[PWRMGR][ERR] :" fmt, ##__VA_ARGS__)
#define pwrmgr_warning(fmt, ...) \
    printf("[PWRMGR][WARN]:" fmt, ##__VA_ARGS__)
#define pwrmgr_info(fmt, ...) \
    printf("[PWRMGR][INFO]:" fmt, ##__VA_ARGS__)
#define pwrmgr_debug(fmt, ...) \
    printf("[PWRMGR][DBG] :" fmt, ##__VA_ARGS__)
#endif /* PWRMGR_HLOG_ENABLE */


extern char boot_type[16];
int PWRMGR_initial(void);


#ifdef __cplusplus
}
#endif

#endif //__POWERMANAGER_H__



