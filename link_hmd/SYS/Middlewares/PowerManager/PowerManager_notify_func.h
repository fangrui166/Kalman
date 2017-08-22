#ifndef __POWERMANAGER_NOTIFY_H
#define __POWERMANAGER_NOTIFY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PowerManager.h"
#include "PowerManager_state_machine.h"

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

typedef enum {
    PWRMGR_FUNC_MIN_LEVEL = 0,
    /* for core CPU */
    PWRMGR_FUNC_CORE_LEVEL = 10,
    /* for CPU interface */
    PWRMGR_FUNC_CORE_INTERFACE_LEVEL = 20,
    /* for Peripheral Driver */
    PWRMGR_FUNC_DRIVER_LEVEL = 30,
    /* for Applicatioon */
    PWRMGR_FUNC_APP_LEVEL = 40,
    PWRMGR_FUNC_MAX_LEVEL = 255,
}__pwrmgr_func_level;

typedef enum
{
    PWRMGR_NOTIFY_BATTERY_STATE     = (1 << 0),
    PWRMGR_NOTIFY_CHARGER_STATE     = (1 << 1),
    PWRMGR_NOTIFY_OVERHEAT_STATE    = (1 << 2),
    PWRMGR_NOTIFY_STOP_STATE        = (1 << 3),
    PWRMGR_NOTIFY_POWER_OFF         = (1 << 4),
    PWRMGR_NOTIFY_SYSTEM_RESET      = (1 << 5),
    PWRMGR_NOTIFY_FACTORY_RESET     = (1 << 6),
    PWRMGR_NOTIFY_DISPLAY_STATE     = (1 << 7),
    PWRMGR_NOTIFY_AUDIO_STATE       = (1 << 8),
    PWRMGR_NOTIFY_USB_HUB_OAC       = (1 << 9),
    PWRMGR_NOTIFY_USB_DP_HOLE       = (1 << 10),
    PWRMGR_NOTIFY_UNKNOWN           = (1 << 11)
} PWRMGR_NOTIFY_FLAG;
typedef enum
{
    PLUG_OUT = 0, PLUG_IN
} PWRMGR_DP_HOLE_STATE;

typedef enum
{
    STOP_ENTER = 0, STOP_LEAVE
} PWRMGR_STOP_STATE;
typedef enum
{
    NOTIFY_NORMAL = 0, NOTIFY_AFTER_ACK
} PWRMGR_NOTIFY_TYPE;

typedef enum{
    LEVER_DESCEND_ORDER = 0,
    LEVER_ASCEND_ORDER
}PWRMGR_NOTIFY_LEVEL_ORDER;

#define PWRMGR_NOTIFY_NEED_ACK(x) ((x) & (PWRMGR_NOTIFY_POWER_OFF | PWRMGR_NOTIFY_SYSTEM_RESET | PWRMGR_NOTIFY_FACTORY_RESET))
#define IS_PWRMGR_NOTIFY(x)       ((x) & (PWRMGR_NOTIFY_UNKNOWN - 1))

#define PWRMGR_POWER_OFF_NORMAL_TIMEOUT      (10)    // unit: sec
#define PWRMGR_POWER_OFF_ABNORMAL_TIMEOUT    (30)    // unit: sec

/* for Driver use, these structure should be fill by driver owner */
struct pwrmgr_notify_func_data {
    const char *func_name;
    void *data;
    uint32_t                notify_flag;
    uint32_t                notify_flag_need_ack;
    PWRMGR_NOTIFY_TYPE      notify_type;
    int (*callback)(uint32_t _notify_flag, uint32_t _state, void *);
    __pwrmgr_func_level func_level;
};

int PWRMGR_register_notify_func(struct pwrmgr_notify_func_data *);
int PWRMGR_unregister_notify_func(struct pwrmgr_notify_func_data *);
int PWRMGR_suspend_func_execute(void);
int PWRMGR_resume_func_execute(void);
void PWRMGR_dump_notify_func(void);
int PWRMGR_reboot_func_execute(PWRMGR_POWEROFF_TYPE _type);
int PWRMGR_shutdown_func_execute(PWRMGR_POWEROFF_TYPE _type);
int PWRMGR_SendNotifyAck (struct pwrmgr_notify_func_data *data);
int PWRMGR_SendNotify(uint32_t _notify_flag, uint32_t _state,
                                PWRMGR_NOTIFY_LEVEL_ORDER lever_order);

#ifdef __cplusplus
}
#endif

#endif /* __POWERMANAGER_SYSTEM_H */
