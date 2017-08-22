#ifndef __POWERMANAGER_SYSTEM_H
#define __POWERMANAGER_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PowerManager.h"

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

#define WAITE_USB_DATA_IN_TIMEOUT   (10*60*1000)
#define WAITE_VIDEO_OUTPUT_TIMEOUT   (10*60*1000)

typedef enum{
    PM_SYS_CMD_SYS_WAKEUP,
    PM_SYS_CMD_USB_DATA_OUT,
    PM_SYS_CMD_USB_DATA_IN,
    PM_SYS_CMD_STOP_TIMEOUT,
    PM_SYS_CMD_PSENSOR_DET,
    PM_SYS_CMD_NO_USB_TIMEOUT,
    PM_SYS_CMD_LOW_BATTERY,
    PM_SYS_CMD_RTC_UPDATE_SOC,
    PM_SYS_CMD_POWERKEY_LONG_PRESS,
    PM_SYS_CMD_POWERKEY_SHORT_PRESS,
    PM_SYS_CMD_WAIT_VIDEO_TIMEOUT,
    PM_SYS_CMD_DP_SYNC_STATE,
}PM_SYS_CMD_TYPE;

typedef enum{
    DP_SYNC_LOST,DP_SYNC_DET,
}DP_SYNC_STATE_T;
typedef struct
{
    PM_SYS_CMD_TYPE cmd;
    uint8_t state;
}PmSysMsgQueue_t;

int pwrmgr_system_send_message(PM_SYS_CMD_TYPE cmd, uint8_t state);
int pwrmgr_system_init(void);
void phone_vbus_det_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWERMANAGER_SYSTEM_H */
