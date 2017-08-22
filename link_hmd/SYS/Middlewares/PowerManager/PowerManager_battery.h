#ifndef __POWERMANAGER_BATTERY_H
#define __POWERMANAGER_BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PowerManager.h"
#include "x_pmic.h"

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/


#define PWRMGR_BATTERY_NORMAL_POLLING_CYCLE    (configTICK_RATE_HZ * 60)


int PWRMGR_battery_initial_service(PmicDrvTypeDef *handle);
int PRMGR_battery_update_uiSoc();
TimerHandle_t get_battery_timer_handle();
int PRMGR_battery_get_uiSoc();

#ifdef __cplusplus
}
#endif

#endif /* __POWERMANAGER_BATTERY_H */
