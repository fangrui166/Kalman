#ifndef __POWERMANAGER_POWER_H
#define __POWERMANAGER_POWER_H

#ifdef __cplusplus
extern "C" {
#endif


typedef enum{
    POWER_STATE_STOP,
    POWER_STATE_REBOOT,
    POWER_STATE_OFF,
    POWER_STATE_ON,
}power_state_t;

int PWRMGR_state_change(int state);

#ifdef __cplusplus
}
#endif

#endif /* __POWERMANAGER_POWER_H */
