#include "FreeRTOS.h"
#include "timers.h"

#include "string.h"
#include "stdio.h"
//#include "component.h"

#include "PowerManager.h"
#include "PowerManager_battery.h"
#include "PowerManager_power.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_peripherals.h"
#include "PowerManager_system.h"
#include "x_pmic.h"

int PWRMGR_initial(void)
{
    PmicDrvTypeDef *pmic_handle = NULL;
    pwrmgr_state_machine_init();
    pmic_handle = bsp_pmic_get_handle();
    if(pmic_handle == NULL){
        pwrmgr_err("%s get pmic handle error\n",__func__);
    }
    else{
        PWRMGR_battery_initial_service(pmic_handle);
    }
    pwrmgr_system_init();
    PWRMGR_peripherals_init();
    pwrmgr_debug("%s: done\n", __func__);
    return 0;
}
