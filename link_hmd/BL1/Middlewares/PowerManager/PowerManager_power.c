#include <string.h>
#include "Stm32f4xx_hal.h"
#include "PowerManager_power.h"
#include "x_pmic.h"


void PWRMGR_SYSTEM_POWER_ENTER_STOP_MODE(void)
{

    printf("\r\nenter stop\n");
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);


}
extern void SystemClock_Config(void);
void PWRMGR_SYSTEM_POWER_EXIT_STOP_MODE(void)
{

    do
    {
        SystemClock_Config();
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    } while (0);
    printf("\r\nexit stop\n");

}
void PWRMGR_SYSTEM_POWER_STANDBY_MODE(void)
{
    HAL_PWR_EnterSTANDBYMode();
}

int PWRMGR_SYSTEM_POWER_RESET(int reason)
{
    __disable_irq();
    HAL_NVIC_SystemReset();
    return 0;
}

int PWRMGR_SYSTEM_POWER_POWEROFF(int reason)
{
    int i = 0;
    for(i=0;i<V_POWER_MAX;i++){
        bsp_pmic_power_enable((power_item_t)i, POWER_DISABLE);
    }
    bsp_pmic_power_off();
    return 0;
}
int PWRMGR_SYSTEM_POWER_POWERON(int reason)
{
    int i = 0;
    int ret = 0;
    for(i=0;i<V_POWER_MAX;i++){
        ret |= bsp_pmic_power_enable((power_item_t)i, POWER_ENABLE);
    }
    if(ret == 0){
        printf("pass\n");
    }
    else{
        printf("fail\n");
    }
    return 0;
}

int PWRMGR_state_change(int state)
{
    switch(state){
        case POWER_STATE_STOP:
            PWRMGR_SYSTEM_POWER_ENTER_STOP_MODE();

            PWRMGR_SYSTEM_POWER_EXIT_STOP_MODE();
            break;
        case POWER_STATE_REBOOT:
            PWRMGR_SYSTEM_POWER_RESET(1);
            break;
        case POWER_STATE_OFF:
            PWRMGR_SYSTEM_POWER_POWEROFF(1);
            break;
        case POWER_STATE_ON:
            PWRMGR_SYSTEM_POWER_POWERON(1);
            break;
    }
    return 0;
}
