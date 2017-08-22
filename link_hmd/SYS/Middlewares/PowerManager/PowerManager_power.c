#include "FreeRTOS.h"
#include "semphr.h"
//#include "component.h"
#include "Stm32f4xx_hal.h"
#include "PowerManager.h"
#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_system.h"
#include <string.h>
#include "x_pmic.h"
#include "gpio.h"
#include "ccgx_ec.h"
#include "rtc_drv.h"

extern void MX_IWDG_Reinit_For_Suspend(void);
extern void MX_IWDG_Init(void);
extern pcbid_t pcb_id;

static volatile uint8_t is_wakeup_by_rtc = 0;
volatile uint32_t wakeup_source = 0;
extern void watchdog_refresh();
void SystemClock_Config_LowPower(void)
{
    #if 0
    /* this config HCLK as 16MHz */
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  __HAL_RCC_HSI_DISABLE();

    #endif
}

/* Enables/Disables the Flash Power Down in Stop mode */
void PWRMGR_SYSTEM_ENTER_FLASH_POWER_ENABLE(uint32_t enable)
{
    if (enable)
        HAL_PWREx_DisableFlashPowerDown();
    else
        HAL_PWREx_EnableFlashPowerDown();
}

/* Return Voltage Scaling Range */
uint32_t PWRMGR_SYSTEM_GET_VOLTAGE_SCALING(void)
{
    return HAL_PWREx_GetVoltageRange();
}

/* Configures the main internal regulator output voltage */
int PWRMGR_SYSTEM_SET_VOLTAGE_SCALING(uint32_t volt_scaling)
{
    HAL_StatusTypeDef ret;
    ret = HAL_PWREx_ControlVoltageScaling(volt_scaling);
    if (ret != HAL_OK)
        return -(int)ret;

    return 0;
}
void PWRMGR_SYSTEM_RTC_WAKEUP(void)
{
    is_wakeup_by_rtc = 1;
    if(wakeup_source == 0){
        wakeup_source |= WAKEUP_SOURCE_RTC_TIMER;
    }
}

int isUSBDataIn(void)
{
#if USB_HOLE_DET_BY_CCG4
    if(DATA_PORT_CONNECT == get_port_status(DATA_PORT)){
        return 1;
    }
    else{
        return 0;
    }
#else
    if(pcb_id == XA0n){
        return !HAL_GPIO_ReadPin(GPIO_PORT_PHONE_DETC, GPIO_PHONE_DETC);
    }
    else if((pcb_id == XC01) || (pcb_id == XC02)){
        return !HAL_GPIO_ReadPin(GPIO_XC_PORT_PHONE_DETC, GPIO_XC_PHONE_DETC);
    }
    else{
        return !HAL_GPIO_ReadPin(GPIO_XB_PORT_PHONE_DETC, GPIO_XB_PHONE_DETC);
    }
#endif
}
int isChargerIn(void)
{
#if USB_HOLE_DET_BY_CCG4
    if(CHARGE_PORT_CONNECT == get_port_status(CHARGE_PORT)){
        return 1;
    }
    else{
        return 0;
    }
#else
    if(pcb_id == XA0n){
        return !HAL_GPIO_ReadPin(GPIO_PORT_CHARGE_DETC, GPIO_CHARGE_DETC);
    }
    else{/* XB02, XC01*/
        return !HAL_GPIO_ReadPin(GPIO_XB_PORT_CHARGE_DETC, GPIO_XB_CHARGE_DETC);
    }
#endif
}

int PWRMGR_SYSTEM_CAN_BE_POWER_OFF(void)
{
    if((pcb_id == XC01) || (XC02 == pcb_id)){
        if(isChargerIn()){
            return 0;
        }
    }
    else{
        if(isUSBDataIn() || isChargerIn()){
            return 0;
        }
    }
    return 1;
}

void PWRMGR_SYSTEM_POWER_SLEEP_MODE(void)
{
    static uint32_t sleep_duration = 0;
    SystemClock_Config_LowPower();
    HAL_SuspendTick();

    MX_IWDG_Reinit_For_Suspend();
    mcu_rtc_wakeUp_alarm_set_time(PWRMGR_RTC_KICK_WDG_PERIOD_S);
loop:
    watchdog_refresh();

    wakeup_source = 0;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
     HAL_PWR_EnterSLEEPMode(PWRMGR_INTER_LOPWR_REGULATOR_ON,PWRMGR_SLEEP_ENTRY_WFI);

    if(is_wakeup_by_rtc){
        is_wakeup_by_rtc = 0;
        sleep_duration ++;
        if((sleep_duration*PWRMGR_RTC_KICK_WDG_PERIOD_S)
            >= PWRMGR_STOP_DURATION){
            sleep_duration = 0;
            if(PWRMGR_SYSTEM_CAN_BE_POWER_OFF()){
                mcu_rtc_wakeUp_alarm_disable();
                MX_IWDG_Init();
                pwrmgr_system_send_message(PM_SYS_CMD_STOP_TIMEOUT, 0);

                return ;
            }
        }
        goto loop;
    }
    else{
        mcu_rtc_wakeUp_alarm_disable();
        MX_IWDG_Init();
        watchdog_refresh();
    }
    sleep_duration = 0;

}

void PWRMGR_SYSTEM_POWER_ENTER_STOP_MODE(void)
{
    static uint32_t stop_duration = 0;
    if(PWRMGR_STAT_MACHINE_STOP_MODE !=pwrmgr_state_going_get()) return ;
    MX_IWDG_Reinit_For_Suspend();
    mcu_rtc_wakeUp_alarm_set_time(PWRMGR_RTC_KICK_WDG_PERIOD_S);
loop:
    watchdog_refresh();

    wakeup_source = 0;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    HAL_PWR_EnterSTOPMode(PWRMGR_INTER_LOPWR_REGULATOR_ON, PWRMGR_STOP_ENTRY_WFI);

    if(is_wakeup_by_rtc){
        is_wakeup_by_rtc = 0;
        stop_duration ++;
        if((stop_duration*PWRMGR_RTC_KICK_WDG_PERIOD_S)
            >= PWRMGR_STOP_DURATION){
            stop_duration = 0;
            if(PWRMGR_SYSTEM_CAN_BE_POWER_OFF()){
                mcu_rtc_wakeUp_alarm_disable();
                MX_IWDG_Init();
                pwrmgr_system_send_message(PM_SYS_CMD_STOP_TIMEOUT, 0);

                return ;
            }
        }
        goto loop;
    }
    else{
        mcu_rtc_wakeUp_alarm_disable();
        MX_IWDG_Init();
    }
    stop_duration = 0;

}
extern void SystemClock_Config(void);
void PWRMGR_SYSTEM_POWER_EXIT_STOP_MODE(void)
{
    if(PWRMGR_STAT_MACHINE_STOP_MODE !=pwrmgr_state_going_get()) return ;

    do
    {
        SystemClock_Config();
        watchdog_refresh();
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    } while (0);

}
void PWRMGR_SYSTEM_POWER_EXIT_SLEEP_MODE(void)
{

    do
    {
        SystemClock_Config();
        watchdog_refresh();
        HAL_ResumeTick();
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        #ifdef HAL_IWDG_MODULE_ENABLED
        HAL_Delay(100);
        #endif
    } while (0);

}

void PWRMGR_SYSTEM_POWER_STANDBY_MODE(void)
{
    HAL_PWR_EnterSTANDBYMode();
}

__no_init char boot_type[16]@0x20002000;

int PWRMGR_SYSTEM_POWER_RESET(int reason)
{
    PWRMGR_reboot_func_execute((PWRMGR_POWEROFF_TYPE)reason);
    memset(boot_type, 0, sizeof(boot_type));
    switch(reason){
        case PWRMGR_MFG:
            memcpy(boot_type, "mfg", strlen("mfg"));
            break;
        case PWRMGR_DFU:
            memcpy(boot_type, "DFU", strlen("DFU"));
            break;
		case PWRMGR_SYSDFU:
            memcpy(boot_type, "SYSDFU", strlen("SYSDFU"));
            break;
        case PWRMGR_ENG_TEST:
        default:
            memcpy(boot_type, "reboot", strlen("reboot"));
            break;

    }
    __disable_irq();
    HAL_NVIC_SystemReset();
    return 0;
}

int PWRMGR_SYSTEM_POWER_POWEROFF(int reason)
{
    PWRMGR_shutdown_func_execute((PWRMGR_POWEROFF_TYPE)reason);
    memset(boot_type, 0, sizeof(boot_type));
    memcpy(boot_type, "poweroff", strlen("poweroff"));
    if(XA0n == pcb_id){
        while(!HAL_GPIO_ReadPin(GPIO_PORT_POWER_KEY, GPIO_POWER_KEY)); // Wate power key release.
    }
    else{
        while(!HAL_GPIO_ReadPin(GPIO_XB_PORT_POWER_KEY, GPIO_XB_POWER_KEY)); // Wate power key release.
    }
    bsp_pmic_power_off();
    return 0;
}
