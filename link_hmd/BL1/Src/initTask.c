#include <stdio.h>
#include <string.h>
#include "task.h"
#include "init.h"
#include "initTask.h"
#include "stm32f4xx_hal.h"
#include "shellTask.h"
#include "dfuTask.h"
#include "GLOBAL.h"
#include "htc_memory_define.h"
#include "misc_data.h"
#include "x_pmic.h"
#include "gpio.h"
#include "hlog.h"


__no_init char boot_type[16]@0x20002000;

typedef void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t  JumpAddress;
extern IRQn_Type dbg_uart_irq_num;
extern pcbid_t pcb_id;


#pragma optimize=none
static void jump_and_execute(uint32_t address)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_DisableIRQ(dbg_uart_irq_num);
    /* Disable SysTick */
    SysTick->CTRL = 0x0;

    /* Disable all IRQ */
    __disable_irq();

    /* Jump code to external PSRAM to execut */
    JumpAddress = *(__IO uint32_t*) (address + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) address);
    Jump_To_Application();
}
void HAL_SYSTICK_Callback(void)
{
    tickcount ++;
    if(++sysTickfor10ms >= 10){
        sysTickfor10ms = 0;
        sys10msFlag = 1;
    }
}

TASK IdelTask(void)
{
    static uint8_t count = 0;
    bool state;
    int soc;
    PmicDrvTypeDef *handle;
    if(sys10msFlag){
        // TODO Add 10ms task at here
        sys10msFlag = 0;
        if(++sysTickfor100ms >= 10){
            sysTickfor100ms = 0;
            sys100msFlag = 1;
        }
    }
    if(sys100msFlag){
        // TODO add 100ms task at here
        sys100msFlag = 0;
        if(++sysTickfor1min >= 600){
            sysTickfor1min = 0;
	    sys1minFlag = 1;
            //printf("%s(%5d)\n",__func__,i++);
        }
    }
    if (sys1minFlag) {
	handle = bsp_pmic_get_handle();
	if (handle != NULL) {
	    if(XA0n == pcb_id){
    		state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0) ? 1 : 0;
	    }
        else{
            state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) ? 1 : 0;
        }
	    bsp_pmic_get_battery_soc(handle, &soc);
	    if (soc >= 50 && state == 1)
	        bsp_pmic_set_charger_enableDisable(handle, 0);
	    else if (soc < 30 && state == 1)
		bsp_pmic_set_charger_enableDisable(handle, 1);
	    //else
		 //printf("%s: battery do not need charge, or charger disconnected.\n", __func__);
        } else
            printf("%s: handler is NULL, cannot set charger state.\n", __func__);
	sys1minFlag = 0;
	if (count++ >= 10) {
	    count = 0;
	}
    }
}

int check_boot_reason(void)
{
    int ret = 0;
    printf("boot_type:%-16.16s\r\n",boot_type);
    if(!strncmp("hTC_Link_hmd", boot_type, strlen("hTC_Link_hmd"))){
        printf("System reboot by Watchdog , saving log ...\r\n");
        hlog_save_logbuf();
    }
    return ret;
}
static int isPowerKeyPress(void){
    if(XA0n == pcb_id){
        return (!HAL_GPIO_ReadPin(GPIO_PORT_POWER_KEY, GPIO_POWER_KEY));
    }
    else{
        return (!HAL_GPIO_ReadPin(GPIO_XB_PORT_POWER_KEY, GPIO_XB_POWER_KEY));
    }
}

int isUSBDataIn(void)
{
    if(pcb_id == XA0n){
        return !HAL_GPIO_ReadPin(GPIO_PORT_PHONE_DETC, GPIO_PHONE_DETC);
    }
    else if((pcb_id == XC01) || (XC02 == pcb_id)){
        return !HAL_GPIO_ReadPin(GPIO_XC_PORT_PHONE_DETC, GPIO_XC_PHONE_DETC);
    }
    else{
        return !HAL_GPIO_ReadPin(GPIO_XB_PORT_PHONE_DETC, GPIO_XB_PHONE_DETC);
    }
}

int isChargerIn(void)
{
#if 0
    if(pcb_id == XA0n){
        return !HAL_GPIO_ReadPin(GPIO_PORT_CHARGE_DETC, GPIO_CHARGE_DETC);
    }
    else{/* XB02, XC01*/
        return !HAL_GPIO_ReadPin(GPIO_XB_PORT_CHARGE_DETC, GPIO_XB_CHARGE_DETC);
    }
#else
    return bsp_pmic_get_pg_status();
#endif
}


TASKCFUNC(Init)
{
    BOOT_MODE mode=ERROR_MODE;
    if (!TaskInitFlag){
        printf("Link BL1 build @ %s - %s\r\n",__DATE__,__TIME__);
        printf("pcb_id:%#x\r\n",pcb_id);
        TaskInitFlag = 1;
    }
    mode = get_bootmode();

    if(mode == BL_MFG_MODE){
        printf("%s boot_type: %s\r\n",__func__, "BL1_mfg mode");
        Shell_init();
        CreatTask(&ShellTaskID, ShellTask);
        ChangeTask(0); // hang_up this task
        printf("CMD>");
        ChangeTask(ShellTaskID);//Start Shell task
    }else if(mode == DFU_MODE){
        printf("%s boot_type: %s\r\n",__func__, "BL1_dfu mode");
        set_bootmode(NORM_MODE);
        CreatTask(&DFUTaskID, DFUTask);
        ChangeTask(0); // hang_up this task
        ChangeTask(DFUTaskID);
    }else if(mode == FOTA_MODE){
        printf("%s boot_type: %s\r\n",__func__, "BL1_fota mode");
        CreatTask(&DFUTaskID, DFUTask);
        ChangeTask(0); // hang_up this task
        ChangeTask(DFUTaskID);
    }else{ // SYS_MFG_MODE,NORM_MODE
        /* normal bootup or reboot */
        check_boot_reason();
        if(isPowerKeyPress() ||\
            isUSBDataIn() ||\
            isChargerIn() ||\
            !strcmp(boot_type, "reboot") ||\
            !strcmp(boot_type, "DFU")
            ){
            jump_and_execute(IMAGE_START_FREERTOS);
        }
        else{
            printf("BL1 bootup_check failed\r\n");
            bsp_pmic_power_off();
        }
    }

}

