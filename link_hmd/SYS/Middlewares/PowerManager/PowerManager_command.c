#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "PowerManager_battery.h"
#include "PowerManager_command.h"
#include "PowerManager_power.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_system.h"
#include "misc_data.h"
#include "rtc_drv.h"
#include "htc_usb_cdc_data_service.h"
//#include "usbd_app.h"

osTimerDef_t reboot_timer_def;
osTimerId reboot_timer_id;
extern RTC_HandleTypeDef hrtc;

int PWRMGR_Wakeup(int argc, char *argv[], _command_source source)
{
    return 0;
}
int PWRMGR_SystemSleep(int argc, char *argv[], _command_source source)
{
    return pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_SLEEP_MODE,
          ((source == CMD_SOURCE_USBCDC)? PWRMGR_MFG_TEST : PWRMGR_ENG_TEST));
}
int PWRMGR_SystemStop(int argc, char *argv[], _command_source source)
{
    return pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_STOP_MODE,
        ((source == CMD_SOURCE_USBCDC)? PWRMGR_MFG_TEST : PWRMGR_ENG_TEST));
}

int PWRMGR_SystemPowerOff(int argc, char *argv[], _command_source source)
{
    return pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_POWEROFF_MODE,
        ((source == CMD_SOURCE_USBCDC)? PWRMGR_MFG_TEST : PWRMGR_ENG_TEST));
}

int PWRMGR_SystemReset(int argc, char *argv[], _command_source source)
{
    int reboot_reason = 0;
    if(argc == 2){
        if(!strcasecmp(argv[1], "board_mfg")){
            set_bootmode(BL_MFG_MODE);
            reboot_reason = PWRMGR_MFG;
        }
        else if(!strcasecmp(argv[1], "dfu")){
            set_bootmode(DFU_MODE);
            reboot_reason = PWRMGR_DFU;
        }
        else if(!strcasecmp(argv[1], "sysdfu")){
            reboot_reason = PWRMGR_SYSDFU;
        }
        else if(!strcasecmp(argv[1], "system_mfg")){
            set_bootmode(SYS_MFG_MODE);
            reboot_reason = PWRMGR_MFG;
        }
        else if(!strcasecmp(argv[1], "normal")){
            set_bootmode(NORM_MODE);
            reboot_reason = PWRMGR_ENG_TEST;
        }
		else
		{
			int count = strtoul((char const*)argv[1], NULL, 10);
			if(count > 0)
			{
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR19, count );
				reboot_reason = PWRMGR_MFG_TEST;
			}
		}
        if(source == CMD_SOURCE_USBCDC){
            usb_cdc_printf("%s","set_bootmode success\r\n");
        }

    }
    else{
        reboot_reason = PWRMGR_ENG_TEST;
    }
    return pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_REBOOT_MODE, reboot_reason);
}

static void stress_reboot_timer_callback(void const *argument)
{
	uint32_t count = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR19);
	if(count > 0)
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR19, count - 1 );
	/*__disable_irq();*/
	/* HAL_NVIC_SystemReset(); */
	/* change to powermanager API */
	pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_REBOOT_MODE, PWRMGR_PQM_TEST);
}

unsigned int myRandom(int start, int end)
{
	return start+(rand()%(end - start + 1));
}

void PWRMGR_Stress_Reboot(void)
{
	//reboot stress test
	int t;
	int seed;
	reboot_timer_def.ptimer = stress_reboot_timer_callback;
	reboot_timer_id = osTimerCreate(&reboot_timer_def, osTimerOnce, &t);
	uint8_t h=0,m=0,s=0;
	int tmp = 0;
	rtc_drv_getTime(&h,&m,&s);
	seed = h*3600+m*60+s;
	srand(seed);
	tmp = myRandom(5, 10);
	//reboot count
	shell_info("*********seed:%d,rand num is:%d, remain reboot count:%d *********\n", \
			seed,tmp, HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR19));
	osTimerStart(reboot_timer_id, tmp*1000);
}

void PWRMGR_reboot_test(void)
{
	//read reboot count value to determine start srand timer(to reboot) or not.
	if( HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR19) > 0)
		PWRMGR_Stress_Reboot();
}

int PWRMGR_DumpNotify(int argc, char *argv[], _command_source source)
{
    PWRMGR_dump_notify_func();
    return 0;
}
int notify_test_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
    switch(_notify_flag){
        case PWRMGR_NOTIFY_STOP_STATE:
            if(_state == STOP_ENTER){
                //shell_err("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
            }
            else if(_state == STOP_LEAVE){
                //shell_err("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
            }
            else{
                shell_err("not support the state!!!\n");
            }
            break;
        default:
            break;
    }
    return 0;
}
int PWRMGR_NotifyTest(int argc, char *argv[], _command_source source)
{
    struct pwrmgr_notify_func_data *PmNotifyData;
    if(argc < 3){
        shell_err("argc < 3\r\n");
        return -2;
    }
    PmNotifyData = pvPortMalloc(sizeof(struct pwrmgr_notify_func_data));
    if (PmNotifyData == NULL){
        shell_err("%s pvPortMalloc error\r\n",__func__);
        return -1;
    }
    PmNotifyData->func_name = argv[1];
    PmNotifyData->data = NULL;
    PmNotifyData->callback= notify_test_callback;
    PmNotifyData->notify_flag = PWRMGR_NOTIFY_STOP_STATE;
    PmNotifyData->func_level = (__pwrmgr_func_level)atoi(argv[2]);
    PWRMGR_register_notify_func(PmNotifyData);
    return 0;
}


int PWRMGR_GetMachineState(int argc, char *argv[], _command_source source)
{
    int ret = 0;
    enum __pwrmgr_stat_machine_mode t = PWRMGR_STAT_MACHINE_MODE_LAST;
    enum __pwrmgr_stat_machine_mode *m = &t;
    pwrmgr_state_machine_get(m);
    shell_info(" machine state: %d(%s)\r\n", *m, __pwrmgr_stat_machine_mode_string[*m]);

    return ret;
}

int PWRMGR_HELP(int argc, char *argv[], _command_source source);

const MONITOR_COMMAND PowercommandTable[] =
{
    {"wakeup",      PWRMGR_Wakeup},
    {"sleep",       PWRMGR_SystemSleep},
    {"stop",        PWRMGR_SystemStop},
    {"reboot",      PWRMGR_SystemReset},
    {"poweroff",    PWRMGR_SystemPowerOff},
    {"getstate",    PWRMGR_GetMachineState},
    {"notify",      PWRMGR_NotifyTest},
    {"dump",        PWRMGR_DumpNotify},

    {"?",           PWRMGR_HELP}, //This must be the last command
};
static const unsigned long ulNumberOfPowerCommands = (sizeof(PowercommandTable) / sizeof(PowercommandTable[0]));

int PWRMGR_HELP(int argc, char *argv[], _command_source source)
{
    int i = 0;

    shell_info("********** Power Command list**********\n");
    for (i=0; i<(ulNumberOfPowerCommands-1); i++)
    {
       shell_info("\t%s\r\n", PowercommandTable[i].command);
    }
    shell_info("**********end**********\n");
    return 0;
}

int PowerDoCommand(int argc, char **argv, _command_source source)
{
    unsigned int uiCount;

    // The first argument should be the command
    for (uiCount = 0; uiCount < ulNumberOfPowerCommands; uiCount++)
    {
        char resut_cmp = (strcmp((char const*)argv[1], (char const*)PowercommandTable[uiCount].command)== 0);
        if ( resut_cmp )
        {
            return(*(PowercommandTable[uiCount].pFunc))(argc-1, (char **)&argv[1], source);
        }
    }

     shell_err("Command error !!!\n");

    return -1;
}

