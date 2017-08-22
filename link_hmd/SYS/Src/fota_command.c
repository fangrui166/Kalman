/*************************************
*file_name: Flash_command.c
*description: fota for device
*lack:no
**************************************/

#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "string.h"
#include "fota_command.h"

typedef uint32_t    (*pFunCmd)(uint32_t argc, const char *argv[]);

typedef enum{
    FOTA_SET_NORMAL_MODE = 1,
    FOTA_SET_BOOTLOAD_MODE ,
    FOTA_GET_DEVICE_INFO,
    FOTA_GET_BATTERY_STATUS,
    FOTA_SET_FIRMWARE_NUMBER,
}FOTA_COMMAND;

typedef struct{
	FOTA_COMMAND cmd;
	const char *help;
	pFunCmd cmdParser;
}fota_cmd_data;



static uint32_t set_runtype_normal(uint32_t argc, const char *argv[]);
static uint32_t set_runtype_fota(uint32_t argc, const char *argv[]);
static uint32_t get_device_info(uint32_t argc, const char *argv[]);
static uint32_t get_battery_state(uint32_t argc, const char *argv[]);


const fota_cmd_data fotaCmdTable[] = {
		{
			FOTA_SET_NORMAL_MODE,
			"format :set runtype ival. ival = 0, for normal reboot; ival = 1, for fota reboot.",
			set_runtype_normal
		},
		{
			FOTA_SET_BOOTLOAD_MODE,
			"format :set runtype ival. ival = 0, for normal reboot; ival = 1, for fota reboot.",
			set_runtype_fota
		},
		{
			FOTA_GET_DEVICE_INFO,
			"format : getdevinfo",
			get_device_info
		},
             {
                    FOTA_GET_BATTERY_STATUS,
                    "format : get battery state",
                    get_battery_state
             },
		/*{
			FOTA_CMD_HELP,
			"",
			fota_cmd_help
		}*/
};
static uint32_t set_runtype_fota(uint32_t argc, const char *argv[]){
	uint32_t runtype = 1;
       fota_info("reboot to fota mode.\r\n");
       fota_reboot(runtype);
	return 1;
}

static uint32_t set_runtype_normal(uint32_t argc, const char *argv[]){
	uint32_t runtype = 0;
	fota_info("reboot to normal mode .\r\n");
       fota_reboot(runtype);
	return 1;
}
static uint32_t get_device_info(uint32_t argc, const char *argv[]){
    char buf[32] = {0};
    int  buf_firm = getFirmwareVersion();
    fota_info("FirmwareVersion=%d\n", buf_firm);
    sprintf(buf, "%d", buf_firm);
    usb_cdc_transmit_data((uint8_t*)buf,strlen(buf));
    return 1;
}

static uint32_t get_battery_state(uint32_t argc, const char *argv[]){
    char buf[32] = {0};
    int buf_state =  PRMGR_battery_get_uiSoc();
    fota_info("vivianbattery=%d\n", buf_state);
    
    sprintf(buf, "%d", buf_state);
    usb_cdc_transmit_data((uint8_t*)buf, strlen(buf));
    return 1;
}

void fota_reboot (uint32_t runtype){
    int reboot_reason = 0;
    if(runtype == 1){
        fota_info("set to fota mode, then system reset.\r\n");
        set_bootmode(DFU_MODE);
        reboot_reason = PWRMGR_DFU;

    }else if (runtype == 0){
       fota_info("set to normal mode , then system reset.\r\n");
       set_bootmode(NORM_MODE);
       reboot_reason = PWRMGR_ENG_TEST; 
    }else {
        fota_err("the runtype status is not handle.\r\n");
    }
    pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_REBOOT_MODE, reboot_reason);
}
static uint32_t fota_cmd_parser(uint8_t * cmdline){

    switch (cmdline[0]){
        case FOTA_SET_NORMAL_MODE:
            set_runtype_normal(0,0);
            break;
        case FOTA_SET_BOOTLOAD_MODE:
            set_runtype_fota(0,0);
            break;
        case FOTA_GET_DEVICE_INFO:
            get_device_info(0,0);
            break;
        case FOTA_GET_BATTERY_STATUS:
            get_battery_state(0,0);
            break;
        default:
            fota_err("the cmd is not analysis.\r\n");
            break;
        }
    return 0;


}


int32_t fota_ProcessRcvData(uint8_t * buf, uint8_t len){
    if(buf == NULL)
    {
        return -1;
    }

    uint8_t cmd_data[8] = {0};
	cmd_data[0] = buf[2];
	fota_debug("this buf is %x \n", cmd_data[0]);
    fota_cmd_parser(cmd_data);
    return 0;
}

void fota_init(){
  fota_debug("entry to Fota command\n");
}

