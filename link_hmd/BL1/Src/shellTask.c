#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "shellTask.h"
#include "PowerManager_power.h"
#include "mfg_console.h"
#include "i2c_drv.h"
#include "misc_data.h"
#include "ccgx_fw_io.h"
#include "ccgx_fw.h"
#include "x_pmic.h"
#include "gpio_exp.h"
#include "audio_alc4040_driver.h"
#include "audio_alc5665_driver.h"
#include "htc_audio_path_service.h"
#include "BMI160_ACC_GYRO_Driver.h"
#include "display_drv.h"

typedef struct{
    uint32_t lenght;
    char buf[CMD_BUF_LEN];
}CMDRxBuf;
CMDRxBuf CMDCacheBuf[CMD_CACHE_LEN] = {0};
uint8_t CMD_Cache_Head, CMD_Cache_Tail;
char *g_mfg_tag_names[MFG_TAG_MAX] = {
	[MFG_TAG_FLASH] = "[FLASH]",
	[MFG_TAG_SENSOR] = "[SENSOR]",
	[MFG_TAG_PMIC] = "[PMIC]",
	[MFG_TAG_RTC] = "[RTC]",
	[MFG_TAG_LED] = "[LED]",
	[MFG_TAG_BT] = "[BT]",
	[MFG_TAG_USB] = "[USB]",
	[MFG_TAG_BUTTON] = "[BTN]",
	[MFG_TAG_TRACKPAD] = "[TRKP]",
	[MFG_TAG_APP] = "[APP]",
	[MFG_TAG_NTF] = "[NTF]",
	[MFG_TAG_ALARM] = "[ALARM]",
	[MFG_TAG_PWRMGR] = "[PWRMGR]",
	[MFG_TAG_SYNCSRV] = "[SYNCSRV]",
	[MFG_TAG_MISC] = "[MISC]",
	[MFG_TAG_COMMON] = "[COMMON]",
	[MFG_TAG_CCG4] = "[CCG4]",
	[MFG_TAG_AUDIO] = "[AUDIO]",
	[MFG_TAG_MUX] = "[MUX]"
};
const unsigned char whiteSpace[] = {' ', '\t', '=', '\r', '\n'};

__weak int get_task_state(int argc, char *argv[], _command_source source)
{
    int i;

    for (i = 0; i < argc; i++)
    {
        printf("argv[%d]:%s\r\n",i,(char const*)argv[i]);
    }

    return 0;
}

unsigned long my_strtoul (char *data)
{
	int base = 10;
	if (data[0] == '0' && (data[1] == 'x' || data[1] == 'X'))
		base = 16;

	return strtoul(data, NULL, base);
}

void show_led_help()
{
	printf("LED Usage:\n \
		led	[R/G][on/off] --> led R on: will turn on red led\n \
		led	?\n");
}

#define portMAX_DELAY  	        0xffff
#define LP5562_REG_ENABLE		0x00
#define LP5562_REG_B_PWM		0x02
#define LP5562_REG_G_PWM		0x03
#define LP5562_REG_CONFIG		0x08
#define LP5562_REG_ENG_SEL		0x70
#define LP5562_ENG_SEL_PWM		0
#define LP5562_PWRSAVE_EN		0x20
int led_mfg_cmd(int argc, char * argv[], _command_source source)
{
	int ret = 0;
	/* chip enable */
	uint8_t enable = 0;
	RTOS_I2C_ReadBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_ENABLE, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
	if(!enable)
	{
		enable = 0x40;
		RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_ENABLE, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
		HAL_Delay(1);
		enable = LP5562_PWRSAVE_EN | 0x01;
		RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
		enable = LP5562_ENG_SEL_PWM;
		RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_ENG_SEL, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
	}

	if (argc == 2)
	{
		if (!strcmp(argv[1], "?"))
		{
			show_led_help();
		}else
			show_led_help();
	}
	else if (argc == 3)
	{
		if (!strcmp(argv[1], "R"))
		{
			if (!strcmp(argv[2], "on"))
			{
				/* enable = 0x0; */
				/* RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_G_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY); */
				enable = 0x3f;
				RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_B_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
			}
			else if (!strcmp(argv[2], "off"))
			{
				enable = 0x0;
				RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_B_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
			}
		}
		else if (!strcmp(argv[1], "G"))
		{
			if (!strcmp(argv[2], "on"))
			{
				/* enable = 0x0; */
				/* RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_B_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY); */
				enable = 0x3f;
				RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_G_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
			}
			else if (!strcmp(argv[2], "off"))
			{
				enable = 0x0;
				RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, LP5562_REG_G_PWM, I2C_MEMADD_SIZE_8BIT, &enable, 1, portMAX_DELAY);
			}
		}
		else
			show_led_help();
	}
	else
		show_led_help();
	return ret;
}

void show_i2c_help()
{
	printf("I2C Usage:\n \
		i2c devices --> dump i2c devices\n \
		i2c read_reg8 0x[dev_addr] 0x[reg] [len] --> register is 8bits\n \
		i2c read_reg16 0x[dev_addr] 0x[reg] [len] --> register is 16bits\n \
		i2c write_reg8 0x[dev_addr] 0x[reg] [len] 0x[byte]... --> register is 8bits\n \
		i2c write_reg16 0x[dev_addr] 0x[reg] [len] 0x[byte]... --> register is 16bits\n \
		i2c ?\n");
}

#define I2C_RW_MAX_LENGTH 16
int i2c_mfg_cmd(int argc, char * argv[], _command_source source)
{
	int ret = 0;
	if (argc == 2)
	{
		if (!strcmp(argv[1], "devices"))
		{
			/* shell_info("I2C devices: %s, Address:0x%0x\n", I2C_DEVICE_TOUCH_NAME, I2C_DEVICE_TOUCH_ADDR); */
			/* shell_info("I2C devices: %s, Address:0x%0x\n", I2C_DEVICE_COMPASS_NAME, I2C_DEVICE_COMPASS_ADDR); */
		}
		else if (!strcmp(argv[1], "?"))
		{
			show_i2c_help();
		}else
			show_i2c_help();
	}
	else if (argc >= 4)
	{
		/* addr = 0x, reg = 0x, value = 0x */
		uint32_t addr = (uint32_t)my_strtoul(argv[2]) & 0x0ff;
		if(I2C_GetHandleByAddr(addr | I2C1_FLAG))
			addr |= I2C1_FLAG;
		else if(I2C_GetHandleByAddr(addr | I2C2_FLAG))
			addr |= I2C2_FLAG;

		uint16_t reg = (uint16_t)my_strtoul(argv[3]);
		uint8_t rx_buf[I2C_RW_MAX_LENGTH] = {0};
		uint8_t len = (uint16_t)my_strtoul(argv[4]);
		if(len > I2C_RW_MAX_LENGTH)
			len = I2C_RW_MAX_LENGTH;
		else if (len < 1)
			len = 1;

		if(!strcmp(argv[1], "read_reg8")){
			ret = RTOS_I2C_ReadBuffer(addr, reg, I2C_MEMADD_SIZE_8BIT, &rx_buf[0], len, portMAX_DELAY);
			if(ret == I2C_OK) {
				printf("I2C read device(address:0x%0x) reg(0x%02x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", rx_buf[i]);
				}
				printf("\r\n");
			}
			else
				printf("I2C read device(address:0x%0x) reg(0x%02x) fail.\r\n",(uint8_t)addr, reg);
			return ret;
		}else if(!strcmp(argv[1], "read_reg16")){
			ret = RTOS_I2C_ReadBuffer(addr, reg, I2C_MEMADD_SIZE_16BIT, &rx_buf[0], len, portMAX_DELAY);
			if(ret == I2C_OK){
				printf("I2C read device(address:0x%0x) reg(0x%04x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", rx_buf[i]);
				}
				printf("\r\n");
			}
			else
				printf("I2C read device(address:0x%0x) reg(0x%04x) fail.\r\n",(uint8_t)addr, reg);
			return ret;
		}else if(!strcmp(argv[1], "write_reg8")){
			uint8_t value[I2C_RW_MAX_LENGTH] = {0};
			for(int i=0; i<len; i++){
				value[i] = (uint8_t)my_strtoul(argv[5+i]);
			}
			ret = RTOS_I2C_WriteBuffer(addr, reg, I2C_MEMADD_SIZE_8BIT, &value[0], len, portMAX_DELAY);
			if(ret == I2C_OK){
				printf("I2C write device(address:0x%0x) reg(0x%02x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0;i < len; i++){
					printf("0x%02x ", value[i]);
				}
				printf("\r\n");
			}
			else
				printf("I2C write device(address:0x%0x) reg(0x%02x) fail.\r\n",(uint8_t)addr, reg);
		}else if(!strcmp(argv[1], "write_reg16")){
			uint8_t value[I2C_RW_MAX_LENGTH] = {0};
			for(int i=0; i<len; i++){
				value[i] = (uint8_t)my_strtoul(argv[5+i]);
			}
#if 0
			uint8_t value[2] = {
			/* high 8bits */
					(uint8_t)my_strtoul(argv[4]),
			/* low 8bits */
					(uint8_t)my_strtoul(argv[5])
			};
#endif
			ret = RTOS_I2C_WriteBuffer(addr, reg, I2C_MEMADD_SIZE_16BIT, &value[0], len, portMAX_DELAY);
			/* uint16_t temp = ((uint16_t)value[0] << 8 | value[1]); */
			if(ret == I2C_OK){
				printf("I2C write device(address:0x%0x) reg(0x%04x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", value[i]);
				}
				printf("\r\n");
			}
			else
				printf("I2C write device(address:0x%0x) reg(0x%04x) fail.\r\n",(uint8_t)addr, reg);
		}else
			show_i2c_help();

	}else
		show_i2c_help();

	return 0;
}

int power_mfg_cmd(int argc, char * argv[], _command_source source)
{
    if(argc == 2){
        if(!strncmp("stop", argv[1], 4)){
            PWRMGR_state_change(POWER_STATE_STOP);
        }
        else if(!strncmp("reboot", argv[1], 6)){
            PWRMGR_state_change(POWER_STATE_REBOOT);
        }
        else if(!strncmp("poweroff", argv[1], 8)){
            PWRMGR_state_change(POWER_STATE_OFF);
        }
        else if(!strncmp("poweron", argv[1], 7)){
            PWRMGR_state_change(POWER_STATE_ON);
        }
    }else if(argc == 3) {
        if(!strncmp("reboot", argv[1], 6)){
            if(!strcasecmp(argv[2], "board_mfg")){
                set_bootmode(BL_MFG_MODE);
            }
            else if(!strcasecmp(argv[2], "dfu")){
                set_bootmode(DFU_MODE);
            }
            else if(!strcasecmp(argv[2], "sysdfu")){
                ;//reboot_reason = PWRMGR_RESET_SYSDFU;
            }
            else if(!strcasecmp(argv[2], "system_mfg")){
                set_bootmode(SYS_MFG_MODE);
            }
            else if(!strcasecmp(argv[2], "normal")){
                set_bootmode(NORM_MODE);
            }

            PWRMGR_state_change(POWER_STATE_REBOOT);
        }

	}
    return 0;
}

int cmd_misc_data(int argc, char * argv[], _command_source source)
{
	char dec[MISC_PARAMETER_SIZE]={0};
	pcbid_t pcbid;
	int value;
	int32_t para_int32;
	int result = 0;
	if(argc == 3 && !strcmp((char const*)argv[1], "get")){
		if (!strcmp((char const*)argv[2], "customid")){
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getCustomID(dec);
			misc_info("CustomID:%s\r\n",dec);
		}
		else if (!strcmp((char const*)argv[2], "skuid"))
		{
			get_skubid(&value);
			misc_info("SkuID:0x%x\r\n",value);
		}
		else if (!strcmp((char const*)argv[2], "SerialNum"))
		{
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getSerialNumber(dec);
			misc_info("SerialNumber:%s\r\n",dec);
		}
		else if (!strcmp((char const*)argv[2], "pcbid"))
		{
			get_pcbid(&pcbid);
			misc_info("pcbid:0x%x\r\n",pcbid);
		}
		else if (!strcmp((char const*)argv[2], "MBSerialNumber"))
		{
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getMBSerialNumber(dec);
			misc_info("MBSerialNumber:%s\r\n",dec);
		}
		else if (!strcmp((char const*)argv[2], "colorid"))
		{
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getColorID(dec);
			misc_info("ColorID:%s\r\n",dec);
		}
		else if (!strcmp((char const*)argv[2], "projectid"))
		{
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getProjectID(dec);
			misc_info("ProjectID:%s\r\n",dec);
		}
		else if (!strcmp((char const*)argv[2], "FirmMainVer"))
		{
			memset(dec,'\0',MISC_PARAMETER_SIZE);
			getFirmwareMainVersion(dec);
			misc_info("FirmwareMainVersion:%s\n",dec);
		}
		else
		{
			misc_err("Input arguments error.\r\n");
			return -1;
		}

	}
	else if(argc == 4 && !strcmp((char const*)argv[1], "set")) {
		if (!strcmp((char const*)argv[2], "customid")){
			result = setCustomID((const char *)argv[3],
					strlen((char *)argv[3]));
			if(0 == result) {
				misc_info("setCustomID success\n");
			}else if(-2 == result) {
				misc_err("The paramater overflow\n");
			}
		}
		else if (!strcmp((char const*)argv[2], "skuid"))
		{
			value = strtoul((char const*)argv[3], NULL, 16);
			misc_info("value is 0x%x\n", value);
			if (set_skuid((int)value) < 0) {
				misc_err("set sku id failed\n");
				return -1;
			}
			else
				misc_info("set sku id success\n");
		}
		else if (!strcmp((char const*)argv[2], "SerialNum"))
		{
			result = setSerialNumber((const char *)argv[3],
					strlen((char *)argv[3]));
			if(0 == result) {
				misc_info("setSerialNumber success\n");
			}else if(-2 == result) {
				misc_err("The paramater overflow\n");
			}
		}
		else if (!strcmp((char const*)argv[2], "pcbid"))
		{
			para_int32 = strtoul((char const*)argv[3], NULL, 16);
			misc_info("para_int32 is 0x%x\n", para_int32);
			if (set_pcbid((pcbid_t)para_int32) < 0) {
				misc_err("set pcb id failed\n");
				return -1;
			}
			else
			{
				misc_info("set pcb id success\n");
			}
		}
		else if (!strcmp((char const*)argv[2], "MBSerialNumber"))
		{
			result = setMBSerialNumber((const char *)argv[3],
					strlen((char *)argv[3]));
			if(0 == result) {
				misc_info("setMBSerialNumber success\n");
			}else if(-2 == result) {
				misc_err("The paramater overflow\n");
			}
		}
		else if (!strcmp((char const*)argv[2], "colorid"))
		{
			result = setColorID((const char *)argv[3],
					strlen((char *)argv[3]));
			if(0 == result) {
				misc_info("setColorID success\n");
			}else if(-2 == result) {
				misc_err("The paramater overflow\n");
			}
		}
		else if (!strcmp((char const*)argv[2], "projectid"))
		{
			result = setProjectID((const char *)argv[3],
					strlen((char *)argv[3]));
			if(0 == result) {
				misc_info("setProjectID success\n");
			}else if(-2 == result) {
				misc_err("The paramater overflow\n");
			}
		}
		else
		{
			misc_err("Input arguments error.\r\n");
			return -1;
		}
	}
	else {
		misc_err("Input arguments error.\r\n");
		return -1;
	}
	return 0;
}

int mfg_example(int argc, char * argv[], _command_source source)
{
	int i=0;
	for( i=0;i<argc;i++){
		MFG_LOG_DEBUG(MFG_TAG_COMMON,"%s\r\n",argv[i]);
		MFG_LOG_INFO(MFG_TAG_COMMON,"%s\r\n",argv[i]);
		MFG_LOG_WARNING(MFG_TAG_COMMON,"%s\r\n",argv[i]);
		MFG_LOG_ERR(MFG_TAG_COMMON,"%s\r\n",argv[i]);
		MFG_LOG_PASS(MFG_TAG_COMMON,"%s\r\n",argv[i]);

	}
	return 0;
}

void disp_test_help(void)
{
	printf("*******DISP Command list******\r\n");
	printf("       mfg_disp h2d getid\r\n");
	printf("       mfg_disp d2h getid\r\n");
	printf("       mfg_disp power on\r\n");
	printf("       mfg_disp power off\r\n");
	printf("       mfg_disp test colorbar\r\n");
	printf("       mfg_disp stop colorbar\r\n");
	printf("       mfg_disp start x\r\n");
	printf("************end************\r\n");
}

int disp_test_cmd(int argc, char * argv[], _command_source source)
{
	uint16_t id = 0;
//	uint8_t b_val = 0;
    pcbid_t pcb_id;

	if (argc >= 3) {
		if (!strcmp(argv[1], "h2d")) {
			if (!strcmp(argv[2], "getid")) {
				if (disp_h2d_getid(&id) == 0) {
					printf("h2d chip id 0x%04x.\n", id);
				} else {
					printf("can't got h2d chip id.\n");
				}
				return 0;
			}
		}

		if (!strcmp(argv[1], "d2h")) {
			if (!strcmp(argv[2], "getid")) {
				uint8_t ids[6];
    			get_pcbid(&pcb_id);
				if ((XA0n == pcb_id) && (disp_d2h_getid(ids, 6) == 0)) {
					printf("d2h chip id 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x.\n",
							ids[0], ids[1], ids[2], ids[3], ids[4], ids[5]);
				} else {
					printf("can't got d2h chip id.\n");
				}
				return 0;
			}
		}

		if (!strcmp(argv[1], "power")) {
			if (!strcmp(argv[2], "on")) {
				disp_power_on();
				return 0;
			}
			if (!strcmp(argv[2], "off")) {
				disp_power_off();
				return 0;
			}
		}

		if (!strcmp(argv[1], "test")) {
			if (!strcmp(argv[2], "colorbar")) {
				disp_test_colorbar();
				return 0;
			}
		}

		if (!strcmp(argv[1], "stop")) {
			if (!strcmp(argv[2], "colorbar")) {
				disp_power_off();
				disp_power_on();
				return 0;
			}
		}

        if (!strcmp(argv[1], "start")) {
			if (!strcmp(argv[2], "x")) {
                disp_startx();
                return 0;
			}
        }

	}

	disp_test_help();
	return -1;
}

int ccg4_read_version(int argc, char * argv[], _command_source source)
{
	uint8_t rx_buf[3]={0};
	uint8_t major_version = 0;
	uint8_t minor_version = 0;
	uint8_t mini_version = 0;
	I2C_STATUS ret;
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_ADDR,0x00, I2C_MEMADD_SIZE_8BIT, &rx_buf[0],3, portMAX_DELAY);

	if(ret == I2C_OK) {
		//rx_buf[0] -->port status
		//rx_buf[1] -->version number
		major_version = (rx_buf[1]>>4)&0x0f;
		minor_version = rx_buf[1]&0x0f;
		mini_version = rx_buf[2];
		MFG_LOG_PASS(MFG_TAG_CCG4,"version:%d.%d.%d",major_version,minor_version,mini_version);
	}else{
		MFG_LOG_ERR(MFG_TAG_CCG4,"get ccg4 version fail");
	}
	return ret;

}
int mux_read_version(int argc, char * argv[], _command_source source)
{
	uint8_t rx_buf[1]={0};
	I2C_STATUS ret;
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_ADDR,0x03, I2C_MEMADD_SIZE_8BIT, &rx_buf[0],1, portMAX_DELAY);

	if(ret == I2C_OK) {
		MFG_LOG_PASS(MFG_TAG_MUX,"version:%d", rx_buf[0]);
	}else{
		MFG_LOG_ERR(MFG_TAG_MUX,"get mux version fail");
	}
	return ret;

}


static int cmd_mag(int argc, char * argv[], _command_source source)
{
	uint8_t yas_chipid;
	uint8_t chipid[2];
	uint16_t akm_chipid;
	char mag_msg[CMD_BUF_LEN] = { 0 };
	if (argc == 2){
		if (!strcmp(argv[1], "getID")){
			//command  get magnetic sensor chip ID
			/*read akm9916 magnetic sensor first*/
			BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
			com_rslt = bmi160_mag_init();
			com_rslt += bmi160_mag_manual_enable();
			com_rslt += bmi160_mag_read_reg(0x00,chipid, 2);
			akm_chipid = (chipid[1]<<8)|chipid[0];
			if((com_rslt == BMI160_SUCCESS)&&(akm_chipid == 0x0948))
			{
				snprintf((char *)&mag_msg,
							CMD_BUF_LEN - 1,
							"mag ak09916 chip id is 0x%04x\r\n",akm_chipid);
				MFG_LOG_PASS(MFG_TAG_SENSOR,"%s", mag_msg);
			}
			else{
				/*read yas537 sensor*/
				I2C_STATUS i2c_ret = I2C_OK;
				i2c_ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_ECOMPASS_ADDR, 0x80, I2C_MEMADD_SIZE_8BIT, &yas_chipid, 1, 0x1000);
				if((i2c_ret == I2C_OK)&&(yas_chipid == 0x07)){
					snprintf((char *)&mag_msg,
							CMD_BUF_LEN - 1,
							"mag yas537 chip id is 0x%02x\r\n",yas_chipid);
					MFG_LOG_PASS(MFG_TAG_SENSOR,"%s", mag_msg);
				}
				else{
					printf("get magnetic ID fail\r\n");
					MFG_LOG_ERR(MFG_TAG_SENSOR,"get magnetic ID fail\r\n");
					return -1;
				}
			}
		}
		else{
			printf("command error:argv %s not support\r\n",argv[1]);
			return -1;
		}
	}
	else{
		printf("command error:argc not match!argc=%d\r\n",argc);
		return -1;
	}
	return 0;
}


static void print_bootmode(BOOT_MODE mode)
{
	switch(mode){
		case FOTA_MODE:
			printf("current mode is fota mode\r\n");
			break;
		case NORM_MODE:
			printf("current mode is normal mode\r\n");
			break;
		case BL_MFG_MODE:
			printf("current mode is board level mfg mode\r\n");
			break;
		case SYS_MFG_MODE:
			printf("current mode is system level mfg mode\r\n");
			break;
		case DFU_MODE:
			printf("current mode is dfu mode\r\n");
			break;
		default:
			printf("current mode is error mode\r\n");
			break;

	}
}
void audio_test_usage(void) {
	printf(" audio command:\r\n"
		"\taudio set path [hdmi|usb|loopback]\r\n"
		"\taudio set power [on|off]\r\n"
		"\taudio set amp [on|off]\r\n"
		"\taudio set output [aj|ear]\r\n"
		"\taudio get power\r\n"
		"\taudio get amp\r\n"
		"\taudio get path\r\n"
		"\taudio get output\r\n"
		"\taudio get cal\r\n"
		"\taudio get chipid\r\n");
}

int audio_mfg_cmd(int argc, char * argv[], _command_source source)
{
	enum __audio_srv_path sr_path;
	unsigned char onoff;
	uint16_t chipid;
	int loop_i;
	uint16_t reg_data;
	char msg[CMD_BUF_LEN] = { 0 };
	if (argc == 3) {
		if (!strcmp(argv[1], "get")) {
			if(!strcmp(argv[2], "regall")) {
				for (loop_i = 0; loop_i < 0x0400; loop_i++) {
					audio_alc5665_i2c_read(loop_i,
								&reg_data);
					snprintf((char *)&msg,
						CMD_BUF_LEN - 1,
						"audio i2c: addr: 0x%4.4X, "
						"data: 0x%4.4X\r\n",
						loop_i, reg_data);
					MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				}
				return 0;
			} else if(!strcmp(argv[2], "cal")) {
				for (loop_i = 490; loop_i < 501; loop_i++) {
					audio_alc5665_i2c_read(loop_i,
								&reg_data);
					snprintf((char *)&msg,
						CMD_BUF_LEN - 1,
						"audio i2c: addr: 0x%4.4X, "
						"data: 0x%4.4X\r\n",
						loop_i, reg_data);
					MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				}
				return 0;
			} else if (!strcmp(argv[2], "path")) {
				audio_path_get(&sr_path);
				if (sr_path == AUDIO_SRV_HDMI_PATH)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"hdmi\r\n");
				else if (sr_path == AUDIO_SRV_USB_PATH)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"usb\r\n");
				else if (sr_path == AUDIO_SRV_LOOPBACK_PATH)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"loopback\r\n");
				else
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"Uknown\r\n");

				MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				return 0;
			} else if (!strcmp(argv[2], "power")) {
				audio_alc5665_audio_power_get(&onoff);
				if (onoff == 0)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"power off\r\n");
				else if (onoff == 1)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"power on\r\n");
				else
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"power unknown\r\n");

				MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				return 0;
			} else if (!strcmp(argv[2], "chipid")) {
				audio_alc5665_i2c_read(ALC5665_DEVICE_ID,
                                                       &chipid);
				if (chipid == ALC5665_6_8_DEVICE_ID)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"chipid:0x6451\r\n");
				else
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"unknown\r\n");

				MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				return 0;
			} else if (!strcmp(argv[2], "output")) {
				audio_path_get(&sr_path);
				if (sr_path == AUDIO_SRV_OUTPUT_TO_AJ)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"aj\r\n");
				else if (sr_path == AUDIO_SRV_OUTPUT_TO_EAR)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"ear\r\n");
				else if (sr_path == AUDIO_SRV_OUTPUT_TO_BOTH)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"both\r\n");
				else
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"Uknown\r\n");

				MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				return 0;
			} else if (!strcmp(argv[2], "amp")) {
				audio_alc5665_audio_amp_power_get(&onoff);
				if (onoff == 0)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"amp power off\r\n");
				else if (onoff == 1)
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"amp power on\r\n");
				else
					snprintf((char *)&msg,
							CMD_BUF_LEN - 1,
							"power unknown\r\n");

				MFG_LOG_PASS(MFG_TAG_AUDIO,"%s", msg);
				return 0;
			}
			audio_test_usage();
			return -1;
		}
	} else if (argc == 4) {
		if (!strcmp(argv[1], "set")) {
			if (!strcmp(argv[2], "path")) {
				if (!strcmp(argv[3], "usb")) {
					sr_path = AUDIO_SRV_USB_PATH;
				} else if (!strcmp(argv[3], "hdmi")) {
					sr_path = AUDIO_SRV_HDMI_PATH;
				} else if (!strcmp(argv[3], "loopback")) {
					sr_path = AUDIO_SRV_LOOPBACK_PATH;
				} else {
					audio_test_usage();
					return -1;
				}
				return audio_path_set(sr_path);
			} else if (!strcmp(argv[2], "power")) {
				if (!strcmp(argv[3], "on"))
					onoff = 1;
				else
					onoff = 0;
				return audio_alc5665_audio_power_set(onoff);
			} else if (!strcmp(argv[2], "output")) {
				if (!strcmp(argv[3], "aj"))
					sr_path = AUDIO_SRV_OUTPUT_TO_AJ;
				else if (!strcmp(argv[3], "ear"))
					sr_path = AUDIO_SRV_OUTPUT_TO_EAR;
				else {
					audio_test_usage();
					return -1;
				}
				return audio_path_set(sr_path);
			} else if (!strcmp(argv[2], "amp")) {
				if (!strcmp(argv[3], "on"))
					onoff = 1;
				else
					onoff = 0;
				return audio_alc5665_audio_amp_power_set(onoff);
			}
			audio_test_usage();
			return -1;
		}
	}
	audio_test_usage();
	return -1;
}
#define COMMAND_GSENSOR_DUMPREG "dumpreg"
#define COMMAND_GSENSOR_READCHIPID "readchipid"
#define COMMAND_GSESNOR_READACC "readacc"
#define COMMAND_GSENSOR_READGYRO "readgyro"
#define BMI160_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_16G  0.488 /**< Sensitivity value for 16g full scale [mg/LSB] */


#define BMI160_GYRO_SENSITIVITY_FOR_FS_125DPS   13.68	//0.0038   /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_250DPS   27.36 //0.0076  /**< Sensitivity value for 250 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_500DPS   54.72 //0.0153  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_1000DPS  109.44//0.0305  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_2000DPS  218.88//0.0610  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */
int gsensor_mfg_cmds(int argc, char * argv[], _command_source source)
{
	uint8_t i,pBuffer;
	float sensitivity;
	us8 fullScale;
	struct bmi160_accel_t accel = {0};
	struct bmi160_gyro_t gyro = {0};

	if (argc == 2) {
		if (!strcmp((char const*)argv[1], "?"))
		{
			printf("**********	Command list	**********\n");
			printf("%s\n%s\n%s\n%s\n", COMMAND_GSENSOR_DUMPREG, COMMAND_GSENSOR_READCHIPID,
									COMMAND_GSESNOR_READACC,COMMAND_GSENSOR_READGYRO);
			printf("**********end**********\n");
			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_DUMPREG))
		{
			for(i = 0x0;i<0x7E;i++)
			{
				bmi160_read_reg(i, &pBuffer, 1);
				printf("[BMI160] addr = 0x%x value = 0x%x\n",i,pBuffer);
				HAL_Delay(1);
			}
			return 0;
		}
		else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READCHIPID))
		{
			bmi160_read_reg(0x0,&pBuffer, 1);
			printf("[BMI160] chipid = 0x%x\n",pBuffer);
			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSESNOR_READACC))
		{

			  /* Read actual full scale selection from sensor. */
			  if (bmi160_get_accel_range(&fullScale) != BMI160_SUCCESS)
			  {
			  	printf("read acc raw data error\n");
				return -1;
			  }

			  /* Store the sensitivity based on actual full scale. */
			  switch( fullScale )
			  {
				case BMI160_ACCEL_RANGE_2G:
				  sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_2G;
				  break;
				case BMI160_ACCEL_RANGE_4G:
				  sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_4G;
				  break;
				case BMI160_ACCEL_RANGE_8G:
				  sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_8G;
				  break;
				case BMI160_ACCEL_RANGE_16G:
				  sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_16G;
				  break;
				default:
				  sensitivity = -1.0f;
				  printf("read acc raw data error\n");
				  return -1;
			  }
			/* Read raw data from BMI160 output register. */
			if (bmi160_read_accel_xyz(&accel) != BMI160_SUCCESS)
			{
				  printf("read acc raw data error\n");
				  return -1;
			}
			printf("[BMI160] accel: x = %dmg y = %dmg z = %dmg\n",
					( int32_ts )( (accel.x) * (sensitivity) ),( int32_ts )( (accel.y) * (sensitivity) ),( int32_ts )( (accel.z) * (sensitivity) ));

			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READGYRO))
		{
			  /* Read actual full scale selection from sensor. */
			  if ( bmi160_get_gyro_range( &fullScale ) !=BMI160_SUCCESS)
			  {
				printf("read gyro raw data error\n");
				return -1;
			  }

			  /* Store the sensitivity based on actual full scale. */
			  switch( fullScale )
			  {
				case BMI160_GYRO_RANGE_125_DEG_SEC:
				  sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_125DPS;
				  break;
				case BMI160_GYRO_RANGE_250_DEG_SEC:
				  sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_250DPS;
				  break;
				case BMI160_GYRO_RANGE_500_DEG_SEC:
				  sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_500DPS;
				  break;
				case BMI160_GYRO_RANGE_1000_DEG_SEC:
				  sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_1000DPS;
				  break;
				case BMI160_GYRO_RANGE_2000_DEG_SEC:
				  sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_2000DPS;
				  break;
				default:
				  sensitivity = -1.0f;
				  printf("read gyro raw data error\n");
				  return -1;
			   }
			  if ( bmi160_read_gyro_xyz(&gyro) != BMI160_SUCCESS)
			  {
				  printf("read gyro raw data error\n");
				  return -1;
			  }
			  printf("[BMI160] gyro: x = %.2fmdps y = %.2fmdps z = %.2fmdps\n",
				(float)( gyro.x * sensitivity )/3600,(float)( gyro.y * sensitivity )/3600,(float)( gyro.z * sensitivity )/3600);
			return 0;
		}else
		{
			printf("input arguments ERROR!\n");
		}
	}else {
		printf("input arguments ERROR!\n");
	}
	return 0;
}

int cmd_bootmode(int argc,char* argv[],_command_source source)
{
	BOOT_MODE mode;

	if(argc==2&&(strcmp(argv[1],"r")==0)){
		mode = get_bootmode();
		print_bootmode(mode);

	}else if(argc==3&&(strcmp(argv[1],"w")==0)){

		if(strcmp(argv[2],"fota")==0){
			mode = FOTA_MODE;
		}else if(strcmp(argv[2],"normal")==0){
			mode = NORM_MODE;
		}else if(strcmp(argv[2],"board_mfg")==0){
			mode = BL_MFG_MODE;
		}else if(strcmp(argv[2],"system_mfg")==0){
			mode = SYS_MFG_MODE;
		}else if(strcmp(argv[2],"dfu")==0){
			mode = DFU_MODE;
		}else{ // error mode
			MFG_LOG_ERR(MFG_TAG_MISC,\
							"please set correct mode");
			mode  = ERROR_MODE;
			return -1;
		}
		if(mode!=ERROR_MODE){
			set_bootmode(mode);
            PWRMGR_state_change(POWER_STATE_REBOOT);
		}
	}
	return 0;
}
int cmd_help(int argc, char * argv[], _command_source source);

static int ccgx_fw_misc_cmds(int argc, char * argv[], _command_source source)
{

	if (argc == 2) {
		if (!strcmp((char const*)argv[1], "dump")) {
			ccgx_dump_fw_info();
			return 0;
		}
	}

	if (argc == 4) {

		int i;
		unsigned char buf [64];
		unsigned  addr;
		unsigned  cnt;

		if (!strcmp((char const*)argv[1], "reg")) {

			addr = strtoul(argv[2], NULL, 0);
			cnt =  strtoul(argv[3], NULL, 0);

			memset(buf, 0x00, sizeof(buf));
			if (ccgx_fw_i2c_read(addr, buf, cnt) == 0) {
				cnt = (cnt + 0x07) & ~0x07;
				for (i = 0; i < cnt; i += 8)
					printf("0x%08x : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n", addr + i,
							buf[i], buf[i + 1], buf[i + 2], buf[i + 3], buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
			}
			return 0;
		}
	}

	printf("Usage : \n");
	printf("\t %s dump\n", argv[0]);
	printf("\t %s reg addr count\n", argv[0]);
	printf("\t %s upgrade fw_idx offset size\n", argv[0]);

	return 0;
}

const MONITOR_COMMAND commandTable[] =
{
    {"task",    get_task_state},
    {"pm",      power_mfg_cmd},
    {"i2c",     i2c_mfg_cmd},
    {"led",     led_mfg_cmd},
    {"mfg_example", mfg_example},
    {"mfg_disp", disp_test_cmd},
    {"misc" ,cmd_misc_data},
    {"mag" ,cmd_mag},
    {"ccgx_fw" ,ccgx_fw_misc_cmds},
    {"ccg4_version", ccg4_read_version},
    {"mux_version",mux_read_version},
    {"audio", audio_mfg_cmd},
    {"gsensor", gsensor_mfg_cmds},
    {"bootmode",cmd_bootmode},
    {"?",       cmd_help}, //This must be the last command
};
const unsigned long ulNumberOfCommands = (sizeof(commandTable) / sizeof(commandTable[0]));

int cmd_help(int argc, char * argv[], _command_source source)
{
    uint8_t i;

    printf("\tAll MFG Command list\r\n");

    for (i=0; i<(ulNumberOfCommands-1); i++){
        printf("\t%s\r\n", commandTable[i].command);
    }
    printf("\t________ ^_^ ________\r\n");
    return 0;
}

int isaspace(unsigned char c)
{
    int     i;

    for (i = 0; i < sizeof(whiteSpace); i++)
    {
        if (c == whiteSpace[i])
        {
            return 1;
        }
    }
    return 0;
}

int ParseCommandAndData(unsigned char *pLineBuf, int *argn,
                                    unsigned char *argv[], int MaxArgs)
{
    int             n;
    int             i;
    unsigned char   quoteChar;

    n = 0;
    while (n < MaxArgs)
    {
        while (isaspace(*pLineBuf))
        {
            pLineBuf++;
        }

        if (*pLineBuf == '"' || *pLineBuf == '\'')
        {
            quoteChar = *pLineBuf;
            *pLineBuf = (unsigned char)1;
            argv[n++] = pLineBuf++;
            while (*pLineBuf && (*pLineBuf != quoteChar))
            {
                pLineBuf++;
            }
            if (*pLineBuf)
            {
                *pLineBuf = 0;
                pLineBuf++;
            }
            else
            {
                n = 0;                     // Error, no matching quote char
                break;
            }
        }
        else if (*pLineBuf)
        {
            argv[n++] = pLineBuf;
            //
            // Go to the next whiteSpace
            //
            while (*pLineBuf && !isaspace(*pLineBuf))
            {
                pLineBuf++;
            }
            if (*pLineBuf)
            {
                *pLineBuf = 0;
                pLineBuf++;
            }
            else break;
        }
        else break;
    }

    if ((n >= 1) && *argv[0] == '?' && *(argv[0] + 1))
    {
        n++;
        if (n <= MaxArgs)
        {
            for (i = 1; i < n; i++)
            {
                argv[i] = argv[i - 1];
            }
            (argv[1])++;
            argv[0] = (unsigned char*)"?";
        }
    }
    if (n > MaxArgs)
    {
        printf("Too many arguments\n");
        n = 0;
    }
    *argn = n;
    return n;
}

/**
*use SecureCRT serial transceivers tools,in the character stream,may contain
*a character that is not needed , such as backspace,move around.Before use the
*command line tool to parse the character,we must delete the invalid character.
* supported character:
*   move up:1B 5B 41
*   move down:1B 5B 42
*   move right:1B 5B 43
*   move left:1B 5B 44
*   carriage retuen and line feed:0D 0A
*   Backspace:08
*/
static uint32_t filter_useless_string(char *dest,const char *src)
{
    uint32_t dest_count=0;
    uint32_t src_count=0;
    while(src[src_count]!=0x0D && src[src_count+1]!=0x0A){
        //if(isprint(src[src_count])){
        if(src[src_count] >= 0x20 && src[src_count] <= 0x7E){
            dest[dest_count++]=src[src_count++];
        }
        else{
            switch(src[src_count]){
                case    0x08:                          //Backspace
                {
                    if(dest_count>0){
                        dest_count --;
                    }
                    src_count ++;
                }
                break;
                case    0x1B:
                {
                    if(src[src_count+1]==0x5B){
                        if(src[src_count+2]==0x41 || src[src_count+2]==0x42){
                            src_count +=3;              //move up and down
                        }
                        else if(src[src_count+2]==0x43){
                            dest_count++;               //move right
                            src_count+=3;
                        }
                        else if(src[src_count+2]==0x44){
                            if(dest_count >0)           //move left
                            {
                                dest_count --;
                            }
                            src_count +=3;
                        }
                        else{
                            src_count +=3;
                        }
                    }
                    else{
                        src_count ++;
                    }
                }
                break;
                default:
                {
                    src_count++;
                }
                break;
            }
        }
    }

    dest[dest_count++]=src[src_count++];
    dest[dest_count++]=src[src_count++];

    return dest_count;
}

int DoCommand(int argn,unsigned char *argv[], _command_source source)
{
    unsigned int uiCount;

    //
    // The first argument should be the command
    //
    for (uiCount = 0; uiCount < ulNumberOfCommands; uiCount++){
        char resut_cmp = (strcmp((char const*)argv[0], (char const*)commandTable[uiCount].command)== 0);
        if ( resut_cmp ){
            return(*(commandTable[uiCount].pFunc))(argn, (char **)argv, source);
        }
    }
     printf("Command error !!!\n");

    return -1;
}

void Shell_parse_cmd(CMDRxBuf *cmd_buf)
{
    int argn = 0;
    CMDRxBuf cmd = {0};
    char cmd_string[]={"CMD>"};
    unsigned char *argv[MAX_ARGS] = {0};
    cmd.lenght = filter_useless_string(cmd.buf,cmd_buf->buf);
    if(cmd.lenght != 0){
        if(0 != ParseCommandAndData((unsigned char *)cmd.buf, &argn, argv,
                                    (sizeof(argv)/sizeof(argv[0])))){
            DoCommand(argn, argv, CMD_SOURCE_UART);
            printf("%s",cmd_string);
        }
        else{
            printf("Shell Command Task %s \r\n", "[No Processor for Command]");
            printf("%s",cmd_string);

        }
    }
    else{
        printf("[No Command]\n");
        printf("%s",cmd_string);
    }
}
void ShellRecvProcess(void)
{
    if(CMD_Cache_Tail != CMD_Cache_Head){
        CMDRxBuf CMDBuffer = {0};
        memcpy(&CMDBuffer,&CMDCacheBuf[CMD_Cache_Tail],
                    CMDCacheBuf[CMD_Cache_Tail].lenght+4);
        Shell_parse_cmd(&CMDBuffer);
        if(++CMD_Cache_Tail >= CMD_CACHE_LEN){
            CMD_Cache_Tail = 0;
        }
    }
}

TASKCFUNC(Shell)
{
    ShellRecvProcess();
}

void Shell_rec_buf(char data)
{
    static uint32_t rec_count=0;
    CMDCacheBuf[CMD_Cache_Head].buf[rec_count] = data;
    if(/*(0x0D == CMDCacheBuf[CMD_Cache_Head].buf[rec_count-1]) &&*/ //support mtty in mfg mode
        (0x0D == CMDCacheBuf[CMD_Cache_Head].buf[rec_count])){
        CMDCacheBuf[CMD_Cache_Head].lenght = rec_count + 1;
        rec_count = 0;
        if(++CMD_Cache_Head >= CMD_CACHE_LEN){
            CMD_Cache_Head = 0;
        }
    }
    else{
        rec_count ++;
        if(rec_count >= CMD_BUF_LEN){
            rec_count = 0;
        }
    }
}

void Shell_init(void)
{
        pcbid_t pcb_id;
        get_pcbid(&pcb_id);
    CMD_Cache_Head = 0;
    CMD_Cache_Tail = 0;
    //ioexp_drv_init();
    //bsp_pmic_init();
    bsp_pmic_power_enable(V_VDD_1V8, POWER_ENABLE);
    BMI160_IO_Init();
    bmi160_init_sensor();
        if(pcb_id == XA0n)
                audio_alc4040_driver_init();
        audio_alc5665_driver_init();
        audio_path_service_init();
}

