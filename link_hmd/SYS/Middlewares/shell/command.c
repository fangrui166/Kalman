#include <string.h>
#include <stdlib.h>
#include "usart_drv.h"
#include "command.h"
#include "PowerManager_command.h"
#include "freertos_trace.h"
#include "gpio.h"
#include "x_pmic.h"
#include "ncp6924.h"
#include "audio_alc5665_driver.h"
#include "audio_alc4040_driver.h"
#include "htc_audio_path_service.h"
#include "anx7737.h"
#include "rtos_i2c_drv.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "max17050_fuel_gauge.h"
#include "gpio_exp.h"
#include "rtc_drv.h"
#include "usbd_comp.h"
#include "htc_usb_cdc_data_service.h"
#include "display_drv.h"
#include "hlog_api.h"
#include "htc_adc_func.h"
#include "misc_data.h"
#include "led_drv.h"
#include "led_hal.h"
#include "gSensor_Calibate.h"
#include "ccgx_fw_io.h"
#include "ccgx_fw.h"
#include "epl88051_driver.h"
#include "sensor_task.h"

#define CDCMSG_BUFFER_SIZE	COMP_CDC_DATA_FS_IN_PACKET_SIZE

static osThreadId commandTaskHandle;
static osMessageQId CommdMsgQueueHandle;
static osSemaphoreId cmd_semaphore;
extern int shell_lsm6dsm_commad(int argc, char * argv[], _command_source source);
extern pcbid_t pcb_id;

unsigned long my_strtoul (char *data);
const unsigned char whiteSpace[] = {' ', '\t', '=', '\r', '\n'};

__weak int get_task_state(int argc, char *argv[], _command_source source)
{
    int i;

    for (i = 0; i < argc; i++)
    {
        shell_debug("argv[%d]:%s\r\n",i,(char const*)argv[i]);
    }

    return 0;
}

int gpio_test_cmd(int argc, char * argv[], _command_source source)
{
    int ret = 0;
    if((argc < 2) ||(!strcmp(argv[1], "?"))||(!strcmp(argv[1], "help"))){
        shell_info("Useage:\n\tgpio dump --> dump all of gpio status\n");
        shell_info("\n\tgpio set PXn=x\n\teg:\n\tgpio set PA1=1\n");
        return -1;
    }
    if(!strcmp(argv[1], "dump")){
        gpio_dump_status();
    }
    else if(!strcmp(argv[1], "set")){
        uint8_t PinState = 0;
        uint8_t gpio_index = 0;
        uint16_t GPIO_PIN_x = 0;
        GPIO_TypeDef *GPIOx;
        if(argc < 4){
            shell_err("Useage:\n\tgpio set PXn=x\neg\t:\n\tgpio set PA1=1\n");
            return -2;
        }
        PinState = atoi(argv[3]);
        gpio_index = atoi(argv[2]+2);
        GPIO_PIN_x = (1 << gpio_index);
        if(!strncasecmp("PA",argv[2],2)){
            GPIOx = GPIOA;
        }
        else if(!strncasecmp("PB",argv[2],2)){
            GPIOx = GPIOB;
        }
        else if(!strncasecmp("PC",argv[2],2)){
            GPIOx = GPIOC;
        }
        else if(!strncasecmp("PD",argv[2],2)){
            GPIOx = GPIOD;
        }
        HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, (GPIO_PinState)PinState);
    }
    return ret;
}

void show_led_help()
{
	shell_info("LED Usage:\n \
		led dump --> dump all led events status\n \
		led [R/G/A] [on/off] --> turn red/green/amber led on/off\n \
		led event [0~12]|0x100 --> change led event(0x10x:enable/0x0x:disable)\n");
	shell_info("led [R/G] [current] [value]--> set read/green led current(0~255)\n \
		led ?\n");
}

int led_test_cmd(int argc, char * argv[], _command_source source)
{
#define LP5562_REG_B_CURRENT	0x05
#define LP5562_REG_G_CURRENT	0x06
	int ret = 0;
	if (argc >= 2) {
		/* uint16_t reg = (uint16_t)my_strtoul(argv[2]); */
		if(!strcmp(argv[1], "R")){
			if(!strcmp(argv[2], "on")){
				led_set_led_state(LED_COLOR_RED, LED_STEADY_ON);
			} else if(!strcmp(argv[2], "off")){
				led_set_led_state(LED_COLOR_RED, LED_STEADY_OFF);
			} else if(!strcmp(argv[2], "current")){
				uint8_t cur = atoi(argv[3]);
				led_reg_write(LP5562_REG_B_CURRENT, cur);
			} else
				show_led_help();
		}else if(!strcmp(argv[1], "G")){
			if(!strcmp(argv[2], "on")){
				led_set_led_state(LED_COLOR_GREEN, LED_STEADY_ON);
			} else if(!strcmp(argv[2], "off")){
				led_set_led_state(LED_COLOR_GREEN, LED_STEADY_OFF);
			} else if(!strcmp(argv[2], "current")){
				uint8_t cur = atoi(argv[3]);
				led_reg_write(LP5562_REG_G_CURRENT, cur);
			} else
				show_led_help();
		}else if(!strcmp(argv[1], "A")){
			if(!strcmp(argv[2], "on")){
				led_set_led_state(LED_COLOR_AMBER, LED_STEADY_ON);
			} else if(!strcmp(argv[2], "off")){
				led_set_led_state(LED_COLOR_AMBER, LED_STEADY_OFF);
			} else
				show_led_help();
		}else if(!strcmp(argv[1], "event")){
			uint32_t event = (uint32_t)my_strtoul(argv[2]);
			/* led_blink_led((LED_EVENT)event); */
			SendLedEventToDeal(event);
		}else if(!strcmp(argv[1], "dump")){
			led_dump_event_status();
		}else
			show_led_help();
	}else
		show_led_help();
	return ret;
}

void show_ioexp_help()
{
	shell_info("IOEXP Usage:\n \
		ioexp dump --> dump ioexp gpio direction and value\n \
		ioexp output get [0~17] --> get gpio output value\n \
		ioexp output set [0~17] [0/1] --> set gpio output value\n \
		ioexp direction get [0~17] --> get gpio direction\n \
		ioexp direction set [0~17] [0/1] --> set gpio direction(1:output/0:input)\n \
		ioexp ?\n");
}

int ioexp_test_cmd(int argc, char * argv[], _command_source source)
{
	int ret = 0;
	int mult = 1;
	if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id))
		mult = 2;
	if (argc == 2) {
		if (!strcmp(argv[1], "?")) {
			show_ioexp_help();
		}
		else if (!strcmp(argv[1], "dump")) {
			uint8_t ioexp_data[2*IOEXP_GPIO_NUM][2] = {0};
			for(int i=0; i<mult*IOEXP_GPIO_NUM; i++){
				int8_t ret = ioexp_gpio_get_value(IOEXP_REG_DIR, i);
				/* ioexp gpio is output */
				if(ret > 0){
					ioexp_data[i][0] = 'O';
					ioexp_data[i][1] = ioexp_gpio_get_value(IOEXP_REG_OUT, i) ? 'H':'L';
				}
				/* ioexp gpio is input */
				else if(ret == 0){
					ioexp_data[i][0] = 'I';
					ioexp_data[i][1] = ioexp_gpio_get_value(IOEXP_REG_STAT, i) ? 'H':'L';
				}
				else
					shell_err("ioexp gpio%d get direction fail!\n", i);

				shell_debug("ioexp gpio[%02d] direction: %c, value: %c\n", i, ioexp_data[i][0], ioexp_data[i][1]);
			}
		}else
			show_ioexp_help();
	}
	else if(argc >= 4) {
		uint8_t gpio = atoi(argv[3]);
		if(gpio >= mult*IOEXP_GPIO_NUM) {
			shell_err("invalid ioexp gpio num!\n");
			return -1;
		}
		uint8_t value = 0;

		if (!strcmp(argv[1], "output")) {
			if (!strcmp(argv[2], "get")) {
				/* set as output first */
				ioexp_gpio_set_value(IOEXP_REG_DIR, gpio, IOEXP_GPIO_OUTPUT);
				ret = ioexp_gpio_get_value(IOEXP_REG_OUT, gpio);
				if(ret >= 0)
					shell_debug("ioexp gpio%d output %s\n", gpio, ret ? "high":"low");
				else
					shell_err("ioexp gpio%d get value fail!\n", gpio);
			}
			else if (!strcmp(argv[2], "set")) {
				value = atoi(argv[4]);
				/* set as output first */
				ioexp_gpio_set_value(IOEXP_REG_DIR, gpio, IOEXP_GPIO_OUTPUT);
				ret = ioexp_gpio_set_value(IOEXP_REG_OUT, gpio, value);
				if(ret >= 0)
					shell_debug("ioexp gpio%d set output %s sucess\n", gpio, value ? "high":"low");
				else
					shell_err("ioexp gpio%d set output fail!\n", gpio);
			}
			else
				show_ioexp_help();
		}
		else if (!strcmp(argv[1], "direction")) {
			if (!strcmp(argv[2], "get")) {
				ret = ioexp_gpio_get_value(IOEXP_REG_DIR, gpio);
				if(ret >= 0)
					shell_debug("ioexp gpio%d direction is %s\n", gpio, ret ? "output":"input");
				else
					shell_err("ioexp gpio%d get direction fail!\n", gpio);
			}
			else if (!strcmp(argv[2], "set")) {
				value = atoi(argv[4]);
				ret = ioexp_gpio_set_value(IOEXP_REG_DIR, gpio, value);
				if(ret >= 0)
					shell_debug("ioexp gpio%d set direction %s sucess\n", gpio, value ? "output":"input");
				else
					shell_err("ioexp gpio%d set direction fail!\n", gpio);
			}else
				show_ioexp_help();
		}else
			show_ioexp_help();
	}

	return ret;
}

void show_i2c_help()
{
	shell_info("I2C Usage:\n \
		i2c devices --> dump i2c devices\n \
		i2c read_reg8 0x[dev_addr] 0x[reg] [len] --> register is 8bits\n \
		i2c read_reg16 0x[dev_addr] 0x[reg] [len] --> register is 16bits\n \
		i2c write_reg8 0x[dev_addr] 0x[reg] [len] 0x[byte]... --> register is 8bits\n \
		i2c write_reg16 0x[dev_addr] 0x[reg] [len] 0x[byte]... --> register is 16bits\n \
		i2c ?\n");
}

#define I2C_RW_MAX_LENGTH 16
int i2c_test_cmd(int argc, char * argv[], _command_source source)
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
				shell_debug("I2C read device(address:0x%0x) reg(0x%02x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", rx_buf[i]);
				}
				printf("\r\n");
			}
			else
				shell_err("I2C read device(address:0x%0x) reg(0x%02x) fail.\r\n",(uint8_t)addr, reg);
			return ret;
		}else if(!strcmp(argv[1], "read_reg16")){
			ret = RTOS_I2C_ReadBuffer(addr, reg, I2C_MEMADD_SIZE_16BIT, &rx_buf[0], len, portMAX_DELAY);
			if(ret == I2C_OK){
				shell_debug("I2C read device(address:0x%0x) reg(0x%04x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", rx_buf[i]);
				}
				printf("\r\n");
			}
			else
				shell_err("I2C read device(address:0x%0x) reg(0x%04x) fail.\r\n",(uint8_t)addr, reg);
			return ret;
		}else if(!strcmp(argv[1], "write_reg8")){
			uint8_t value[I2C_RW_MAX_LENGTH] = {0};
			for(int i=0; i<len; i++){
				value[i] = (uint8_t)my_strtoul(argv[5+i]);
			}
			ret = RTOS_I2C_WriteBuffer(addr, reg, I2C_MEMADD_SIZE_8BIT, &value[0], len, portMAX_DELAY);
			if(ret == I2C_OK){
				shell_debug("I2C write device(address:0x%0x) reg(0x%02x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0;i < len; i++){
					printf("0x%02x ", value[i]);
				}
				printf("\r\n");
			}
			else
				shell_err("I2C write device(address:0x%0x) reg(0x%02x) fail.\r\n",(uint8_t)addr, reg);
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
				shell_debug("I2C write device(address:0x%0x) reg(0x%04x) ok, values are:\r\n",(uint8_t)addr, reg);
				for(int i=0; i < len; i++){
					printf("0x%02x ", value[i]);
				}
				printf("\r\n");
			}
			else
				shell_err("I2C write device(address:0x%0x) reg(0x%04x) fail.\r\n",(uint8_t)addr, reg);
		}else
			show_i2c_help();

	}else
		show_i2c_help();

	return 0;
}
int ccg4_read_version(int argc, char * argv[], _command_source source)
{
	uint8_t rx_buf[3]={0};
	uint8_t major_version = 0;
	uint8_t minor_version = 0;
	uint8_t mini_version = 0;
    char usb_buffer[PARAMETER_SIZE] = {0};
	I2C_STATUS ret;
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_ADDR,0x00, I2C_MEMADD_SIZE_8BIT, &rx_buf[0],3, portMAX_DELAY);

	if(ret == I2C_OK) {
		//rx_buf[0] -->port status
		//rx_buf[1] -->version number
		major_version = (rx_buf[1]>>4)&0x0f;
		minor_version = rx_buf[1]&0x0f;
		mini_version = rx_buf[2];
		sprintf(usb_buffer,"ccg4 version:%d.%d.%d\r\n",major_version,minor_version,mini_version);

		if (source == CMD_SOURCE_USBCDC)
		{
			usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
		}else if(source == CMD_SOURCE_UART){
			shell_info("%s",usb_buffer);
		}
	}else{
		sprintf(usb_buffer,"get ccg4 version fail\r\n");
		if (source == CMD_SOURCE_USBCDC)
		{
			usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
		}else if(source == CMD_SOURCE_UART){
			shell_info("%s",usb_buffer);
		}
	}
	return ret;

}
int ccgx_i2c2_test(int argc, char * argv[], _command_source source)
{
	int ret = 0;
	uint8_t rx_buf[16] = {0};
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_I2C2_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &rx_buf[0], 16, portMAX_DELAY);
	for(int i=0;i<16;i++)
	shell_debug("%s: read CCGX_2 4 bytes 0x%02x\n", __func__, rx_buf[i]);
        return ret;
}
void pmic_test_help(void)
{
	shell_info("PMIC Usage:\n \
	pmic pid --> dump chip pid\n \
	pmic dump --> dump all of power status\n \
	pmic set itemX=x eg: pmic set item2=1,V_DP will output\n \
	pmic voltage get [index: 0~5]\n \
	pmic voltage set [index: 0~5] 0x[value]\n \
	pmic register read 0x[reg] eg: pmic register read 0x14\n \
	pmic register write 0x[reg] 0x[value]\n \
	pmic ?\n");
}

int pmic_test_cmd(int argc, char * argv[], _command_source source)
{
    int ret = 0;
    if((argc < 2) ||(!strcmp(argv[1], "?"))||(!strcmp(argv[1], "help"))){
		pmic_test_help();
        return -1;
    }
    else if(!strcmp(argv[1], "pid")){
		uint8_t rx_buf = 0;
		/* use FID instead of PID(read value is 0) */
		ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_PMIC_ADDR, NCP6924_REG_FID, I2C_MEMADD_SIZE_8BIT, &rx_buf, 1, portMAX_DELAY);
		shell_debug("pmic pid:0x%02x\n", rx_buf);
    }
    else if(!strcmp(argv[1], "dump")){
        ret = bsp_pmic_power_state_dump();
    }
    else if(!strcmp(argv[1], "set")){
        power_item_t item;
        uint8_t enable;
        if(argc < 4){
			pmic_test_help();
            return -2;
        }
        item = (power_item_t)atoi(argv[2]+4);
        enable = atoi(argv[3]);
        ret = bsp_pmic_power_enable(item, enable);
    }
    else if(!strcmp(argv[1], "voltage")){
		uint8_t index = atoi(argv[3]);
		if(!strcmp(argv[2], "get"))
		{
			uint8_t value = 0;
			ret = ncp6924_reg_read(pmic_vreg[index].reg_addr, &value);
		}
		else if(!strcmp(argv[2], "set"))
		{
			uint8_t volt = (uint8_t)my_strtoul(argv[4]);
			ret = ncp6924_reg_write(pmic_vreg[index].reg_addr, volt);
		}
		else
			pmic_test_help();
	}
    else if(!strcmp(argv[1], "register")){
        if(argc < 4){
			pmic_test_help();
            return -3;
        }
		/* reg = 0x, value = 0x */
		uint8_t reg = (uint8_t)my_strtoul(argv[3]);
		uint8_t value = 0;
		if(!strcmp(argv[2], "read"))
		{
			ret = ncp6924_reg_read(reg, &value);
		}
		else if(!strcmp(argv[2], "write"))
		{
			value = (uint8_t)my_strtoul(argv[4]);
			ret = ncp6924_reg_write(reg, value);
		}
		else
			pmic_test_help();
    }
    return ret;
}

static void print_bootmode(BOOT_MODE mode, _command_source source)
{
	char buffer[100]={0};
	switch(mode){
		case FOTA_MODE:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is fota mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);
			break;
		case NORM_MODE:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is normal mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);

			break;
		case BL_MFG_MODE:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is board level mfg mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);
			break;
		case SYS_MFG_MODE:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is system level mfg mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);
			break;
		case DFU_MODE:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is dfu mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);
			break;
		default:
			memset(buffer,0,sizeof(buffer));
			strcpy(buffer,"current mode is error mode\r\n");
			if(source == CMD_SOURCE_USBCDC){
				usb_cdc_printf("%s",buffer);
			}else
				shell_info("%s",buffer);
			break;

	}
}
int cmd_bootmode(int argc,char* argv[],_command_source source)
{
	BOOT_MODE mode;

	if(argc==2&&(strcmp(argv[1],"r")==0)){
		mode = get_bootmode();
		print_bootmode(mode,source);

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
			shell_err("please set correct mode");
			mode  = ERROR_MODE;
			return -1;
		}
		if(mode!=ERROR_MODE){
			set_bootmode(mode);
            //PWRMGR_state_change(POWER_STATE_REBOOT);
		}
	}
	return 0;
}

/*
  * anx command:
  * anx dump reg
  */

void anx_test_usage(void) {
	shell_info(" anx command:\r\n"
		"\tanx dump slave_address\r\n"
                  );
}

unsigned long my_strtoul (char *data)
{
	int base = 10;
	if (data[0] == '0' && (data[1] == 'x' || data[1] == 'X'))
		base = 16;

	return strtoul(data, NULL, base);
}

int anx_test_cmd(int argc, char * argv[], _command_source source)
{
	uint16_t addr = 0;
	int loop_i;
	uint16_t reg_data;
	if (argc == 3) {
		if (!strcmp(argv[1], "dump")) {
			addr = (uint16_t)my_strtoul(argv[2]);
			for (loop_i = 0; loop_i < 0x0400; loop_i++) {
				anx7737_i2c_read(addr, loop_i, &reg_data);
				shell_info("anx i2c: addr: 0x%4.4X reg: 0x%4.4X "
					"data: 0x%4.4X\r\n",
						addr, loop_i, reg_data);
			}
			anx_test_usage();
			return -1;
		}
	}
	anx_test_usage();
	return -1;
}

/*
  * audio command:
  * audio set reg address value
  * audio set path [hdmi|usb|loopback]
  * audio set output [aj|ear]
  * audio get reg address
  * audio get regall
  * audio get path
  * audio get output
  * audio get cal
  */

void audio_test_usage(void) {
	shell_info(" audio command:\r\n"
		"\taudio set reg address value\r\n"
		"\taudio set path [hdmi|usb|loopback]\r\n"
		"\taudio set power [on|off]\r\n"
		"\taudio set output [aj|ear]\r\n"
		"\taudio set amp [on|off]\r\n"
		"\taudio get reg address\r\n"
		"\taudio get regall\r\n"
		"\taudio get power\r\n"
		"\taudio get amp\r\n"
		"\taudio get output\r\n"
		"\taudio get cal\r\n"
		"\taudio get path\r\n");
}

int audio_test_cmd(int argc, char * argv[], _command_source source)
{
	enum __audio_srv_path sr_path;
	uint16_t reg = 0;
	uint16_t data = 0;
	unsigned char onoff;
	int loop_i;
	uint16_t reg_data;
	char msg[CDCMSG_BUFFER_SIZE] = { 0 };
	if (argc == 3) {
		if (!strcmp(argv[1], "get")) {
			if(!strcmp(argv[2], "regall")) {
				for (loop_i = 0; loop_i < 0x0400; loop_i++) {
					audio_alc5665_i2c_read(loop_i,
								&reg_data);
					snprintf((char *)&msg,
						CDCMSG_BUFFER_SIZE - 1,
						"audio i2c: addr: 0x%4.4X, "
						"data: 0x%4.4X\r\n",
						loop_i, reg_data);
					shell_info("%s", msg);
				}
				return 0;
			} else if(!strcmp(argv[2], "cal")) {
				for (loop_i = 490; loop_i < 501; loop_i++) {
					audio_alc5665_i2c_read(loop_i,
								&reg_data);
					snprintf((char *)&msg,
						CDCMSG_BUFFER_SIZE - 1,
						"audio i2c: addr: 0x%4.4X, "
						"data: 0x%4.4X\r\n",
						loop_i, reg_data);
					shell_info("%s", msg);
				}
				return 0;
			} else if (!strcmp(argv[2], "path")) {
				audio_path_get(&sr_path);
				if (sr_path == AUDIO_SRV_HDMI_PATH)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"hdmi\r\n");
				else if (sr_path == AUDIO_SRV_USB_PATH)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"usb\r\n");
                                else if (sr_path == AUDIO_SRV_LOOPBACK_PATH)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"loopback\r\n");
				else
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"Uknown\r\n");

				shell_info("%s", msg);
				if (source == CMD_SOURCE_USBCDC)
					usb_cdc_transmit_data((uint8_t *)&msg,
								strlen(msg));
				return 0;
			} else if (!strcmp(argv[2], "power")) {
				audio_alc5665_audio_power_get(&onoff);
				if (onoff == 0)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"power off\r\n");
				else if (onoff == 1)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"power on\r\n");
				else
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"power unknown\r\n");

				shell_info("%s", msg);
				if (source == CMD_SOURCE_USBCDC)
					usb_cdc_transmit_data((uint8_t *)&msg,
								strlen(msg));

				return 0;
			} else if (!strcmp(argv[2], "output")) {
				audio_path_get(&sr_path);
				if (sr_path == AUDIO_SRV_OUTPUT_TO_AJ)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"aj\r\n");
				else if (sr_path == AUDIO_SRV_OUTPUT_TO_EAR)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"ear\r\n");
				else if (sr_path == AUDIO_SRV_OUTPUT_TO_BOTH)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"both\r\n");
				else
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"Uknown\r\n");

				shell_info("%s", msg);
				if (source == CMD_SOURCE_USBCDC)
					usb_cdc_transmit_data((uint8_t *)&msg,
								strlen(msg));
				return 0;
			} else if (!strcmp(argv[2], "amp")) {
				audio_alc5665_audio_amp_power_get(&onoff);
				if (onoff == 0)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"amp power off\r\n");
				else if (onoff == 1)
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"amp power on\r\n");
				else
					snprintf((char *)&msg,
							CDCMSG_BUFFER_SIZE - 1,
							"power unknown\r\n");

				shell_info("%s", msg);
				if (source == CMD_SOURCE_USBCDC)
					usb_cdc_transmit_data((uint8_t *)&msg,
								strlen(msg));
				return 0;
			}
			audio_test_usage();
			return -1;
		}
	} else if (argc == 4) {
		if (!strcmp(argv[1], "get")) {
			if (!strcmp(argv[2], "reg")) {
				reg = (uint16_t)my_strtoul(argv[3]);
				audio_alc5665_i2c_read(reg, &data);
				snprintf((char *)&msg, CDCMSG_BUFFER_SIZE - 1,
					"reg: 0x%4.4X, val: 0x%4.4X\r\n",
					reg, data);
				shell_info("%s", msg);
				if (source == CMD_SOURCE_USBCDC)
					usb_cdc_transmit_data((uint8_t *)&msg,
								strlen(msg));
				return 0;
			}
			audio_test_usage();
			return -1;
		} else if (!strcmp(argv[1], "set")) {
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
		audio_test_usage();
		return -1;
	} else if (argc == 5) {
		if (!strcmp(argv[1], "get")) {
			audio_test_usage();
			return -1;
		} else if (!strcmp(argv[1], "set")) {
			if (!strcmp(argv[2], "reg")) {
				reg = (uint16_t)my_strtoul(argv[3]);
				data = (uint16_t)my_strtoul(argv[4]);
				return audio_alc5665_i2c_write(reg, data);
			}
		}
		audio_test_usage();
		return -1;
	}
	audio_test_usage();
	return -1;
}

int show_mag_help()
{
	shell_info (" Usage:\r\n");
	shell_info ("mag getdata\t--- get magnetic sensor data\r\n");
	shell_info ("mag getID	\t--- get magnetic sensor chip ID\r\n");
	shell_info ("mag getaccuracy\t--- get magnetic sensor auto calibration accuracy\r\n");
	shell_info ("mag selftest\t--- magnetic sensor self test command\r\n");
	shell_info ("mag  ?	\t--- show magnetic sensor command help\r\n");
	return 0;
}

int cmd_mag(int argc, char * argv[], _command_source source)
{
	extern void *MAGNETIC_handle;
	uint16_t chip_id;
	SensorAxes_t MAG_Value;
	uint8_t accuracy;
	char mag_string[80];
	if(MAGNETIC_handle == NULL){
		shell_err("command error:magnetic sensor initial fail\r\n");
		if (source == CMD_SOURCE_USBCDC){
			usb_cdc_transmit_data("command error:magnetic sensor initial fail\r\n",strlen("command error:magnetic sensor initial fail\r\n"));
		}
		return -1;
	}
	if (argc == 2){
		if (!strcmp(argv[1], "getdata")){
			//command get magnetic sensor data
			if (BSP_MAGNETO_Get_Axes(MAGNETIC_handle, &MAG_Value) != COMPONENT_OK){
				shell_err("mag get data fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("mag get data fail\r\n",strlen("mag get data fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("mag x=%d,y=%d,z=%d\r\n",MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(mag_string,sizeof(mag_string)-1,"mag x=%d,y=%d,z=%d\r\n",MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);
					usb_cdc_transmit_data((uint8_t *)mag_string,strlen(mag_string));
				}
			}
		}
		else if (!strcmp(argv[1], "getID")){
			//command  get magnetic sensor chip ID
			if (BSP_MAGNETO_Get_WhoAmI(MAGNETIC_handle, &chip_id) != COMPONENT_OK){
				shell_err("mag get chip id fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("mag get chip id fail\r\n",strlen("mag get chip id fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("mag chip id is 0x%02x\r\n",chip_id);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(mag_string,sizeof(mag_string)-1,"mag chip id is 0x%02x\r\n",chip_id);
					usb_cdc_transmit_data((uint8_t *)mag_string,strlen(mag_string));
				}
			}
		}
		else if (!strcmp(argv[1], "selftest")){
			//command  magnetic sensor selftest
			if (BSP_MAGNETO_Sensor_Selftest(MAGNETIC_handle) != COMPONENT_OK){
				shell_err("mag selftest fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("mag selftest fail\r\n",strlen("mag selftest fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("mag selftest pass\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("mag selftest pass\r\n",strlen("mag selftest pass\r\n"));
				}
			}
		}
		else if (!strcmp(argv[1], "getaccuracy")){
			//command  get magnetic sensor auto calibration accuracy level
			if (BSP_MAGNETO_Get_Accuracy(MAGNETIC_handle,&accuracy) != COMPONENT_OK){
				shell_err("mag get accuracy fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("mag get accuracy fail\r\n",strlen("mag get accuracy fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("mag accuracy=%d\r\n",accuracy);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(mag_string,sizeof(mag_string)-1,"mag accuracy=%d\r\n",accuracy);
					usb_cdc_transmit_data((uint8_t *)mag_string,strlen(mag_string));
				}
			}
		}
		else if (!strcmp(argv[1], "?")){
			//show magnetic sensor command list
			show_mag_help();
		}
		else{
			shell_err("command error:argv %s not support\r\n",argv[1]);
			if (source == CMD_SOURCE_USBCDC){
				snprintf(mag_string,sizeof(mag_string)-1,"command error:argv %s not support\r\n",argv[1]);
				usb_cdc_transmit_data((uint8_t *)mag_string,strlen(mag_string));
			}
			show_mag_help();
		}
	}
	else{
		shell_err("command error:argc not match!argc=%d\r\n",argc);
		if (source == CMD_SOURCE_USBCDC){
			snprintf(mag_string,sizeof(mag_string)-1,"command error:argc not match!argc=%d\r\n",argc);
			usb_cdc_transmit_data((uint8_t *)mag_string,strlen(mag_string));
		}
		show_mag_help();
	}
	return 0;
}

int show_prox_help()
{
	shell_info ("Usage:\r\n");
	shell_info ("\tprox getdata\t--- get proximity sensor data\r\n");
	shell_info ("\tprox getID\t--- get proximity sensor chip ID\r\n");
	shell_info ("\tprox getprox\t--- get proximity sensor proximity status\r\n");
	shell_info ("\tprox getthreshold\t--- get proximity sensor threshold value\r\n");
	shell_info ("\tprox calixtalk\t--- do proximity sensor open air calibration\r\n");
	shell_info ("\tprox  ?	\t--- show proximity sensor command help\r\n");
	return 0;
}

int cmd_prox(int argc, char * argv[], _command_source source)
{
	uint8_t chip_id,proximity;
	uint16_t adc_value,xtalk,low_threshold,high_threshold;
	char prox_string[80];
	if (argc == 2){
		if (!strcmp(argv[1], "getdata")){
			//command get proximity sensor data
			if(epl88051_ps_adc_value(&adc_value ) != PS_OK)
			{
				shell_err("prox get data fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox get data fail\r\n",strlen("prox get data fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("prox adc value=%d\r\n",adc_value);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(prox_string,sizeof(prox_string)-1,"prox adc value=%d\r\n",adc_value);
					usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
				}
			}
		}
		else if (!strcmp(argv[1], "getID")){
			//command  get proximity sensor chip ID
			if(epl88051_ps_get_revno(&chip_id ) != PS_OK)
			{
				shell_err("prox get ID fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox get ID fail\r\n",strlen("prox get ID fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("prox ID is 0x%02x\r\n",chip_id);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(prox_string,sizeof(prox_string)-1,"prox ID is 0x%02x\r\n",chip_id);
					usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
				}
			}
		}
		else if (!strcmp(argv[1], "getprox")){
			//command  get proximity sensor proximity status
			if(epl88051_cmd_ps_proximity(&proximity) != PS_OK)
			{
				shell_err("prox get proximity fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox get proximity fail\r\n",strlen("prox get proximity fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("prox %s\r\n",proximity?"near":"far");
				if (source == CMD_SOURCE_USBCDC){
					snprintf(prox_string,sizeof(prox_string)-1,"prox %s\r\n",proximity?"near":"far");
					usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
				}
			}
		}
		else if (!strcmp(argv[1], "getthreshold")){
			//command  get proximity sensor threshold value
			if(epl88051_ps_get_threshold(&low_threshold,&high_threshold) != PS_OK)
			{
				shell_err("prox get threshold fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox get threshold fail\r\n",strlen("prox get threshold fail\r\n"));
				}
				return -1;
			}
			else{
				shell_info("prox low threshold=%d,high threshold=%d\r\n",low_threshold,high_threshold);
				if (source == CMD_SOURCE_USBCDC){
					snprintf(prox_string,sizeof(prox_string)-1,"prox low threshold=%d,high threshold=%d\r\n",low_threshold,high_threshold);
					usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
				}
			}
		}
		else if (!strcmp(argv[1], "calixtalk")){
			//command  proximity sensor calibration
			if(epl88051_ps_Xtalk_calibration(&xtalk,&low_threshold,&high_threshold) != PS_OK)
			{
				shell_err("prox open air calibration fail\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox open air calibration fail\r\n",strlen("prox open air calibration fail\r\n"));
				}
				return -1;
			}
			else{
				if(set_ps_threshold(low_threshold,high_threshold))
				{
					shell_err("prox open air calibration fail\r\n");
					if (source == CMD_SOURCE_USBCDC){
						usb_cdc_transmit_data("prox open air calibration fail\r\n",strlen("prox open air calibration fail\r\n"));
					}
					return -1;
				}
				shell_info("prox open air calibration pass\r\n");
				if (source == CMD_SOURCE_USBCDC){
					usb_cdc_transmit_data("prox open air calibration pass\r\n",strlen("prox open air calibration pass\r\n"));
				}
			}
		}
		else if (!strcmp(argv[1], "dump")){
			//command  get proximity sensor threshold value
			shell_info("prox register dump start\r\n");
			if(epl88051_ps_register_dump() != PS_OK)
			{
				shell_err("prox register dump fail!\r\n");
				return -1;
			}
			else{
				shell_info("prox register dump done!\r\n");
			}
		}
		else if (!strcmp(argv[1], "?")){
			//command show proximity sensor command list
			show_prox_help();
		}
		else{
			shell_err("command error:argv %s not support\r\n",argv[1]);
			if (source == CMD_SOURCE_USBCDC){
				snprintf(prox_string,sizeof(prox_string)-1,"command error:argv %s not support\r\n",argv[1]);
				usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
			}
			show_prox_help();
		}
	}
	else{
		shell_err("command error:argc not match!argc=%d\r\n",argc);
		if (source == CMD_SOURCE_USBCDC){
			snprintf(prox_string,sizeof(prox_string)-1,"command error:argc not match!argc=%d\r\n",argc);
			usb_cdc_transmit_data((uint8_t *)prox_string,strlen(prox_string));
		}
		show_prox_help();
	}
	return 0;
}

void gauge_test_help(void)
{
	shell_info("gauge Usage:\r\n \
	fg current 1 --> read average current\r\n \
	fg current 0 --> read present current\r\n \
	fg soc --> read Soc\r\n \
	fg temp --> read temperture\r\n \
	fg voltage --> read voltage\r\n \
	fg fullcapacity --> read full capacity\r\n \
	fg help --> printf gauge command list\r\n \
	fg ? --> printf gauge command list\r\n");
}


int gauge_test_cmd(int argc, char * argv[], _command_source source){
	int soc = 0;
	int temperature;
	int current;
	uint32_t voltage;
	uint32_t capacity;
	if((argc > 3) ||(!strcmp(argv[1], "?"))||(!strcmp(argv[1], "help"))){
		gauge_test_help();
        return -1;
    }
    else if (argc == 3 && !strcmp(argv[1], "current") && !strcmp(argv[2], "1")) {
		current = htc_batt_get_current(1);
		if(current != -1){
			shell_info("Battery average current: %d mA\r\n",(current / 1000));
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Battery average current: %d mA\r\n",(current / 1000));
			return 0;
		}else{
			shell_err("Get battery average current fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery average current fail\r\n");
			return -1;
		}
    }
	else if (argc == 3 && !strcmp(argv[1], "current") && !strcmp(argv[2], "0")) {
		current = htc_batt_get_current(0);
		if(current != -1){
			shell_info("Battery present current: %d mA\r\n",(current / 1000));
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Battery present current: %d mA\r\n",(current / 1000));
			return 0;
		}else{
			shell_err("Get battery present current fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery present current fail\r\n");
			return -1;
		}
    }
    else if (argc == 2 && !strcmp(argv[1], "soc")) {
		soc = htc_batt_get_soc();
		if(soc != -1){
			shell_info("Battery soc: %d\r\n",soc);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Battery soc: %d\r\n",soc);
			return 0;
		}else{
			shell_err("Get battery soc fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery soc fail\r\n");
			return -1;
		}
    }
	else if (argc == 2 && !strcmp(argv[1], "temp")) {
		temperature = htc_batt_get_temperature();
		if(soc != -1){
			shell_info("battery temperature: %d\r\n",temperature/10);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("battery temperature: %d\r\n",temperature/10);
			return 0;
		}else{
			shell_err("Get battery temperature fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery temperature fail\r\n");
			return -1;
		}
	}
	else if (argc == 2 && !strcmp(argv[1], "voltage")) {
		voltage = htc_batt_get_voltage();
		if(voltage != -1){
			shell_info("battery voltage: %d\r\n",voltage);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("battery voltage: %d\r\n",voltage);
			return 0;
		}else{
			shell_err("Get battery voltage fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery voltage fail\r\n");
			return -1;
		}
	}
	else if (argc == 2 && !strcmp(argv[1], "fullcapacity")) {
		capacity = htc_batt_get_full_capacity();
		if(capacity != -1){
			shell_info("battery full capacity: %d\r\n",capacity);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("battery full capacity: %d\r\n",capacity);
			return 0;
		}else{
			shell_err("Get battery full capacity fail\r\n");
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Get battery full capacity fail\r\n");
			return -1;
		}
	}
	else {
		shell_info("Input argument error.\r\n");
		if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("Input argument error.\r\n");
		gauge_test_help();
        return -1;
	}
}

extern uint8_t pa_test;
static void charger_test_help()
{
	shell_info("*******charger command list*********\r\n \
					chg getadcvolt\r\n \
					chg getchgstat\r\n \
					chg setpatestflag\r\n");
}

int charger_test_cmd(int argc, char * argv[], _command_source source)
{
	if ((argc > 3) || (!strcmp(argv[1], "?")) || (!strcmp(argv[1], "help"))) {
		charger_test_help();
        	return -1;
	} else if ((argc == 2) && !strcmp(argv[1], "getadcvolt")) {
		float volt;
		htc_get_batt_average_voltage(&volt);
		shell_info("%s: used to test battery ID volt = %.3f\n", __func__, volt);
		if (source == CMD_SOURCE_USBCDC)
			usb_cdc_printf("%s: used to test battery ID volt = %.3f\n", __func__, volt);
		return 0;
	} else if ((argc == 2) && !strcmp(argv[1], "getchgstat")) {
		int charging_stat;
		extern int htc_get_charger_state(void);
		charging_stat = htc_get_charger_state();
		if (charging_stat < 0) {
			shell_info("%s: get charging state failed.\n", __func__);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("%s: get charging state failed.\n", __func__);
		} else {
			shell_info("%s: current charging state = %d.\n", __func__, charging_stat);
			if (source == CMD_SOURCE_USBCDC)
				usb_cdc_printf("%s: current charging state = %d.\n", __func__, charging_stat);
		}
		return 0;
	}  else if ((argc == 2) && !strcmp(argv[1], "setpatestflag")) {
		pa_test = 1;
		shell_info("%s: set PA test flag = %d\n", __func__, pa_test);
		if (source == CMD_SOURCE_USBCDC)
			usb_cdc_printf("%s: set PA test flag = %d\n", __func__, pa_test);
		return 0;
	} else {
		shell_err("Input argument error.\r\n");
		charger_test_help();
		return -1;
	}
}

void rtc_test_help(void)
{
	shell_info("*******RTC Command list******\r\n");
	shell_info("	   rtc gettime\r\n");
	shell_info("	   rtc getdate\r\n");
	shell_info("	   rtc getalarmA\r\n");
	shell_info("	   rtc getalarmB\r\n");
	shell_info("	   rtc settime\r\n");
	shell_info("	   rtc setdate\r\n");
	shell_info("	   rtc setalarmA\r\n");
	shell_info("	   rtc setalarmB\r\n");
	shell_info("	   rtc settimer\r\n");
	shell_info("	   rtc ?\r\n");
	shell_info("************end************\r\n");

}

static int rtc_test_cmd(int argc, char * argv[], _command_source source)
{
	int ret = 0;
	uint8_t u_hour = 0, u_min = 0, u_sec = 0;
	uint8_t u_year = 0, u_month = 0, u_day = 0;
	RTC_AlarmTypeDef sAlarm = { {0} };

	if (argc == 2)
	{
		if (!strcmp(argv[1], "gettime"))
		{

			ret = rtc_drv_getTime(&u_hour, &u_min, &u_sec);
			if(ret == 0)
				shell_info("Current time is %02d:%02d:%02d\n", u_hour, u_min, u_sec);
			else
			{
				shell_err("rtctest get time ERROR\n");
				return -1;
			}

		}
		else if (!strcmp(argv[1], "getdate"))
		{

			ret = rtc_drv_getDate(&u_year, &u_month, &u_day);
			if(ret == 0)
				shell_info("Current time is %d/%02d/%02d\n", (u_year + 2000), u_month, u_day);
			else
			{
				shell_err("rtctest get date ERROR!\n");
				return -1;
			}

		}
		else if (!strcmp(argv[1], "getalarmA"))
		{
			ret = mcu_rtc_get_alarm(&sAlarm, RTC_ALARM_A);
			if(ret == 0)
				shell_info("Current Alarm A time is %02d:%02d:%02d\n",
					sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds);
			else
			{
				shell_err("rtctest get alarm A ERROR!\n");
				return -1;
			}
		}
		else if (!strcmp(argv[1], "getalarmB"))
		{
			ret = mcu_rtc_get_alarm(&sAlarm, RTC_ALARM_B);
			if(ret == 0)
				shell_info("Current Alarm B time is %02d:%02d:%02d\n",
					sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds);
			else
			{
				shell_err("rtctest get alarm B ERROR!\n");
				return -1;
			}

		}
		else if (!strcmp(argv[1], "?"))
		{
			rtc_test_help();
		}
		else
			shell_err("input arguments ERROR!\n");

	}
	else if (argc == 3)
	{
		if  ((!strcmp(argv[1], "settimer")) &&
			(strcmp(argv[2], "disable")))
		{
			uint32_t count  = 0;
			count = atoi(argv[2]);
			ret = mcu_rtc_wakeUp_alarm_set_time(count);
			if(ret != 0)
			{
				shell_err("rtctest Set Wake Up Counter ERROR!\n");
				return -1;
			}
		}
		else if ((!strcmp(argv[1], "settimer")) &&
			(!strcmp(argv[2], "disable")))
		{
			ret = mcu_rtc_wakeUp_alarm_disable();
			if (ret != 0)
			{
				shell_err("rtctest Wake up timer Disable ERROR!\n");
				return -1;
			}
		}
		else
			shell_err("input arguments ERROR!\n");
	}
	else if (argc == 5)
	{
		if (!strcmp(argv[1], "settime"))
		{
			u_hour = atoi(argv[2]);
			u_min = atoi(argv[3]);
			u_sec = atoi(argv[4]);
			ret = rtc_drv_setTime(u_hour, u_min, u_sec);
			if(ret != 0)
			{
				shell_err("rtctest set time ERROR!\n");
				return -1;
			}
		}
		else if (!strcmp(argv[1], "setdate"))
		{
			u_year = atoi(argv[2]);
			u_month = atoi(argv[3]);
			u_day = atoi(argv[4]);
			ret = rtc_drv_setDate(u_year, u_month, u_day);
			if(ret != 0)
			{
				shell_err("rtctest set set date ERROR!\n");
				return -1;
			}
		}
		else if (!strcmp(argv[1], "setalarmA"))
		{
			u_hour = atoi(argv[2]);
			u_min = atoi(argv[3]);
			u_sec = atoi(argv[4]);
			rtc_drv_setAlarmA(u_hour, u_min, u_sec);
		}
		else if (!strcmp(argv[1], "setalarmB"))
		{
			u_hour = atoi(argv[2]);
			u_min = atoi(argv[3]);
			u_sec = atoi(argv[4]);
			rtc_drv_setAlarmB(u_hour, u_min, u_sec);

		}
		else
			shell_err("input arguments ERROR!\n");
	}
	else
		shell_err("input arguments ERROR!\n");

	return 0;
}

void disp_test_help(void)
{
	shell_info("*******DISP Command list******\r\n");
	shell_info("	   disp h2d getid\r\n");
	shell_info("	   disp d2h getid\r\n");
//	shell_info("	   disp lcd getid\r\n");
	shell_info("	   disp lcd suspend\r\n");
	shell_info("	   disp lcd resume\r\n");
	shell_info("	   disp lcd backlight [value](0-255)\r\n");
	shell_info("	   disp fps {dump | [value](0:m10,1:ocean note,2:ocean)}\r\n");
	shell_info("	   disp power off\r\n");
	shell_info("	   disp power on\r\n");
	shell_info("	   disp test colorbar\r\n");
	shell_info("	   disp test dummy\r\n");
	shell_info("	   disp test hdcp\r\n");
	shell_info("	   disp test dummy24\r\n");
	shell_info("	   disp read comp\r\n");
	shell_info("	   disp write comp\r\n");
	shell_info("	   disp read gamma\r\n");
	shell_info("	   disp read aid\r\n");
	shell_info("	   disp write aid\r\n");
	shell_info("************end************\r\n");

}

int disp_test_cmd(int argc, char * argv[], _command_source source)
{
    uint16_t id = 0;
    uint8_t b_val = 0;
	pcbid_t pcbid;

    if (argc >= 3) {
        if (!strcmp(argv[1], "h2d")) {
            if (!strcmp(argv[2], "getid")) {
                if (disp_h2d_getid(&id) == 0) {
                    shell_info("h2d chip id 0x%04x.\n", id);
                } else {
                    shell_info("can't got h2d chip id.\n");
                }
                return 0;
            }
        }

        if (!strcmp(argv[1], "d2h")) {
            if (!strcmp(argv[2], "getid")) {
                uint8_t ids[6];
                get_pcbid(&pcbid);
                if ((XA0n == pcbid) && (disp_d2h_getid(ids, 6) == 0)) {
                    shell_info("d2h chip id 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x.\n",
                            ids[0], ids[1], ids[2], ids[3], ids[4], ids[5]);
                } else {
                    shell_info("can't got d2h chip id.\n");
                }
                return 0;
            }
        }

#if 0
		if (!strcmp(argv[1], "lcd")) {
			if (!strcmp(argv[2], "getid")) {
				if (disp_show_lcd_id()) {
					shell_info("can't got lcd chip id.\n");
                }

                return 0;
			}
		}
#endif

        if (!strcmp(argv[1], "lcd")) {
            if (!strcmp(argv[2], "suspend")) {
                disp_lcd_suspend();
                return 0;
            }
            if (!strcmp(argv[2], "resume")) {
                disp_lcd_resume();
                return 0;
            }
            if (!strcmp(argv[2], "backlight")) {
                if (argc >= 4) {
                    b_val = atoi(argv[3]);
                    disp_lcd_backlight(b_val);
                    return 0;
                }
            }
        }
        if (!strcmp(argv[1], "power")) {
            if (!strcmp(argv[2], "off")) {
                disp_power_off();
                return 0;
            }
            if (!strcmp(argv[2], "on")) {
                disp_power_on();
                return 0;
            }
        }

		if (!strcmp(argv[1], "fps")) {
            if (!strcmp(argv[2], "dump")) {
				float fps = disp_fps_get();
				shell_info("got disp fps = %f.\n", fps);
			} else {
				b_val = atoi(argv[2]);
				if (disp_phone_type_set(b_val) == -1) {
				    shell_info("error phone type, 0:m10,1:ocean note,2:ocean.\n");
				}
			}
			return 0;
		}

        if (!strcmp(argv[1], "test")) {
            if (!strcmp(argv[2], "colorbar")) {
                disp_test_colorbar();
                return 0;
            }
            if (!strcmp(argv[2], "dummy")) {
                disp_test_dummy();
                return 0;
            }
            if (!strcmp(argv[2], "hdcp")) {
                disp_test_hdcp();
                return 0;
            }
            if (!strcmp(argv[2], "dummy24")) {
                disp_test_dummy24();
                return 0;
            }
        }
        if (!strcmp(argv[1], "read")) {
            if (!strcmp(argv[2], "comp")) {
                disp_read_comp();
                return 0;
            }
            if (!strcmp(argv[2], "gamma")) {
                disp_read_gamma();
                return 0;
            }
            if (!strcmp(argv[2], "aid")) {
                disp_read_aid();
                return 0;
            }
        }
        if (!strcmp(argv[1], "write")) {
            if (!strcmp(argv[2], "comp")) {
                if (argc >= 3) {
                    disp_write_comp(argv[3]);
                    return 0;
                }
            }
            if (!strcmp(argv[2], "aid")) {
                if (argc >= 3) {
                    disp_write_aid(argv[3]);
                    return 0;
                }
            }
        }
    }

    disp_test_help();
    return -1;
}

int cmd_help(int argc, char * argv[], _command_source source);

static int hlog_cmd(int argc, char * argv[], _command_source source)
{
    if(argc ==2){
    	if (!strcmp((char const*)argv[1], "dump")){
            hlog_dump_logbuf();
    	}
        else if (!strcmp((char const*)argv[1], "usbdump")){
            hlog_cdc_dump_logbuf();
    	}
        else if (!strcmp((char const*)argv[1], "flashdump")){
            hlog_dump_flashlog();
    	}
    	else if (!strcmp((char const*)argv[1], "clear")){
            hlog_clear_logbuf();
    	}
    	else if (!strcmp((char const*)argv[1], "flashclear")){
            hlog_erase_flashlog();
    	}
        else if(!strcmp((char const*)argv[1], "exptest")){
            #ifdef MVR_DEBUG_BUILD
            ((void (*)(void))(0x00001000))();
            #endif
        }
        else if(!strcmp((char const*)argv[1], "wdttest")){
            #ifdef MVR_DEBUG_BUILD
            vTaskSuspendAll();
            #endif
        }
    	else{
            shell_err("input arguments ERROR!\n");
    	}
    }
    else if(argc ==3){
    	if (!strcmp((char const*)argv[1], "get")){
            if (!strcmp((char const*)argv[2], "level")){
                shell_info("cur_log_level=%d\r\n",hlog_get_level());
            }
    	}
    }
    else if(argc ==4){
    	if (!strcmp((char const*)argv[1], "set")){
            if (!strcmp((char const*)argv[2], "level")){
                hlog_set_level(atoi(argv[3]));
            }
    	}
    }
    else{
        shell_err("input arguments ERROR!\n");
    }
    return 0;
}

int cmd_misc_data(int argc, char * argv[], _command_source source)
{
    char dec[MISC_PARAMETER_SIZE]={0};
    float para;
    BOOL para_Bool;
    int32_t para_int32;
    pcbid_t pcbid;
    int value;
    char usb_buffer[PARAMETER_SIZE] = {0};
    float f_value[8];
    double d_value[2];
    pcbid_t my_pcbid;
    int result;
    if(argc == 2){
        if (!strcmp((char const*)argv[1], "dump")) {
          if(source == CMD_SOURCE_USBCDC) {
              dump_property_for_usb();
          } else {
              shell_warning("This command just support usb port.\n");
              return -1;
          }
        } else {
          shell_err("input arguments ERROR!\n");
          return -1;
        }
    }
    else if(argc == 3)
    {
        if (!strcmp((char const*)argv[1], "get"))
        {
            if (!strcmp((char const*)argv[2], "TrSysName"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getTrackingSystemName(dec);
                shell_info("TrackingSystemName:%s\n",dec);
            }
            else if (!strcmp((char const*)argv[2], "SerialNum"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getSerialNumber(dec);
                shell_info("SerialNumber:%s\n",dec);
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"SerialNumber:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "MaFacName"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getManufacturerName(dec);
                shell_info("ManufacturerName:%s\n",dec);
            }
            else if (!strcmp((char const*)argv[2], "TrFirmVer"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getTrackingFirmwareVersion(dec);
                shell_info("TrackingFirmwareVersion:%s\n",dec);
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"TrackingFirmwareVersion:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "HardRevision_str"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getHardwareRevision_string(dec);
                shell_info("HardwareRevision_string:%s\n",dec);
            }
            else if (!strcmp((char const*)argv[2], "DevIsWireless"))
            {
                shell_info("DeviceIsWireless:%d\n",getDeviceIsWireless());
            }
            else if (!strcmp((char const*)argv[2], "DevIsCharging"))
            {
                shell_info("DeviceIsCharging:%d\n",getDeviceIsCharging());
            }
            else if (!strcmp((char const*)argv[2], "DevProBatSta"))
            {
                shell_info("DeviceProvidesBatteryStatus:%d\n",getDeviceProvidesBatteryStatus());
            }
            else if (!strcmp((char const*)argv[2], "DevCanPowerOff"))
            {
                shell_info("DeviceCanPowerOff:%d\n",getDeviceCanPowerOff());
            }
            else if (!strcmp((char const*)argv[2], "ContainsProximitySensor"))
            {
                shell_info("ContainsProximitySensor:%d\n",getContainsProximitySensor());
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"ContainsProximitySensor:%d\r\n",getContainsProximitySensor());
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "HasCamera"))
            {
                shell_info("HasCamera:%d\n",getHasCamera());
            }
            else if (!strcmp((char const*)argv[2], "FirmupdateAva"))
            {
                shell_info("Firmware_UpdateAvailable:%d\n",getFirmware_UpdateAvailable());
            }
            else if (!strcmp((char const*)argv[2], "DevBatPer"))
            {
                shell_info("DeviceBatteryPercentage:%f\n",getDeviceBatteryPercentage());
            }
            else if (!strcmp((char const*)argv[2], "UserIpdMeters"))
            {
                shell_info("UserIpdMeters:%f\n",getUserIpdMeters());
                if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("UserIpdMeters:%f\r\n",getUserIpdMeters());
                    }
            }
            else if (!strcmp((char const*)argv[2], "LensCenterLeftU"))
            {
                shell_info("LensCenterLeftU:%f\n",getLensCenterLeftU());
            }
            else if (!strcmp((char const*)argv[2], "LensCenterLeftV"))
            {
                shell_info("LensCenterLeftV:%f\n",getLensCenterLeftV());
            }
            else if (!strcmp((char const*)argv[2], "LensCenterRightU"))
            {
                shell_info("LensCenterRightU:%f\n",getLensCenterRightU());
            }
            else if (!strcmp((char const*)argv[2], "LensCenterRightV"))
            {
                shell_info("LensCenterRightV:%f\n",getLensCenterRightV());
            }
            else if (!strcmp((char const*)argv[2], "UserHeadToEyeDepthMeters"))
            {
                shell_info("UserHeadToEyeDepthMeters:%f\n",getUserHeadToEyeDepthMeters());
            }
            else if (!strcmp((char const*)argv[2], "HardRevision_int"))
            {
                shell_info("dwMFGAreaSignature:0x%x\n",getHardwareRevision_int());
            }
            else if (!strcmp((char const*)argv[2], "FirmVer"))
            {
                shell_info("FirmwareVersion:%d\n",getFirmwareVersion());
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"FirmwareVersion_int:%d\r\n",getFirmwareVersion());
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "pcbid"))
            {
                get_pcbid(&pcbid);
                shell_info("pcbid:0x%x\n",pcbid);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"pcbid:0x%x\r\n",pcbid);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "BoardInfoMagicNb"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getBoardInformationMagicNumber(dec);
                shell_info("BoardInformationMagicNumber:%s\n",dec);
            }
            else if (!strcmp((char const*)argv[2], "customid"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getCustomID(dec);
                shell_info("CustomID:%s\n",dec);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"CustomID:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "skuid"))
            {
                get_skubid(&value);
                shell_info("SkuID:0x%x\n",value);
            }
            else if (!strcmp((char const*)argv[2], "FirmMainVer"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                getFirmwareMainVersion(dec);
                shell_info("FirmwareMainVersion:%s\n",dec);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"FirmwareMainVersion:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "modelname"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getModelName(dec);
                shell_info("ModelName:%s\n",dec);
            }
            else if (!strcmp((char const*)argv[2], "cProductDate"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getcProductDate(dec);
                shell_info("cProductDate:%s\n",dec);
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"cProductDate:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "MBSerialNumber"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getMBSerialNumber(dec);
                shell_info("MBSerialNumber:%s\n",dec);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"MBSerialNumber:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "colorid"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getColorID(dec);
                shell_info("ColorID:%s\n",dec);
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"ColorID:%s\r\n",dec);
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "projectid"))
            {
                memset(dec,'\0',MISC_PARAMETER_SIZE);
                getProjectID(dec);
                shell_info("ProjectID:%s\n",dec);
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("ProjectID:%s\r\n",dec);
                }
            }
            else if (!strcmp((char const*)argv[2], "dwMFGAreaSignature"))
            {
                getdwMFGAreaSignature(&para_int32);
                shell_info("dwMFGAreaSignature:0x%x\n",para_int32);
            }
            else if (!strcmp((char const*)argv[2], "DeviceClass"))
            {
                getDeviceClass(&value);
                shell_info("DeviceClass:%d\n",value);
            }
            else if (!strcmp((char const*)argv[2], "engineerid"))
            {
                getEngineerID(&value);
                shell_info("EngineerID:%d\n",value);
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("EngineerID:%d\r\n",value);
                }
            }
            else if (!strcmp((char const*)argv[2], "CameraToHeadTransform"))
            {
                getCameraToHeadTransform_Matrix34();
            }
            else if (!strcmp((char const*)argv[2], "StatusDisplayTransform"))
            {
                getStatusDisplayTransform_Matrix34();
            }
            else if (!strcmp((char const*)argv[2], "default"))
            {
                if (!getDefaultData())
                    shell_info("misc get default success.\r\n");
                else {
                    shell_err("misc get default fail.\r\n");
                    return -1;
                }
            }
			else if (!strcmp((char const*)argv[2],"semi")){
				char print_buf[30];
				SEMI_STATUS ear_pod,jack,dp;

				memset(print_buf,0,sizeof(print_buf));
				if(!getSemiPara(&ear_pod,0)){
					sprintf(print_buf,"semi earpod test %s\r\n",
					(ear_pod==SEMI_FINISH)?"finish":"unfinish");
				}else {
					sprintf(print_buf,"get semi earpodtest fail\r\n");
				}
				shell_info(print_buf);
				if(source == CMD_SOURCE_USBCDC){
					usb_cdc_printf(print_buf);
				}

				memset(print_buf,0,sizeof(print_buf));
				if(!getSemiPara(&jack,1)){
					sprintf(print_buf,"semi jack test %s\r\n",
					(jack==SEMI_FINISH)?"finish":"unfinish");
				}else {
					sprintf(print_buf,"get semi jack test fail\r\n");
				}
				shell_info(print_buf);
				if(source == CMD_SOURCE_USBCDC){
					usb_cdc_printf(print_buf);
				}

				memset(print_buf,0,sizeof(print_buf));
				if(!getSemiPara(&dp,2)){
					sprintf(print_buf,"semi display test %s\r\n",
					(dp==SEMI_FINISH)?"finish":"unfinish");
				}else {
					sprintf(print_buf,"get semi display test fail\r\n");
				}
				shell_info(print_buf);
				if(source == CMD_SOURCE_USBCDC){
					usb_cdc_printf(print_buf);
				}

			}
            else if (!strcmp((char const*)argv[2], "BuildTime")){
                extern char build_date[16];
                extern char build_time[16];
                shell_info("\nSystem build @ %s-%s\n\n",build_date, build_time);
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("\nSystem build @ %s-%s\n\n",build_date, build_time);
                }
            }
            else
            {
                shell_err("input arguments ERROR!\n");
                return -1;
            }
        }
	else
        {
            shell_err("input arguments ERROR!\n");
            return -1;
        }
    }
    else if(argc ==4)
    {
        if (!strcmp((char const*)argv[1], "set"))
        {
            if (!strcmp((char const*)argv[2], "TrSysName"))
            {
                result = setTrackingSystemName((const char *)argv[3],
                                               strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setTrackingSystemName success\n");
                } else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "SerialNum"))
            {
                result = setSerialNumber((const char *)argv[3],
                                         strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setSerialNumber success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setSerialNumber success\r\n");
                    }
                } else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "MaFacName"))
            {
                result = setManufacturerName((const char *)argv[3],
                                             strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setManufacturerName success\n");
                } else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                }
            }
#if 0
            else if (!strcmp((char const*)argv[2], "TrFirmVer"))
            {
                setTrackingFirmwareVersion((char const*)argv[3],
			       strlen((char *)argv[3]));
                shell_info("setTrackingFirmwareVersion success\n");
            }
#endif
            else if (!strcmp((char const*)argv[2], "HardRevision_str"))
            {
                result = setHardwareRevision_string((const char *)argv[3],
                                                    strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setHardwareRevision_string success\n");
                } else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "DevIsWireless"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setDeviceIsWireless((BOOL)para_Bool);
                shell_info("setDeviceIsWireless success\n");
            }
            else if (!strcmp((char const*)argv[2], "DevIsCharging"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setDeviceIsCharging((BOOL)para_Bool);
                shell_info("setDeviceIsCharging success\n");
            }
            else if (!strcmp((char const*)argv[2], "DevProBatSta"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setDeviceProvidesBatteryStatus((BOOL)para_Bool);
                shell_info("setDeviceProvidesBatteryStatus success\n");
            }
            else if (!strcmp((char const*)argv[2], "DevCanPowerOff"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setDeviceCanPowerOff((BOOL)para_Bool);
                shell_info("setDeviceCanPowerOff success\n");
            }
            else if (!strcmp((char const*)argv[2], "ContainsProximitySensor"))
            {
                value = atoi((char const*)argv[3]);
                setContainsProximitySensor(value);
                shell_info("setContainsProximitySensor success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    sprintf(usb_buffer,"setContainsProximitySensor success\r\n");
                    usb_cdc_transmit_data((uint8_t *)usb_buffer,strlen(usb_buffer));
                }
            }
            else if (!strcmp((char const*)argv[2], "HasCamera"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setHasCamera((BOOL)para_Bool);
                shell_info("setHasCamera success\n");
            }
            else if (!strcmp((char const*)argv[2], "FirmupdateAva"))
            {
                para_Bool = atoi((char const*)argv[3]);
                setFirmware_UpdateAvailable((BOOL)para_Bool);
                shell_info("setFirmware_UpdateAvailable success\n");
            }
            else if (!strcmp((char const*)argv[2], "DevBatPer"))
            {
                para = atof((char const*)argv[3]);
                setDeviceBatteryPercentage((float)para);
                shell_info("setDeviceBatteryPercentage success\n");
            }
            else if (!strcmp((char const*)argv[2], "UserIpdMeters"))
            {
                para = atof((char const*)argv[3]);
                setUserIpdMeters((float)para);
                shell_info("setUserIpdMeters success\n");
                if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setUserIpdMeters success\r\n");
                    }
            }
            else if (!strcmp((char const*)argv[2], "LensCenterLeftU"))
            {
                para = atof((char const*)argv[3]);
                setLensCenterLeftU((float)para);
                shell_info("setLensCenterLeftU success\n");
            }
            else if (!strcmp((char const*)argv[2], "LensCenterLeftV"))
            {
                para = atof((char const*)argv[3]);
                setLensCenterLeftV((float)para);
                shell_info("setLensCenterLeftV success\n");
            }
            else if (!strcmp((char const*)argv[2], "LensCenterRightU"))
            {
                para = atof((char const*)argv[3]);
                setLensCenterRightU((float)para);
                shell_info("setLensCenterRightU success\n");
            }
            else if (!strcmp((char const*)argv[2], "LensCenterRightV"))
            {
                para = atof((char const*)argv[3]);
                setLensCenterRightV((float)para);
                shell_info("setLensCenterRightV success\n");
            }
            else if (!strcmp((char const*)argv[2], "UserHeadToEyeDepthMeters"))
            {
                para = atof((char const*)argv[3]);
                setUserHeadToEyeDepthMeters((float)para);
                shell_info("setUserHeadToEyeDepthMeters success\n");
            }
            else if (!strcmp((char const*)argv[2], "HardRevision_int"))
            {
                para_int32 = strtoul((char const*)argv[3], NULL, 16);
                setdwMFGAreaSignature(para_int32);
                shell_info("setHardwareRevision_int success\n");
            }
#if 0
            else if (!strcmp((char const*)argv[2], "FirmVer"))
            {
                value = atoi((char const*)argv[3]);
                setFirmwareVersion(value);
                shell_info("setFirmwareVersion success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_transmit_data("setFirmwareVersion success\r\n",strlen("setFirmwareVersion success\r\n"));
                }
            }
#endif
            else if (!strcmp((char const*)argv[2], "pcbid"))
            {
                para_int32 = strtoul((char const*)argv[3], NULL, 16);
                if (set_pcbid((pcbid_t)para_int32) < 0) {
                    shell_err("set pcb id failed\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_transmit_data("set pcb id failed\r\n",strlen("set pcb id failed\r\n"));
                    }
                    return -1;
                }
                else {
                    shell_info("set pcb id success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_transmit_data("set pcb id success\r\n",strlen("set pcb id success\r\n"));
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "BoardInfoMagicNb"))
            {
                result = setBoardInformationMagicNumber((const char *)argv[3],
                                                        strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setBoardInformationMagicNumber success\n");
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "customid"))
            {
                result = setCustomID((const char *)argv[3],
                                     strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setCustomID success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setCustomID success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "skuid"))
            {
                value = strtoul((char const*)argv[3], NULL, 16);
                if (set_skuid((int)value) < 0) {
                    shell_err("set sku id failed\n");
                    return -1;
                }
                else
                    shell_info("set sku id success\n");
            }
#if 0
            else if (!strcmp((char const*)argv[2], "FirmMainVer"))
            {
                int i,j;
                setFirmwareMainVersion((const char*)argv[3],
                                       strlen((char *)argv[3]));
                setTrackingFirmwareVersion((char const*)argv[3],
                                           strlen((char *)argv[3]));
                shell_info("setFirmwareMainVersion success\n");
                shell_info("setTrackingFirmwareVersion success\n");
                memset(usb_buffer,'\0',PARAMETER_SIZE);
                j = 0;
                for (i=0;i<strlen((char *)argv[3]);i++) {
                    if(argv[3][i] == '.'){
                      continue;
                    }
                    else{
                        usb_buffer[j] = argv[3][i];
                        j++;
                    }
                }
                usb_buffer[j] = '\0';
                value = atoi((char const*)usb_buffer);
                setFirmwareVersion(value);
                shell_info("setFirmwareVersion success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_transmit_data("setFirmwareMainVersion success\r\n",strlen("setFirmwareMainVersion success\r\n"));
                    usb_cdc_transmit_data("setTrackingFirmwareVersion success\r\n",strlen("setTrackingFirmwareVersion success\r\n"));
                    usb_cdc_transmit_data("setFirmwareVersion_int success\r\n",strlen("setFirmwareVersion_int success\r\n"));
                }
            }
#endif
            else if (!strcmp((char const*)argv[2], "modelname"))
            {
                result = setModelName((const char *)argv[3],
                                      strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setModelName success\n");
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "cProductDate"))
            {
                result = setcProductDate((const char *)argv[3],
                                         strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setcProductDate success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setcProductDate success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "MBSerialNumber"))
            {
                result = setMBSerialNumber((const char *)argv[3],
                                           strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setMBSerialNumber success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setMBSerialNumber success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "colorid"))
            {
                result = setColorID((const char *)argv[3],
                                    strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setColorID success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setColorID success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "projectid"))
            {
                result = setProjectID((const char *)argv[3],
                                    strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setProjectID success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setProjectID success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "dwMFGAreaSignature"))
            {
                para_int32 = strtoul((char const*)argv[3], NULL, 16);
                setdwMFGAreaSignature(para_int32);
                shell_info("setdwMFGAreaSignature success\n");
            }
            else if (!strcmp((char const*)argv[2], "DeviceClass"))
            {
                value = atoi((char const*)argv[3]);
                setDeviceClass(value);
                shell_info("setDeviceClass success\n");
            }
            else if (!strcmp((char const*)argv[2], "engineerid"))
            {
                value = atoi((char const*)argv[3]);
                setEngineerID(value);
                shell_info("setEngineerID success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setEngineerID success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "HorizontalFOV"))
            {
                para = atof((char const*)argv[3]);
                setScreenshotHorizontalFOV((float)para);
                shell_info("setScreenshotHorizontalFOV success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setScreenshotHorizontalFOV success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "VerticalFOV"))
            {
                para = atof((char const*)argv[3]);
                setScreenshotVerticalFOV((float)para);
                shell_info("setScreenshotVerticalFOV success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setScreenshotVerticalFOV success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "ContainsRecenter"))
            {
                value = atoi((char const*)argv[3]);
                setContainsRecenter(value);
                shell_info("setContainsRecenter success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setContainsRecenter success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "DistanceEyeToLens"))
            {
                para = atof((char const*)argv[3]);
                setDistanceEyeToLens((float)para);
                shell_info("setDistanceEyeToLens success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setDistanceEyeToLens success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "DistanceLensToScreen"))
            {
                para = atof((char const*)argv[3]);
                setDistanceLensToScreen((float)para);
                shell_info("setDistanceLensToScreen success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setDistanceLensToScreen success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "LensFocalLength"))
            {
                para = atof((char const*)argv[3]);
                setLensFocalLength((float)para);
                shell_info("setLensFocalLength success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setLensFocalLength success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "DistanceScaleX"))
            {
                para = atof((char const*)argv[3]);
                setDistanceScaleX((float)para);
                shell_info("setDistanceScaleX success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setDistanceScaleX success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "DistanceScaleY"))
            {
                para = atof((char const*)argv[3]);
                setDistanceScaleY((float)para);
                shell_info("setDistanceScaleY success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setDistanceScaleY success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "RenderOverfill"))
            {
                para = atof((char const*)argv[3]);
                setRenderOverfill((float)para);
                shell_info("setRenderOverfill success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setRenderOverfill success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "VendorPartNum"))
            {
                result = setVendorPartNumber((const char *)argv[3],
                                             strlen((char *)argv[3]));
                if(0 == result) {
                    shell_info("setVendorPartNumber success\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("setVendorPartNumber success\r\n");
                    }
                }else if(-2 == result) {
                    shell_err("The paramater overflow\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("The paramater overflow\r\n");
                    }
                }
            }
            else if (!strcmp((char const*)argv[2], "DisplayOnDevice"))
            {
                value = atoi((char const*)argv[3]);
                setDisplayOnDevice(value);
                shell_info("setDisplayOnDevice success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setDisplayOnDevice success\r\n");
                }
            }
            else if(!strcmp((char const*)argv[2],"semi"))
            {
                uint8_t offset;
                SEMI_STATUS value;
                char print_buf[30];
                if (!strcmp((char const*)argv[3], "earpod_yes")){
                    offset = 0;
                    value = SEMI_FINISH;
                }
                else if(!strcmp((char const*)argv[3], "earpod_no")){
                    offset = 0;
                    value = SEMI_UNFINISH;
                }
                else if (!strcmp((char const*)argv[3], "jack_yes")){
                    offset = 1;
                    value = SEMI_FINISH;
                }
                else if (!strcmp((char const*)argv[3], "jack_no")){
                    offset = 1;
                    value = SEMI_UNFINISH;
                }
                else if (!strcmp((char const*)argv[3], "disp_yes")){
                    offset = 2;
                    value= SEMI_FINISH;
               }
                else if (!strcmp((char const*)argv[3], "disp_no")){
                    offset = 2;
                    value= SEMI_UNFINISH;
               }
               else{
                    memset(print_buf,0,sizeof(print_buf));
                    sprintf(print_buf,"misc set semi fail\r\n");
                    shell_err(print_buf);
                    if(source==CMD_SOURCE_USBCDC)
                        usb_cdc_printf(print_buf);
                    return -1;
                }
                if(!setSemiPara(&value,offset)){
                    memset(print_buf,0,sizeof(print_buf));
                    sprintf(print_buf,"misc set semi success\r\n");
                    shell_err(print_buf);

                    if(source==CMD_SOURCE_USBCDC)
                        usb_cdc_printf(print_buf);
                    return 0;
                 }else{
                     memset(print_buf,0,sizeof(print_buf));
                     sprintf(print_buf,"misc set semi fail\r\n");
                     shell_err(print_buf);

                     if(source==CMD_SOURCE_USBCDC)
                         usb_cdc_printf(print_buf);
                     return -1;

				}

			}
            else if (!strcmp((char const*)argv[2], "default"))
            {
            	my_pcbid = UNKNOWN;
            	if (!strcmp((char const*)argv[3], "XB02"))
                    my_pcbid = XB02;
                if (!strcmp((char const*)argv[3], "XA0n"))
                    my_pcbid = XA0n;
                if (!strcmp((char const*)argv[3], "XC01"))
                    my_pcbid = XC01;
                if (!strcmp((char const*)argv[3], "XC02"))
                    my_pcbid = XC02;
                if (my_pcbid == UNKNOWN) {
                    shell_err("misc set default fail, invalid PCBID\r\n");
                    return -1;
                }
                if(!setDefaultData(my_pcbid))
                    shell_info("misc set default success.\r\n");
                else {
                    shell_err("misc set default fail.\r\n");
                    return -1;
                }
            }
            else
            {
                shell_err("input arguments ERROR!\n");
                return -1;
            }
        }
        else
        {
            shell_err("input arguments ERROR!\n");
            return -1;
        }
    }
    else if(argc == 5) {
        if(!strcmp((char const*)argv[1], "set")){
            if (!strcmp((char const*)argv[2], "RecommendedRenderTargetSize"))
            {
                d_value[0] = atof((char const*)argv[3]);
                d_value[1] = atof((char const*)argv[4]);
                setRecommendedRenderTargetSize(d_value);
                shell_info("setRecommendedRenderTargetSize success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setRecommendedRenderTargetSize success\r\n");
                }
            }
            else if (!strcmp((char const*)argv[2], "RealScreenSize"))
            {
                f_value[0] = atof((char const*)argv[3]);
                f_value[1] = atof((char const*)argv[4]);
                setRealScreenSize(f_value);
                shell_info("setRealScreenSize success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                  usb_cdc_printf("setRealScreenSize success\r\n");
                }
            }
            else
            {
              shell_err("input arguments ERROR!\n");
              return -1;
            }
        }
    }
    else if(argc == 6) {
        if(!strcmp((char const*)argv[1], "set")){
            if (!strcmp((char const*)argv[2], "SensorToHead"))
            {
                f_value[0] = atof((char const*)argv[3]);
                f_value[1] = atof((char const*)argv[4]);
                f_value[2] = atof((char const*)argv[5]);
                setSensorToHead(f_value);
                shell_info("setSensorToHead success\n");
                if (source == CMD_SOURCE_USBCDC)
                {
                    usb_cdc_printf("setSensorToHead success\r\n");
                }
            }
            else
            {
                shell_err("input arguments ERROR!\n");
                return -1;
            }
        }
    }
	else if(argc == 11) {
		if (!strcmp((char const*)argv[1], "set")) {
                    memset(f_value,0,sizeof(f_value));
                    for (int i=0;i<8;i++){
                        f_value[i] = atof((char const*)argv[i+3]);
                    }
                    if (!strcmp((char const*)argv[2], "PCRed")) {
                        setPolynomialCoeffsRed(f_value);
                        if (source == CMD_SOURCE_USBCDC)
                        {
                            usb_cdc_printf("setPolynomialCoeffsRed success\r\n");
                        }
                    }
                    else if(!strcmp((char const*)argv[2], "PCGreen")) {
                        setPolynomialCoeffsGreen(f_value);
                        if (source == CMD_SOURCE_USBCDC)
                        {
                            usb_cdc_printf("setPolynomialCoeffsGreen success\r\n");
                        }
                    }
                    else if(!strcmp((char const*)argv[2], "PCBlue")){
                        setPolynomialCoeffsBlue(f_value);
                        if (source == CMD_SOURCE_USBCDC)
                        {
                            usb_cdc_printf("setPolynomialCoeffsBlue success\r\n");
                        }
                    }
                    else {
                        shell_err("input arguments ERROR!\n");
                        if (source == CMD_SOURCE_USBCDC)
                        {
                            usb_cdc_printf("input arguments ERROR!\r\n");
                        }
                    }
                }else {
                    shell_err("input arguments ERROR!\n");
                    if (source == CMD_SOURCE_USBCDC)
                    {
                        usb_cdc_printf("input arguments ERROR!\r\n");
                    }
                }
        }
        else
        {
            shell_err("input arguments ERROR!\n");
            return -1;
        }
	return 0;
}

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

     if (argc == 5) {
               int fw_idx;
               unsigned offset;
               unsigned size;

               if (!strcmp((char const*)argv[1], "upgrade")) {

                    fw_idx = strtoul(argv[2], NULL, 0);
                    offset = strtoul(argv[3], NULL, 0);
                    size = strtoul(argv[4], NULL, 0);

                    printf("FW%d : 0x%08x@0x%08x\n", fw_idx + 1, size, offset);

                    ccgx_set_fw_info(fw_idx, offset, size);
                    ccgx_upgrade_fw(fw_idx);
                    return 0;
               }

     }

     shell_info("Usage : \n");
     shell_info("\t %s dump\n", argv[0]);
     shell_info("\t %s reg addr count\n", argv[0]);
     shell_info("\t %s upgrade fw_idx offset size\n", argv[0]);

     return 0;
}
#define RAM_WRITE_LEN 256

static int ramverify(uint32_t u_size)
{
    char* s_TestPtr = NULL;
    int b_ret = 0;
    uint32_t u_idx;
    uint8_t u_PassCnt = 0;
    uint32_t u_length = 0;

    s_TestPtr = (char*)malloc(u_size);
    if(s_TestPtr == NULL)
    {
        shell_err("%s allocate %d failed\n", __func__, u_size);
        return -1;
    }
    else
    {
        for(u_length = 0; u_length < u_size; u_length += RAM_WRITE_LEN)
        {
            //Write
            for(u_idx = 0; u_idx < RAM_WRITE_LEN; u_idx++)
            {
                s_TestPtr[u_length+u_idx] = u_idx;
            }

            //Read to verify
            for(u_idx = 0; u_idx < RAM_WRITE_LEN; u_idx++)
            {
                if (s_TestPtr[u_idx+u_length] != u_idx)
                {
                    shell_err("verify failed, 0x%8x, %d\n", &s_TestPtr[u_idx+u_length], u_idx);
                    b_ret = -1;
                    break;
                }
            }
            if(b_ret == 0)
            {
                u_PassCnt++;
            }
            else
            {
                break;
            }
            shell_info("verify Address, 0x%8x------------Pass\n",&s_TestPtr[u_length]);

        }
    }

    if(b_ret == 0)
    {
        if(u_PassCnt == (u_size/RAM_WRITE_LEN))
        {
            b_ret = 0;
        }
    }

    free(s_TestPtr);
    return b_ret;
}


static int cmd_ramtest(int argc, char * argv[], _command_source source)
{
    uint16_t cycle;
    uint32_t u_size;

    if (argc < 2 || argc > 3) {
        shell_err("usage: ramtest [size] [cycle]\n");
        return -1;
    }

    /* Get First Parameter for req size */
    u_size = atoi((char const*)argv[1]);

    /* Get Second Parameter for req cycle */
    if (argc == 2)
        cycle = 1;
    else {
        cycle = atoi((char const*)argv[2]);
    }

    /* 256bytes alignment */
    u_size -= u_size % RAM_WRITE_LEN;

    shell_info("ramtest [%u] [%u]\n", u_size, cycle);

    while (cycle--) {
        if (ramverify(u_size)) {
            shell_err("ramtest failed\n");
            return -1;
        }
    }

    shell_info("ramtest done\n");

    return 0;
}

#if SENSOR_LATENCY_TEST
extern uint32_t latency_data[SENSOR_LATENCY_TEST_COUNT][SENSOR_LATENCY_TEST_POINT];
extern volatile uint32_t latency_index;
extern uint32_t latency_lock;
extern void watchdog_refresh();

#endif
static int performance_test(int argc, char * argv[], _command_source source)
{
#if SENSOR_LATENCY_TEST
    uint32_t i = 0;
    uint32_t index;
    uint32_t sum[3] = {0};
    latency_lock = 1;
    osDelay(10); // wait for lock effect

    index= latency_index;
    shell_info("latency_index = %d\r\n", index);
    for(i=0; i<SENSOR_LATENCY_TEST_COUNT; i++){
        watchdog_refresh();
        shell_info("<%2d> Notify:%d \t read data:%d \t IMU:%d \r\n",i,
            (latency_data[i][1]-latency_data[i][0]),
            (latency_data[i][2]-latency_data[i][1]),
            (latency_data[i][3]-latency_data[i][2]));
        if(i == index) continue;
        sum[0] += (latency_data[i][1]-latency_data[i][0]);
        sum[1] += (latency_data[i][2]-latency_data[i][1]);
        sum[2] += (latency_data[i][3]-latency_data[i][2]);
    }
    latency_lock = 0;
    shell_info("<everage> Notify:%.2fus \t read data:%.2fus \t IMU:%.2fus \r\n\n",
        (float)(sum[0]/(float)((SENSOR_LATENCY_TEST_COUNT-1)*96)),
        (float)(sum[1]/(float)((SENSOR_LATENCY_TEST_COUNT-1)*96)),
        (float)(sum[2]/(float)((SENSOR_LATENCY_TEST_COUNT-1)*96)));


#endif
    return 0;
}
const MONITOR_COMMAND commandTable[] =
{
    {"task",    get_task_state},
    {"misc",    cmd_misc_data},
    {"pm",      PowerDoCommand},
    {"gsensor",  shell_lsm6dsm_commad},
    {"gpio",    gpio_test_cmd},
    {"pmic",    pmic_test_cmd},
    {"i2c",     i2c_test_cmd},
    {"ioexp",   ioexp_test_cmd},
    {"led",     led_test_cmd},
    {"fg",    gauge_test_cmd},//fuel gauge
    {"chg", charger_test_cmd},
    {"rtc",   rtc_test_cmd},//rtc
    {"log", hlog_cmd},
    {"ccgx_fw", ccgx_fw_misc_cmds},
    {"ccg4_version", ccg4_read_version},
    {"audio",   audio_test_cmd },
    {"anx",   anx_test_cmd },
    {"mag",   cmd_mag},
    {"prox",   cmd_prox},
    {"disp",   disp_test_cmd},
	{"ccg4_test",ccgx_i2c2_test},
    {"k_acc",cmd_mfg_bmi160_k_acc},
    {"k_gyro",cmd_mfg_bmi160_k_gyro},
    {"ktest_gyro",cmd_mfg_bmi160_test_gyro},
    {"bootmode",cmd_bootmode},
    {"ramtest",cmd_ramtest},
    {"sensor",  performance_test},
    {"?",       cmd_help}, //This must be the last command
};
const unsigned long ulNumberOfCommands = (sizeof(commandTable) / sizeof(commandTable[0]));
int cmd_help(int argc, char * argv[], _command_source source)
{
    uint8_t i;

    shell_info("\tAll Command list\r\n");

    for (i=0; i<(ulNumberOfCommands-1); i++){
        shell_info("\t%s\r\n", commandTable[i].command);
    }
    shell_info("________ ^_^ ________\r\n");
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

int ParseCommandAndData(unsigned char *pLineBuf, int *argn, unsigned char *argv[], int MaxArgs)
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
        shell_err("Too many arguments\n");
        n = 0;
    }
    *argn = n;
    return n;
}


int DoCommand(int argn,unsigned char *argv[], _command_source source)
{
    unsigned int uiCount;

    //
    // The first argument should be the command
    //
    for (uiCount = 0; uiCount < ulNumberOfCommands; uiCount++)
    {
        char resut_cmp = (strcmp((char const*)argv[0], (char const*)commandTable[uiCount].command)== 0);
        if ( resut_cmp )
        {
            return(*(commandTable[uiCount].pFunc))(argn, (char **)argv, source);

        }
    }

     shell_err("Command error !!!\n");

    return -1;
}

void StartCommandTask(void const * argument)
{
    for(;;)
    {
        int argn = 0;
        cmd_struct commdBuf = {0};
        unsigned char *argv[MAX_ARGS] = {0};

        if(!CommdMsgQueueHandle) continue;
        xQueueReceive( CommdMsgQueueHandle, &commdBuf,  portMAX_DELAY );

        memset(commdBuf.cmd_buf+commdBuf.cmd_length-1,'\0',CMD_BUF_LEN-commdBuf.cmd_length);
        if(!cmd_semaphore) continue;
        osSemaphoreWait(cmd_semaphore, osWaitForever);

        if(commdBuf.cmd_length!= 0)
        {
            if(0 != ParseCommandAndData((unsigned char *)commdBuf.cmd_buf, &argn, argv, (sizeof(argv) / sizeof(argv[0]))))
            {
                DoCommand(argn, argv, CMD_SOURCE_UART);
            }
            else
            {
                shell_err("CommandManger_Thread %s \r\n", "[No Processor for Command]");
            }
        }
        else
        {
            shell_err("[No Command]\n");
        }
        osSemaphoreRelease(cmd_semaphore);

    }

}

osMessageQId get_queue_commd(void)
{
    return CommdMsgQueueHandle;
}

osThreadId get_commd_task_handle(void)
{
    return commandTaskHandle;
}


void cmd_init(void)
{
    osThreadDef(commandTask, StartCommandTask, osPriorityAboveNormal, 0,
                configMINIMAL_STACK_SIZE*4);
    commandTaskHandle = osThreadCreate(osThread(commandTask), NULL);

    osMessageQDef(CommdMsgQueue, 1, cmd_struct);
    CommdMsgQueueHandle = osMessageCreate(osMessageQ(CommdMsgQueue), NULL);

    osSemaphoreDef(cmd_semaphore);
    cmd_semaphore = osSemaphoreCreate(osSemaphore(cmd_semaphore),1);

    shell_info("cmd_init success!\n");

}
int cmd_mfg_bmi160_k_acc(int argc, char * argv[], _command_source source)
{
	 if(argc == 2)
	ACC_Calibate((unsigned char *)argv[1]);

	return 0;
}
int cmd_mfg_bmi160_k_gyro(int argc, char * argv[], _command_source source)
{
	if(Gyro_Calibate() == 0)
	{
		usb_cdc_transmit_data("OK\n",3);
	}
	else
	{
		usb_cdc_transmit_data("fail\n",5);
	}
	return 0;
}
int cmd_mfg_bmi160_test_gyro(int argc, char * argv[], _command_source source)
{
	if(test_gyro() == 0)
	{
		usb_cdc_transmit_data("OK\n",3);
	}
	else
	{
		usb_cdc_transmit_data("fail\n",5);
	}
	return 0;

}
