#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "integer.h"
#include "gpio.h"
//#include "hlog_api.h
#include "ccgx_ccg4.h"
#include "ccgx_ec.h"
#include "PowerManager_system.h"
#include "gpio_exp.h"
#include "led_hal.h"
extern pcbid_t pcb_id;
static ccgx_device ccgx_dev={0};
/*******************************************************************/

void ccg4_reset()
{
	if((XB02 == pcb_id)||(XC01 == pcb_id) ||\
        XC02 == pcb_id) {
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_CCG4_XRES, IOEXP_GPIO_LOW);
		HAL_Delay(100);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_CCG4_XRES, IOEXP_GPIO_HIGH);
	}
	return ;
}

/* Use this function to switch between different Sink PDOs when system is transitioning
 * between different states (i.e S0 -> S4 or S4 -> S0) */
void select_sink_pdo(uint8_t mask)
{

	mask |= 0x01;		/* 5V should always be enabled */

	reg_write(CCG_SELECT_SINK_PDO, &mask, 4);

}

/* Use this function to get the current voltage when system is consuming power.
 * It returns 0 if there is no PD Adapter connected or if a Power Sink is connected.
 * If a power adapter/source is connected, it returns the voltage range supplied by the source.
 * RETURNS:
 *      max_voltage - Maximum voltage from the source in 50mV units
 *      min_voltage - Minimum voltage from the source in 50mV units */
void get_sink_voltage(uint16_t * max_voltage, uint16_t * min_voltage)
{

	uint32_t pd_status, current_pdo;

	reg_read((uint16_t) CCG_PD_STATUS, (uint8_t *) & pd_status, 4);

	/* If no PD contract exists or if there is a contract but CCG is a source, return 0 */
	if (!(pd_status & PD_STATUS_CONTRACT_EXISTS)
	    || (pd_status & PD_STATUS_SOURCE)) {

		*max_voltage = 0;

		*min_voltage = 0;

		return;

	}

	reg_read(CCG_CURRENT_PDO, (uint8_t *) & current_pdo, 4);

	/* Get the type of power supply connected */
	switch (current_pdo & ((uint32_t) 0x03 << 30)) {

	case 0x00:		/* This is a fixed voltage and current supply. So, max and min voltages are same */

		*max_voltage = (current_pdo >> 10) & (0x000003FF);

		*min_voltage = *max_voltage;

		break;

	case ((uint32_t) 0x01 << 30):	/* This is a battery supply: output voltage depends on current drawn
					   and the supply can give a maximum amount of power */

	case ((uint32_t) 0x02 << 30):	/* This is a variable supply: output voltage is in a range and system can draw
					   upto a maximum limit of current */

		*max_voltage = (current_pdo >> 20) & (0x000003FF);

		*min_voltage = (current_pdo >> 10) & (0x000003FF);

		break;

	}

}

/* Use this function to get the current or power that can be drawn from power adapter.
 * RETURNS:
 *      is_power - This is 'true' if the output variables are Power values.
 *                 If output variables are current values, this is 'false'.
 *      operating_cur_power - Operating current (or power) from the source in 10mA (or 250mW) units
 *      max_op_cur_power - Maximum operating current (or power) from the source in 10mA (or 250mW) units */
void get_sink_cur_power(uint16_t * max_op_cur_power,
			uint16_t * operating_cur_power, bool * is_power)
{

	uint32_t pd_status, current_rdo, current_pdo;

	reg_read(CCG_PD_STATUS, (uint8_t *) & pd_status, 4);

	/* If no PD contract exists or if there is a contract but CCG is a source, return 0 */
	if (!(pd_status & PD_STATUS_CONTRACT_EXISTS)
	    || (pd_status & PD_STATUS_SOURCE)) {

		*max_op_cur_power = 0;

		*operating_cur_power = 0;

		return;

	}

	/* Read CURRENT_PDO to know what type of supply is connected */
	reg_read(CCG_CURRENT_PDO, (uint8_t *) & current_pdo, 4);

	/* Read CURRENT_RDO for the current/power information */
	reg_read(CCG_CURRENT_RDO, (uint8_t *) & current_rdo, 4);

	*max_op_cur_power = (current_rdo) & (0x000003FF);

	*operating_cur_power = (current_rdo >> 10) & (0x000003FF);

	/* Get the type of power supply connected */
	switch (current_pdo & ((uint32_t) 0x03 << 30)) {

	case (0x00 << 30):	/* This is a fixed supply that can supply a fixed max current at fixed voltage */

	case ((uint32_t) 0x02 << 30):	/* This is a variable supply which can supply a fixed maximum current at variable voltage */

		*is_power = false;

		break;

	case ((uint32_t) 0x01 << 30):	/* This is a battery supply that supplies a maximum power but at a voltage and current range */

		*is_power = true;

		break;

	}

}

/* Use this function to change how CCG responds to swap requests.
 * For example, to reject PR_Swap, accept DR_Swap and VCONN_Swap, call as follows:
 *      change_swap_response(ACCEPT_SWAP, REJECT_SWAP, ACCEPT_SWAP);
 */
void change_swap_response(uint8_t dr_swap_response, uint8_t pr_swap_response,
			  uint8_t vconn_swap_response)
{

	uint8_t swap_response;

	swap_response =
	    (dr_swap_response << DATA_ROLE_SWAP_POS) | (pr_swap_response <<
							POWER_ROLE_SWAP_POS)
	    | (vconn_swap_response << VCONN_SRC_SWAP_POS);

	reg_write(CCG_SWAP_RESPONSE, &swap_response, 1);

}

/* adapting for project link */

osThreadId ccgx_th;
//ccg4 i2c1 handler
void ccgx_irq_handler(void)
{
	ccgx_device* dev = &ccgx_dev;
	BaseType_t need_resched = pdFALSE;

	if (NULL == dev->ccgx_th)
		return;
	if(dev->ccgx_msg_handler!=NULL){
		xQueueSendFromISR(dev->ccgx_msg_handler, &dev, &need_resched);
	}

	if (pdTRUE == need_resched)
		taskYIELD();


}

static void init_port_status(void)
{
	uint8_t buffer[0x10];
	if(reg_read_i2c1(0x00,buffer,2)==HAL_OK) {
		ccgx_dev.current_port_status = buffer[0];

		if((buffer[0]&0xF0) == CHARGE_PORT_CONNECT)
			ccgx_pr_INFO("charge port plug in\n");

		if((buffer[0]&0xF0) == CHARGE_PORT_DISCONNECT)
			ccgx_pr_INFO("charge port plug out\n");

		if((buffer[0]&0x0f) == DATA_PORT_CONNECT)
			ccgx_pr_INFO("data port plug in\n");

		if((buffer[0]&0x0f) == DATA_PORT_DISCONNECT)
			ccgx_pr_INFO("data port plug out\n");
	}else{
		ccgx_pr_ERROR("Init ccgx port status error\r\n");
	}
}
enumPortStatus get_port_status(enumPortID portID)
{
	enumPortStatus status=DATA_PORT_DISCONNECT;

	if(portID == DATA_PORT){

		if((ccgx_dev.current_port_status&0x0f) == DATA_PORT_CONNECT){
			status = DATA_PORT_CONNECT;

		} else if((ccgx_dev.current_port_status&0x0f) == DATA_PORT_DISCONNECT){
			status = DATA_PORT_DISCONNECT;

		}else{
			status = DATA_PORT_ERROR;
		}

	} else if(portID == CHARGE_PORT){

		if((ccgx_dev.current_port_status&0xF0) == CHARGE_PORT_CONNECT){
			status = CHARGE_PORT_CONNECT;

		}else if((ccgx_dev.current_port_status &0xF0) == CHARGE_PORT_DISCONNECT){
			status = CHARGE_PORT_DISCONNECT;

		}else{
			status = CHARGE_PORT_ERROR;
		}

	} else {
		ccgx_pr_ERROR("wrong  port ID\n");
		return PORT_ERROR;
	}
	return status;
}
static void Data_Ovp_Timer_Detect_Callback(void const* argument)
{
    GPIO_PinState pin_state = GPIO_PIN_SET;
	if(XB02 == pcb_id){
        pin_state = HAL_GPIO_ReadPin(GPIOB,GPIO_XB_CC_FLAG_OVP_PHONE);
	}
    else if((XC01 == pcb_id) || (XC02 == pcb_id)){
        pin_state = HAL_GPIO_ReadPin(GPIO_XC_PORT_OVP_PHONE,
            GPIO_XC_CC_FLAG_OVP_PHONE);
	}
    else{
        /* */
	}

    if(pin_state == GPIO_PIN_RESET){
        ccgx_pr_ERROR("Phone port CC OVP error happen\r\n");
    }
}
static void Charge_Ovp_Timer_Detect_Callback(void const* argument)
{
	if((XB02 == pcb_id)){
		if(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOC,GPIO_XB_CC_FLAG_OVP_CHARG)){
			ccgx_pr_ERROR("Charge port CC OVP error happen\r\n");

		}else{
			//ccgx_pr_DEBUG("Charge port CC ovp ok\r\n");
		}

	}else{
		return;
	}
}
void ccgx_task_thread_i2c1(void const *args)
{
#define OVP_TIMER_PERIOD	(3000)
	ccgx_device* dev = &ccgx_dev;
	ccgx_device* device;
	//ccg_event_t event;
	uint8_t buffer[0x10];

	if(get_port_status(DATA_PORT)==DATA_PORT_CONNECT){
		osTimerStart(dev->ccgx_data_ovp_timer,OVP_TIMER_PERIOD);
	}
	if(get_port_status(CHARGE_PORT)==CHARGE_PORT_CONNECT){
		if(XB02 == pcb_id){
			osTimerStart(dev->ccgx_charge_ovp_timer,OVP_TIMER_PERIOD);
		}
	}
	while (1) {
		xQueueReceive(dev->ccgx_msg_handler, &device, portMAX_DELAY);

		if (device == &ccgx_dev){

			if(reg_read_i2c1(0x00,buffer,2)==HAL_OK) {

				if(ccgx_dev.current_port_status==buffer[0])
						continue;

				if((buffer[0]&0xf0) != (ccgx_dev.current_port_status&0xf0)) {

					if((buffer[0]&0xF0) == CHARGE_PORT_CONNECT){
						ccgx_pr_INFO("charge port plug in\n");

						if(XB02 == pcb_id){
							osTimerStart(dev->ccgx_charge_ovp_timer,OVP_TIMER_PERIOD);
						}
					}
					if((buffer[0]&0xF0) == CHARGE_PORT_DISCONNECT){
						ccgx_pr_INFO("charge port plug out\n");
						if(XB02 == pcb_id){
							osTimerStop(dev->ccgx_charge_ovp_timer);
						}
					}
					ccgx_dev.current_port_status=(ccgx_dev.current_port_status&0x0F)|(buffer[0]&0xF0 );

				}

				if((buffer[0]&0x0f) != (ccgx_dev.current_port_status&0x0f)) {

					if((buffer[0]&0x0f) == DATA_PORT_CONNECT) {
						ccgx_pr_INFO("data port plug in\n");
						osTimerStart(dev->ccgx_data_ovp_timer,OVP_TIMER_PERIOD);
                        #if USB_HOLE_DET_BY_CCG4
						pwrmgr_system_send_message(PM_SYS_CMD_USB_DATA_IN, 0);
                        #endif
					}
					if((buffer[0]&0x0f) == DATA_PORT_DISCONNECT) {
						ccgx_pr_INFO("data port plug out\n");
						osTimerStop(dev->ccgx_data_ovp_timer);
                        #if USB_HOLE_DET_BY_CCG4
						pwrmgr_system_send_message(PM_SYS_CMD_USB_DATA_OUT, 0);
                        #endif
					}

					ccgx_dev.current_port_status = (ccgx_dev.current_port_status&0xF0)|(buffer[0]&0x0F );
				}
			}else{
				ccgx_pr_ERROR("ccg4 i2c1 read error\n");

			}
		}

	}

}
void ccg4_init()
{

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_CCG4_INT);

	if((XB02 == pcb_id)||(XC01 == pcb_id) ||\
        (XC02 == pcb_id)){
		ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_CCG4_XRES, IOEXP_GPIO_OUTPUT);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_CCG4_XRES, IOEXP_GPIO_HIGH);
	}
	init_port_status();
	TRACE();

}
int ccgx_init(void)
{
	if(ccgx_dev.is_initialized == 1)
		return 0;
	ccg4_init();
	osThreadDef(ccgx, ccgx_task_thread_i2c1, osPriorityHigh, 0,
		    configMINIMAL_STACK_SIZE * 4);

	ccgx_dev.ccgx_th = osThreadCreate(osThread(ccgx), NULL);

	if(ccgx_dev.ccgx_th==NULL)
		return -1;
	ccgx_dev.ccgx_data_ovp_timer_callback.ptimer = Data_Ovp_Timer_Detect_Callback;
	ccgx_dev.ccgx_data_ovp_timer = osTimerCreate(&ccgx_dev.ccgx_data_ovp_timer_callback,\
					osTimerPeriodic,&ccgx_dev);
	ccgx_dev.ccgx_charge_ovp_timer_callback.ptimer = Charge_Ovp_Timer_Detect_Callback;
	ccgx_dev.ccgx_charge_ovp_timer = osTimerCreate(&ccgx_dev.ccgx_charge_ovp_timer_callback,\
					osTimerPeriodic,&ccgx_dev);

	osMessageQDef(ccgx_int_queue,4,ccgx_device*);
	ccgx_dev.ccgx_msg_handler = osMessageCreate(osMessageQ(ccgx_int_queue),NULL);
	ccgx_dev.is_initialized = 1;
	return 0;
}

void ccgx_test()
{
	int count;
	int retval;
	unsigned char buf[32];

	buf[0] = 0;
	reg_read(CCG_DEVICE_MODE_ADDR, buf, 1);
	printf("CCG_DEVICE_MODE : 0x%02x\n", buf[0]);

	memset(buf, 0x00, 2);
	reg_read(CCG_SILICON_ID, buf, 2);
	printf("CCG_SILICON_ID : 0x%02x 0x%02x\n", buf[0], buf[1]);

	memset(buf, 0x00, 16);
	reg_read(CCG_GET_VERSION, buf, 16);
	printf
	    ("FW1_VERSION : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n",
	     buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
	     buf[15]);

	memset(buf, 0x00, 8);
	reg_read(CCG_GET_VERSION + 0x10, buf, 8);
	printf
	    ("FW2_VERSION : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n",
	     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	retval = 0;
	count = 10;
	send_power_role_swap();
	osDelay(2000);
	while (count--) {
		retval = swap_status();
		if (retval != SWAP_IN_PROGRESS)
			break;

		osDelay(1000);
	}
	printf(" power role swap : %d\n", retval);

	retval = 0;
	count = 10;
	send_data_role_swap();
	osDelay(2000);
	while (count--) {
		retval = swap_status();
		if (retval != SWAP_IN_PROGRESS)
			break;

		osDelay(1000);
	}
	printf(" data role swap : %d\n", retval);

	return;
}
