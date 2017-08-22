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
//#include "hlog_api.h"

#include "ccgx_ccg4.h"
#include "ccgx_ec.h"
#include "Rtos_i2c_drv.h"
/* adapt for ccgx */

static int ccgx_i2c_read(u16 reg_addr, unsigned char *data, unsigned count)
{

        int retval = HAL_OK;
        retval =RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_ADDR,
			     reg_addr, I2C_MEMADD_SIZE_16BIT, data,
			     count, I2C_FLAG_TIMEOUT);
        if (retval != HAL_OK)
          ccgx_pr_ERROR("READ ERROR %d\n", retval);

        return retval == HAL_OK ? 0 : -1;

}

static int ccgx_i2c_write(u16 reg_addr, unsigned char *data,
			  unsigned count)
{

	int retval = HAL_OK;
        retval = RTOS_I2C_WriteBuffer(I2C_DEVICE_CCG4_ADDR,
			     reg_addr, I2C_MEMADD_SIZE_16BIT, data,
			     count, I2C_FLAG_TIMEOUT);
	if (retval != HAL_OK)
          ccgx_pr_ERROR("WRITE ERROR %d\n", retval);

	return retval == HAL_OK ? 0 : -1;

}
static int ccgx_i2c1_read(u16 reg_addr, unsigned char *data, unsigned count)
{

	int retval = HAL_OK;
    retval =RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_ADDR,
			     reg_addr, I2C_MEMADD_SIZE_8BIT, data,
			     count, I2C_FLAG_TIMEOUT);

	if (retval != HAL_OK)
		ccgx_pr_ERROR("READ ERROR %d\n", retval);

	return retval == HAL_OK ? 0 : -1;

}



/* adapt for ccgx end */

/* ---- GLOBAL VARIABLES --- */
bool gl_swap_in_progress = false;

swap_response_t gl_swap_status = SWAP_IN_PROGRESS;

/* ---- LOW LEVEL FUNCTIONS ---- */

/* FUNCTIONS THAT THE CUSTOMER HAS TO IMPLEMENT:
 *      1. reg_read
 *      2. reg_write
 *      3. i2c_intr_status
 *      4. register ccg_irq_handler() as the interrupt
 *         handler for the I2C interrupt pin
 */

/* DESCRIPTION:
 * Function to read from a CCG register via I2C2 (not SMBus)
 *
 * RETURN:
 *      num_bytes_read - this returns the number of bytes read from CCG
 */
void reg_read(uint16_t reg_addr, uint8_t * reg_data, uint16_t num_bytes_read)
{

	/* Send I2C command as follows:
	   [I2C Start]
	   Byte 0: [7-bit address: 0x08] [write]
	   Byte 1: reg_addr
	   [I2C Restart]
	   Byte 2: [7-bit address: 0x08] [read]
	   Byte 3..n: 'size' bytes of register contents sent by CCG
	   [I2C Stop]
	 */
	ccgx_i2c_read(reg_addr, reg_data, num_bytes_read);

	return;

}

/* DESCRIPTION:
 * Function to write to a CCG register via I2C2 (not SMBus)
 *
 * RETURN:
 *      nothing
 */
void reg_write(uint16_t reg_addr, uint8_t * reg_data, uint16_t size)
{

	/* Send I2C command as follows:
	   [I2C Start]
	   Byte 0: [7-bit address: 0x08] [write]
	   Byte 1: reg_addr
	   Byte 2: reg_data[0]
	   Byte 3: reg_data[1]
	   ...
	   [I2C Stop]
	 */
	ccgx_i2c_write(reg_addr, reg_data, size);

	return;

}
int reg_read_i2c1(uint16_t reg_addr, uint8_t * reg_data, uint16_t num_bytes_read)
{

	/* Send I2C command as follows:
	   [I2C Start]
	   Byte 0: [7-bit address: 0x08] [write]
	   Byte 1: reg_addr
	   [I2C Restart]
	   Byte 2: [7-bit address: 0x08] [read]
	   Byte 3..n: 'size' bytes of register contents sent by CCG
	   [I2C Stop]
	 */
	return ccgx_i2c1_read(reg_addr, reg_data, num_bytes_read);
}

/* DESCRIPTION:
 * Function to read the I2C interrupt status
 *
 * RETURN:
 *      true - if the interrupt is pending (i.e I2C_INT pin is low)
 *      false - if the interrupt is not pending (i.e I2C_INT pin is high)
 */
bool i2c_intr_pending()
{

	/* return true if PD_INT/I2C_INT pin is LOW; else return false */
	return __HAL_GPIO_EXTI_GET_IT(GPIO_CCG4_INT) != RESET;

}

/* DESCRIPTION:
 * This function is the INTERRUPT REQUEST HANDLER for the I2C interrupt
 * from CCG i.e it is called if PD_INT/I2C_INT goes low
 *
 * RETURN:
 *      nothing
 */


/* DESCRIPTION:
 * This function reads the RESPONSE register to know what the response
 * to the previous command is. Or it reads the event information sent
 * by CCG to the EC
 *
 * RETURN:
 *      event structure:
 *          event.event_code - has the event sent by CCG. Look at ccg_response_t
 *                             type in ccg.h for all types of events
 *          event.event_length - if the event has extra data, this field has the size
 *                               of that data
 *          event.event_data - this array has the extra data sent by CCG
 */
ccg_event_t read_event_reg()
{

	ccg_event_t event;

	reg_read(CCG_RESPONSE, (uint8_t *) & event, 2);

	/* If there's data to read, fill into the buffer */
	if (event.event_length > 0) {

		reg_read(CCG_RESPONSE_DATA_MEMORY, event.event_data,
			 event.event_length);

	}

	return event;

}

/* DESCRIPTION:
 * This function clears the I2C interrupt request so that the next interrupt
 * can be serviced
 *
 * RETURN:
 *      nothing
 */
void clear_i2c_intr()
{

	uint8_t clear_intr = 0x01;

	reg_write(CCG_INTR_REG, &clear_intr, 1);

}

/* DESCRIPTION:
 * This flushes the event queue in CCG. Use it only during initialization.
 *
 * RETURN:
 *      nothing
 */
void flush_ccg_event_queue()
{

	while (i2c_intr_pending() == true) {

		clear_i2c_intr();

		osDelay(1000);	/* Add a small delay */

	}

}

/* ---- GENERAL INFORMATION ---- */

/* As soon as CCG boots up, it sends the 'RESET_COMPLETE' event. And before you read/write
 * to any other register, you need to read and clear this event.
 * You can either read the event register or just clear the I2C interrupt */

/* All of the following helper functions return a response code received from CCG.
 * The values are defined in ccg.h in "ccg_response_t".
 * Usually, you should receive 0x02 (RESP_SUCCESS) indicating a successful command */

/* ---- BASIC USB-PD AND TYPE-C FUNCTIONS ---- */

/* DESCRIPTION:
 * This function returns the status of a PD contract. Any PD command cannot be sent
 * if the PD contract does not exist
 *
 * RETURN:
 *      true - if the PD contract exists
 *      false - if the PD contract does not exist (i.e port is not connected or if Type-C
 *                                                  only device is connected)
 */
bool pd_contract_exists()
{

	uint32_t pd_status;

	reg_read(CCG_PD_STATUS, (uint8_t *) & pd_status, 4);

	if (pd_status & PD_STATUS_CONTRACT_EXISTS)
		/* Bit 10 */
		return true;

	else

		return false;

}

/* ---- ROLE SWAP FUNCTIONS ---- */

/* Note that these 3 functions only INITIATE a role swap. They do not guarantee
 * successful transmission or completion of the swap. This information is obtained
 * in the last function called swap_status() */

/* DESCRIPTION:
 * Send a PR_SWAP command to the port partner. This exchanges the power role
 * from Source to Sink or vice versa
 *
 * RETURN:
 *      true - if the swap command was successfully sent
 *      false - if the PD contract does not exist
 */
bool send_power_role_swap()
{

	pd_ctrl_t pr_swap_cmd = PD_CTRL_POWER_ROLE_SWAP;

	/* If PD contract doesn't exist, we cannot send this command */
	if (pd_contract_exists() == false)

		return false;

	reg_write(CCG_PD_CONTROL, (uint8_t *) & pr_swap_cmd, 1);

	gl_swap_in_progress = true;

	return true;

}

/* DESCRIPTION:
 * Send DR_SWAP command to the port partner. This exchanges the data roles
 * from DFP/Host to UFP/Device or vice versa
 *
 * RETURN:
 *      true - if the swap command was successfully sent
 *      false - if the PD contract does not exist
 */
bool send_data_role_swap()
{

	pd_ctrl_t dr_swap_cmd = PD_CTRL_DATA_ROLE_SWAP;

	/* If PD contract doesn't exist, we cannot send this command */
	if (pd_contract_exists() == false)

		return false;

	reg_write(CCG_PD_CONTROL, (uint8_t *) & dr_swap_cmd, 1);

	gl_swap_in_progress = true;

	return true;

}

/* DESCRIPTION:
 * Send VCONN_SWAP command to port partner. This exchanges the VCONN source
 * responsibility. Initially, the DFP/Source is the port supplying VCONN.
 * If this command is sent or received, the Sink will start supplying VCONN and
 * if this command is sent again, the Source will once again supply VCONN
 *
 * RETURN:
 *      true - if the swap command was successfully sent
 *      false - if the PD contract does not exist
 */
bool send_vconn_source_swap()
{

	pd_ctrl_t vconn_swap_cmd = PD_CTRL_VCONN_SOURCE_SWAP;

	/* If PD contract doesn't exist, we cannot send this command */
	if (pd_contract_exists() == false)

		return false;

	reg_write(CCG_PD_CONTROL, (uint8_t *) & vconn_swap_cmd, 1);

	gl_swap_in_progress = true;

	return true;

}

/* DESCRIPTION:
 * This checks if the swap command sent previously was complete or not. The swap process
 * may take some to finish and so you must call this function repeatedly to check the status
 * of the swap
 *
 * RETURN:
 *      SWAP_IN_PROGRESS - if the swap is still in progress. Please call the function again
 *                         to check if the swap is complete
 *      SWAP_COMMAND_FAILED - if the port partner does not send GoodCRC for swap command
 *                            i.e if the command could not be sent successfully
 *      SWAP_NO_RESPONSE_FROM_PORT - if the port partner send a GoodCRC but does not respond to the command
 *      SWAP_ACCEPTED - if the port partner sends Accept to the swap request and if the swap completed
 *      SWAP_REJECTED - if the port partner sends Reject to the swap request and if the swap completed
 *      SWAP_WAIT_RECEIVED - if the port partner sends Wait to the swap request and if the swap completed
 *      SWAP_HARD_RESET_SENT - if the swap process was started but failed to finish. So CCG sent a
 *                             HARD_RESET to the port partner.
 */
swap_response_t swap_status()
{

	if (gl_swap_in_progress)

		return SWAP_IN_PROGRESS;

	return gl_swap_status;

}
