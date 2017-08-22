/* 
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 20013-2014,
##  All Rights Reserved
##  UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed
##  by the license agreement included in the file
##
##     <install>/license/license.txt
##
##  where <install> is the Cypress software
##  installation root directory path.
##
## ===========================
*/  
    
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

//#include "FreeRTOS.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
#include "integer.h"
//#include "gpio.h"
#include "htc_memory_define.h"
#include "flash_drv.h"
#include "i2c_drv.h"
//#include "hlog_api.h"
//#include "ccgx_ccg4.h"
//#include "ccgx_ec.h"
#include "ccgx_fw_io.h"
#include "ccgx_fw.h"

int ccgx_fw_i2c_read(unsigned reg_addr, unsigned char *data, unsigned count)
{

	int retval = HAL_OK;

	unsigned reg_addr_xchg;
	watchdog_refresh();
#if 0
	reg_addr_xchg = ((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);

	retval =
		HAL_I2C_Mem_Read(CCGX_I2C_ADAPTER_HANDLER, CCGX_I2C_CHIP_ADDR,
				reg_addr_xchg, I2C_MEMADD_SIZE_16BIT, data,
				count, I2C_FLAG_TIMEOUT);

#elif 0
	while (count-- > 0) {

		reg_addr_xchg =
			((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);

		retval =
			HAL_I2C_Mem_Read(CCGX_I2C_ADAPTER_HANDLER,
					CCGX_I2C_CHIP_ADDR, reg_addr_xchg,
					I2C_MEMADD_SIZE_16BIT, data++, 0x01,
					I2C_FLAG_TIMEOUT);

		reg_addr++;

		if (retval != HAL_OK)

			break;

	}
#elif 0
	reg_addr_xchg = reg_addr;
	retval =
		RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_I2C2_ADDR, reg_addr_xchg,
				I2C_MEMADD_SIZE_8BIT, data,
				count, I2C_FLAG_TIMEOUT);
#elif 1
	reg_addr_xchg = ((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);
	retval =
		RTOS_I2C_ReadBuffer(I2C_DEVICE_CCG4_I2C2_ADDR, reg_addr_xchg,
				I2C_MEMADD_SIZE_16BIT, data,
				count, I2C_FLAG_TIMEOUT);
#endif
#if 0        

	if (retval != HAL_OK)

		ccgx_pr("READ ERROR %d\n", retval);
#endif
	watchdog_refresh();
	return retval ? -1 : 0;

}

int ccgx_fw_i2c_write(unsigned reg_addr, unsigned char *data,
		unsigned count)
{

	int retval = HAL_OK;

	unsigned reg_addr_xchg;
	watchdog_refresh();
#if 0
	reg_addr_xchg = ((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);

	retval =
		HAL_I2C_Mem_Read(CCGX_I2C_ADAPTER_HANDLER, CCGX_I2C_CHIP_ADDR,
				reg_addr_xchg, I2C_MEMADD_SIZE_16BIT, data,
				count, I2C_FLAG_TIMEOUT);

#elif 0

	while (count-- > 0) {

		reg_addr_xchg =
			((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);

		retval =
			HAL_I2C_Mem_Write(CCGX_I2C_ADAPTER_HANDLER,
					CCGX_I2C_CHIP_ADDR, reg_addr_xchg,
					I2C_MEMADD_SIZE_8BIT, data++, 0x01,
					I2C_FLAG_TIMEOUT);

		reg_addr++;

		if (retval != HAL_OK)

			break;

	}
#elif 0
	reg_addr_xchg = reg_addr;
	retval =
		RTOS_I2C_WriteBuffer(I2C_DEVICE_CCG4_I2C2_ADDR, reg_addr_xchg,
				I2C_MEMADD_SIZE_8BIT, data,
				count, I2C_FLAG_TIMEOUT);
#elif 1
	reg_addr_xchg = ((reg_addr & 0xff) << 8) | ((reg_addr & 0xff00) >> 8);
	retval =
		RTOS_I2C_WriteBuffer(I2C_DEVICE_CCG4_I2C2_ADDR, reg_addr_xchg,
				I2C_MEMADD_SIZE_16BIT, data,
				count, I2C_FLAG_TIMEOUT);

#endif

#if 0

	if (retval != HAL_OK)

		ccgx_pr("WRITE ERROR %d\n", retval);
#endif
	watchdog_refresh();
	return retval ? -1 : 0;

}

CY_HANDLE CyFindDevices(void) 
{
	return I2C_DEVICE_CCG4_I2C2_ADDR;
}

bool CyUsRegisterWrite(CY_HANDLE bridge_handle, uint16_t slaveAddr, uint16_t addr,
		uint16_t length, uint8_t * txBuffer, uint16_t * dataWritten) 
{	
	int retval;
	(void)bridge_handle;
	(void)slaveAddr;
	*dataWritten = length;
	retval = ccgx_fw_i2c_write(addr, txBuffer, length);
	return !retval;
}

bool CyUsRegisterRead(CY_HANDLE bridge_handle, uint16_t slaveAddr, uint16_t addr, uint16_t length, 
		uint8_t * rxBuffer, uint16_t * dataRead) 
{

	int retval;

	(void)bridge_handle;
	(void)slaveAddr;
	*dataRead = length;

	retval = ccgx_fw_i2c_read(addr, rxBuffer, length);
	return !retval;
}

int ccgx_read_flash(unsigned offset, uint8_t * buff, unsigned size)
{
	int retval;

	watchdog_refresh();
	retval = VR_flash_read(offset + REGION_FLASH_START, buff, size);
	watchdog_refresh();

	return retval;
}

int ccgx_fw_fgets(uint8_t * buff, unsigned size, unsigned *offset)
{

	int i;
	ccgx_read_flash(*offset, buff, size);

	for (i = 0; i < size; i++) {
		if (buff[i] == '\n') {
			buff[++i] = '\0';
			*offset += i;
			return i;
		}
	}

	return -1;
}

