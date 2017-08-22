//#include "FreeRTOS.h"

//#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include "i2c_drv.h"
#include "anx7737.h"

#include <stdio.h>
#include <string.h>

#define ANX7737_I2C_RETRY_MAX_COUNT	(10)
#define ANX7737_I2C_TIMEOUT_MAX		0x1000


int anx7737_i2c_read(uint16_t addr, uint16_t reg, uint16_t *value)
{
	int retry_count = 0;
	uint8_t priv_data[2] = { 0 };
	I2C_STATUS ret = I2C_OK;
	while (retry_count++ < ANX7737_I2C_RETRY_MAX_COUNT) {
		ret = RTOS_I2C_ReadBuffer(addr, reg,
		I2C_MEMADD_SIZE_8BIT, priv_data, 2, ANX7737_I2C_TIMEOUT_MAX );
		if (ret == I2C_OK)
			break;
	}
	/* Check the communication status */
	if (ret != I2C_OK) {
		/* Execute user timeout callback */
		anx7737_err("%s: status %d, failed\n", __func__, ret);
		return -1;
	}
	*value = (priv_data[0] << 8) | priv_data[1];
	return 0;
}

int anx7737_i2c_write(uint16_t addr, uint16_t reg, uint16_t value)
{
	int retry_count = 0;
	I2C_STATUS ret = I2C_OK;
	uint8_t priv_data[2];
	priv_data[0] = value >> 8;
	priv_data[1] = value & 0xFF;
	while (retry_count++ < ANX7737_I2C_RETRY_MAX_COUNT) {
		ret = RTOS_I2C_WriteBuffer(addr, reg,
		I2C_MEMADD_SIZE_8BIT, priv_data, 2, ANX7737_I2C_TIMEOUT_MAX );
		if (ret == I2C_OK)
			break;
	}
	
	/* Check the communication status */
	if (ret != I2C_OK) {
		/* Execute user timeout callback */
		anx7737_err("%s: status %d, failed\r\n", __func__, ret);
		return -1;
	}
	return 0;
}

int anx7737_driver_init(void)
{
       	anx7737_debug("%s: driver initial successfully\n", __func__);
	return 0;
}

