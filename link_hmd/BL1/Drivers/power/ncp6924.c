#include "x_pmic.h"
#include "ncp6924.h"
#include "i2c_drv.h"
#include "gpio.h"

static int is_ncp6924_init = 0;
extern pcbid_t pcb_id;

ncp6924_vreg pmic_vreg[6] = {
    [NCP6924C_BUCK1] = {NCP6924_REG_DCDC1, NCP6924_EN_REG_BIT_DCDC1},
    [NCP6924C_BUCK2] = {NCP6924_REG_DCDC2, NCP6924_EN_REG_BIT_DCDC2},
    [NCP6924C_LDO1] = {NCP6924_REG_LDO1,  NCP6924_EN_REG_BIT_LDO1 },
    [NCP6924C_LDO2] = {NCP6924_REG_LDO2,  NCP6924_EN_REG_BIT_LDO2 },
    [NCP6924C_LDO3] = {NCP6924_REG_LDO3,  NCP6924_EN_REG_BIT_LDO3 },
    [NCP6924C_LDO4] = {NCP6924_REG_LDO4,  NCP6924_EN_REG_BIT_LDO4 },
};

int8_t ncp6924_reg_read(uint8_t reg, uint8_t *value)
{
	I2C_STATUS ret;
	uint8_t rx_buf;    /*read from I2C register*/
	/* read register value */
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_PMIC_ADDR, reg, I2C_8BIT, &rx_buf, 1, I2C_SHORT_DELAY);
	if(ret != I2C_OK)
	{
		pm_err("%s ***read reg(0x%2x)*** fail\r\n", __func__, reg);
		return -1;
	}
	else
	{
		*value = rx_buf;
		//pm_debug("%s ***read reg(0x%2x) value(0x%2x) *** sucess\r\n", __func__, reg, rx_buf);
	}

	return ret;
}

int8_t ncp6924_reg_write(uint8_t reg, uint8_t value)
{
    I2C_STATUS ret;

    /*write register value into register*/
    ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_PMIC_ADDR, reg, I2C_8BIT, &value, 1, I2C_SHORT_DELAY);
    if(ret != I2C_OK)
    {
		pm_debug("%s ***write reg(0x%2x) value(0x%2x) *** fail\r\n", __func__, reg, value);
        return -1;
    }
	else
	{
		//pm_debug("%s ***write reg(0x%2x) value(0x%2x) *** sucess\r\n", __func__, reg, value);
    }

    return ret;
}

static int8_t ncp6924_reg_write_enable_bit(uint8_t index, uint8_t value)
{
	I2C_STATUS ret;
	uint8_t tx_buf[2] = {NCP6924_REG_ENABLE, 0}; /*[0]:register, [1]:data write to register*/
	uint8_t rx_buf;                              /*read from I2C register*/

	/* read register value */
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_PMIC_ADDR, tx_buf[0], I2C_8BIT, &rx_buf, 1, I2C_SHORT_DELAY);
	if(ret != I2C_OK)
	{
		return -1;
	}
	//pm_debug("%s ***read reg(0x%2x) value(0x%2x) *** sucess\r\n", __func__, tx_buf[0], rx_buf);

	/* only change the index bit of enable register */
	if (value)
		tx_buf[1] = rx_buf | (1 << pmic_vreg[index].enable_bit);
	else
		tx_buf[1] = rx_buf & ~(1 << pmic_vreg[index].enable_bit);

	/* write register value */
	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_PMIC_ADDR, tx_buf[0], I2C_8BIT, &tx_buf[1], 1, I2C_SHORT_DELAY);
	if(ret != I2C_OK)
	{
		pm_err("%s ***write reg(0x%2x)*** fail\r\n", __func__, tx_buf[0]);
		return -2;
	}
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_PMIC_ADDR, tx_buf[0], I2C_8BIT, &rx_buf, 1, I2C_SHORT_DELAY);
	if(ret != I2C_OK)
	{
		return -3;
	}
	//pm_debug("%s ***read reg(0x%2x) value(0x%2x) *** sucess\r\n", __func__, tx_buf[0], rx_buf);

	return ret;
}

static int NCP6924_LDO_SetVoltage(PmicDrvTypeDef *handle, LDO_Index_t index,
                                        uint32_t voltage)
{
	ncp6924_reg_write(pmic_vreg[index].reg_addr, voltage);

	return 0;
}

static int NCP6924_Deinit(PmicDrvTypeDef *handle)
{
    /* disable DCDC/LDO by custom sequence or by chip */
    /* disable ncp6924, all supplies are disabled and outputs are discharged simultaneously */
    if(XA0n == pcb_id){
        HAL_GPIO_WritePin(GPIO_PORT_PMIC_EN, GPIO_PMIC_EN, GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIO_XB_PORT_PMIC_EN, GPIO_XB_PMIC_EN,
            GPIO_PIN_RESET);
    }
    is_ncp6924_init = 0;

    return 0;
}

static int NCP6924_LDO_Enable(PmicDrvTypeDef *handle, LDO_Index_t index,
                                    uint8_t enable)
{
    int ret = 0;
    if(!is_ncp6924_init) {
        pm_err("%s ncp6924 is not init\n",__func__);
        return -1;
    }
    ret = ncp6924_reg_write_enable_bit(index, enable);

    return ret;
}
static int NCP6924_LDO_GetEnableState(PmicDrvTypeDef *handle, LDO_Index_t index,
                                    uint8_t *state)
{
	int ret = 0;
	if(!is_ncp6924_init){
		pm_err("%s ncp6924 is not init\n",__func__);
		return -1;
	}
	else
	{
		uint8_t value = 0;
		ret = ncp6924_reg_read(NCP6924_REG_ENABLE, &value);
		if(ret < 0)
		{
			pm_err("%s GetEnableState fail\r\n", __func__);
			return -2;
		}
		else
		{
			*state = (value >> pmic_vreg[index].enable_bit) & 0x01;
			pm_debug("%s GetEnableState %d\r\n", __func__, *state);
		}
	}

	return ret;
}

static void NCP6924_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    if(XA0n == pcb_id){
    	GPIO_InitStruct.Pin = GPIO_PMIC_EN;
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    	HAL_GPIO_Init(GPIO_PORT_PMIC_EN, &GPIO_InitStruct);
    }
    else{
    	GPIO_InitStruct.Pin = GPIO_XB_PMIC_EN;
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    	HAL_GPIO_Init(GPIO_XB_PORT_PMIC_EN, &GPIO_InitStruct);
    }
}

/*
*  when enabled the chip by pull high the HWEN pin,
*  the different power rails can be independently enable/disable
*  by write register or extern gpio pin
*/
static int NCP6924_Init(PmicDrvTypeDef *handle)
{
    int ret = 0;
    /* disable all buck and ldo output */
	/* enable DCDC1 as PWM mode */
    ret |= ncp6924_reg_write(NCP6924_REG_ENABLE, (1 << NCP6924_EN_BIT_DCDC1_PWM) );

    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_BUCK1, VOLT_DCDC_1V15);
    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_BUCK2, VOLT_DCDC_1V8);
    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_LDO1, VOLT_LDO_3V3);
    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_LDO2, VOLT_LDO_3V3);
    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_LDO3, VOLT_LDO_3V3);
    ret |= NCP6924_LDO_SetVoltage(handle, NCP6924C_LDO4, VOLT_LDO_3V3);
    if(ret){
        pm_err("%s error:%d\n",__func__,ret);
        //return ret;
    }
    /* enable PMIC NCP6924 */
	NCP6924_pin_init();
    if(XA0n == pcb_id){
        HAL_GPIO_WritePin(GPIO_PORT_PMIC_EN, GPIO_PMIC_EN, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIO_XB_PORT_PMIC_EN, GPIO_XB_PMIC_EN, GPIO_PIN_SET);
    }
    is_ncp6924_init = 1;

    return 0;
}

LDO_DRV_t NCP6924_Drv = {
    NCP6924_Init,
    NCP6924_Deinit,
    NCP6924_LDO_SetVoltage,
    NCP6924_LDO_Enable,
    NCP6924_LDO_GetEnableState,
};

