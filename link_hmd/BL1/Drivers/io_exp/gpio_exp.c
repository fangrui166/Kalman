#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *			   I N C L U D E S
 *****************************************************************************/
#include "i2c_drv.h"
#include "gpio_exp.h"
#include "gpio.h"

extern pcbid_t pcb_id;

/* TCA6418 GPIO MAP: gpio num start from 0 to 17*/
static unsigned char TCA6418_GPIO_BITMAP[IOEXP_GPIO_NUM] = {7, 6, 5, 4, 3, 2, 1, 0, 0,
	1, 2, 3, 4, 5, 6, 7, 0, 1};

static uint8_t IOEXP_REG_DATA[4][3] = {
	{TCA6418E_Reg_GPIO_DAT_STAT1, TCA6418E_Reg_GPIO_DAT_STAT2, TCA6418E_Reg_GPIO_DAT_STAT3},
	{TCA6418E_Reg_GPIO_DAT_OUT1,  TCA6418E_Reg_GPIO_DAT_OUT2,  TCA6418E_Reg_GPIO_DAT_OUT3},
	{TCA6418E_Reg_GPIO_DIR1,      TCA6418E_Reg_GPIO_DIR2,      TCA6418E_Reg_GPIO_DIR3},
	{TCA6418E_Reg_GPIO_PULLDOWN1, TCA6418E_Reg_GPIO_PULLDOWN2, TCA6418E_Reg_GPIO_PULLDOWN3}
};

/*get register throuth gpio number*/
static uint8_t ioexp_get_reg_addr(uint8_t index, uint8_t gpio)
{
	uint8_t reg = 0;
/* #ifdef HTC_PCB_XB02 */
	if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id))
	{
		if(gpio >= IOEXP_GPIO_NUM)
			gpio -= IOEXP_GPIO_NUM;
	}
/* #endif */
	if (gpio <= 7)
		reg = IOEXP_REG_DATA[index][0];
	else if ((gpio >= 8) && (gpio <= 15))
		reg = IOEXP_REG_DATA[index][1];
	else if ((gpio >= 16) && (gpio <= 17))
		reg = IOEXP_REG_DATA[index][2];
	else {
		ioexp_log_e("pin:%d invalid!\r\n", gpio);
		return 0x0ff;
	}
	return reg;
}

/* ioexp1 on I2C1: gpio0~gpio17 */
/* index: IOEXP_REG_STAT/IOEXP_REG_OUT/IOEXP_REG_DIR/IOEXP_REG_PULL */
int8_t ioexp_gpio_get_value(uint8_t index, uint8_t gpio)
{
	uint8_t tx_buf[2];          /*[0]:register, [1]:data write to register*/
	uint8_t rx_buf;             /*read from I2C register*/
	uint32_t dev_addr = I2C_DEVICE_IOEXP1_ADDR;

	if(gpio >= IOEXP_GPIO_NUM)
	{
		dev_addr = I2C_DEVICE_IOEXP2_ADDR;
		gpio -= IOEXP_GPIO_NUM;
	}

	/*get register throuth gpio number*/
	tx_buf[0] = ioexp_get_reg_addr(index, gpio);
	if(tx_buf[0] == 0x0ff)
		return -1;

	/*read register data*/
	RTOS_I2C_ReadBuffer(dev_addr, tx_buf[0], I2C_8BIT, &rx_buf, 1, I2C_SHORT_DELAY);

	return ((rx_buf >> TCA6418_GPIO_BITMAP[gpio]) & 0x01);
}


/* gpio: 0~17 */
/* index: IOEXP_REG_STAT/IOEXP_REG_OUT/IOEXP_REG_DIR/IOEXP_REG_PULL */
int8_t ioexp_gpio_set_value(uint8_t index, uint8_t gpio, uint8_t value)
{
    int8_t ret = 0;
	uint8_t tx_buf[2];          /*[0]:register, [1]:data write to register*/
	uint8_t rx_buf;             /*read from I2C register*/
	uint32_t dev_addr = I2C_DEVICE_IOEXP1_ADDR;

	if(gpio >= IOEXP_GPIO_NUM)
	{
		dev_addr = I2C_DEVICE_IOEXP2_ADDR;
		gpio -= IOEXP_GPIO_NUM;
	}

	/*get register throuth gpio number*/
	tx_buf[0] = ioexp_get_reg_addr(index, gpio);
	if(tx_buf[0] == 0x0ff)
		return -1;

	/*read register value*/
	ret |= RTOS_I2C_ReadBuffer(dev_addr, tx_buf[0], I2C_8BIT, &rx_buf, 1, I2C_SHORT_DELAY);

	/*fill gpio value into register value*/
	if (value)
        tx_buf[1] = rx_buf | (1 << TCA6418_GPIO_BITMAP[gpio]);
	else
        tx_buf[1] = rx_buf & ~(1 << TCA6418_GPIO_BITMAP[gpio]);

	/*write register value into register*/
	ret |= RTOS_I2C_WriteBuffer(dev_addr, tx_buf[0], I2C_8BIT, &tx_buf[1], 1, I2C_SHORT_DELAY);

	return ret;
}

/* need HW offer gpio init table */
static void ioexp_gpio_init(void)
{
	/* enable led controller chip */
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_LED_DRV_EN, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_LED_DRV_EN, IOEXP_GPIO_HIGH);
	if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
		ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_CCG4_XRES, IOEXP_GPIO_OUTPUT);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_CCG4_XRES, IOEXP_GPIO_HIGH);
	}

}

/**io expender reset pin Configuration */
static void ioexp_reset_pin_init(void)
{
	/* output high to disable reset */
	HAL_GPIO_WritePin(GPIO_PORT_IO_EXP_RST, GPIO_IO_EXP_RST, GPIO_PIN_SET);
/* #ifdef HTC_PCB_XB02 */
	if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id))
	{
		/* output high to disable reset */
		HAL_GPIO_WritePin(GPIO_XB_PORT_IO_EXP_RST, GPIO_XB_IO_EXP_RST, GPIO_PIN_SET);
	}
/* #endif */
}

int8_t ioexp_drv_init(void)
{
	/* inited in gpio.c */
	ioexp_reset_pin_init();
	ioexp_gpio_init();

	return 0;
}

#ifdef __cplusplus
}
#endif
