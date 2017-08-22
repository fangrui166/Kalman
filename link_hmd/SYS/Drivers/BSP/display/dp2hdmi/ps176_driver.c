#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "rtos_i2c_drv.h"
#include "x_pmic.h"
#include "gpio_exp.h"

#include "display_drv.h"
#include "hdmi2dsi_driver.h"
#include "dp2hdmi_ps176.h"


void dp2h_power_en_ps176(int enable)
{
	if (enable) {
		bsp_pmic_power_enable(V_DP_EN, 1);
		HAL_Delay(2);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PS176_PD_N, 1);
		HAL_Delay(2);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_RST_DP_N, 1);
		HAL_Delay(2);
	} else {
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_RST_DP_N, 0);
		HAL_Delay(2);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PS176_PD_N, 0);
		HAL_Delay(2);
		bsp_pmic_power_enable(V_DP_EN, 0);
		HAL_Delay(5);
	}
}  

void dp2h_gpio_config_ps176(void)
{
	dp2h_power_en_ps176(0);
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_RST_DP_N, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_RST_DP_N, 0);
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_PS176_PD_N, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PS176_PD_N, 0);
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_PS176_LANE_MODE, IOEXP_GPIO_OUTPUT); //2 lan mode
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PS176_LANE_MODE, 0);
	HAL_Delay(5);
}

int disp_d2h_getid_ps176(uint8_t *id, int size)
{

    if (is_h2d_init() == 0) {
        return -1;
    }

#if 0 // check size
	if (size < PS176_CHIP_ID_SIZE) {
		disp_err("%s arg error, size must be %d\n", __func__, PS176_CHIP_ID_SIZE);
		return -1;
	}
#endif

    return 0;
}

