#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "rtos_i2c_drv.h"
#include "x_pmic.h"

#include "display_drv.h"
#include "hdmi2dsi_driver.h"
#include "dp2hdmi_anx7737.h"

#define ANX7737_I2C_TIMEOUT  0x1000

#define ANX7737_CHIP_ID_SIZE   (6)

void dp2h_power_en_anx7737(int enable)
{
	if (enable) {
		bsp_pmic_power_enable(V_DP_EN, 1);
	} else {
		bsp_pmic_power_enable(V_DP_EN, 0);
	}
}

static int anx7737_rx1_i2c_read8(uint8_t reg, uint8_t *value)
{
    I2C_STATUS ret = I2C_OK;

    ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_ANX_RX_ADDR1, reg, I2C_MEMADD_SIZE_8BIT, value, 1, ANX7737_I2C_TIMEOUT);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

int disp_d2h_getid_anx7737(uint8_t *id, int size)
{
    uint8_t reg = 0x37;
    int i = 0;

    if (is_h2d_init() == 0) {
        return -1;
    }

    if (size < ANX7737_CHIP_ID_SIZE) {
        disp_err("%s arg error, size must be %d\n", __func__, ANX7737_CHIP_ID_SIZE);
        return -1;
    }

    for (i = 0; i < ANX7737_CHIP_ID_SIZE; i++) {
        if (anx7737_rx1_i2c_read8(reg+i, id+i) != 0) {
            return -1;
        }
    }

    return 0;
}

