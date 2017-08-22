//#include "FreeRTOS.h"

//#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
//#include "component.h"
#include "Stm32f4xx_hal.h"

#include "display_drv.h"
#include "hdmi2dsi_driver.h"

int display_drv_init()
{
    hdmi2dsi_driver_init();
    return 0;
}

