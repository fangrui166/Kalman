#ifndef __HDMI2DSI_DRIVER_H__
#define __HDMI2DSI_DRIVER_H__
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/**
  * @brief Init HDMI to DSI bridge chip TC358870XBG
  * 
  * @retval 0:success.
  */
int hdmi2dsi_driver_init(void);

/**
  * @brief is TC358870 chip initialized.
  *
  * @retval 1: true 0: false.
  */
int is_h2d_init(void);

#ifdef __cplusplus
}
#endif
#endif /*__HDMI2DSI_BRIDGE_DRIVER_H__ */
