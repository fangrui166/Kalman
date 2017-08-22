/**
  ******************************************************************************
  * @file    SensorTile.h
  * @author  Central Labs
  * @version V1.1.0
  * @date    12-Sept-2016
  * @brief   This file contains definitions for SensorTile.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* IMPORTANT: in order to compile with RevA following flag shall be defined  */
/* in the preprocessor options:  USE_SENSORTILE_REVA !!!!!!!!!! */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORTILE_H
#define __SENSORTILE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "accelerometer.h"
#include "gyroscope.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup SENSORTILE
  * @{
  */
      
  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Types SENSORTILE_LOW_LEVEL Exported Types
  * @{
  */
typedef enum
{
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LSM6DSM = 0,                  /* LSM6DSM. */
} SPI_Device_t;

/**
  * @}
  */ 

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Constants SENSORTILE_LOW_LEVEL Exported Constants
  * @{
  */ 

#define LSM6DSM_INT2_GPIO_PORT           GPIOB
#define LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define LSM6DSM_INT2_PIN                 GPIO_PIN_0
#define LSM6DSM_INT2_EXTI_IRQn           EXTI0_IRQn

                                              
#define SENSORTILE_SENSORS_SPI                    SPI2

#define SENSORTILE_SENSORS_SPI_Port               GPIOB
#define SENSORTILE_SENSORS_SPI_MOSI_Pin           GPIO_PIN_15
#define SENSORTILE_SENSORS_SPI_MSOI_Pin           GPIO_PIN_14
#define SENSORTILE_SENSORS_SPI_SCK_Pin            GPIO_PIN_13

#define SENSORTILE_SENSORS_SPI_CLK_ENABLE()       __SPI1_CLK_ENABLE()
#define SENSORTILE_SENSORS_SPI_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

#define SENSORTILE_LSM6DSM_SPI_CS_Port	          GPIOB
#define SENSORTILE_LSM6DSM_SPI_CS_Pin     	  GPIO_PIN_12
#define SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
                                              




                                              


/**
  * @}
  */ 


/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Macros SENSORTILE_LOW_LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
uint8_t Sensor_IO_SPI_CS_Disable(void *handle);
#if 0
DrvStatusTypeDef Sensor_IO_SPI_Init( void );
uint8_t Sensor_IO_SPI_CS_Init(void *handle);
DrvStatusTypeDef LSM6DSM_Sensor_IO_ITConfig( void );
#endif
/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
