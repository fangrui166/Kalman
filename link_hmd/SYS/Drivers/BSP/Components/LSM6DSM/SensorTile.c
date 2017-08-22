/**
******************************************************************************
* @file    SensorTile.c
* @author  Central Labs
* @version V1.1.0
* @date    12-Sept-2016
* @brief   This file provides low level functionalities for Sensor Tile board
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

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.h"
#include "stm32f4xx_hal_spi.h"
/** @addtogroup BSP
* @{
*/ 

/** @addtogroup SENSORTILE
* @{
*/

  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @brief This file provides a set of low level firmware functions 
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Private_TypesDefinitions SENSORTILE_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL__Private_Defines SENSORTILE_LOW_LEVEL Private Defines
* @{
*/

//#define SYNCHRO_WAIT(NB)       for(int i=0; i<NB; i++){__asm("dsb\n");}
//#define SYNCHRO_SPI_DELAY     (1)

/**
* @brief SensorTile BSP Driver version number V1.0.0
*/
#define __SensorTile_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __SensorTile_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __SensorTile_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __SensorTile_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __SensorTile_BSP_VERSION         ((__SensorTile_BSP_VERSION_MAIN << 24)\
|(__SensorTile_BSP_VERSION_SUB1 << 16)\
  |(__SensorTile_BSP_VERSION_SUB2 << 8 )\
    |(__SensorTile_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL_Private_Variables SENSORTILE_LOW_LEVEL Private Variables 
* @{
*/
//static SPI_HandleTypeDef SPI_Sensor_Handle;
extern SPI_HandleTypeDef hspi2;


/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */

/**
* @brief  This method returns the STM32446E EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __SensorTile_BSP_VERSION;
}
#if 0
/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_Init( void )
{
  if(HAL_SPI_GetState( &SPI_Sensor_Handle) == HAL_SPI_STATE_RESET )
  {
    SPI_Sensor_Handle.Instance = SENSORTILE_SENSORS_SPI;
    SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    SPI_Sensor_Handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
    SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_Sensor_Handle.Init.CRCPolynomial = 10;
	
    HAL_SPI_Init(&SPI_Sensor_Handle);
  }
  return COMPONENT_OK;
}
uint8_t Sensor_IO_SPI_CS_Init(void *handle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = SENSORTILE_LSM6DSM_SPI_CS_Pin;
    HAL_GPIO_Init(SENSORTILE_LSM6DSM_SPI_CS_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  default:
    return COMPONENT_NOT_IMPLEMENTED;
  }
  return COMPONENT_OK;
}

/**
 * @brief  Configures sensor interrupts interface for LSM6DSM sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DSM_Sensor_IO_ITConfig( void )
{

  /* At the moment this feature is only implemented for LSM6DSM */
  GPIO_InitTypeDef GPIO_InitStructureInt2;

  /* Enable INT2 GPIO clock */
  LSM6DSM_INT2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = LSM6DSM_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL; 
  HAL_GPIO_Init(LSM6DSM_INT2_GPIO_PORT, &GPIO_InitStructureInt2);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  return COMPONENT_OK;
}
#endif

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 1)
  {
    return Sensor_IO_SPI_Write( handle, WriteAddr, pBuffer, nBytesToWrite );
  }
  
  return COMPONENT_ERROR;
}

/**
 * @brief  Reads from the sensor to a buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 1)
  {
    return Sensor_IO_SPI_Read( handle, ReadAddr, pBuffer, nBytesToRead );
  }
  
  return COMPONENT_ERROR;
}
/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
	uint8_t array[64];
	uint16_t i =0;
	HAL_StatusTypeDef status = HAL_OK;

	array[0] =  WriteAddr; 
	array[1] = *pBuffer;

	for(i = 0;i<nBytesToWrite;i++)
	{
		array[i+1] = *pBuffer;
		pBuffer++;
	}
	Sensor_IO_SPI_CS_Enable(handle);
	status = HAL_SPI_Transmit(&hspi2,array, (nBytesToWrite+1), 1000); 
	Sensor_IO_SPI_CS_Disable(handle);
	/* Check the communication status */
	if(status != HAL_OK)
	{
	 //printf("status = %d\n",status);
	 return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
	uint8_t array[64] = {0};
	uint8_t array_pTx[8] = {0};
	uint8_t stringpos;
	HAL_StatusTypeDef status = HAL_OK;
	nBytesToRead++;

	array_pTx[0] = ReadAddr | 0x80;
	Sensor_IO_SPI_CS_Enable(handle);
	status = HAL_SPI_TransmitReceive(&hspi2, array_pTx,array, (uint16_t)nBytesToRead, 1000);
	Sensor_IO_SPI_CS_Disable(handle);
	/* Check the communication status */
	if(status != HAL_OK)
	{
	   //printf("status = %d\n",status);
	   return COMPONENT_ERROR;
	}
	for (stringpos = 0; stringpos < nBytesToRead; stringpos++)
		*(pBuffer + stringpos) = array[stringpos+ 1];
  	return COMPONENT_OK;
}


uint8_t Sensor_IO_SPI_CS_Enable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  }
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Disable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  }
  return COMPONENT_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
