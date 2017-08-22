/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_proximity.c
 * @author  hTC BSP Team
 * @version V1.0.0
 * @date    31-August-2016
 * @brief   This file provides a set of functions needed to manage the proximity sensor
 ******************************************************************************
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
#include <string.h>
#include "x_nucleo_iks01a1_proximity.h"
#include "EPL88051_PROXIMITY_driver_HL.h"

/** X_NUCLEO_IKS01A1_PROXIMITY_Private_Variables Private variables**/

static DrvContextTypeDef PROXIMITY_SensorHandle[ PROXIMITY_SENSORS_MAX_NUM ];
static PROXIMITY_Data_t PROXIMITY_Data[ PROXIMITY_SENSORS_MAX_NUM ]; // Proximity - all.

static DrvStatusTypeDef BSP_EPL88051_PROXIMITY_Init( void **handle );


/**
 * @brief Initialize a proximity sensor
 * @param id the proximity sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Init( PROXIMITY_ID_t id, void **handle )
{

	*handle = NULL;

	switch(id)
	{
		case PROXIMITY_SENSORS_AUTO:
		default:
			{
				/*try to initial ePL88051 first*/
				if( BSP_EPL88051_PROXIMITY_Init(handle)  == COMPONENT_ERROR )
				{
					return COMPONENT_ERROR;
				}
				break;
			}
		case EPL88051:
			{
				if( BSP_EPL88051_PROXIMITY_Init(handle)  == COMPONENT_ERROR )
				{
					return COMPONENT_ERROR;
				}
				break;
			}
	}

	return COMPONENT_OK;
}

/**
 * @brief initialize specific ePL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_EPL88051_PROXIMITY_Init( void **handle )
{
	PROXIMITY_Drv_t *driver = NULL;

	if(PROXIMITY_SensorHandle[ EPL88051].isInitialized == 1)
	{
		/* We have reached the max num of instance for this component */
		return COMPONENT_ERROR;
	}
	/* Setup proximity sensor handle. */
	PROXIMITY_SensorHandle[ EPL88051 ].who_am_i      = EPL88051_REVNO;
	PROXIMITY_SensorHandle[ EPL88051 ].address       = EPL88051_ADDR;
	PROXIMITY_SensorHandle[ EPL88051 ].instance      = EPL88051;
	PROXIMITY_SensorHandle[ EPL88051 ].isInitialized = 0;
	PROXIMITY_SensorHandle[ EPL88051 ].isEnabled     = 0;
	PROXIMITY_SensorHandle[ EPL88051 ].isCombo       = 0;
	PROXIMITY_SensorHandle[ EPL88051 ].pData         = ( void * )&PROXIMITY_Data[ EPL88051 ];
	PROXIMITY_SensorHandle[ EPL88051 ].pVTable       = ( void * )&EPL88051Drv;
	PROXIMITY_SensorHandle[ EPL88051 ].pExtVTable    = 0;

	PROXIMITY_Data[ EPL88051 ].pComponentData = 0;
	PROXIMITY_Data[ EPL88051 ].pExtData       = 0;

	*handle = (void *)&PROXIMITY_SensorHandle[ EPL88051 ];

	driver = ( PROXIMITY_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

	if ( driver->Init == NULL )
	{
		memset((*handle), 0, sizeof(DrvContextTypeDef));
		*handle = NULL;
		return COMPONENT_ERROR;
	}

	if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
	{
		memset((*handle), 0, sizeof(DrvContextTypeDef));
		*handle = NULL;
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}
/**
 * @brief Deinitialize a proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_DeInit( void **handle )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( driver->DeInit == NULL )
	{
		return COMPONENT_ERROR;
	}

	if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	memset(ctx, 0, sizeof(DrvContextTypeDef));

	*handle = NULL;

	return COMPONENT_OK;
}


/**
 * @brief Enable proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Sensor_Enable( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( driver->Sensor_Enable == NULL )
	{
		return COMPONENT_ERROR;
	}

	if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Disable proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Sensor_Disable( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( driver->Sensor_Disable == NULL )
	{
		return COMPONENT_ERROR;
	}

	if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}


/**
 * @brief Check if the proximity sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_IsInitialized( void *handle, uint8_t *status )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	if ( status == NULL )
	{
		return COMPONENT_ERROR;
	}

	*status = ctx->isInitialized;

	return COMPONENT_OK;
}


/**
 * @brief Check if the proximity sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_IsEnabled( void *handle, uint8_t *status )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	if ( status == NULL )
	{
		return COMPONENT_ERROR;
	}

	*status = ctx->isEnabled;

	return COMPONENT_OK;
}


/**
 * @brief Check if the proximity sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_IsCombo( void *handle, uint8_t *status )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	if ( status == NULL )
	{
		return COMPONENT_ERROR;
	}

	*status = ctx->isCombo;

	return COMPONENT_OK;
}


/**
 * @brief Get the proximity sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Get_Instance( void *handle, uint8_t *instance )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	if ( instance == NULL )
	{
		return COMPONENT_ERROR;
	}

	*instance = ctx->instance;

	return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the proximity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( who_am_i == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_WhoAmI == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Check_WhoAmI( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( driver->Check_WhoAmI == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}


/**
 * @brief Get the proximity value
 * @param handle the device handle
 * @param proximity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_PROXIMITY_Get_Proximity( void *handle, uint8_t *proximity )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	PROXIMITY_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( PROXIMITY_Drv_t * )ctx->pVTable;

	if ( proximity == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Proximity== NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Proximity( ctx, proximity ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
