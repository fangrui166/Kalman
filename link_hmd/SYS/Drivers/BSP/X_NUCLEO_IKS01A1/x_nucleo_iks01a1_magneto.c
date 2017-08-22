/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_magneto.c
 * @author  MEMS Application Team
 * @version V2.0.0
 * @date    10-December-2015
 * @brief   This file provides a set of functions needed to manage the magnetometer sensor
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
#include "x_nucleo_iks01a1_magneto.h"



static DrvContextTypeDef MAGNETO_SensorHandle[ MAGNETO_SENSORS_MAX_NUM ];//magnetometer sensor hadle all.
static MAGNETO_Data_t MAGNETO_Data[ MAGNETO_SENSORS_MAX_NUM ]; // Magnetometer data structure all.
static AKM099XX_Data_t AKM099XX_Data; // magnetic sensor AKM099XX special component data.
static YAS537_Data_t YAS537_Data; // magnetic sensor yas537 special component data.


/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_MAGNETO_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_AKM099XX_MAGNETO_Init( void **handle );
static DrvStatusTypeDef BSP_YAS537_MAGNETO_Init( void **handle );


/**
 * @brief Initialize a magnetometer sensor
 * @param id the magnetometer sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle )
{

	*handle = NULL;

	switch(id)
	{
		case MAGNETO_SENSORS_AUTO:
		default:
			{
				/*default to initial AKM099XX first*/
				if( BSP_AKM099XX_MAGNETO_Init(handle)  == COMPONENT_ERROR )
				{
					if( BSP_YAS537_MAGNETO_Init(handle)  == COMPONENT_ERROR )
					{
						return COMPONENT_ERROR;
					}
				}
				break;
			}
		case AKM099XX:
			{
				if( BSP_AKM099XX_MAGNETO_Init(handle)  == COMPONENT_ERROR )
				{
					return COMPONENT_ERROR;
				}
				break;
			}
		case YAS537:
			{
				if( BSP_YAS537_MAGNETO_Init(handle)  == COMPONENT_ERROR )
				{
					return COMPONENT_ERROR;
				}
				break;
			}
	}

	return COMPONENT_OK;
}



/*AKM099XX magneto sensor init*/
static DrvStatusTypeDef BSP_AKM099XX_MAGNETO_Init( void **handle )
{
	MAGNETO_Drv_t *driver = NULL;

	if(MAGNETO_SensorHandle[ AKM099XX ].isInitialized == 1)
	{
		/* We have reached the max num of instance for this component */
		return COMPONENT_ERROR;
	}
	/* Setup sensor handle. */
	MAGNETO_SensorHandle[ AKM099XX ].who_am_i= 0;
	MAGNETO_SensorHandle[ AKM099XX ].address       = 0;
	MAGNETO_SensorHandle[ AKM099XX ].instance      = AKM099XX;
	MAGNETO_SensorHandle[ AKM099XX ].isInitialized = 0;
	MAGNETO_SensorHandle[ AKM099XX ].isEnabled     = 0;
	MAGNETO_SensorHandle[ AKM099XX ].isCombo       = 0;
	MAGNETO_SensorHandle[ AKM099XX ].pData         = ( void * )&MAGNETO_Data[ AKM099XX ];
	MAGNETO_SensorHandle[ AKM099XX ].pVTable       = ( void * )&AKM099XXDrv;
	MAGNETO_SensorHandle[ AKM099XX ].pExtVTable    = 0;

	MAGNETO_Data[ AKM099XX ].pComponentData = ( void * )&AKM099XX_Data;
	MAGNETO_Data[ AKM099XX ].pExtData       = 0;

	*handle = (void *)&MAGNETO_SensorHandle[ AKM099XX ];

	driver = ( MAGNETO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

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

/*yas537 magneto sensor init*/
static DrvStatusTypeDef BSP_YAS537_MAGNETO_Init( void **handle )
{
	MAGNETO_Drv_t *driver = NULL;

	if(MAGNETO_SensorHandle[ YAS537 ].isInitialized == 1)
	{
		/* We have reached the max num of instance for this component */
		return COMPONENT_ERROR;
	}
	/* Setup sensor handle. */
	MAGNETO_SensorHandle[ YAS537 ].who_am_i= 0;
	MAGNETO_SensorHandle[ YAS537 ].address       = 0;
	MAGNETO_SensorHandle[ YAS537 ].instance      = YAS537;
	MAGNETO_SensorHandle[ YAS537 ].isInitialized = 0;
	MAGNETO_SensorHandle[ YAS537 ].isEnabled     = 0;
	MAGNETO_SensorHandle[ YAS537 ].isCombo       = 0;
	MAGNETO_SensorHandle[ YAS537 ].pData         = ( void * )&MAGNETO_Data[ YAS537 ];
	MAGNETO_SensorHandle[ YAS537 ].pVTable       = ( void * )&YAS537Drv;
	MAGNETO_SensorHandle[ YAS537 ].pExtVTable    = ( void * )&YAS537Callback;;

	MAGNETO_Data[ YAS537 ].pComponentData = ( void * )&YAS537_Data;
	MAGNETO_Data[ YAS537 ].pExtData       = 0;

	*handle = (void *)&MAGNETO_SensorHandle[ YAS537 ];

	driver = ( MAGNETO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

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
 * @brief Deinitialize a magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle )
{
	DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

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
 * @brief Enable magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

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
 * @brief Disable magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

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
 * @brief Check if the magnetometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status )
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
 * @brief Check if the magnetometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status )
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
 * @brief Check if the magnetometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status )
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
 * @brief Get the magnetometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance )
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
 * @brief Get the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint16_t *who_am_i )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

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
 * @brief Check the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

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
 * @brief Get the magnetometer sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written [mgauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( magnetic_field == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Axes == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Axes( ctx, magnetic_field ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( value == NULL )
	{
		return COMPONENT_ERROR;
	}

	if ( driver->Get_AxesRaw == NULL )
	{
		return COMPONENT_ERROR;
	}

	if ( driver->Get_AxesRaw( ctx, value ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}




/**
 * @brief Get the magnetometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( sensitivity == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Sensitivity == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( odr == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_ODR == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_ODR == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_ODR_Value == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( fullScale == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_FS == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_FS == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_FS_Value == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief Set the magnetometer sensor mode
 * @param handle the device handle
 * @param SensorMode the sensor mode enum to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_Mode( void *handle, SensorMode_t SensorMode )
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_Sensor_Mode == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_Sensor_Mode( ctx, SensorMode ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief magnetometer sensor selftest
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Selftest( void *handle)
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Sensor_Selftest == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Sensor_Selftest( ctx) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief Get magnetometer sensor calibration accuracy
 * @param handle the device handle
 * @param accuracy pointer where the accuracy is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Accuracy( void *handle,uint8_t * accuracy)
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Get_Accuracy == NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_Accuracy( ctx,accuracy) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief Set magnetometer sensor to do user calibration
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_User_Calibration( void *handle,uint8_t enable)
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Set_User_Calibration== NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Set_User_Calibration( ctx,enable) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief Get magnetometer sensor user calibration info
 * @param handle the device handle
 * @param info magnetic sensor calibration info
 * @ param length the length of magnetic sensor calibration info
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_User_Calibration( void *handle,uint8_t * info,uint8_t length)
{

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	MAGNETO_Drv_t *driver = NULL;

	if(ctx == NULL)
	{
		return COMPONENT_ERROR;
	}

	driver = ( MAGNETO_Drv_t * )ctx->pVTable;

	if ( driver->Get_User_Calibration== NULL )
	{
		return COMPONENT_ERROR;
	}
	if ( driver->Get_User_Calibration( ctx,info,length) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}
