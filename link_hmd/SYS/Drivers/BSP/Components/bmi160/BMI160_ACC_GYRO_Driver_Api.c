#include "BMI160_ACC_GYRO_Driver_Api.h"
#include "PowerManager_notify_func.h"
#include "gSensor_Calibate.h"
#include <math.h>
#include "misc_data.h"



//BMI160_DataTypeDef bmi160_offset = {0};
gSensorCalTypeDef gSensorCalStatus = GSENSORNONCAL;

static struct pwrmgr_notify_func_data Bmi160PmNotifyData = {0};
BMI160_DataTypeDef bmi160_offset = {0};
static DrvStatusTypeDef BMI160_X_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef BMI160_X_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef BMI160_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef BMI160_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef BMI160_X_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef BMI160_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef BMI160_X_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef BMI160_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef BMI160_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef BMI160_X_Get_Axes_Status( DrvContextTypeDef *handle, us8 *xyz_enabled );
static DrvStatusTypeDef BMI160_X_Set_Axes_Status( DrvContextTypeDef *handle, us8 *enable_xyz );

static DrvStatusTypeDef BMI160_G_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_G_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_G_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_G_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_G_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef BMI160_G_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity );
static DrvStatusTypeDef BMI160_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef BMI160_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef BMI160_G_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef BMI160_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef BMI160_G_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef BMI160_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale );
static DrvStatusTypeDef BMI160_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef BMI160_G_Get_Axes_Status( DrvContextTypeDef *handle, us8 *xyz_enabled );
static DrvStatusTypeDef BMI160_G_Set_Axes_Status( DrvContextTypeDef *handle, us8 *enable_xyz );


static DrvStatusTypeDef BMI160_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef BMI160_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef BMI160_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef BMI160_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef BMI160_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef BMI160_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef BMI160_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

static DrvStatusTypeDef BMI160_X_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode );



ACCELERO_Drv_t BMI160_X_Drv =
{
  BMI160_X_Init,
  BMI160_X_DeInit,
  BMI160_X_Sensor_Enable,
  BMI160_X_Sensor_Disable,
  BMI160_X_Get_WhoAmI,
  BMI160_X_Check_WhoAmI,
  BMI160_X_Get_Axes,
  BMI160_X_Get_AxesRaw,
  BMI160_X_Get_Sensitivity,
  BMI160_X_Get_ODR,
  BMI160_X_Set_ODR,
  BMI160_X_Set_ODR_Value,
  BMI160_X_Get_FS,
  BMI160_X_Set_FS,
  BMI160_X_Set_FS_Value,
  BMI160_X_Get_Axes_Status,
  BMI160_X_Set_Axes_Status,
  BMI160_X_Set_Mode
};
GYRO_Drv_t BMI160_G_Drv =
{
  BMI160_G_Init,
  BMI160_G_DeInit,
  BMI160_G_Sensor_Enable,
  BMI160_G_Sensor_Disable,
  BMI160_G_Get_WhoAmI,
  BMI160_G_Check_WhoAmI,
  BMI160_G_Get_Axes,
  BMI160_G_Get_AxesRaw,
  BMI160_G_Get_Sensitivity,
  BMI160_G_Get_ODR,
  BMI160_G_Set_ODR,
  BMI160_G_Set_ODR_Value,
  BMI160_G_Get_FS,
  BMI160_G_Set_FS,
  BMI160_G_Set_FS_Value,
  BMI160_G_Get_Axes_Status,
  BMI160_G_Set_Axes_Status,
};

BMI160_Combo_Data_t BMI160_Combo_Data[BMI160_SENSORS_MAX_NUM];


static DrvStatusTypeDef BMI160_X_Init( DrvContextTypeDef *handle )
{
  //us8 axes_status[] = {1,1,1};
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

  if ( BMI160_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  //pComponentData->Previous_ODR = BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
  pComponentData->Previous_ODR = 800.0f;
  pComponentData->comboData->lastODR = 800.0f;

  /* Output data rate selection - power down. */
  if ( bmi160_set_accel_output_data_rate(BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ,BMI160_ACCEL_NORMAL_AVG4) != BMI160_SUCCESS )
  {
    return COMPONENT_ERROR;
  }
   bmi160_delay_ms(5);
  /* Full scale selection */
  if ( BMI160_X_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes
  if ( BMI160_X_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }*/

  handle->isInitialized = 1;

  return COMPONENT_OK;
}
static DrvStatusTypeDef BMI160_X_DeInit( DrvContextTypeDef *handle )
{
	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

	  if ( BMI160_Check_WhoAmI( handle ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  /* Disable the component */
	  if ( BMI160_X_Sensor_Disable( handle ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  /* Reset output data rate. */
	  pComponentData->Previous_ODR = 0.0f;

	  handle->isInitialized = 0;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Sensor_Enable( DrvContextTypeDef *handle )
{
	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;
	  us8 v_accel_power_mode_stat_us8 = 0;

	  /* Check if the component is already enabled */
	  if ( handle->isEnabled == 1 )
	  {
		return COMPONENT_OK;
	  }

	  /* Output data rate selection */
	  if ( BMI160_X_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
      /*enable accel fifo mode
	  bmi160_delay_ms(5);
      if(bmi160_set_fifo_accel_enable(0x01) != BMI160_SUCCESS)
 	  {
	    return COMPONENT_ERROR;
      }
	  bmi160_delay_ms(5);
      if(bmi160_set_fifo_down_accel(0x01) != BMI160_SUCCESS)
      {
	    return COMPONENT_ERROR;
      }
	  bmi160_delay_ms(5);
	  if(bmi160_set_fifo_header_enable(BMI160_ENABLE) != BMI160_SUCCESS)
	  {
	    return COMPONENT_ERROR;
      }*/
	  bmi160_delay_ms(5);
	  if ( bmi160_accel_mode(ACCEL_MODE_NORMAL) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(200);
	  if( bmi160_get_accel_power_mode_stat(&v_accel_power_mode_stat_us8) != BMI160_SUCCESS)
	  {
		 return COMPONENT_ERROR;
	  }

	  if(v_accel_power_mode_stat_us8 != 1)
	  {
		 return COMPONENT_ERROR;
	  }
	  pComponentData->comboData->isEnabled = 1;
	  handle->isEnabled = 1;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Sensor_Disable( DrvContextTypeDef *handle )
{
	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

	  /* Check if the component is already disabled */
	  if ( handle->isEnabled == 0 )
	  {
		return COMPONENT_OK;
	  }

	  /* Store actual output data rate. */
	  if ( BMI160_X_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  /* Output data rate selection - power down.
	  if ( bmi160_set_accel_output_data_rate(BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ,BMI160_ACCEL_NORMAL_AVG4) != BMI160_SUCCESS )
	  {
		return COMPONENT_ERROR;
	  }*/

	  if(bmi160_accel_mode(ACCEL_SUSPEND) != BMI160_SUCCESS )
	  {
		return COMPONENT_ERROR;
	  }
	  pComponentData->comboData->isEnabled = 0;
	  pComponentData->comboData->lastODR = 800.0f;
	  handle->isEnabled = 0;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Get_WhoAmI( DrvContextTypeDef *handle, us8 *who_am_i )
{
    return BMI160_Get_WhoAmI( handle, who_am_i );
}
static DrvStatusTypeDef BMI160_X_Check_WhoAmI( DrvContextTypeDef *handle )
{
	return BMI160_Check_WhoAmI( handle );
}
static DrvStatusTypeDef BMI160_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{

	struct bmi160_accel_t accel = {0};
	float sensitivity = 0;


	/* Get BMI160 actual sensitivity. */
	if ( BMI160_X_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
	{
		return COMPONENT_ERROR;
	}
	/* Read raw data from BMI160 output register. */
	if (bmi160_read_accel_xyz(&accel) != BMI160_SUCCESS)
	{
		return COMPONENT_ERROR;
	}

	  //printf("before acc:AXIS_X = %d  AXIS_Y = %d  AXIS_Z = %d\n",acceleration->AXIS_X,acceleration->AXIS_Y,acceleration->AXIS_Z);
	  if(bmi160_offset.isAccValid == 1)
	  {
	  	switch(gSensorCalStatus)
	  	{
	  		case GSENSORCAL_6:
			{
				acceleration->AXIS_X = ( int32_ts )(bmi160_offset.accel_k_xyz[0][0]*((accel.x)*(sensitivity)-bmi160_offset.accel_x) + \
								   				bmi160_offset.accel_k_xyz[0][1]*((accel.y)*(sensitivity)-bmi160_offset.accel_y) + \
								   				bmi160_offset.accel_k_xyz[0][2]*((accel.z)*(sensitivity)-bmi160_offset.accel_z));

				acceleration->AXIS_Y = ( int32_ts )(bmi160_offset.accel_k_xyz[1][0]*((accel.x)*(sensitivity)-bmi160_offset.accel_x) + \
									  				bmi160_offset.accel_k_xyz[1][1]*((accel.y)*(sensitivity)-bmi160_offset.accel_y) + \
									  				bmi160_offset.accel_k_xyz[1][2]*((accel.z)*(sensitivity)-bmi160_offset.accel_z));

				acceleration->AXIS_Z = ( int32_ts )(bmi160_offset.accel_k_xyz[2][0]*((accel.x)*(sensitivity)-bmi160_offset.accel_x) + \
									   				bmi160_offset.accel_k_xyz[2][1]*((accel.y)*(sensitivity)-bmi160_offset.accel_y) + \
									   				bmi160_offset.accel_k_xyz[2][2]*((accel.z)*(sensitivity)-bmi160_offset.accel_z));
				//printf("axis_6\n");
				break;
			}
			case GSENSORCAL_1:
			{
			  	acceleration->AXIS_X = ( int32_ts )( (accel.x) * (sensitivity) - bmi160_offset.accel_x);
			  	acceleration->AXIS_Y = ( int32_ts )( (accel.y) * (sensitivity) - bmi160_offset.accel_y);
			 	acceleration->AXIS_Z = ( int32_ts )( (accel.z) * (sensitivity) - bmi160_offset.accel_z);
				//printf("axis_1\n");
				break;
			}
	  	}
		//printf("after acc:AXIS_X = %d  AXIS_Y = %d  AXIS_Z = %d\n",acceleration->AXIS_X,acceleration->AXIS_Y,acceleration->AXIS_Z);
	  }
	  else
	  {
		 /* Calculate the data. */
	  	acceleration->AXIS_X = ( int32_ts )( (accel.x) * (sensitivity) );
	  	acceleration->AXIS_Y = ( int32_ts )( (accel.y) * (sensitivity) );
	 	acceleration->AXIS_Z = ( int32_ts )( (accel.z) * (sensitivity) );
		//printf("axis raw\n");
	  }
	  return COMPONENT_OK;
}


static DrvStatusTypeDef BMI160_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

	  struct bmi160_accel_t accel = {0};

	  /* Read raw data from BMI160 output register. */
	  if (bmi160_read_accel_xyz(&accel) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  /* Set the raw data. */
	  value->AXIS_X = accel.x;
	  value->AXIS_Y = accel.y;
	  value->AXIS_Z = accel.z;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

	  us8 fullScale;

	  /* Read actual full scale selection from sensor. */
	  if (bmi160_get_accel_range(&fullScale) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  /* Store the sensitivity based on actual full scale. */
	  switch( fullScale )
	  {
		case BMI160_ACCEL_RANGE_2G:
		  *sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_2G;
		  break;
		case BMI160_ACCEL_RANGE_4G:
		  *sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_4G;
		  break;
		case BMI160_ACCEL_RANGE_8G:
		  *sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_8G;
		  break;
		case BMI160_ACCEL_RANGE_16G:
		  *sensitivity = ( float )BMI160_ACC_SENSITIVITY_FOR_FS_16G;
		  break;
		default:
		  *sensitivity = -1.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

	  us8 odr_low_level;
	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

	  /* Accelerometer ODR forced to be same like gyroscope ODR. */
	  if(pComponentData->comboData->isEnabled == 1)
	  {
		*odr = pComponentData->comboData->lastODR;
		return COMPONENT_OK;
	  }

	  if ( bmi160_get_accel_output_data_rate(&odr_low_level) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  switch( odr_low_level )
	  {
		case BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED:
		  *odr =   0.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ:
		  *odr =  0.78f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ:
		  *odr =  1.56f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ:
		  *odr = 3.12f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ:
		  *odr = 6.25f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ:
		  *odr = 12.5f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ:
		  *odr = 25.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ:
		  *odr = 50.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ:
		  *odr = 100.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ:
		  *odr = 200.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ:
		  *odr = 400.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ:
		  *odr = 800.0f;
		  break;
		case BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ:
		  *odr = 1600.0f;
		  break;
		default:
		  *odr =  0.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  if(handle->isEnabled == 1)
	  {
		if(BMI160_X_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }
	  else
	  {
		if(BMI160_X_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

	  if(handle->isEnabled == 1)
	  {
		if(BMI160_X_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }
	  else
	  {
		if(BMI160_X_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

	  us8 fs_low_level;

	  if ( bmi160_get_accel_range(&fs_low_level) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  switch( fs_low_level )
	  {
		case BMI160_ACCEL_RANGE_2G:
		  *fullScale =	2.0f;
		  break;
		case BMI160_ACCEL_RANGE_4G:
		  *fullScale =	4.0f;
		  break;
		case BMI160_ACCEL_RANGE_8G:
		  *fullScale =	8.0f;
		  break;
		case BMI160_ACCEL_RANGE_16G:
		  *fullScale = 16.0f;
		  break;
		default:
		  *fullScale = -1.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs )
{
    us8 new_fs;
	switch( fs )
	{
		case FS_LOW:
		  new_fs = BMI160_ACCEL_RANGE_2G;
		  break;
		case FS_MID:
		  new_fs = BMI160_ACCEL_RANGE_4G;
		  break;
		case FS_HIGH:
		  new_fs = BMI160_ACCEL_RANGE_8G;
		  break;
		default:
		  return COMPONENT_ERROR;
	}
	if ( bmi160_set_accel_range(new_fs) != BMI160_SUCCESS)
	{
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

	  us8 new_fs;

	  new_fs =   ( fullScale <= 2.0f ) ? BMI160_ACCEL_RANGE_2G
			   : ( fullScale <= 4.0f ) ? BMI160_ACCEL_RANGE_4G
			   : ( fullScale <= 8.0f ) ? BMI160_ACCEL_RANGE_8G
			   :						 BMI160_ACCEL_RANGE_16G;

	  if ( bmi160_set_accel_range(new_fs) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Get_Axes_Status( DrvContextTypeDef *handle, us8 *xyz_enabled )
{

	  us8 xyzStatus;

	  if ( bmi160_get_accel_power_mode_stat(&xyzStatus) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }


	  xyz_enabled[0] = ( xyzStatus == ACCEL_MODE_NORMAL ) ? 1 : 0;
	  xyz_enabled[1] = ( xyzStatus == ACCEL_MODE_NORMAL ) ? 1 : 0;
	  xyz_enabled[2] = ( xyzStatus == ACCEL_MODE_NORMAL ) ? 1 : 0;


	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_Axes_Status( DrvContextTypeDef *handle, us8 *enable_xyz )
{
      us8 xyzStatus;

	  if((enable_xyz[0]+enable_xyz[1]+enable_xyz[2]) == 0)
	  {
          xyzStatus = ACCEL_SUSPEND;
	  }
	  else
	  {
	      xyzStatus = ACCEL_MODE_NORMAL;
	  }
	  if (bmi160_accel_mode(xyzStatus) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}
static DrvStatusTypeDef BMI160_G_Init( DrvContextTypeDef *handle )
{

	  //us8 axes_status[] = { 1, 1, 1 };
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  if ( BMI160_Check_WhoAmI( handle ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  /* Select default output data rate. */
	  pComponentData->Previous_ODR = 800.0f;
	  pComponentData->comboData->lastODR = 800.0f;

	  /* Output data rate selection - power down. */
	  if (bmi160_set_gyro_output_data_rate(BMI160_GYRO_OUTPUT_DATA_RATE_800HZ) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  /* Full scale selection */
	  if ( BMI160_G_Set_FS( handle, FS_HIGH ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  /* Enable axes
	  if ( BMI160_G_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }*/

	  handle->isInitialized = 1;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_DeInit( DrvContextTypeDef *handle )
{
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  if ( BMI160_Check_WhoAmI( handle ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  /* Disable the component */
	  if ( BMI160_G_Sensor_Disable( handle ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  /* Reset output data rate. */
	  pComponentData->Previous_ODR = 800.0f;

	  handle->isInitialized = 0;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Sensor_Enable( DrvContextTypeDef *handle )
{
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;
	  us8 v_accel_power_mode_stat_us8 = 0;

	  /* Check if the component is already enabled */
	  if ( handle->isEnabled == 1 )
	  {
		return COMPONENT_OK;
	  }

	  /* Output data rate selection */
	  if ( BMI160_G_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  /*enable  gyro fifo
	  if (bmi160_set_fifo_gyro_enable(0x01) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(5);
	  if (bmi160_set_fifo_down_gyro(0x01) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	   bmi160_delay_ms(5);
	  if(bmi160_set_fifo_header_enable(BMI160_ENABLE) != BMI160_SUCCESS)
	  {
	    return COMPONENT_ERROR;
      }*/
	  bmi160_delay_ms(5);
	  if ( bmi160_gyro_mode(GYRO_MODE_NORMAL) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	  bmi160_delay_ms(200);
	  if( bmi160_get_gyro_power_mode_stat(&v_accel_power_mode_stat_us8) != BMI160_SUCCESS)
	  {
		 return COMPONENT_ERROR;
	  }

	  if(v_accel_power_mode_stat_us8 != 1)
	  {
		 return COMPONENT_ERROR;
	  }
	  pComponentData->comboData->isEnabled = 1;
	  handle->isEnabled = 1;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Sensor_Disable( DrvContextTypeDef *handle )
{
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  /* Check if the component is already disabled */
	  if ( handle->isEnabled == 0 )
	  {
		return COMPONENT_OK;
	  }

	  /* Store actual output data rate.
	  if ( BMI160_G_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }*/
	  bmi160_delay_ms(5);
	  /* Output data rate selection - power down.
	  if ( bmi160_set_gyro_output_data_rate(BMI160_GYRO_OUTPUT_DATA_RATE_100HZ) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }*/
	  if ( bmi160_gyro_mode(GYRO_MODE_SUSPEND) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  pComponentData->comboData->isEnabled = 0;
	  pComponentData->comboData->lastODR = 800.0f;
	  handle->isEnabled = 0;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_WhoAmI( DrvContextTypeDef *handle, us8 *who_am_i )
{

	  return BMI160_Get_WhoAmI(handle, who_am_i);
}

static DrvStatusTypeDef BMI160_G_Check_WhoAmI( DrvContextTypeDef *handle )
{

	  return BMI160_Check_WhoAmI(handle);
}

static DrvStatusTypeDef BMI160_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity )
{

	  struct bmi160_gyro_t gyro = {0,0,0};
	  float sensitivity = 0;

	  /* Get BMI160 actual sensitivity. */
	  if ( BMI160_G_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  if ( bmi160_read_gyro_xyz(&gyro) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }
	  /* Calculate the data. */

	  angular_velocity->AXIS_X = ( int32_ts )( gyro.x * sensitivity );
	  angular_velocity->AXIS_Y = ( int32_ts )( gyro.y * sensitivity );
	  angular_velocity->AXIS_Z = ( int32_ts )( gyro.z * sensitivity );
	  //printf("before gyro:AXIS_X =%d AXIS_Y = %d AXIS_Z = %d\n",angular_velocity->AXIS_X,angular_velocity->AXIS_Y,angular_velocity->AXIS_Z);
	  if(bmi160_offset.isGyroValid == 1)
	  {
		angular_velocity->AXIS_X = angular_velocity->AXIS_X - ( int32_ts )((bmi160_offset.gyro_x) *(sensitivity));
		angular_velocity->AXIS_Y = angular_velocity->AXIS_Y - ( int32_ts )((bmi160_offset.gyro_y) *(sensitivity));
		angular_velocity->AXIS_Z = angular_velocity->AXIS_Z - ( int32_ts )((bmi160_offset.gyro_z) *(sensitivity));
		//printf("after gyro:AXIS_X =%d AXIS_Y = %d AXIS_Z = %d\n",angular_velocity->AXIS_X,angular_velocity->AXIS_Y,angular_velocity->AXIS_Z);
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

	  struct bmi160_gyro_t gyro = {0,0,0};


	  if ( bmi160_read_gyro_xyz(&gyro) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  /* Set the raw data. */
	  value->AXIS_X = gyro.x;
	  value->AXIS_Y = gyro.y;
	  value->AXIS_Z = gyro.z;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

	  us8 fullScale;

	  /* Read actual full scale selection from sensor. */
	  if ( bmi160_get_gyro_range( &fullScale ) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  /* Store the sensitivity based on actual full scale. */
	  switch( fullScale )
	  {
		case BMI160_GYRO_RANGE_125_DEG_SEC:
		  *sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_125DPS;
		  break;
		case BMI160_GYRO_RANGE_250_DEG_SEC:
		  *sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_250DPS;
		  break;
		case BMI160_GYRO_RANGE_500_DEG_SEC:
		  *sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_500DPS;
		  break;
		case BMI160_GYRO_RANGE_1000_DEG_SEC:
		  *sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_1000DPS;
		  break;
		case BMI160_GYRO_RANGE_2000_DEG_SEC:
		  *sensitivity = ( float )BMI160_GYRO_SENSITIVITY_FOR_FS_2000DPS;
		  break;
		default:
		  *sensitivity = -1.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

	  us8 odr_low_level;

	  if (bmi160_get_gyro_output_data_rate(&odr_low_level) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  switch( odr_low_level )
	  {
		case BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED:
		  *odr =   0.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_25HZ:
		  *odr =  25.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_50HZ:
		  *odr =  50.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_100HZ:
		  *odr = 100.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_200HZ:
		  *odr = 200.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_400HZ:
		  *odr = 400.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_800HZ:
		  *odr = 800.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ:
		  *odr = 1600.0f;
		  break;
		case BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ:
		  *odr = 3200.0f;
		  break;
		default:
		  *odr =  -1.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  if(handle->isEnabled == 1)
	  {
		if(BMI160_G_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }
	  else
	  {
		if(BMI160_G_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

	  if(handle->isEnabled == 1)
	  {
		if(BMI160_G_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }
	  else
	  {
		if(BMI160_G_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
		{
		  return COMPONENT_ERROR;
		}
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

	  us8 fs_low_level;

	  if ( bmi160_get_gyro_range( &fs_low_level ) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  switch( fs_low_level )
	  {
		case BMI160_GYRO_RANGE_125_DEG_SEC:
		  *fullScale =	125.0f;
		  break;
		case BMI160_GYRO_RANGE_250_DEG_SEC:
		  *fullScale =	250.0f;
		  break;
		case BMI160_GYRO_RANGE_500_DEG_SEC:
		  *fullScale =  500.0f;
		  break;
		case BMI160_GYRO_RANGE_1000_DEG_SEC:
		  *fullScale =	1000.0f;
		  break;
		case BMI160_GYRO_RANGE_2000_DEG_SEC:
		  *fullScale =  2000.0f;
		  break;
		default:
		  *fullScale =	 -1.0f;
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

	  us8 new_fs;

	  switch( fullScale )
	  {
		case FS_LOW:
		  new_fs = BMI160_GYRO_RANGE_125_DEG_SEC;
		  break;
		case FS_MID:
		  new_fs = BMI160_GYRO_RANGE_500_DEG_SEC;
		  break;
		case FS_HIGH:
		  new_fs = BMI160_GYRO_RANGE_2000_DEG_SEC;
		  break;
		default:
		  return COMPONENT_ERROR;
	  }

	  if ( bmi160_set_gyro_range(new_fs) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

	  us8 new_fs;

	  new_fs =   ( fullScale <= 125.0f ) ? BMI160_GYRO_RANGE_125_DEG_SEC
			   : ( fullScale <= 250.0f ) ? BMI160_GYRO_RANGE_250_DEG_SEC
			   : ( fullScale <= 500.0f ) ? BMI160_GYRO_RANGE_500_DEG_SEC
			   : ( fullScale <=1000.0f ) ? BMI160_GYRO_RANGE_1000_DEG_SEC
			   :						   BMI160_GYRO_RANGE_2000_DEG_SEC;

	  if ( bmi160_set_gyro_range(new_fs) != BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Get_Axes_Status( DrvContextTypeDef *handle, us8 *xyz_enabled )
{

		  us8 xyzStatus;

		  if ( bmi160_get_gyro_power_mode_stat(&xyzStatus) != BMI160_SUCCESS)
		  {
			return COMPONENT_ERROR;
		  }


		  xyz_enabled[0] = ( xyzStatus == GYRO_MODE_NORMAL ) ? 1 : 0;
		  xyz_enabled[1] = ( xyzStatus == GYRO_MODE_NORMAL ) ? 1 : 0;
		  xyz_enabled[2] = ( xyzStatus == GYRO_MODE_NORMAL ) ? 1 : 0;


		  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_Axes_Status( DrvContextTypeDef *handle, us8 *enable_xyz )
{
   us8 xyzStatus;

   if((enable_xyz[0]+enable_xyz[1]+enable_xyz[2]) == 0)
   {
          xyzStatus = GYRO_MODE_SUSPEND;
   }
   else
   {
	      xyzStatus = GYRO_MODE_NORMAL;
   }
   if (bmi160_gyro_mode(xyzStatus) != BMI160_SUCCESS)
   {
		  return COMPONENT_ERROR;
   }

   return COMPONENT_OK;
}
static DrvStatusTypeDef BMI160_Get_WhoAmI( DrvContextTypeDef *handle, us8 *who_am_i )
{
	/* Read CHIP ID register */
	if ( bmi160_read_reg(BMI160_USER_CHIP_ID_ADDR,who_am_i, 1) != BMI160_SUCCESS )
	{
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_Check_WhoAmI( DrvContextTypeDef *handle )
{

	  us8 who_am_i = 0x00;

	  if ( BMI160_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }
	  if ( who_am_i != handle->who_am_i )
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  us8 new_odr;

	  switch( odr )
	  {
		case ODR_LOW:
		  new_odr = BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ;
		  break;
		case ODR_MID_LOW:
		  new_odr = BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ;
		  break;
		case ODR_MID:
		  new_odr = BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ;
		  break;
		case ODR_MID_HIGH:
		  new_odr = BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ;
		  break;
		case ODR_HIGH:
		  new_odr = BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ;
		  break;
		default:
		  return COMPONENT_ERROR;
	  }

	  if ( bmi160_set_accel_output_data_rate(new_odr,BMI160_ACCEL_NORMAL_AVG4) != BMI160_SUCCESS )
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

	  switch( odr )
	  {
		case ODR_LOW:
		  pComponentData->Previous_ODR = 0.78f;
		  break;
		case ODR_MID_LOW:
		  pComponentData->Previous_ODR = 100.0f;
		  break;
		case ODR_MID:
		  pComponentData->Previous_ODR = 200.0f;
		  break;
		case ODR_MID_HIGH:
		  pComponentData->Previous_ODR = 400.0f;
		  break;
		case ODR_HIGH:
		  pComponentData->Previous_ODR = 800.0f;
		  break;
		default:
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

	  us8 new_odr;

	  new_odr =  ( odr <=	0.78f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ
				: ( odr <=  1.56f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ
				: ( odr <=  3.12f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ
				: ( odr <=  6.25f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ
				: ( odr <=  12.5f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ
				: ( odr <=  25.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
				: ( odr <=  50.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ
				: ( odr <= 100.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ
				: ( odr <= 200.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ
				: ( odr <= 400.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ
				: ( odr <= 800.0f ) ? BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ
				:					  BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ;

	  if ( bmi160_set_accel_output_data_rate(new_odr,BMI160_ACCEL_NORMAL_AVG4) != BMI160_SUCCESS )
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

	  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
	  BMI160_X_Data_t *pComponentData = ( BMI160_X_Data_t * )pData->pComponentData;

	  pComponentData->Previous_ODR =  ( odr <=  0.78f ) ? 0.78f
									 : ( odr <=  1.56f ) ? 1.56f
									 : ( odr <=  3.12f ) ? 3.12f
									 : ( odr <=  6.25f ) ? 6.25f
									 : ( odr <=  12.5f ) ? 12.5f
									 : ( odr <=  25.0f ) ? 25.0f
									 : ( odr <=  50.0f ) ? 50.0f
									 : ( odr <= 100.0f ) ? 100.0f
									 : ( odr <= 200.0f ) ? 200.0f
									 : ( odr <= 400.0f ) ? 400.0f
									 : ( odr <= 800.0f ) ? 800.0f
									 :					   1600.0f;

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  us8  new_odr;
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  switch( odr )
	  {
		case ODR_LOW:
		  new_odr = BMI160_GYRO_OUTPUT_DATA_RATE_50HZ;
		  break;
		case ODR_MID_LOW:
		  new_odr = BMI160_GYRO_OUTPUT_DATA_RATE_100HZ;
		  break;
		case ODR_MID:
		  new_odr = BMI160_GYRO_OUTPUT_DATA_RATE_200HZ;
		  break;
		case ODR_MID_HIGH:
		  new_odr = BMI160_GYRO_OUTPUT_DATA_RATE_400HZ;
		  break;
		case ODR_HIGH:
		  new_odr = BMI160_GYRO_OUTPUT_DATA_RATE_800HZ;
		  break;
		default:
		  return COMPONENT_ERROR;
	  }

	  if ( bmi160_set_gyro_output_data_rate(new_odr) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  if(BMI160_G_Get_ODR( handle, &pComponentData->comboData->lastODR ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}


static DrvStatusTypeDef BMI160_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  switch( odr )
	  {
		case ODR_LOW:
		  pComponentData->Previous_ODR = 50.0f;
		  pComponentData->comboData->lastODR = 50.0f;
		  break;
		case ODR_MID_LOW:
		  pComponentData->Previous_ODR = 100.0f;
		  pComponentData->comboData->lastODR = 100.0f;
		  break;
		case ODR_MID:
		  pComponentData->Previous_ODR = 200.0f;
		  pComponentData->comboData->lastODR = 200.0f;
		  break;
		case ODR_MID_HIGH:
		  pComponentData->Previous_ODR = 400.0f;
		  pComponentData->comboData->lastODR = 400.0f;
		  break;
		case ODR_HIGH:
		  pComponentData->Previous_ODR = 800.0f;
		  pComponentData->comboData->lastODR = 800.0f;
		  break;
		default:
		  return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

	  us8 new_odr;
	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  new_odr = ( odr <=  0.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
				: ( odr <=	25.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_25HZ
				: ( odr <= 50.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_50HZ
				: ( odr <= 100.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_100HZ
				: ( odr <= 200.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_200HZ
				: ( odr <= 400.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_400HZ
				: ( odr <= 800.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_800HZ
				: ( odr <= 1600.0f ) ? BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ
				:					  BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ;

	  if ( bmi160_set_gyro_output_data_rate(new_odr) !=BMI160_SUCCESS)
	  {
		return COMPONENT_ERROR;
	  }

	  if(BMI160_G_Get_ODR( handle, &pComponentData->comboData->lastODR ) == COMPONENT_ERROR )
	  {
		return COMPONENT_ERROR;
	  }

	  return COMPONENT_OK;
}

static DrvStatusTypeDef BMI160_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

	  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
	  BMI160_G_Data_t *pComponentData = ( BMI160_G_Data_t * )pData->pComponentData;

	  pComponentData->Previous_ODR = ( odr <=  0.0f ) ? 0.0f
									 : ( odr <=  25.0f ) ? 25.0f
									 : ( odr <= 50.0f ) ? 50.0f
									 : ( odr <= 100.0f ) ? 100.0f
									 : ( odr <= 200.0f ) ? 200.0f
									 : ( odr <= 400.0f ) ? 400.0f
									 : ( odr <= 800.0f ) ? 800.0f
									 : ( odr <= 1600.0f ) ? 1600.0f
									 :					   3200.0f;

	  pComponentData->comboData->lastODR =  ( odr <=  0.0f ) ? 0.0f
									 			: ( odr <=  25.0f ) ? 25.0f
									 			: ( odr <= 50.0f ) ? 50.0f
									 			: ( odr <= 100.0f ) ? 100.0f
									 			: ( odr <= 200.0f ) ? 200.0f
									 			: ( odr <= 400.0f ) ? 400.0f
									 			: ( odr <= 800.0f ) ? 800.0f
									 			: ( odr <= 1600.0f ) ? 1600.0f
									 			:					   3200.0f;

	  return COMPONENT_OK;
}

char BMI160_INIT_SENSOR_API(void)
{
	char com_rslt = 0;
	bmi160_offset.isAccValid = 0;
	bmi160_offset.isGyroValid = 0;
	com_rslt  = bmi160_init_sensor();
	com_rslt += bmi160_init_gpio();
	com_rslt += getBmi160OffsetValue((void *)(&bmi160_offset));//read accel and gyro offset value
	if((bmi160_offset.accel_k_xyz[0][0] == 1) && (bmi160_offset.accel_k_xyz[0][1] == 1) && (bmi160_offset.accel_k_xyz[0][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[1][0] == 1) && (bmi160_offset.accel_k_xyz[1][1] == 1) && (bmi160_offset.accel_k_xyz[1][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[2][0] == 1) && (bmi160_offset.accel_k_xyz[2][1] == 1) && (bmi160_offset.accel_k_xyz[2][2] == 1) )
	{
		gSensorCalStatus = GSENSORCAL_1;
	}
	else
	{
		gSensorCalStatus = GSENSORCAL_6;
	}
	com_rslt += bmi160_set_intr_data_rdy(BMI160_INTR1_MAP_DATA_RDY,BMI160_ENABLE);//data ready intr ---> GYRO_S_INT(PB0)
	//com_rslt += bmi160_init_fifo();
	//bmi160_delay_ms(20);
	//com_rslt += bmi160_set_intr_fifo_wm(BMI160_INTR1_MAP_DATA_RDY, BMI160_ENABLE);
	bmi160_delay_ms(20);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_init_fifo(void)
{
	char com_rslt = 0;
	us8  init_value = 0;
	com_rslt  = bmi160_write_reg(BMI160_USER_FIFO_CONFIG_0_ADDR,&init_value,1);
	bmi160_delay_ms(5);
	com_rslt += bmi160_write_reg(BMI160_USER_FIFO_CONFIG_1_ADDR,&init_value,1);
	bmi160_delay_ms(5);
	com_rslt += bmi160_write_reg(BMI160_USER_FIFO_DOWN_ADDR,&init_value,1);
	bmi160_delay_ms(5);
	com_rslt += bmi160_set_fifo_wm(0x5A);
	bmi160_delay_ms(5);
	com_rslt += bmi160_set_fifo_header_enable(BMI160_ENABLE);
	return com_rslt;

}

/*!
 *	@brief This function used for interrupt configuration
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_init_gpio(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
	us8 init_value = 0;
	struct bmi160_t *bmi160_temp_p;
        /* Configure the in/out control of interrupt*/
	bmi160_temp_p = bmi160_get_ptr();
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    init_value = 0xBB;
	com_rslt = bmi160_write_reg(BMI160_USER_INTR_OUT_CTRL_ADDR,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	init_value = 0x01;
	com_rslt += bmi160_write_reg(BMI160_USER_INTR_LATCH_ADDR,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	init_value = 0x00;//init disable
	com_rslt += bmi160_write_reg(0x50,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	init_value = 0x10;
	com_rslt += bmi160_write_reg(0x51,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	init_value = 0x00;
	com_rslt += bmi160_write_reg(0x52,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	init_value = 0x00;//map init 0x0(0x55-0x57)
	com_rslt += bmi160_write_reg(0x55,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	com_rslt += bmi160_write_reg(0x56,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	com_rslt += bmi160_write_reg(0x57,&init_value,1);
	bmi160_temp_p->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	return com_rslt;
}

/*
#define ACCEL_MODE_NORMAL	(0x11)
#define	ACCEL_LOWPOWER		(0X12)
#define	ACCEL_SUSPEND		(0X10)
*/

BMI160_RETURN_FUNCTION_TYPE bmi160_accel_mode(us8 accel_power_mode)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(accel_power_mode);
    return com_rslt;
}
/*
#define GYRO_MODE_SUSPEND		(0x14)
#define GYRO_MODE_NORMAL		(0x15)
#define GYRO_MODE_FASTSTARTUP	(0x17)
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_gyro_mode(us8 gyro_power_mode)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(gyro_power_mode);
    return com_rslt;
}
/*
#define MAG_MODE_SUSPEND	(0x18)
#define MAG_MODE_NORMAL	(0x19)
#define MAG_MODE_LOWPOWER	(0x1A)
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_mode(us8 mag_power_mode)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(mag_power_mode);
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_flush(void)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(0xB0);
	bmi160_delay_ms(3);
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_int_reset(void)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(0xB1);
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_softreset(void)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_ERROR;
    com_rslt = bmi160_set_command_register(0xB6);
    return com_rslt;
}
#include "stm32f4xx.h"
int bmi160_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{

    us8 v_accel_power_mode = 0xff,v_gyro_power_mode = 0xff,count = 0;
    switch(_notify_flag){
        case PWRMGR_NOTIFY_STOP_STATE:{
            if(_state == STOP_ENTER){

			//HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			//printf("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			while(1)
			{
				bmi160_accel_mode(ACCEL_SUSPEND);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_SUSPEND) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x0 ) && (v_gyro_power_mode == 0x0 ))
				{
					gsensor_debug("bmi160 set supend success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("bmi160 set supend fail\n");
					break;
				}
			}
            }
            else if(_state == STOP_LEAVE){
			//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			while(1)
			{
				bmi160_accel_mode(ACCEL_MODE_NORMAL);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_NORMAL) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x01 ) && (v_gyro_power_mode == 0x01 ))
				{
					gsensor_info("bmi160 set normal success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("bmi160 set normal fail\n");
					break;
				}
			}
            }
            else{
                gsensor_err("%s state error\n",__func__);
            }
            break;}
        case PWRMGR_NOTIFY_USB_DP_HOLE:{
            if(_state == PLUG_OUT){

			//HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			//printf("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			while(1)
			{
				bmi160_accel_mode(ACCEL_SUSPEND);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_SUSPEND) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x0 ) && (v_gyro_power_mode == 0x0 ))
				{
					gsensor_info("[USB PLUG OUT]bmi160 set supend success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("[USB PLUG OUT]bmi160 set supend fail\n");
					break;
				}
			}
            }
            else if(_state == PLUG_IN){
			//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			while(1)
			{
				bmi160_accel_mode(ACCEL_MODE_NORMAL);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_NORMAL) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x01 ) && (v_gyro_power_mode == 0x01 ))
				{
					gsensor_info("[USB PLUG IN]bmi160 set normal success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("[USB PLUG IN]bmi160 set normal fail\n");
					break;
				}
			}
            }
            else{
                gsensor_err("%s state error\n",__func__);
            }
            break;}
        default:
            gsensor_info("bmi160 don't care this flag\n");
            break;
    }
    return 0;
}
void bmi160_postinit(void)
{
    Bmi160PmNotifyData.func_name = "bmi160_drv";
    Bmi160PmNotifyData.data = NULL;
    Bmi160PmNotifyData.callback= bmi160_notify_callback;
    Bmi160PmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE |
								 PWRMGR_NOTIFY_USB_DP_HOLE;
    Bmi160PmNotifyData.func_level = PWRMGR_FUNC_DRIVER_LEVEL;
    PWRMGR_register_notify_func(&Bmi160PmNotifyData);

    gsensor_info("bmi160_postinit success!\n");

}

static DrvStatusTypeDef BMI160_X_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode )
{
	us8 v_accel_power_mode = 0xff,v_gyro_power_mode = 0xff,count = 0;
	if(SensorMode == SUSPEND){
		//printf("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			while(1)
			{
				bmi160_accel_mode(ACCEL_SUSPEND);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_SUSPEND) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x0 ) && (v_gyro_power_mode == 0x0 ))
				{
					gsensor_info("bmi160 set supend success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("bmi160 set supend fail\n");
					break;
				}
			}
	}
	else if(SensorMode == RESUME){
			while(1)
			{
				bmi160_accel_mode(ACCEL_MODE_NORMAL);
				HAL_Delay(10);
				bmi160_gyro_mode(GYRO_MODE_NORMAL) ;
				HAL_Delay(100);
				bmi160_get_accel_power_mode_stat(&v_accel_power_mode);
				HAL_Delay(2);
				bmi160_get_gyro_power_mode_stat(&v_gyro_power_mode);
				if((v_accel_power_mode == 0x01 ) && (v_gyro_power_mode == 0x01 ))
				{
					gsensor_info("bmi160 set normal success\n");
					break;
				}
				count++;
				if(count == 5)
				{
					gsensor_err("bmi160 set normal fail\n");
					break;
				}
			}
	}

	return COMPONENT_OK;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_mag_init(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	us8 v_data_us8  = 0;
	v_data_us8  = 0x19;
	com_rslt  = bmi160_write_reg(0x7E,&v_data_us8,1);
	bmi160_delay_ms(100);

	v_data_us8  = 0x37;
	com_rslt += bmi160_write_reg(0x7E,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = 0x9A;
	com_rslt += bmi160_write_reg(0x7E,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = 0xC0;
	com_rslt += bmi160_write_reg(0x7E,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = 0x90;
	com_rslt += bmi160_write_reg(0x7F,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = 0x80;
	com_rslt += bmi160_write_reg(0x7F,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = (0x0C << 1);//I2C ADDR
	com_rslt += bmi160_write_reg(0x4B,&v_data_us8,1);
	v_data_us8  = 0x20;
	com_rslt += bmi160_write_reg(0x6B,&v_data_us8,1);
	bmi160_delay_ms(5);

	v_data_us8  = 0X08;//
	com_rslt += bmi160_write_reg(0x44,&v_data_us8,1);

	v_data_us8  = 0x80 |0x02;
	com_rslt += bmi160_write_reg(0x4C,&v_data_us8, 1);

	return com_rslt;
}
static BMI160_RETURN_FUNCTION_TYPE bmi_is_mag_manual(us8 * enable)
{
	us8 v_data_us8;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	com_rslt = bmi160_read_reg(0x4C,&v_data_us8, 1);
	if(v_data_us8 & 0x80)
		*enable = 1;
	else
		*enable = 0;
	return com_rslt;
}
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_write_reg(us8 v_addr_us8,us8 *v_data)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	us8 v_data_us8  = 0;
	com_rslt = bmi_is_mag_manual(&v_data_us8);
	if(!v_data_us8){
		com_rslt += bmi160_mag_manual_enable();
	}

	com_rslt += bmi160_write_reg(0x4F,v_data, 1);//data to write
	bmi160_delay_ms(2);
	com_rslt += bmi160_write_reg(0x4E,&v_addr_us8, 1);//address to write
	bmi160_delay_ms(2);

	com_rslt += bmi_is_mag_manual(&v_data_us8);
	if(v_data_us8){
		com_rslt += bmi160_mag_manual_disable(0x11,8);
	}
	return com_rslt;
}
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_read_reg(us8 v_addr_us8,us8 *v_data,us8 v_len_us8 )
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	us8 v_data_us8;


	if(v_len_us8 != 9)
	{
		com_rslt = bmi_is_mag_manual(&v_data_us8);
		if(!v_data_us8){
			com_rslt += bmi160_mag_manual_enable();
		}

		com_rslt += bmi160_write_reg(0x4D,&v_addr_us8, 1);
		bmi160_delay_ms(2);
		com_rslt += bmi160_read_reg(0x04,v_data, v_len_us8);//return reg value

		com_rslt += bmi_is_mag_manual(&v_data_us8);
		if(v_data_us8){
			com_rslt += bmi160_mag_manual_disable(0x11,8);
		}

	}else
	{
		com_rslt = bmi_is_mag_manual(&v_data_us8);
		if(v_data_us8){
			com_rslt += bmi160_mag_manual_disable(0x11,8);
		}

		*v_data = 0x01;
		com_rslt += bmi160_read_reg(0x04,v_data+1, v_len_us8-1);//return rawdata
	}
	return com_rslt;
}
BMI160_RETURN_FUNCTION_TYPE  bmi160_mag_manual_disable(us8 v_addr_us8,us8 v_len_us8 )
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	us8 v_data_us8  = 0;


	switch(v_len_us8)
	{
		case 1:
			v_data_us8  = 0x00;
			break;
		case 2:
			v_data_us8  = 0x01;
			break;
		case 3:
		case 4:
		case 5:
		case 6:
			v_data_us8  = 0x02;
			break;
		case 7:
		case 8:
		case 9:
			v_data_us8  = 0x03;
			break;
		default:
			return 0;

	}
	com_rslt = bmi160_write_reg(0x4D,&v_addr_us8, 1);
	bmi160_delay_ms(2);
	com_rslt += bmi160_write_reg(0x4C,&v_data_us8, 1);
	bmi160_delay_ms(104);
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_mag_manual_enable(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	us8 v_data_us8  = 0;

	v_data_us8  = 0x80|0x02;
	com_rslt = bmi160_write_reg(0x4C,&v_data_us8, 1);
	bmi160_delay_ms(104);
	return com_rslt;
}
