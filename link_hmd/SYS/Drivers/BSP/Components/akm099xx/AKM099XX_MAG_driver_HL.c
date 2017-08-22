/**
 ******************************************************************************
 * @file    AKM099XX_MAG_driver_HL.c
 * @author  htc BSP Team
 * @version V2.0.0
 * @date    10-November-2016
 * @brief   This file contains definitions for the AKM099XX_MAG_driver_HL.h firmware driver
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
#include "stm32f4xx_hal.h"
#include "AKM099XX_MAG_driver_HL.h"
#include "AKS_APIs.h"
#include "AKH_APIs.h"
#include "ak099xx_register.h"
#include "AKM_CustomerSpec.h"
#include "common/akl_lib_func.h"
#include "PowerManager_notify_func.h"
#include "lib3d/akl_smart_compass.h"

static DrvStatusTypeDef AKM099XX_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef AKM099XX_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef AKM099XX_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef AKM099XX_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef AKM099XX_Get_WhoAmI( DrvContextTypeDef *handle, uint16_t *who_am_i );
static DrvStatusTypeDef AKM099XX_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef AKM099XX_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field );
static DrvStatusTypeDef AKM099XX_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef AKM099XX_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef AKM099XX_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef AKM099XX_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef AKM099XX_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef AKM099XX_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode );
static DrvStatusTypeDef AKM099XX_Sensor_Selftest( DrvContextTypeDef *handle);
static DrvStatusTypeDef AKM099XX_Get_Accuracy( DrvContextTypeDef *handle,uint8_t * accuracy);
static DrvStatusTypeDef AKM099XX_Set_User_Calibration( DrvContextTypeDef *handle,uint8_t enable);
static DrvStatusTypeDef AKM099XX_Get_User_Calibration( DrvContextTypeDef *handle,uint8_t *info,uint8_t len);


struct AKL_SCL_PRMS *prm = NULL;
static uint8_t accuracy_akm = 0;
static struct AKL_NV_PRMS akm_nv_data = {0};
static uint8_t isAkmUserCalibration = 0;
/**
 * @brief AKM099XX driver structure
 */
MAGNETO_Drv_t AKM099XXDrv =
{
	AKM099XX_Init,
	AKM099XX_DeInit,
	AKM099XX_Sensor_Enable,
	AKM099XX_Sensor_Disable,
	AKM099XX_Get_WhoAmI,
	AKM099XX_Check_WhoAmI,
	AKM099XX_Get_Axes,
	AKM099XX_Get_AxesRaw,
	AKM099XX_Get_Sensitivity,
	AKM099XX_Get_ODR,
	AKM099XX_Set_ODR,
	AKM099XX_Set_ODR_Value,
	NULL,
	NULL,
	NULL,
	AKM099XX_Set_Mode,
	AKM099XX_Sensor_Selftest,
	AKM099XX_Get_Accuracy,
	AKM099XX_Set_User_Calibration,
	AKM099XX_Get_User_Calibration
};

/*magnetic sensor register power manager notify function*/
static struct pwrmgr_notify_func_data akmPmNotifyData = {0};

/*magnetic sensor callback function for power manager*/
int akmSensor_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
	int16_t ret = AKM_SUCCESS;
	switch(_notify_flag){
		case PWRMGR_NOTIFY_STOP_STATE:
			//mag_info("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			if(_state == STOP_ENTER){
				ret = AKS_Stop(AKM_ST_MAG);
				ret += AKH_Disable();
				if(ret != AKM_SUCCESS){
					mag_err("%s magnetic sensor enter suspend fail\n",__func__);
					return COMPONENT_ERROR;
				}
			}
			else if(_state == STOP_LEAVE){
				//ret = AKH_Enable();
				ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_100Hz);
				ret += AKH_Enable();
				if(ret != AKM_SUCCESS){
					mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
					return COMPONENT_ERROR;
				}
			}
			else{
				mag_warning("%s notify state is not support,please check it.\n",__func__);
			}
			break;
		case PWRMGR_NOTIFY_POWER_OFF:
			if(accuracy_akm >0){
				stop_and_save(prm);
			}
			PWRMGR_SendNotifyAck(&akmPmNotifyData);
			break;
		case PWRMGR_NOTIFY_USB_DP_HOLE:
			//mag_info("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			if(_state == PLUG_OUT){
				ret = AKS_Stop(AKM_ST_MAG);
				ret += AKH_Disable();
				if(ret != AKM_SUCCESS){
					mag_err("%s magnetic sensor enter suspend fail\n",__func__);
					return COMPONENT_ERROR;
				}
			}
			else if(_state == PLUG_IN){
				//ret = AKH_Enable();
				ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_100Hz);
				ret += AKH_Enable();
				if(ret != AKM_SUCCESS){
					mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
					return COMPONENT_ERROR;
				}
			}
			else{
				mag_warning("%s notify state is not support,please check it.\n",__func__);
			}
			break;
		default:
			mag_warning("magnetic sensor don't care this flag\n");
			break;
	}
	return 0;
}

static void akm099xx_pwrmgr_func(void)
{
	/*register magnetic sensor suspend and resume function*/
	akmPmNotifyData.func_name = "magneticSensor";
	akmPmNotifyData.data = NULL;
	akmPmNotifyData.callback= akmSensor_notify_callback;
	akmPmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE | PWRMGR_NOTIFY_POWER_OFF | PWRMGR_NOTIFY_USB_DP_HOLE;
	akmPmNotifyData.func_level = PWRMGR_FUNC_DRIVER_LEVEL;
	PWRMGR_register_notify_func(&akmPmNotifyData);
}
/**
 * @brief Initialize the AKM099XX sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Init( DrvContextTypeDef *handle )
{
	uint8_t axis_order[3];
	uint8_t axis_sign[3];

	/* Initialize hardware */
	AKH_Init();

	if ( AKM099XX_Check_WhoAmI( handle ) != COMPONENT_OK )
	{
		mag_err("%s AKM099xx check error!\n",__func__);
		AKH_Disable();
		return COMPONENT_ERROR;
	}

	/* Initialize magnetic sensor */
	axis_order[0] = (uint8_t)AKM_CUSTOM_MAG_AXIS_ORDER_X;
	axis_order[1] = (uint8_t)AKM_CUSTOM_MAG_AXIS_ORDER_Y;
	axis_order[2] = (uint8_t)AKM_CUSTOM_MAG_AXIS_ORDER_Z;
	axis_sign[0] = (uint8_t)AKM_CUSTOM_MAG_AXIS_SIGN_X;
	axis_sign[1] = (uint8_t)AKM_CUSTOM_MAG_AXIS_SIGN_Y;
	axis_sign[2] = (uint8_t)AKM_CUSTOM_MAG_AXIS_SIGN_Z;

	if(AKS_Init(AKM_ST_MAG, axis_order, axis_sign) !=AKM_SUCCESS)
	{
		mag_err("AKM Sensor initial error!\n");
		return COMPONENT_ERROR;
	}

	/*Initialize akm library*/
	print_version();
	if (library_init(&prm) != AKM_SUCCESS)
	{
		mag_err("AKM Sensor library initial error!\n");
		return COMPONENT_ERROR;
	}
	if (load_and_start(prm) != AKM_SUCCESS)
	{
		mag_err("AKM Sensor load and start library error!\n");
		return COMPONENT_ERROR;
	}
	akm_nv_data = *(prm->ps_nv);
	akm099xx_pwrmgr_func();

	handle->isInitialized = 1;
	mag_info("AKM099xx initial done!\n");
	return COMPONENT_OK;
}


/**
 * @brief Deinitialize the AKM099XX sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_DeInit( DrvContextTypeDef *handle )
{

	if ( AKM099XX_Check_WhoAmI( handle ) != COMPONENT_OK )
	{
		mag_err("%s AKM099xx check error!\n",__func__);
		return COMPONENT_ERROR;
	}

	/* Disable the component */
	if ( AKM099XX_Sensor_Disable( handle ) != COMPONENT_OK )
	{
		mag_err("%s AKM099XX_Sensor_Disable fail\n",__func__);
		return COMPONENT_ERROR;
	}

	handle->isInitialized = 0;
	mag_info("AKM099xx deinitial done!\n");
	return COMPONENT_OK;
}



/**
 * @brief Enable the AKM099XX sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Sensor_Enable( DrvContextTypeDef *handle )
{
	/* Check if the component is already enabled */
	if ( handle->isEnabled == 1 )
	{
		return COMPONENT_OK;
	}

	/*set akm sensor odr 100Hz default when enabled*/
	if(AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_100Hz) != AKM_SUCCESS)
	{
		mag_err("enable AKM sensor error!\n");
		return COMPONENT_ERROR;
	}

	handle->isEnabled = 1;
	mag_info("AKM099XX enable success!\n");
	return COMPONENT_OK;
}



/**
 * @brief Disable the AKM099XX sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Sensor_Disable( DrvContextTypeDef *handle )
{

	/* Check if the component is already disabled */
	if ( handle->isEnabled == 0 )
	{
		return COMPONENT_OK;
	}

	if(AKS_Stop(AKM_ST_MAG) != AKM_SUCCESS)
	{
		mag_err("disable AKM sensor error!\n");
		return COMPONENT_ERROR;
	}

	handle->isEnabled = 0;
	mag_info("AKM099XX disable success!\n");
	return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the AKM099XX sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_WhoAmI( DrvContextTypeDef *handle, uint16_t *who_am_i )
{
	/* Read WHO AM I register */
	if(AKS_GetWhoami(AKM_ST_MAG,who_am_i) != AKM_SUCCESS)
	{
		mag_err("Get AKM099XX Who Am I error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the AKM099XX sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Check_WhoAmI( DrvContextTypeDef *handle )
{

	uint16_t who_am_i = 0;

	if (AKM099XX_Get_WhoAmI( handle, &who_am_i ) != COMPONENT_OK )
	{
		mag_err("AKM099XX Device ID read error!\n");
		return COMPONENT_ERROR;
	}
	if ( who_am_i != AK09916C_WIA_VAL )
	{
		mag_err("AKM099XX Device ID check error!\n");
		mag_err("AKM099XX Device ID read out is %#x\n",who_am_i);
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Get the AKM099XX sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field )
{
	uint8_t num =1;
	int16_t ret;
	struct AKM_SENSOR_DATA mag_data;
	#define GET_VEC_BUF_SIZE   6
	int32_t  data[GET_VEC_BUF_SIZE];
	int32_t  st;
	int64_t  timestamp;
	if (AKS_GetData(AKM_ST_MAG,&mag_data,&num) != AKM_SUCCESS)
	{
		mag_err("akm099xx sensor get data error!\n");
		return COMPONENT_ERROR;
	}
	//mag_info("before calibration Q16 format x=%d,y=%d,z=%d\n",mag_data.u.s.x,mag_data.u.s.y,mag_data.u.s.z);

	/*akm099xx calibration*/

	ret = AKL_SetVector(prm, &mag_data, 1) ;
	if (ret != AKM_SUCCESS)
	{
		mag_err("send akm099xx sensor data to library fail!,ret =%d\n",ret);
		return COMPONENT_ERROR;
	}
	if (AKL_GetVector(AKM_VT_MAG, prm, data, GET_VEC_BUF_SIZE, &st, &timestamp) != AKM_SUCCESS)
	{
		mag_err("get akm099xx sensor data from library fail!\n");
		return COMPONENT_ERROR;
	}
	accuracy_akm = (uint8_t)st;
	//mag_info("after calibration Q16 format x=%d,y=%d,z=%d,accuracy=%d\n",data[0],data[1],data[2],st);
	//mag_info("calibration bias x=%d,y=%d,z=%d\n",data[3],data[4],data[5]);
	if((accuracy_akm != akm_nv_data.a_hsuc_hdst)&&(accuracy_akm >0))
	{
		akm_nv_data = *(prm->ps_nv);
		stop_and_save(prm);
		/*
		if(isAkmUserCalibration && (accuracy_akm == 3) ){
			//send calibration status and level to usb service
			isAkmUserCalibration = 0;
			mag_info("send AKM calibration pass command to phone\n");
		}
		*/
	}
	magnetic_field->AXIS_X = data[0];
	magnetic_field->AXIS_Y = data[1];
	magnetic_field->AXIS_Z = data[2];

	return COMPONENT_OK;
}



/**
 * @brief Get the AKM099XX sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{
	uint8_t num =1;
	struct AKM_SENSOR_DATA mag_data;
	int32_t  raw_to_micro_q16[3];
	struct AKS_DEVICE_INFO sensor_info;
	#define SENS_IN_Q16    ((int32_t)(9830)) /* 0.15uT in Q16 format */

	if (AKS_GetData(AKM_ST_MAG,&mag_data,&num) != AKM_SUCCESS)
	{
		mag_err("akm099xx sensor get data error!\n");
		return COMPONENT_ERROR;
	}

	if(AKS_GetDeviceInfo(AKM_ST_MAG,&sensor_info,&num) != AKM_SUCCESS)
	{
		mag_err("disable AKM sensor error!\n");
		return COMPONENT_ERROR;
	}
	for(int i=0;i<3;i++){
		/*
		* ASA store Struct AKS_DEVICE_INFO parameter[2..4]
		* coeff = ((ASA + 128) x 9830) >> 8
		*/
		raw_to_micro_q16[i]=((int32_t)(sensor_info.parameter[i+2] + 128) * SENS_IN_Q16) >> 8;
	}

	value->AXIS_X = (int16_t)(mag_data.u.s.x/raw_to_micro_q16[0]);
	value->AXIS_Y = (int16_t)(mag_data.u.s.y/raw_to_micro_q16[1]);
	value->AXIS_Z = (int16_t)(mag_data.u.s.z/raw_to_micro_q16[2]);

	return COMPONENT_OK;
}

/**
 * @brief Get the AKM099XX sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{
	*sensitivity = 0.15f;//unit uT
	return COMPONENT_OK;
}

/**
 * @brief translate mode to output data rate
 * @param mode akm099xx sensor mode
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Mode2ODR( uint8_t mode,float *odr )
{
	switch(mode)
	{
		case AK099XX_MODE_CONT_MEASURE_MODE1:
			*odr = 10.0f;
			break;
		case AK099XX_MODE_CONT_MEASURE_MODE2:
			*odr = 20.0f;
			break;
		case AK099XX_MODE_CONT_MEASURE_MODE3:
			*odr = 50.0f;
			break;
		case AK099XX_MODE_CONT_MEASURE_MODE4:
			*odr = 100.0f;
			break;
		case AK099XX_MODE_CONT_MEASURE_MODE5:
			*odr = 200.0f;
			break;
		case AK099XX_MODE_CONT_MEASURE_MODE6:
			*odr = 1.0f;
			break;
		case AK099XX_MODE_SNG_MEASURE:
		case AK099XX_MODE_SELF_TEST:
			*odr = 0.0f;
		default:
			return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

/**
 * @brief Get the AKM099XX sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_ODR( DrvContextTypeDef *handle, float *odr )
{
	uint8_t mode;
	if(AKS_GetMode(AKM_ST_MAG,&mode) != AKM_SUCCESS)
	{
		mag_err("akm099xx sensor get mode error!\n");
		return COMPONENT_ERROR;
	}
	if (AKM099XX_Mode2ODR(mode,odr) == COMPONENT_ERROR)
	{
		mag_err("akm099xx sensor translate mode to odr error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}



/**
 * @brief Set the AKM099XX sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{
	AKM099XX_MAG_ODR_t new_odr;
	switch( odr )
	{
		case ODR_LOW:
			new_odr = AKM099XX_MAG_ODR_10Hz;
			break;
		case ODR_MID_LOW:
			new_odr = AKM099XX_MAG_ODR_20Hz;
			break;
		case ODR_MID:
			new_odr = AKM099XX_MAG_ODR_50Hz;
			break;
		case ODR_MID_HIGH:
			new_odr = AKM099XX_MAG_ODR_100Hz;
			break;
		case ODR_HIGH:
			new_odr = AKM099XX_MAG_ODR_200Hz;
			break;
		default:
			return COMPONENT_ERROR;
	}

	if(AKS_Start(AKM_ST_MAG,new_odr) != AKM_SUCCESS)
	{
		mag_err("set AKM sensor odr error!\n");
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}


/**
 * @brief Set the AKM099XX sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{
	int ret = AKM_SUCCESS;

	if(odr > 200.0f){
		/*higher than 200Hz*/
		return COMPONENT_ERROR;
	}else if (odr > 100.0f){
		/*200 Hz*/
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_200Hz);
	}else if (odr > 50.0f){
		/*100 Hz*/
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_100Hz);
	}else if (odr > 20.0f){
		/*50 Hz*/
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_50Hz);
	}else if (odr > 10.0f){
		/*20 Hz*/
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_20Hz);
	}else if (odr > 1.0f){
		/*10 Hz or slower*/
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_10Hz);
	}

	if(ret != AKM_SUCCESS)
	{
		mag_err("set AKM sensor odr value error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

/**
 * @brief Set the AKM099XX sensor work mode
 * @param handle the device handle
 * @param SensorMode the work mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode )
{
	int16_t ret = AKM_SUCCESS;

	if(SensorMode == SUSPEND){
		ret = AKS_Stop(AKM_ST_MAG);
		ret += AKH_Disable();
		if(ret != AKM_SUCCESS){
			mag_err("%s magnetic sensor enter suspend fail\n",__func__);
			return COMPONENT_ERROR;
		}
	}
	else if(SensorMode == RESUME){
		ret = AKS_Start(AKM_ST_MAG,AKM099XX_MAG_ODR_100Hz);
		ret += AKH_Enable();
		if(ret != AKM_SUCCESS){
			mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
			return COMPONENT_ERROR;
		}
	}
	return COMPONENT_OK;
}

/**
 * @brief Set the AKM099XX sensor selftest
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Sensor_Selftest( DrvContextTypeDef *handle )
{
	int32_t	test_result;
	if (AKS_SelfTest(AKM_ST_MAG, &test_result) !=AKM_SUCCESS){
		mag_err("%s magnetic sensor selftest failed with error code:0x%08x\n",__func__,test_result);
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

/**
 * @brief Get the AKM099XX sensor calibration accuracy
 * @param handle the device handle
 * @param accuracy the pointer accuracy written to
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_Accuracy( DrvContextTypeDef *handle,uint8_t * accuracy)
{
	*accuracy = accuracy_akm;
	return COMPONENT_OK;
}

/**
 * @brief Set the AKM099XX sensor to do user calibration
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Set_User_Calibration( DrvContextTypeDef *handle,uint8_t enable)
{
	if(enable){
		isAkmUserCalibration = 1;
	}
	else{
		isAkmUserCalibration = 0;
	}
	//akm_nv_data.a_hsuc_hdst = AKSC_HDST_UNSOLVED;
	return COMPONENT_OK;
}

/**
 * @brief Get the AKM099XX sensor user calibration info
 * @param handle the device handle
 * @param info magnetic sensor calibration info
 * @ param length the length of magnetic sensor calibration info
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef AKM099XX_Get_User_Calibration( DrvContextTypeDef *handle,uint8_t *info,uint8_t len)
{
	if (len != 4){
		return COMPONENT_ERROR;
	}
	if(accuracy_akm == 3){
		isAkmUserCalibration = 0;
		info[0] = 'p';//user calibration pass
	}else{
		info[0] = 'f';//user calibration fail
	}
	if(isAkmUserCalibration){
		info[1] = 'c';//user calibration is calibrating
	}else{
		info[1] = 'd';//user calibration have done
	}
	info[2] = accuracy_akm;// user calibration level value
	info[3] = 't';// whether have user calibration :true  or false

	return COMPONENT_OK;
}

