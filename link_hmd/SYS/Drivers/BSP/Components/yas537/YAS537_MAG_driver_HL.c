/**
 *******************************************************************************
 * @file    YAS537_MAG_driver_HL.c
 * @author  MEMS Application Team
 * @version V2.0.0
 * @date    10-December-2015
 * @brief   This file provides a set of high-level functions needed to manage
 the YAS537 sensor
 *******************************************************************************
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
#include "YAS537_MAG_driver_HL.h"
#include <math.h>
#include "yas.h"
#include "yas_drv.h"
#include "yas_algo.h"
#include <stdio.h>
#include "misc_data.h"
#include "stm32f4xx_hal.h"
#include "rtos_i2c_drv.h"
#include "PowerManager_notify_func.h"


static DrvStatusTypeDef YAS537_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef YAS537_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef YAS537_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef YAS537_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef YAS537_Get_WhoAmI( DrvContextTypeDef *handle, uint16_t *who_am_i );
static DrvStatusTypeDef YAS537_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef YAS537_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field );
static DrvStatusTypeDef YAS537_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef YAS537_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef YAS537_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef YAS537_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef YAS537_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef YAS537_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode );
static DrvStatusTypeDef YAS537_Sensor_Selftest( DrvContextTypeDef *handle);
static DrvStatusTypeDef YAS537_Get_Accuracy( DrvContextTypeDef *handle,uint8_t * accuracy);
static DrvStatusTypeDef YAS537_Set_User_Calibration( DrvContextTypeDef *handle,uint8_t enable);
static DrvStatusTypeDef YAS537_Get_User_Calibration( DrvContextTypeDef *handle,uint8_t *info,uint8_t len);


static int yas_device_open(int32_t type);
static int yas_device_close(int32_t type);
static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf, int len);
static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len);
static uint32_t yas_device_current_time(void);
static void yas_device_usleep(int32_t usec);

static uint8_t accuracy_m = 0;
static struct YAS_NV_PRMS yas_nv_data = {0};
static uint8_t isYasUserCalibration;

/**
 * @brief YAS537 driver structure
 */
MAGNETO_Drv_t YAS537Drv =
{
	YAS537_Init,
	YAS537_DeInit,
	YAS537_Sensor_Enable,
	YAS537_Sensor_Disable,
	YAS537_Get_WhoAmI,
	YAS537_Check_WhoAmI,
	YAS537_Get_Axes,
	YAS537_Get_AxesRaw,
	YAS537_Get_Sensitivity,
	YAS537_Get_ODR,
	YAS537_Set_ODR,
	YAS537_Set_ODR_Value,
	NULL,
	NULL,
	NULL,
	YAS537_Set_Mode,
	YAS537_Sensor_Selftest,
	YAS537_Get_Accuracy,
	YAS537_Set_User_Calibration,
	YAS537_Get_User_Calibration
};
struct yas_driver_callback  YAS537Callback = {

	//int (*device_open)(int32_t type);
	yas_device_open,
	//int (*device_close)(int32_t type);
	yas_device_close,
	//int (*device_write)(int32_t type, uint8_t addr, const uint8_t *buf,int len);
	yas_device_write,
	//int (*device_read)(int32_t type, uint8_t addr, uint8_t *buf, int len);
	yas_device_read,
	//void (*usleep)(int usec);
	yas_device_usleep,
	//uint32_t (*current_time)(void);
	yas_device_current_time,
};
static int yas_device_open(int32_t type)
{
	return YAS_NO_ERROR;
}

static int yas_device_close(int32_t type)
{
	return YAS_NO_ERROR;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf, int len)
{
	I2C_STATUS ret = I2C_OK;
	if (ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_ECOMPASS_ADDR, addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, MAGNETO_I2C_TIMEOUT_MAX)){
		mag_err("%s I2C write error ret=%d\n",__func__,ret);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	else
		return YAS_NO_ERROR;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{
	I2C_STATUS ret = I2C_OK;
	if (ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_ECOMPASS_ADDR, addr, I2C_MEMADD_SIZE_8BIT, buf, len, MAGNETO_I2C_TIMEOUT_MAX)){
		mag_err("%s I2C read error ret=%d\n",__func__,ret);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	else
		return YAS_NO_ERROR;
}

static uint32_t yas_device_current_time(void)
{
	return HAL_GetTick();

}

static void yas_device_usleep(int32_t usec)
{
	/* user-written implementation-specific source code */
	HAL_Delay(usec/1000 + 1);
}

/*magnetic sensor register power manager notify function*/
static struct pwrmgr_notify_func_data magPmNotifyData = {0};

/*magnetic sensor callback function for power manager*/
int magSensor_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
	uint8_t tmp;
	switch(_notify_flag){
		case PWRMGR_NOTIFY_STOP_STATE:
			//mag_info("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			if(_state == STOP_ENTER){
				tmp = 0x00;//disable magnetic sensor sample data to stand by mode
				if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
					mag_err("%s magnetic sensor enter suspend fail\n",__func__);
					return -1;
				}
			}
			else if(_state == STOP_LEAVE){
				tmp = 0x21;//enable magnetic sensor sample data
				if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
					mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
					return -1;
				}
			}
			else{
				mag_warning("%s notify state is not support,please check it.\n",__func__);
			}
			break;
		case PWRMGR_NOTIFY_POWER_OFF:
			if(accuracy_m > 0){
				set_yas_calibrate(yas_nv_data);
			}
			PWRMGR_SendNotifyAck(&magPmNotifyData);
			break;
		case PWRMGR_NOTIFY_USB_DP_HOLE:
			//mag_info("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
			if(_state == PLUG_OUT){
				tmp = 0x00;//disable magnetic sensor sample data to stand by mode
				if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
					mag_err("%s magnetic sensor enter suspend fail\n",__func__);
					return -1;
				}
				mag_info("magnetic sensor suspend done\n");
			}
			else if(_state == PLUG_IN){
				tmp = 0x21;//enable magnetic sensor sample data
				if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
					mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
					return -1;
				}
				mag_info("magnetic sensor resume done\n");
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

static void yas537_pwrmgr_func(void)
{
	/*register magnetic sensor suspend and resume function*/
	magPmNotifyData.func_name = "magneticSensor";
	magPmNotifyData.data = NULL;
	magPmNotifyData.callback= magSensor_notify_callback;
	magPmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE |PWRMGR_NOTIFY_POWER_OFF | PWRMGR_NOTIFY_USB_DP_HOLE;
	magPmNotifyData.func_level = PWRMGR_FUNC_DRIVER_LEVEL;
	PWRMGR_register_notify_func(&magPmNotifyData);
}

/**
 * @brief Initialize the YAS537 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Init( DrvContextTypeDef *handle )
{
  	if ( YAS537_Check_WhoAmI( handle ) == COMPONENT_ERROR )
	{
		mag_err("%s YAS537_Check_WhoAmI error!\n",__func__);
		return COMPONENT_ERROR;
	}
	struct yas_driver_callback *f = NULL;
	f=(struct yas_driver_callback *)handle->pExtVTable;
	if(yas_driver_init(f) < 0)
	{
		mag_err("%s yas_driver_init initial fail\n",__func__);
	}
	if (yas_driver_set_position(YAS_TYPE_MAG, 3) != YAS_TYPE_MAG)
	{
		mag_err("%s yas_driver_set_position  fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_driver_set_delay(YAS_TYPE_MAG, 10 /* msec */) != YAS_TYPE_MAG)
	{
		mag_err("%s yas_driver_set_delay  fail\n",__func__);
		return COMPONENT_ERROR;
	}
	yas537_pwrmgr_func();
	handle->isInitialized = 1;
	mag_info("%s success!\n",__func__);
	return COMPONENT_OK;
}



/**
 * @brief Deinitialize the YAS537 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_DeInit( DrvContextTypeDef *handle )
{

	if ( YAS537_Check_WhoAmI( handle ) == COMPONENT_ERROR )
	{
		mag_err("%s YAS537_Check_WhoAmI error!\n",__func__);
		return COMPONENT_ERROR;
	}

	/* Disable the component */
	if ( YAS537_Sensor_Disable( handle ) == COMPONENT_ERROR )
	{
		mag_err("%s YAS537_Sensor_Disable fail\n",__func__);
		return COMPONENT_ERROR;
	}

	handle->isInitialized = 0;

	return COMPONENT_OK;
}



/**
 * @brief Enable the YAS537 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
struct yas_matrix static_matrix = { {10000, 0, 0, 0, 10000, 0, 0, 0, 10000} };
static DrvStatusTypeDef YAS537_Sensor_Enable( DrvContextTypeDef *handle )
{
	/*******************************
	struct yas_algo_state state;
	state.offset[yas_magnetic].v[0] = 127;
	state.offset[yas_magnetic].v[1] = 127;
	state.offset[yas_magnetic].v[2] = 127;
	state.accuracy[yas_magnetic] = 1;
	********************************/

	/* Check if the component is already enabled */
	if ( handle->isEnabled == 1 )
	{
		return COMPONENT_OK;
	}
	if ( yas_driver_set_enable(YAS_TYPE_M_MAG, 1) != YAS_TYPE_M_MAG)
	{
		mag_err("%s yas_driver_set_enable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_init() < 0)
	{
		mag_err("%s yas_algo_init fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_set_calib_enable(YAS_TYPE_MAG, 1) < 0)
	{
		mag_err("%s yas_algo_set_calib_enable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_set_filter_enable(YAS_TYPE_MAG, 1) < 0)
	{
		mag_err("%s yas_algo_set_filter_enable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_driver_ext(YAS_TYPE_MAG, YAS537_SET_STATIC_MATRIX, &static_matrix) < 0)
	{
		mag_err("%s yas_driver_ext set static matrix fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_driver_ext(YAS_TYPE_MAG, YAS537_GET_STATIC_MATRIX, &static_matrix) < 0)
	{
		mag_err("%s yas_driver_ext get static matrix fail\n",__func__);
		return COMPONENT_ERROR;
	}

	if(get_yas_calibrate(&yas_nv_data)){
		mag_err("%s get_yas_calibrate get calibrate nv data fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if(yas_nv_data.magic == YAS_NV_MAGIC_NUMBER){
		struct yas_algo_state state;
		state.offset[yas_magnetic].v[0] = yas_nv_data.offset[0];
		state.offset[yas_magnetic].v[1] = yas_nv_data.offset[1];
		state.offset[yas_magnetic].v[2] = yas_nv_data.offset[2];
		state.accuracy[yas_magnetic] = yas_nv_data.accuracy;
		yas_algo_set_state(&state);
		mag_info("load yas537 calibrate data success\n");
	}

	handle->isEnabled = 1;
	mag_info("%s success!\n",__func__);
	return COMPONENT_OK;
}



/**
 * @brief Disable the YAS537 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Sensor_Disable( DrvContextTypeDef *handle )
{

	/* Check if the component is already disabled */
	if ( handle->isEnabled == 0 )
	{
		return COMPONENT_OK;
	}

	/* Operating mode selection - power down */
	if ( yas_driver_set_enable(YAS_TYPE_M_MAG, 0) != YAS_TYPE_M_MAG )
	{
		mag_err("%s yas537 disable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_set_calib_enable(YAS_TYPE_MAG, 0) < 0)
	{
		mag_err("%s yas_algo_set_calib_disable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_set_filter_enable(YAS_TYPE_MAG, 0) < 0)
	{
		mag_err("%s yas_algo_set_filter_disable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	if (yas_algo_term() < 0){
		mag_err("%s yas_algo_term error\n",__func__);
		return COMPONENT_ERROR;
	}
	mag_info("%s done!\n",__func__);
	handle->isEnabled = 0;

	return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the YAS537 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Get_WhoAmI( DrvContextTypeDef *handle, uint16_t *who_am_i )
{

	/* Read WHO AM I register */
	int32_t mag_type = 0;
	if (yas_device_read(mag_type, YAS537_REG_DIDR,(uint8_t *)who_am_i , 1) < 0){
		mag_err("Read YAS537 chip ID error!\n");
		return COMPONENT_ERROR;
	}
	*who_am_i = (*who_am_i)&0x00FF;
	return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the YAS537 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Check_WhoAmI( DrvContextTypeDef *handle )
{

	uint16_t who_am_i = 0x0000;

	if ( YAS537_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
	{
		mag_err("YAS537 Device ID read error!\n");
		return COMPONENT_ERROR;
	}
	if ( (who_am_i & 0x00FF) != YAS537_DEVICE_ID )
	{
		mag_err("YAS537 Device ID check error!\n");
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/**
 * @brief Get the YAS537 sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field )
{
	struct yas_data uncal, cal_and_filtered;

	if(yas_driver_measure(YAS_TYPE_MAG, &uncal, 1) != 1)
	{
		mag_err("%s yas_driver_measure fail! \n",__func__);
		return COMPONENT_ERROR;
	}
	//mag_info("magnetic uncalibration data x = %d,y = %d,z = %d,accuracy =%d\n",uncal.xyz.v[0],uncal.xyz.v[1],uncal.xyz.v[2],uncal.accuracy);

	yas_algo_update(&uncal, &cal_and_filtered, 1);
	//mag_info("magnetic calibration data x = %d,y = %d,z = %x,accuracy =%d\n",cal_and_filtered.xyz.v[0],cal_and_filtered.xyz.v[1],cal_and_filtered.xyz.v[2],cal_and_filtered.accuracy);
	accuracy_m = cal_and_filtered.accuracy;
	if((accuracy_m != yas_nv_data.accuracy)&& (accuracy_m >0))
	{
		yas_nv_data.magic = YAS_NV_MAGIC_NUMBER;
		yas_nv_data.offset[0] = cal_and_filtered.caliboffset.v[0];
		yas_nv_data.offset[1] = cal_and_filtered.caliboffset.v[1];
		yas_nv_data.offset[2] = cal_and_filtered.caliboffset.v[2];
		yas_nv_data.accuracy = cal_and_filtered.accuracy;
		set_yas_calibrate(yas_nv_data);
		/*
		if(isYasUserCalibration && (accuracy_m == 3)){
			//send calibration status and level to usb service
			isYasUserCalibration = 0;
			mag_info("send YAS calibration pass command to phone\n");
		}
		*/
	}

	/* Calculate the data. */
	magnetic_field->AXIS_X = ( int32_t )( cal_and_filtered.xyz.v[0] );
	magnetic_field->AXIS_Y = ( int32_t )( cal_and_filtered.xyz.v[1] );
	magnetic_field->AXIS_Z = ( int32_t )( cal_and_filtered.xyz.v[2] );
	return COMPONENT_OK;
}



/**
 * @brief Get the YAS537 sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{
	int32_t mag_type = 0;
	uint8_t data[6];
	#define DATA_MASK 0x3FFF
	if (yas_device_read(mag_type, YAS537_REG_DIDR+2,data, 6) < 0){
		mag_err("Read YAS537 raw data error!\n");
		return COMPONENT_ERROR;
	}
	/* Set the raw data. */
	value->AXIS_X = (data[0] | data[1]<<8)&DATA_MASK;
	value->AXIS_Y = (data[2] | data[3]<<8)&DATA_MASK;
	value->AXIS_Z = (data[4] | data[5]<<8)&DATA_MASK;

	return COMPONENT_OK;
}

/**
 * @brief Get the YAS537 sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{
	*sensitivity = 0.3f; //unit uT
	return COMPONENT_OK;
}


/**
 * @brief Get the YAS537 sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Get_ODR( DrvContextTypeDef *handle, float *odr )
{
	int yas_sample_interval = 0;
	yas_sample_interval = yas_driver_get_delay(YAS_TYPE_MAG);
	*odr = ( yas_sample_interval <= YAS537_MAG_DO_120Hz ) ? 120.000f
		:( yas_sample_interval <= YAS537_MAG_DO_80Hz ) ? 80.000f
		:( yas_sample_interval <= YAS537_MAG_DO_60Hz ) ? 60.000f
		:( yas_sample_interval <= YAS537_MAG_DO_30Hz ) ? 30.000f
		:( yas_sample_interval <= YAS537_MAG_DO_15Hz ) ? 15.000f
		:5.000f;

	return COMPONENT_OK;
}



/**
 * @brief Set the YAS537 sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{
	YAS537_MAG_DO_t new_odr;

	switch( odr )
	{
		case ODR_LOW:
			new_odr = YAS537_MAG_DO_15Hz;
			break;
		case ODR_MID_LOW:
			new_odr = YAS537_MAG_DO_30Hz;
			break;
		case ODR_MID:
			new_odr = YAS537_MAG_DO_60Hz;
			break;
		case ODR_MID_HIGH:
			new_odr = YAS537_MAG_DO_80Hz;
			break;
		case ODR_HIGH:
			new_odr = YAS537_MAG_DO_120Hz;
			break;
		default:
			return COMPONENT_ERROR;
	}
	if(yas_driver_set_delay(YAS_TYPE_MAG, new_odr) != YAS_TYPE_MAG){
		mag_err("%s yas_driver_set_delay fail\n",__func__);
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}


/**
 * @brief Set the YAS537 sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef YAS537_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{
	int yas_delay;
	yas_delay =(int)(odr);
	if(yas_driver_set_delay(YAS_TYPE_MAG, yas_delay) != YAS_TYPE_MAG){
		mag_err("%s yas_driver_set_delay fail\n",__func__);
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

static DrvStatusTypeDef YAS537_Set_Mode( DrvContextTypeDef *handle, SensorMode_t SensorMode )
{
	uint8_t tmp;
	if(SensorMode == SUSPEND){
		tmp = 0x00;//disable magnetic sensor sample data to stand by mode
		if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
			mag_err("%s magnetic sensor enter suspend fail\n",__func__);
			return COMPONENT_ERROR;
		}
	}
	else if(SensorMode == RESUME){
		tmp = 0x21;//enable magnetic sensor sample data
		if (yas_device_write(YAS_TYPE_MAG, YAS537_REG_CMDR, &tmp, 1)){
			mag_err("%s magnetic sensor resume from suspend fail\n",__func__);
			return COMPONENT_ERROR;
		}
	}
	return COMPONENT_OK;
}

static DrvStatusTypeDef YAS537_Sensor_Selftest( DrvContextTypeDef *handle )
{
	struct yas537_self_test_result test_result;
	if (yas_driver_ext(YAS_TYPE_MAG, YAS537_SELF_TEST, &test_result) < 0){
		mag_err("%s magnetic sensor selftest error\n",__func__);
		return COMPONENT_ERROR;
	}
	mag_info("mag selftest result:device id =%x,dir=%d,x=%x,y=%x,z=%x\n",	 \
		test_result.id,test_result.dir,test_result.xyz[0],test_result.xyz[1],test_result.xyz[2]);
	return COMPONENT_OK;
}
static DrvStatusTypeDef YAS537_Get_Accuracy( DrvContextTypeDef *handle,uint8_t * accuracy)
{
	*accuracy = accuracy_m;
	return COMPONENT_OK;
}

static DrvStatusTypeDef YAS537_Set_User_Calibration( DrvContextTypeDef *handle,uint8_t enable)
{
	if(enable){
		isYasUserCalibration = 1;
	}
	else{
		isYasUserCalibration = 0;
	}
	//yas_nv_data.accuracy = 0;
	return COMPONENT_OK;
}

static DrvStatusTypeDef YAS537_Get_User_Calibration( DrvContextTypeDef *handle,uint8_t *info,uint8_t len)
{
	if (len != 4){
		return COMPONENT_ERROR;
	}
	if(accuracy_m == 3){
		isYasUserCalibration = 0;
		info[0] = 'p';//user calibration pass
	}else{
		info[0] = 'f';//user calibration fail
	}
	if(isYasUserCalibration){
		info[1] = 'c';//user calibration is going
	}else{
		info[1] = 'd';//user calibration have done
	}
	info[2] = accuracy_m;// user calibration level value
	info[3] = 't';// whether have user calibration :true

	return COMPONENT_OK;
}
