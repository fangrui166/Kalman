/**
 *******************************************************************************
 * @file    EPL88051_PROXIMITY_driver_HL.c
 * @author  hTC BSP Team
 * @version V1.0.0
 * @date    10-November-2016
 * @brief   This file provides a set of high-level functions needed to manage
 the ePL88051 proximity sensor
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
#include "epl88051_driver.h"
#include "rtos_i2c_drv.h"
#include "misc_data.h"
#include "proximity.h"

typedef struct
{
	int ps_initialized;
	int ps_enabled;
	uint16_t ps_threshold_high;
	uint16_t ps_threshold_low;
	int ps_suspend;
	uint16_t ps_flash_kadc_low;
	uint16_t ps_flash_kadc_high;
	uint8_t ps_proximity;
	uint8_t ps_sens_integration;
	uint8_t ps_sens_adc;
	uint8_t ps_sens_filter;
}epl88051_driver;

static epl88051_driver epld = {0};

/**
 * @brief  EPL88051 proximity sensor I2C write function
 * @param addr the register of proximity sensor
 * @buf the buffer written to proximity sensor register
 * @len the length of byte will be written
 * @retval PS_OK in case of success or failure
 */
static PSDrvRet epl88051_device_write( uint8_t addr, uint8_t *buf, int len)
{
	I2C_STATUS ret = I2C_OK;
	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_PSENSOR_ADDR, addr, I2C_MEMADD_SIZE_8BIT, buf, len, PROXIMITY_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ){
		prox_err("%s I2C write error ret=%d\n",__func__,ret);
		return PS_ERROR_COMMUNICATION;
	}
	else
		return PS_OK;
}

/**
 * @brief  EPL88051 proximity sensor I2C read function
 * @param addr the register of proximity sensor
 * @buf the buffer pointer to store data read from proximity sensor
 * @len the length of byte will be read
 * @retval PS_OK in case of success or failure
 */
static PSDrvRet epl88051_device_read(uint8_t addr, uint8_t *buf, int len)
{
	I2C_STATUS ret = I2C_OK;
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_PSENSOR_ADDR, addr, I2C_MEMADD_SIZE_8BIT, buf, len, PROXIMITY_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ){
		prox_err("%s I2C write error ret=%d\n",__func__,ret);
		return PS_ERROR_COMMUNICATION;
	}
	else
		return PS_OK;
}

/**
 * @brief setup EPL88051 proximity sensor register in initialize
 * @retval PS_OK in case of success or failures
 */
static PSDrvRet epl88051_ps_setup()
{
	uint8_t tmp;
	uint8_t threshold_value[2];
	tmp = EPL_RESET | EPL_ACTIVE;
	if (epl88051_device_write( REG_17, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor sleep mode fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	tmp = EPL_PS_MODE |EPL_WTIME_DISABLE;
	if (epl88051_device_write( REG_0, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor mode fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	tmp = EPL_PS_L_GAIN | EPL_PS_INTEG_384;
	if (epl88051_device_write( REG_3, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor gain and integration time fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	epld.ps_sens_integration = (tmp & (0x0F<<2))>>2;

	tmp = EPL_PS_FILTER_16ORDER;
	if (epl88051_device_write( REG_4, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor filter order fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	epld.ps_sens_adc = (tmp & (0x03<<3))>>3;
	epld.ps_sens_filter = tmp & 0x07;

	tmp = EPL_PS_IR_DRIVE_100MA | EPL_PS_IR_MODE_CURRENT |EPL_PS_IR_LED_ON;
	if (epl88051_device_write( REG_5, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor driver mode and strength fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	tmp = EPL_INT_ACTIVE_LOW | EPL_PS_PERSIST_1FRAME|EPL_PS_INT_CTRL;
	if (epl88051_device_write( REG_6, &tmp, EPL_SINGLE_BYTE)){
		prox_err("%s setup proximity sensor interrupt configure fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	threshold_value[0] = (epld.ps_threshold_low) & 0xFF;
	threshold_value[1] = (epld.ps_threshold_low >>8) & 0xFF;
	if (epl88051_device_write( REG_PS_LOW_THRESH_LSB,threshold_value, EPL_DOUBLE_BYTE)){
		prox_err("%s setup proximity sensor low threshold register fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	threshold_value[0] = (epld.ps_threshold_high) & 0xFF;
	threshold_value[1] = (epld.ps_threshold_high >>8) & 0xFF;
	if (epl88051_device_write( REG_PS_HIGH_THRESH_LSB,threshold_value, EPL_DOUBLE_BYTE)){
		prox_err("%s setup proximity sensor high threshold register fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	tmp = EPL_PS_UNLOCK |EPL_PS_CMP_RSTN;
	if (epl88051_device_write(REG_27, &tmp,EPL_SINGLE_BYTE)){
		prox_err("%s unlock epl88051 proximity sensor data register and reset Comparator fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	tmp = EPL_PS_UNLOCK |EPL_PS_CMP_NORMAL;
	if (epl88051_device_write(REG_27, &tmp,EPL_SINGLE_BYTE)){
		prox_err("%s unlock epl88051 proximity sensor data register and reset Comparator fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}

	return PS_OK;
}
/**
 * @brief initialize EPL88051 proximity sensor
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_init()
{
	uint8_t revisionNumber;
	uint16_t ps_low_threshold = 0,ps_high_threshold = 0;
	if(epld.ps_initialized)
	{
		prox_err("epl88051 proximity sensor have initialized\n");
		return PS_ERROR;
	}
	if (epl88051_device_read(REG_REVNO, &revisionNumber, EPL_SINGLE_BYTE)){
		prox_err("%s read proximity sensor revision number fail!\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	if (revisionNumber != EPL88051_REVNO){
		prox_err("%s check proximity revision number error!\n",__func__);
		return PS_ERROR;
	}
#if 1
	if(get_ps_threshold(&ps_low_threshold, &ps_high_threshold) < 0){
		prox_err("epl88051 proximity sensor get threshold form flash fail\n");
		return PS_ERROR;
	}

	epld.ps_flash_kadc_low = ps_low_threshold;
	epld.ps_flash_kadc_high = ps_high_threshold;
	if (epld.ps_flash_kadc_low != 0 && epld.ps_flash_kadc_high != 0 && epld.ps_flash_kadc_low != 0xFFFF && epld.ps_flash_kadc_high != 0xFFFF){
		epld.ps_threshold_low = epld.ps_flash_kadc_low;
		epld.ps_threshold_high = epld.ps_flash_kadc_high;
	}
	else {
		epld.ps_threshold_high = EPL_HIGH_THRESH;
		epld.ps_threshold_low = EPL_LOW_THRESH;
	}
#else
	epld.ps_threshold_high = EPL_HIGH_THRESH;
	epld.ps_threshold_low = EPL_LOW_THRESH;
#endif

	if (epl88051_ps_setup() < 0){
		prox_err("proximity sensor setup error!\n");
		return PS_ERROR;
	}
	epld.ps_initialized = 1;
	return PS_OK;
}

/**
 * @brief deinitialize EPL88051 proximity sensor
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_deinit()
{
	if(epld.ps_initialized == 0)
	{
		prox_err("epl88051 proximity sensor have deinitialized\n");
		return PS_ERROR;
	}
	if (epl88051_ps_enable(0) < 0){
		prox_err("%s Disable proximity sensor fail\n",__func__);
		return PS_ERROR;
	}
	epld.ps_initialized = 0;
	return PS_OK;
}

/**
 * @brief enable or disable EPL88051 proximity sensor
 * @param epld the EPL88051 proximity sensor driver handle
 * @enable enable proximity sensor if 1 ,or disable proximity sensor
 * @retval PS_OK in case of success
 * @retval PS_ERROR in case of failure
 */
PSDrvRet epl88051_ps_enable(uint8_t enable)
{
	uint8_t tmp;
	if(epld.ps_enabled == enable)
	{
		prox_err("epl88051 proximity sensor have %s it\n",enable?"enable":"disable");
		return PS_OK;
	}
	if(enable){
		tmp = EPL_ACTIVE | EPL_START;
		if (epl88051_device_write( REG_17, &tmp, EPL_SINGLE_BYTE)){
			prox_err("%s setup proximity sensor active mode fail\n",__func__);
			return PS_ERROR_COMMUNICATION;
		}
	}
	else{
		tmp = EPL_SLEEP | EPL_START;
		if (epl88051_device_write( REG_17, &tmp, EPL_SINGLE_BYTE)){
			prox_err("%s setup proximity sensor sleep mode fail\n",__func__);
			return PS_ERROR_COMMUNICATION;
		}
	}
	epld.ps_enabled = enable;
	return PS_OK;
}

/**
 * @brief get EPL88051 proximity sensor revision number
 * @revision_number that get from register
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_get_revno(uint8_t *revision_number )
{
	if (epl88051_device_read(REG_REVNO, revision_number,EPL_SINGLE_BYTE )){
		prox_err("%s get epl88051 proximity sensor revision number fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	return PS_OK;
}

/**
 * @brief get EPL88051 proximity sensor proximity
 * @ps_proximity pointer of proximity value get from register
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_proximity(uint8_t *ps_proximity )
{
	uint8_t ps_state;
	if (epl88051_device_read(REG_27, &ps_state,EPL_SINGLE_BYTE)){
		prox_err("%s get epl88051 proximity sensor interrupt state register fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	/*if *ps_proximity = 1 proximity colse, else proximity far*/
	if (ps_state & EPL_PS_INT_MASK){
		if (ps_state & EPL_PS_CMP_H_MASK){
			epld.ps_proximity = 1;
		}
		else{
			epld.ps_proximity = 0;
		}
		*ps_proximity = epld.ps_proximity;
	}

	ps_state = EPL_PS_UNLOCK |EPL_PS_CMP_NORMAL;
	if (epl88051_device_write(REG_27, &ps_state,EPL_SINGLE_BYTE)){
		prox_err("%s unlock epl88051 proximity sensor data register and run fail\n",__func__);
		return PS_ERROR_COMMUNICATION;
	}
	return PS_OK;
}
#if 1
/**
 * @brief get EPL88051 proximity sensor sensing time in milliseconds
 * @delay_time pointer of delay time
 * @retval PS_OK in case of success or failure
 */
static PSDrvRet epl88051_ps_sensing_time(uint32_t *delay_time)
{
	int64_t sensing_us_time;
	int sensing_ms_time;
	int ps_integration,ps_adc,ps_filter;

	ps_integration =ps_integration_value[epld.ps_sens_integration];
	ps_adc = ps_adc_value[epld.ps_sens_adc];
	ps_filter = ps_filter_value[epld.ps_sens_filter];

	sensing_us_time = (ps_integration * 3 + ps_adc *2*3) * ps_filter;
	sensing_ms_time = sensing_us_time /1000;
	*delay_time = sensing_ms_time +5;
	return PS_OK;
}

/**
 * @brief get EPL88051 proximity sensor adc value
 * @adc_value pointer of adc value get from register
 * @retval PS_OK in case of success
 * @retval PS_ERROR in case of failure
 */
PSDrvRet epl88051_ps_adc_value(uint16_t *adc_value ){
	uint8_t ps_data[2],tmp;
	uint32_t sample_time;

	/*unlock data and interrupt register for mfg command*/
	tmp = EPL_PS_UNLOCK |EPL_PS_CMP_NORMAL;
	if (epl88051_device_write(REG_27, &tmp,EPL_SINGLE_BYTE)){
		prox_err("%s unlock epl88051 proximity sensor data register fail\n",__func__);
		return PS_ERROR;
	}

	epl88051_ps_sensing_time(&sample_time);
	HAL_Delay(sample_time);//delay for sampling data

	if (epl88051_device_read(REG_PS_DATA_LSB, ps_data,EPL_DOUBLE_BYTE )){
		prox_err("%s get epl88051 proximity sensor data fail\n",__func__);
		return PS_ERROR;
	}
	*adc_value = ((ps_data[1])<<8)|(ps_data[0]);

	return PS_OK;
}

/**
 * @brief get EPL88051 proximity sensor proximity for command
 * @ps_proximity pointer of proximity value get from register
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_cmd_ps_proximity(uint8_t *ps_proximity ){
	uint8_t ps_state;
	uint8_t tmp;
	uint32_t sample_time;

	tmp = EPL_PS_UNLOCK |EPL_PS_CMP_NORMAL;
	if (epl88051_device_write(REG_27, &tmp,EPL_SINGLE_BYTE)){
		prox_err("%s unlock epl88051 proximity sensor data register and run fail\n",__func__);
		return PS_ERROR;
	}
	epl88051_ps_sensing_time(&sample_time);
	HAL_Delay(sample_time);//delay for sampling data

	if (epl88051_device_read(REG_27, &ps_state,EPL_SINGLE_BYTE)){
		prox_err("%s get epl88051 proximity sensor interrupt state register fail\n",__func__);
		return PS_ERROR;
	}
	/*if *ps_proximity = 1 proximity colse, else proximity far*/
	if (ps_state & EPL_PS_CMP_H_MASK){
		epld.ps_proximity = 1;
	}
	else{
		epld.ps_proximity = 0;
	}
	*ps_proximity = epld.ps_proximity;
	return PS_OK;
}
/**
 * @brief get EPL88051 proximity sensor threshold value
 * @low_threshold pointer of proximity sensor low threshold vale
 * @high_threshold pointer of proximity sensor high threshold vale
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_get_threshold(uint16_t * low_threshold,uint16_t * high_threshold )
{
	uint8_t register_value[2];
	if (epl88051_device_read(REG_PS_LOW_THRESH_LSB, register_value,EPL_DOUBLE_BYTE)){
		prox_err("get epl88051 proximity sensor low threshold value fail\n");
		return PS_ERROR;
	}
	*low_threshold = (register_value[0])|(register_value[1]<<8);
	if (epl88051_device_read(REG_PS_HIGH_THRESH_LSB, register_value,EPL_DOUBLE_BYTE)){
		prox_err("get epl88051 proximity sensor high threshold value fail\n");
		return PS_ERROR;
	}
	*high_threshold = (register_value[0])|(register_value[1]<<8);
	return PS_OK;
}
/**
 * @brief set EPL88051 proximity sensor threshold value
 * @low_threshold pointer of proximity sensor low threshold vale
 * @high_threshold pointer of proximity sensor high threshold vale
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_set_threshold(uint16_t * low_threshold,uint16_t * high_threshold )
{
	uint8_t threshold_value[2];
	if ((low_threshold == NULL)&&(high_threshold == NULL)){
		return PS_ERROR;
	}
	if (low_threshold != NULL){
		threshold_value[0] = (*low_threshold) & 0xFF;
		threshold_value[1] = (*low_threshold >>8) & 0xFF;
		if (epl88051_device_write( REG_PS_LOW_THRESH_LSB,threshold_value, EPL_DOUBLE_BYTE)){
			prox_err("%s setup proximity sensor low threshold register fail\n",__func__);
			return PS_ERROR;
		}
		epld.ps_threshold_low = * low_threshold;
	}
	if (high_threshold != NULL){
		threshold_value[0] = (*high_threshold) & 0xFF;
		threshold_value[1] = (*high_threshold >>8) & 0xFF;
		if (epl88051_device_write( REG_PS_HIGH_THRESH_LSB,threshold_value, EPL_DOUBLE_BYTE)){
			prox_err("%s setup proximity sensor high threshold register fail\n",__func__);
			return PS_ERROR;
		}
		epld.ps_threshold_high = * high_threshold;
	}
	return PS_OK;
}
/**
 * @brief dump proximity sensor register
 * @retval PS_OK in case of success or failure
 */
PSDrvRet epl88051_ps_register_dump( void )
{
	uint8_t register_value;
	int i;
	if (epl88051_device_read(REG_0, &register_value,EPL_SINGLE_BYTE)){
		prox_err("get epl88051 proximity sensor register %#X value fail\n",REG_0);
		return PS_ERROR;
	}
	prox_info("proximity sensor REG_%#x = %#x\n",REG_0,register_value);

	for (i=REG_3;i<=REG_6;i++)
	{
		if (epl88051_device_read(i, &register_value,EPL_SINGLE_BYTE)){
			prox_err("get epl88051 proximity sensor register %#X value fail\n",i);
			return PS_ERROR;
		}
		prox_info("proximity sensor REG_%#x = %#x\n",i,register_value);
	}

	for (i=REG_12;i<=REG_15;i++)
	{
		if (epl88051_device_read(i, &register_value,EPL_SINGLE_BYTE)){
			prox_err("get epl88051 proximity sensor register %#X value fail\n",i);
			return PS_ERROR;
		}
		prox_info("proximity sensor REG_%#x = %#x\n",i,register_value);
	}

	if (epl88051_device_read(REG_17, &register_value,EPL_SINGLE_BYTE)){
		prox_err("get epl88051 proximity sensor register %#X value fail\n",REG_17);
		return PS_ERROR;
	}
	prox_info("proximity sensor REG_%#x = %#x\n",REG_17,register_value);

	for (i=REG_27;i<=REG_31;i++)
	{
		if (epl88051_device_read(i, &register_value,EPL_SINGLE_BYTE)){
			prox_err("get epl88051 proximity sensor register %#X value fail\n",i);
			return PS_ERROR;
		}
		prox_info("proximity sensor REG_%#x = %#x\n",i,register_value);
	}

	for (i=REG_33;i<=REG_35;i++)
	{
		if (epl88051_device_read(i, &register_value,EPL_SINGLE_BYTE)){
			prox_err("get epl88051 proximity sensor register %#X value fail\n",i);
			return PS_ERROR;
		}
		prox_info("proximity sensor REG_%#x = %#x\n",i,register_value);
	}

	return PS_OK;
}

/**
 * @brief proximity sensor open air calibration
 * @xtalk pointer of proximity sensor cross talk value in open air
 * @low_threshold pointer of proximity sensor high threshold vale after calibration
 * @high_threshold pointer of proximity sensor high threshold vale after calibration
 * @retval PS_OK in case of success or failure
 */
PSDrvRet  epl88051_ps_Xtalk_calibration(uint16_t * xtalk,uint16_t * low_threshold,uint16_t * high_threshold )
{
	int i;
	uint16_t adc_value;
	uint32_t adc_sum = 0;
	int ret = PS_OK;
	#define CAL_NUM 100
	uint16_t ps_int_low,ps_int_high;

	for(i=0;i <CAL_NUM;i++){
		ret += epl88051_ps_adc_value(&adc_value);
		adc_sum += adc_value;
	}
	if (ret != PS_OK){
		prox_err("proximity sensor open air calibration read adc error\n");
		return PS_ERROR;
	}
	adc_value = adc_sum/CAL_NUM;
	ps_int_low = adc_value + EPL_LOW_THRESH;
	ps_int_high = adc_value + EPL_HIGH_THRESH;

	if (epl88051_ps_set_threshold(&ps_int_low,&ps_int_high) != PS_OK){
		prox_err("proximity sensor open air calibration set threshold register error\n");
		return PS_ERROR;
	}
	*xtalk = adc_value;
	* low_threshold = ps_int_low;
	* high_threshold = ps_int_high;
	return PS_OK;
}
#endif
