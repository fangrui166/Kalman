#include "FreeRTOS.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "Stm32f4xx_hal_adc.h"
#include "cmsis_os.h"
#include "htc_adc_func.h"

#define AVE_NUM 	5

struct adcx_func_data {
	ADC_HandleTypeDef *adc_handle;
	//osSemaphoreDef_t adc_Semaphore;
	//osSemaphoreId adc_Semaphore_id;
	osMessageQId adc_queue_id;
	//uint8_t data_sync_flag;
	__IO uint32_t convert_value[2];
};


static struct adcx_func_data adc1_data = { 0 };

static float ln(float a)
{
	int N = 15;
	int k, nk;
	float x, xx, y;

	x = (a - 1) / (a + 1);
	xx = x*x;
	nk = 2 * N + 1;
	y = 1.0 / nk;

	for (k = N; k > 0; k--)
	{
		nk = nk - 2;
		y = 1.0 / nk + xx * y;
	}

	return 2.0 * x * y;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//static BaseType_t xHigherPriorityTaskWoken = pdFALSE;	
        if (HAL_ADC_Stop_DMA(adc1_data.adc_handle) != HAL_OK) {
               return;
       }
	//adc_info("DMA_ADC convert complete, convert_value[0] = %d, convert_value[1] = %d\n",
		//adc1_data.convert_value[0], adc1_data.convert_value[1]);
	osMessagePut( adc1_data.adc_queue_id, 0, osWaitForever);
	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static int htc_get_adc1_value(void)/*uint32_t *data*/
{
        if (HAL_ADC_Start_DMA(adc1_data.adc_handle,
			(uint32_t *)&adc1_data.convert_value[0], 2) != HAL_OK) {
                adc_err("%s: ADC start failed\n", __func__);
                return -1;
        }

	return 0;
}

static int htc_get_batt_volt(float *voltage)
{
	osEvent event;
	if (htc_get_adc1_value() != 0)
		return -1;

	while (1) {
		event = osMessageGet( adc1_data.adc_queue_id, osWaitForever );
		if (event.status == osEventMessage) {
			*voltage = (float)ADC_VALUE_TO_VOLT(adc1_data.convert_value[0]);
			//adc_info("%s: Current batt id pin Voltage = %.3f\n", __func__, *voltage);
			return 0;
		}
	}
}

static int htc_get_usb_temp(float * temperature)
{
	float rt;
	osEvent event;
	if (htc_get_adc1_value() != 0)
		return -1;

	while (1) {
		event = osMessageGet( adc1_data.adc_queue_id, osWaitForever );
		if (event.status == osEventMessage) {
			rt = ADC_VALUE_TO_RT(adc1_data.convert_value[1]);
			*temperature = (1 / ((ln(rt / 100) / 4250) + (1 / 298.5)) - 273.5);
			//adc_info("%s: Current USB temperature = %.3f\n", __func__, *temperature);
			return 0;
		}
	}
}

int htc_get_batt_average_voltage(float *volt)
{
	int ret = 0, i = 0;
	float voltage[AVE_NUM] = {0};
	float max_voltage = 0, min_voltage = 0;
	*volt = 0;
	for (i = 0; i < AVE_NUM; i++)
	{
		ret = htc_get_batt_volt(&voltage[i]);
		if (ret != 0) {
			adc_err("Controller get voltage ERROR!\n");
			return ret;
		}
		else
			*volt += voltage[i];
	}
	max_voltage = voltage[0];
	min_voltage = voltage[0];
	for (i = 1; i < AVE_NUM; i++)
	{
		if (max_voltage < voltage[i])
			max_voltage = voltage[i];
		if (min_voltage > voltage[i])
			min_voltage = voltage[i];
	}
	*volt = (*volt - max_voltage - min_voltage) / (AVE_NUM -2);

	return 0;
}

int htc_get_usb_average_temperature(float *temp)
{
	int ret = 0, i = 0;
	float temperature[AVE_NUM] = {0};
	float max_temp = 0, min_temp = 0;

	*temp = 0;
	for (i = 0; i < AVE_NUM; i++) {
		ret = htc_get_usb_temp(&temperature[i]);
		if (ret != 0) {
			adc_err("%s: get temperature ERROR!\n", __func__);
			return ret;
		} else
			*temp += temperature[i];
	}

	max_temp = temperature[0];
	min_temp = temperature[0];
	for (i = 1; i < AVE_NUM; i++) {
		if (max_temp < temperature[i])
			max_temp = temperature[i];
		if (min_temp > temperature[i])
			min_temp = temperature[i];
	}
	*temp = (*temp - max_temp - min_temp) / (AVE_NUM - 2);
	
	return 0;
}

int htc_adc1_func_initial(ADC_HandleTypeDef *adc_handle)
{
	adc1_data.adc_handle = adc_handle;
	osMessageQDef(adc_messageQ, 1, int);
	adc1_data.adc_queue_id = osMessageCreate(osMessageQ(adc_messageQ), NULL);
	if (adc1_data.adc_queue_id == NULL) {
		adc_err("create adc messageQ failed\n");
		return -1;
	}
	adc_info("%s: initial done\n", __func__);
	return 0;
}

