/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLLER_ADC_FUNC_H__
#define __CONTROLLER_ADC_FUNC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "timers.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "hlog_api.h"

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

#define ADC_VALUE_TO_VOLT(ADC_VAL) 	((((float)ADC_VAL * 3300) / 4096) / 1000)
#define ADC_VALUE_TO_RT(ADC_VAL)	((float)((100 * ADC_VAL) / (4096 - ADC_VAL)))

#define ADC_HLOG_ENABLE		1

#ifdef ADC_HLOG_ENABLE
#define adc_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_ADC, fmt, ##__VA_ARGS__)
#define adc_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_ADC, fmt, ##__VA_ARGS__)
#define adc_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_ADC, fmt, ##__VA_ARGS__)
#define adc_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_ADC, fmt, ##__VA_ARGS__)
#define adc_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_ADC, fmt, ##__VA_ARGS__)
#else /* ADC_HLOG_ENABLE */
#define adc_emerg(fmt, ...) \
	printf("[ADC][EMR] :" fmt, ##__VA_ARGS__)
#define adc_err(fmt, ...) \
	printf("[ADC][ERR] :" fmt, ##__VA_ARGS__)
#define adc_warning(fmt, ...) \
	printf("[ADC][WARN]:" fmt, ##__VA_ARGS__)
#define adc_info(fmt, ...) \
	printf("[ADC][INFO]:" fmt, ##__VA_ARGS__)
#define adc_debug(fmt, ...) \
	printf("[ADC][DBG] :" fmt, ##__VA_ARGS__)
#endif /* ADC_HLOG_ENABLE */

int htc_get_batt_average_voltage(float *);
int htc_get_usb_average_temperature(float *);
int htc_adc1_func_initial(ADC_HandleTypeDef *);

#ifdef __cplusplus
}
#endif

#endif /* __HMD_ADC_FUNC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
