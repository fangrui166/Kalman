#include "stdio.h"
#include "cmsis_os.h"
#include "hlog_api.h"
#include "Stm32f4xx_hal.h"

typedef struct RTC_driver{
	RTC_HandleTypeDef *rtcHandle;
	osThreadId intr_handler;
	BaseType_t intr_handler_token;
	osMessageQId rtcMsgQueueHandle;
	//RTC_TimeTypeDef sTime;
	//RTC_DateTypeDef sDate;
}rtc_driver;

typedef enum
{
    RTCR_OK = 0, RTCR_FAIL, RTCR_FAIL_PARM, RTCR_FAIL_TX, RTCR_FAIL_I2C, RTCR_FAIL_INIT
} RTC_RET;


#define TIME_DATE_ENABLE		0x32F2


#define RTC_HLOG_ENABLE 1

#if RTC_HLOG_ENABLE
#define rtc_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_RTC, fmt, ##__VA_ARGS__)
#define rtc_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_RTC, fmt, ##__VA_ARGS__)
#define rtc_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_RTC, fmt, ##__VA_ARGS__)
#define rtc_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_RTC, fmt, ##__VA_ARGS__)
#define rtc_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_RTC, fmt, ##__VA_ARGS__)
#else /* RTC_HLOG_ENABLE */
#define rtc_emerg(fmt, ...) \
	printf("[RTC][EMR] :" fmt, ##__VA_ARGS__)
#define rtc_err(fmt, ...) \
	printf("[RTC][ERR] :" fmt, ##__VA_ARGS__)
#define rtc_warning(fmt, ...) \
	printf("[RTC][WARN]:" fmt, ##__VA_ARGS__)
#define rtc_info(fmt, ...) \
	printf("[RTC][INFO]:" fmt, ##__VA_ARGS__)
#define rtc_debug(fmt, ...) \
	printf("[RTC][DBG] :" fmt, ##__VA_ARGS__)
#endif /* RTC_HLOG_ENABLE */

void rtc_gettime(void);
void rtc_drv_init(RTC_HandleTypeDef *);
void rtc_drv_setAlarmA(uint8_t , uint8_t , uint8_t );
void rtc_drv_setAlarmB(uint8_t , uint8_t , uint8_t );
RTC_RET rtc_drv_setDate (uint8_t , uint8_t , uint8_t);
RTC_RET rtc_drv_setTime (uint8_t , uint8_t , uint8_t );
RTC_RET rtc_drv_getTime (uint8_t* , uint8_t* , uint8_t*);
RTC_RET rtc_drv_getDate (uint8_t* , uint8_t* , uint8_t* );
HAL_StatusTypeDef mcu_rtc_wakeUp_alarm_set_time(uint32_t);
HAL_StatusTypeDef mcu_rtc_get_alarm(RTC_AlarmTypeDef* , uint32_t);
HAL_StatusTypeDef mcu_rtc_alarm_disable(uint32_t Alarm);
uint32_t mcu_rtc_wakeUp_alarm_disable(void);







