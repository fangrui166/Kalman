#include "stdio.h"
#include "rtc_drv.h"
#include "cmsis_os.h"
#include <time.h>
#include <stdlib.h>
#include "Stm32f4xx_hal.h"
#include "PowerManager_power.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_system.h"


//static int is_timezone_hour = 8;
//static int is_timezone_minute = 0;

rtc_driver internal_rtc = { 0 };
//extern RTC_HandleTypeDef internal_rtc.rtcHandle;

static const unsigned char rtc_days_in_month[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};


static  unsigned long int rtc_is_leap_year(uint32_t year)
{
	return (!(year % 4) && (year % 100)) || !(year % 400);
}


unsigned long int rtc_month_days(uint32_t month, uint32_t year)
{
	return rtc_days_in_month[month] + (rtc_is_leap_year(year) && month == 1);
}


int rtc_valid_tm(uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	if (_hour >= 24 || _min >= 60 || _sec >= 60)
		return -1;
	return 0;
}


int rtc_valid_dt(uint8_t _year, uint8_t _mon, uint8_t _day)
{
	if (_year < 70 || _mon >= 12 || _day < 1
		|| _day > rtc_month_days(_mon, _year + 2000))
		return -1;

	return 0;
}

HAL_StatusTypeDef mcu_rtc_get_time(RTC_TimeTypeDef* RTC_TimeStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_RTC_GetTime(internal_rtc.rtcHandle,RTC_TimeStruct, FORMAT_BIN);

	return ret;
}

HAL_StatusTypeDef mcu_rtc_get_date(RTC_DateTypeDef* RTC_DateStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_RTC_GetDate(internal_rtc.rtcHandle,RTC_DateStruct, FORMAT_BIN);

	return ret;

}

HAL_StatusTypeDef mcu_rtc_set_time(RTC_TimeTypeDef* RTC_TimeStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;

	if( (ret = HAL_RTC_SetTime(internal_rtc.rtcHandle, RTC_TimeStruct,FORMAT_BIN))!= HAL_OK)
	{
		rtc_err("HAL_RTC_SetTime  fail[%d] \r\n", ret);
	}

	return ret;
}

HAL_StatusTypeDef mcu_rtc_set_date(RTC_DateTypeDef* RTC_DateStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;

	if( (ret = HAL_RTC_SetDate(internal_rtc.rtcHandle, RTC_DateStruct,FORMAT_BIN))!= HAL_OK)
	{
		rtc_err("mcu_rtc_set_date  fail[%d] \r\n", ret);
	}

	return ret;
}

HAL_StatusTypeDef mcu_rtc_set_alarm(RTC_AlarmTypeDef* RTC_AlarmStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;
	RTC_AlarmStruct->Alarm = RTC_ALARM_A;
	ret = HAL_RTC_SetAlarm_IT(internal_rtc.rtcHandle, RTC_AlarmStruct, FORMAT_BIN);
	rtc_debug("RTC set Alarm A %02d:%02d:%02d\n",
		RTC_AlarmStruct->AlarmTime.Hours, RTC_AlarmStruct->AlarmTime.Minutes, RTC_AlarmStruct->AlarmTime.Seconds);

	return ret;
}

HAL_StatusTypeDef mcu_rtc_set_alarmB(RTC_AlarmTypeDef* RTC_AlarmStruct)
{
	HAL_StatusTypeDef ret = HAL_OK;
	RTC_AlarmStruct->Alarm = RTC_ALARM_B;
	ret = HAL_RTC_SetAlarm_IT(internal_rtc.rtcHandle, RTC_AlarmStruct, FORMAT_BIN);

	rtc_debug("RTC set Alarm B %02d:%02d:%02d\n",
		RTC_AlarmStruct->AlarmTime.Hours, RTC_AlarmStruct->AlarmTime.Minutes, RTC_AlarmStruct->AlarmTime.Seconds);

	return ret;
}


HAL_StatusTypeDef mcu_rtc_alarm_disable(uint32_t Alarm)
{
	HAL_StatusTypeDef ret = HAL_OK;
	ret = HAL_RTC_DeactivateAlarm(internal_rtc.rtcHandle, Alarm);
	return ret;
}


HAL_StatusTypeDef mcu_rtc_get_alarm(RTC_AlarmTypeDef* RTC_AlarmStruct,uint32_t Alarm)
{
	HAL_StatusTypeDef ret = HAL_OK;
	ret = HAL_RTC_GetAlarm(internal_rtc.rtcHandle, RTC_AlarmStruct,Alarm, FORMAT_BIN);
	return ret;
}



uint32_t mcu_rtc_wakeUp_alarm_get_time(void)
{
	uint32_t ret = 0;

	ret = HAL_RTCEx_GetWakeUpTimer(internal_rtc.rtcHandle);

	return ret;
}


HAL_StatusTypeDef mcu_rtc_wakeUp_alarm_set_time(uint32_t WakeUpCounter)
{
	HAL_StatusTypeDef ret = HAL_OK;
	HAL_RTCEx_DeactivateWakeUpTimer(internal_rtc.rtcHandle);

	ret = HAL_RTCEx_SetWakeUpTimer_IT(internal_rtc.rtcHandle,WakeUpCounter, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	rtc_debug("mcu_rtc_wakeUp_alarm_set_time  WakeUpCounter = %d \t\n",WakeUpCounter);

	return ret;

}

uint32_t mcu_rtc_wakeUp_alarm_disable(void)
{
	uint32_t ret = HAL_OK;

	ret = HAL_RTCEx_DeactivateWakeUpTimer(internal_rtc.rtcHandle);
	if (ret != HAL_OK)
		rtc_err("mcu_rtc_wakeUp_alarm_disable ERROR!\n");

	return ret;
}


void mcu_rtc_init_default_dateAndTime(void)
{
        RTC_DateTypeDef rtc_DefaultDate;
        RTC_TimeTypeDef rtc_DefaultTime;

        /* Set Date: Friday June 24th 2016 */
        rtc_DefaultDate.Year = 0x10;
        rtc_DefaultDate.Month = RTC_MONTH_JUNE;
        rtc_DefaultDate.Date = 0x18;
        rtc_DefaultDate.WeekDay = RTC_WEEKDAY_FRIDAY;

        mcu_rtc_set_date(&rtc_DefaultDate);

        /* Set Time: 00:00:00 */
        rtc_DefaultTime.Hours = 0x00;
        rtc_DefaultTime.Minutes = 0x00;
        rtc_DefaultTime.Seconds = 0x00;
        rtc_DefaultTime.TimeFormat = RTC_HOURFORMAT12_AM;
        rtc_DefaultTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
        rtc_DefaultTime.StoreOperation = RTC_STOREOPERATION_RESET;

        mcu_rtc_set_time(&rtc_DefaultTime);

}


RTC_RET rtc_drv_getDate (uint8_t* _year, uint8_t* _mon, uint8_t* _day)
{
	RTC_RET ret = RTCR_OK;
	HAL_StatusTypeDef hal_ret = HAL_OK;
	RTC_DateTypeDef mMcuDate;

	hal_ret = mcu_rtc_get_date(&mMcuDate);
	if(hal_ret != HAL_OK)
		return RTCR_FAIL_PARM;

	*_year = mMcuDate.Year;
	*_mon  = mMcuDate.Month;
	*_day  = mMcuDate.Date;

	return ret;
}


RTC_RET rtc_drv_getTime (uint8_t* _hour, uint8_t* _min, uint8_t* _sec)
{
	RTC_RET ret = RTCR_OK;
	HAL_StatusTypeDef hal_ret = HAL_OK;
	RTC_TimeTypeDef mMcuTime = { 0 };
	RTC_DateTypeDef mMcuDate = { 0 };

	hal_ret = mcu_rtc_get_time(&mMcuTime);
	if(hal_ret != HAL_OK)
		return RTCR_FAIL_PARM;

	hal_ret = mcu_rtc_get_date(&mMcuDate);
	if(hal_ret != HAL_OK)
		return RTCR_FAIL_PARM;

	*_hour =  mMcuTime.Hours;
	*_min  =  mMcuTime.Minutes;
	*_sec  =  mMcuTime.Seconds;

	return ret;
}

RTC_RET rtc_drv_setTime (uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	RTC_RET ret = RTCR_OK;
	HAL_StatusTypeDef hal_ret = HAL_OK;
	RTC_TimeTypeDef rtc_DefaultTime;

	_hour = (_hour % 24);
	_min = (_min % 60);
	_sec = (_sec % 60);

	rtc_DefaultTime.Hours = _hour;
	rtc_DefaultTime.Minutes = _min;
	rtc_DefaultTime.Seconds = _sec;
	//rtc_DefaultTime.TimeFormat = RTC_HOURFORMAT12_AM;
	rtc_DefaultTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	rtc_DefaultTime.StoreOperation = RTC_STOREOPERATION_RESET;

	hal_ret = mcu_rtc_set_time(&rtc_DefaultTime);
	if(hal_ret != HAL_OK)
		return RTCR_FAIL_PARM;

	return ret;
}


RTC_RET rtc_drv_setDate (uint8_t _year, uint8_t _mon, uint8_t _day)
{
	RTC_RET ret = RTCR_OK;
	HAL_StatusTypeDef hal_ret = HAL_OK;
	RTC_DateTypeDef rtc_DefaultDate = { 0 };

	_year = (_year % 100);
	_mon = (_mon % 13);
	_day = (_day % 32);

	rtc_DefaultDate.Year = _year;
	rtc_DefaultDate.Month = _mon;
	rtc_DefaultDate.Date = _day;
	rtc_DefaultDate.WeekDay = 1;

	hal_ret = mcu_rtc_set_date(&rtc_DefaultDate);
	if(hal_ret != HAL_OK)
		return RTCR_FAIL_PARM;

	return ret;
}

void rtc_drv_setAlarmA(uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	RTC_AlarmTypeDef mRTC_Alarm = { 0 };

	if(rtc_valid_tm(_hour, _min, _sec) != 0)
	{
            rtc_err("%s: set Alarm A args ERROR!\n", __func__);
            return;
	}
	mRTC_Alarm.Alarm = RTC_ALARM_A;
	mRTC_Alarm.AlarmDateWeekDay = 1;
	mRTC_Alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	mRTC_Alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	mRTC_Alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	mRTC_Alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	mRTC_Alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	mRTC_Alarm.AlarmTime.Hours = _hour;
	mRTC_Alarm.AlarmTime.Minutes = _min;
	mRTC_Alarm.AlarmTime.Seconds = _sec;
	mRTC_Alarm.AlarmTime.SubSeconds = 0;

	mcu_rtc_set_alarm(&mRTC_Alarm);
}

void rtc_drv_setAlarmB(uint8_t _hour, uint8_t _min, uint8_t _sec)
{
	RTC_AlarmTypeDef mRTC_Alarm = { 0 };

	if(rtc_valid_tm(_hour, _min, _sec) != 0)
	{
            rtc_err("%s:set Alarm B args ERROR!\n", __func__);
            return;
	}
	mRTC_Alarm.Alarm = RTC_ALARM_B;
	mRTC_Alarm.AlarmDateWeekDay = 1;
	mRTC_Alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	mRTC_Alarm.AlarmMask = RTC_ALARMMASK_NONE;
	mRTC_Alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	mRTC_Alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	mRTC_Alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	mRTC_Alarm.AlarmTime.Hours = _hour;
	mRTC_Alarm.AlarmTime.Minutes = _min;
	mRTC_Alarm.AlarmTime.Seconds = _sec;
	mRTC_Alarm.AlarmTime.SubSeconds = 0;

	mcu_rtc_set_alarmB(&mRTC_Alarm);
}


void rtc_gettime(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(internal_rtc.rtcHandle, &sTime, 0);
	HAL_RTC_GetDate(internal_rtc.rtcHandle, &sDate, 0);
	rtc_info("current time is %d:%d:%d.%d\n", sTime.Hours, sTime.Minutes, sTime.Seconds, sTime.SubSeconds);
	rtc_info("current time is %d/%d/%d\n", sDate.Year, sDate.Month, sDate.Date);

}

extern int PRMGR_battery_update_uiSoc();
static void rtc_intr_work(void const * argument)
{
	//uint32_t ulNotificationValue;
	uint32_t intr_alarm;
	do {
		//xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);
		xQueueReceive( internal_rtc.rtcMsgQueueHandle, &intr_alarm, portMAX_DELAY );
		internal_rtc.intr_handler_token = pdFALSE;

		if ((intr_alarm & RTC_ALARM_A) == RTC_ALARM_A) {
			rtc_info("%s: alarm A interrupt occured.\n", __func__);
			PRMGR_battery_update_uiSoc();

			if(!(wakeup_source & (~WAKEUP_SOURCE_RTC_ALARMA))){
				enum __pwrmgr_stat_machine_mode mode = PWRMGR_STAT_MACHINE_STOP_MODE;
				while(mode == PWRMGR_STAT_MACHINE_STOP_MODE){
					pwrmgr_state_machine_get(&mode);
				}
				pwrmgr_system_send_message(PM_SYS_CMD_RTC_UPDATE_SOC, 0);
			}
		}
		if ((intr_alarm & RTC_ALARM_B) == RTC_ALARM_B) {
			rtc_info("%s: alarm B interrupt occured.\n", __func__);
		}
	} while(1);
}

static void rtc_intr_handler(uint32_t Alarm)
{
	if (internal_rtc.intr_handler == NULL)
		return;

	internal_rtc.intr_handler_token = pdFALSE;
	if (internal_rtc.intr_handler != NULL)
		 xQueueSendFromISR(internal_rtc.rtcMsgQueueHandle, &Alarm, &internal_rtc.intr_handler_token);
		//xTaskNotifyFromISR(internal_rtc.intr_handler, Alarm, eSetBits, &internal_rtc.intr_handler_token);

	portYIELD_FROM_ISR(internal_rtc.intr_handler_token);
}

static int rtc_creat_intr_thread()
{
	internal_rtc.intr_handler_token = pdFALSE;
	osThreadDef(rtc_intr, rtc_intr_work, osPriorityNormal, 0,
						configMINIMAL_STACK_SIZE * 2);
	internal_rtc.intr_handler = osThreadCreate(osThread(rtc_intr), &internal_rtc);
	if (internal_rtc.intr_handler == NULL)
		return -1;

	return 0;
}

extern void PWRMGR_SYSTEM_RTC_WAKEUP(void);

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	PWRMGR_SYSTEM_RTC_WAKEUP();
	rtc_debug("Wake Up Timer IRQ generated, Enter %s\n", __func__);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	if(wakeup_source  == 0){
		wakeup_source |= WAKEUP_SOURCE_RTC_ALARMA;
	}
	rtc_intr_handler(RTC_ALARM_A);
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_intr_handler(RTC_ALARM_B);
}

void rtc_drv_init(RTC_HandleTypeDef *hrtc)
{
	int ret;
	internal_rtc.rtcHandle = hrtc;

	osMessageQDef(rtc_intr_queue, 4, uint32_t);
	internal_rtc.rtcMsgQueueHandle = osMessageCreate(osMessageQ(rtc_intr_queue), NULL);

	ret = rtc_creat_intr_thread();
	if (ret != 0) {
		rtc_err("%s: rtc driver creat thread ERROR.\n", __func__);
		return;
	}

	if(HAL_RTCEx_BKUPRead(internal_rtc.rtcHandle, RTC_BKP_DR0) != TIME_DATE_ENABLE)
	{
		mcu_rtc_init_default_dateAndTime();
		HAL_RTCEx_BKUPWrite(internal_rtc.rtcHandle, RTC_BKP_DR0, TIME_DATE_ENABLE);
		rtc_debug("RTC set init date and time\n");
	}
	rtc_debug("RTC initial DONE!\n");
}

