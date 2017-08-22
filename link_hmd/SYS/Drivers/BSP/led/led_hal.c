#include "cmsis_os.h"
#include "hlog_api.h"
#include "led_hal.h"
#include "led_drv.h"
#include "PowerManager_notify_func.h"

uint8_t led_event = 0xff;

#if LED_USE_TASK
osThreadId LedTaskHandle;
#endif
#if LED_USE_QUEUE
osMessageQId LedMsgQueueHandle;
#endif

static struct pwrmgr_notify_func_data PmNotifyData = {0};
static led_event_data led_data[LED_EVENT_MAX]= {
	{ LED_EVENT_ON,  LED_EVENT_NORMAL,          LED_PRIORITY_NORMAL          },
	{ LED_EVENT_OFF, LED_EVENT_CONNECTED,       LED_PRIORITY_CONNECTED       },
	{ LED_EVENT_OFF, LED_EVENT_STAND_BY,        LED_PRIORITY_STAND_BY        },
	{ LED_EVENT_OFF, LED_EVENT_CHG_FULL,        LED_PRIORITY_CHG_FULL        },
	{ LED_EVENT_OFF, LED_EVENT_LOW_BAT,         LED_PRIORITY_LOW_BAT         },
	{ LED_EVENT_OFF, LED_EVENT_CONNECTING,      LED_PRIORITY_CONNECTING      },
	{ LED_EVENT_OFF, LED_EVENT_CONNECTING_FAIL, LED_PRIORITY_CONNECTING_FAIL },
	{ LED_EVENT_OFF, LED_EVENT_CHG_ING,         LED_PRIORITY_CHG_ING         },
	{ LED_EVENT_OFF, LED_EVENT_CHG_FAULT,       LED_PRIORITY_CHG_FAULT       },
	{ LED_EVENT_ON,  LED_EVENT_LOADING,         LED_PRIORITY_LOADING         },
	/* { LED_EVENT_OFF, LED_EVENT_KERNEL_MODE,     LED_PRIORITY_KERNEL_MODE     }, */
	{ LED_EVENT_OFF, LED_EVENT_POWER_OFF,       LED_PRIORITY_POWER_OFF       },
	{ LED_EVENT_OFF, LED_EVENT_ALL_LEDS_OFF,    LED_PRIORITY_ALL_LEDS_OFF    },
	{ LED_EVENT_OFF, LED_EVENT_ESTRESS_TEST,    LED_PRIORITY_ESTRESS_TEST    }
};
const char *led_event_str[] = {
    [LED_EVENT_NORMAL] = "NORMAL",
    [LED_EVENT_CONNECTED] = "CONNECTED",
    [LED_EVENT_STAND_BY] = "STANDBY",
    [LED_EVENT_CHG_FULL] = "CHG_FULL",
    [LED_EVENT_LOW_BAT] = "LOW_BAT",
    [LED_EVENT_CONNECTING] = "CONNECTING",
    [LED_EVENT_CONNECTING_FAIL] = "CONNECTING_FAIL",
    [LED_EVENT_CHG_ING] = "CHG_ING",
    [LED_EVENT_CHG_FAULT] = "CHG_FAULT",
    [LED_EVENT_LOADING] = "LOADING",
    [LED_EVENT_POWER_OFF] = "POWER_OFF",
    [LED_EVENT_ALL_LEDS_OFF] = "ALL_LEDS_OFF",
    [LED_EVENT_ESTRESS_TEST] = "ESTESS_TEST",
};

uint32_t led_get_event_status(LED_EVENT e)
{
	return led_data[e].enable;
}

void led_set_event_status(LED_EVENT e, uint32_t s)
{
	led_data[e].enable = s;
}

LED_EVENT led_get_event_priority(void)
{
	int i = LED_EVENT_MAX-1;
	for(; i>=0; i--)
	{
		if(led_data[i].enable)
			break;
	}
	return (LED_EVENT)i;
}

void led_dump_event_status(void)
{
	uint8_t i = 0;
	for(; i<LED_EVENT_MAX; i++)
	{
		led_log_i("led event status: %s/%d\r\n", led_event_str[i], led_data[i].enable);
	}
}

#if LED_USE_TASK
void StartLedTask(void const * argument)
{
	LED_EVENT evt;
	osEvent notify_data;
	while(1)
	{
		/* if(xQueueReceive( LedMsgQueueHandle, &evt, 1 ) == pdTRUE) */
		notify_data = osMessageGet( LedMsgQueueHandle, osWaitForever );
		uint32_t tmp = notify_data.value.signals;
		evt = (LED_EVENT)(tmp & 0xff);
		led_data[evt].enable = (tmp >> 8);

#if 1
		if(led_data[evt].enable)
		{
			switch(evt)
			{
				case LED_EVENT_CHG_FULL:
					led_data[LED_EVENT_LOW_BAT].enable = 0;
					led_data[LED_EVENT_CHG_ING].enable = 0;
					break;
				case LED_EVENT_LOW_BAT:
					led_data[LED_EVENT_CHG_FULL].enable = 0;
					led_data[LED_EVENT_CHG_ING].enable = 0;
					break;
				case LED_EVENT_CHG_ING:
					led_data[LED_EVENT_LOW_BAT].enable = 0;
					led_data[LED_EVENT_CHG_FULL].enable = 0;
					break;
				case LED_EVENT_CONNECTED:
					led_data[LED_EVENT_CONNECTING_FAIL].enable = 0;
					led_data[LED_EVENT_CONNECTING].enable = 0;
					break;
				case LED_EVENT_CONNECTING:
					led_data[LED_EVENT_CONNECTING_FAIL].enable = 0;
					led_data[LED_EVENT_CONNECTED].enable = 0;
					break;
				case LED_EVENT_CONNECTING_FAIL:
					led_data[LED_EVENT_CONNECTING].enable = 0;
					led_data[LED_EVENT_CONNECTED].enable = 0;
					break;
				default:break;
			}
		}
#endif
		/* select the highest priority event which is enabled */
		int i = LED_EVENT_MAX - 1;
		int selected_event = 0;
		int selected_priority = 0;
		for(; i >=0; i--)
		{
			if(led_data[i].enable)
			{
				/* find the highest priority event */
				if(selected_priority == led_data[i].priority)
				{
					/* if the same priority, select the event current received */
					/* if(led_data[i].enable) */
						/* selected_event = evt; */
				}
				else if (selected_priority < led_data[i].priority)
				{
					selected_priority = led_data[i].priority;
					selected_event = i;
				}
			}
		}

        led_log_i("Recv %s %s, and show %s ON\r\n",
                led_event_str[evt],
                (led_data[evt].enable ? "ON" : "OFF"),
                ((evt==selected_event) ? "it" : led_event_str[selected_event]));

		led_event_deal((LED_EVENT)selected_event);
	}
}
#endif

void SendLedEventToDeal(uint32_t e)
{
	/* xQueueSend(LedMsgQueueHandle, &e, 0); */
	osMessagePut(LedMsgQueueHandle, e, 0);
}

int led_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
	int ret = 0;
	switch(_notify_flag)
	{
		case PWRMGR_NOTIFY_STOP_STATE:
			if(_state == STOP_ENTER)
			{
				led_drv_suspend();
			}
			else if(_state == STOP_LEAVE)
			{
				led_drv_resume();
			}
			else
				ret = -1;
			break;
		case PWRMGR_NOTIFY_POWER_OFF:
			led_drv_deinit();
			PWRMGR_SendNotifyAck(&PmNotifyData);
			break;
		default: break;
	}
	return ret;
}

void led_post_init(void)
{
    PmNotifyData.func_name = "led_drv";
    PmNotifyData.data = NULL;
    PmNotifyData.callback= led_notify_callback;
    PmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE | PWRMGR_NOTIFY_POWER_OFF;
    PmNotifyData.func_level = PWRMGR_FUNC_DRIVER_LEVEL;
    PWRMGR_register_notify_func(&PmNotifyData);
}


#if LED_USE_QUEUE
static int led_create_queue(void)
{
	osMessageQDef(led_queue_t, 4, uint32_t);
	LedMsgQueueHandle = osMessageCreate(osMessageQ(led_queue_t), LedTaskHandle);
	if ( LedMsgQueueHandle == NULL )
		return -1;

	return 0;
}
#endif


void led_init(void)
{
	int8_t ret = 0;
	ret = led_drv_init();
	if(ret)
	{
		led_log_e("%s: led driver init fail", __func__);
		return ;
	}

#if LED_USE_TASK
	osThreadDef(Led_Task, StartLedTask, osPriorityNormal, 0, 128*2);
    LedTaskHandle = osThreadCreate(osThread(Led_Task), NULL);
#endif

#if LED_USE_QUEUE
	led_create_queue();
#endif

	led_post_init();
}

