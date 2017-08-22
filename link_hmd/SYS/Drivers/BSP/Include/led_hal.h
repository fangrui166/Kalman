#ifndef __LED_HAL_H
#define __LED_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "hlog_api.h"

#define LED_USE_TASK        1
#define LED_USE_QUEUE       1
#define LED_USE_PRIORITY    1

#define LED_COLOR_RED       0x01
#define LED_COLOR_GREEN     0x02
#define LED_COLOR_BLUE      0x04
#define LED_COLOR_AMBER     (LED_COLOR_RED | LED_COLOR_GREEN)
#define LED_COLOR_WRITE     (LED_COLOR_RED | LED_COLOR_GREEN | LED_COLOR_BLUE)

/* not 0: led event enable, 0: led event disable */
#define	LED_EVENT_OFF 		0
#define	LED_EVENT_ON 		0x100

/* H = led On, L = led Off */
typedef enum
{
	LED_STEADY_OFF = 0,
	LED_STEADY_ON,
	/* LED_BLINK_OFF = 0x0f, */
	LED_BLINK_ON
}LED_STATE;

#if LED_USE_PRIORITY
typedef enum
{
	LED_PRIORITY_NORMAL = 0,
	LED_PRIORITY_CONNECTED = LED_PRIORITY_NORMAL,
	LED_PRIORITY_STAND_BY,
	LED_PRIORITY_CHG_FULL,
	LED_PRIORITY_LOW_BAT,
	LED_PRIORITY_CONNECTING,
	LED_PRIORITY_CONNECTING_FAIL,
	LED_PRIORITY_CHG_ING,
	LED_PRIORITY_CHG_FAULT,
	/* LED_PRIORITY_KERNEL_MODE, */
	LED_PRIORITY_LOADING,
	LED_PRIORITY_POWER_OFF,
	LED_PRIORITY_ALL_LEDS_OFF = LED_PRIORITY_POWER_OFF,
	LED_PRIORITY_ESTRESS_TEST,
	LED_PRIORITY_MAX
}LED_PRIORITY;
#endif

typedef enum
{
	LED_EVENT_NORMAL = 0,
	LED_EVENT_CONNECTED,
	LED_EVENT_STAND_BY,
	LED_EVENT_CHG_FULL,
	LED_EVENT_LOW_BAT,
	LED_EVENT_CONNECTING,
	LED_EVENT_CONNECTING_FAIL,
	LED_EVENT_CHG_ING,
	LED_EVENT_CHG_FAULT,
	/* LED_EVENT_KERNEL_MODE, */
	LED_EVENT_LOADING,
	LED_EVENT_POWER_OFF,
	LED_EVENT_ALL_LEDS_OFF,
	LED_EVENT_ESTRESS_TEST,
	LED_EVENT_MAX
}LED_EVENT;

/* note:
	on:off
	m:n  --> blink
	0:1  --> steady off
	1:0  --> steady on
*/
typedef struct
{
	uint32_t enable;
	LED_EVENT event;
#if LED_USE_PRIORITY
	LED_PRIORITY priority;
#endif
	/* uint8_t blink_color; */
	/* uint8_t blink_cnt_on; */
	/* uint8_t blink_cnt_off; */
}led_event_data;

#if LED_USE_QUEUE
extern osMessageQId LedMsgQueueHandle;
#endif

extern uint8_t led_event;

LED_EVENT led_get_event_priority(void);
uint32_t led_get_event_status(LED_EVENT e);
void led_set_event_status(LED_EVENT e, uint32_t s);
void led_dump_event_status(void);
void SendLedEventToDeal(uint32_t e);
void led_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_HAL_H */
