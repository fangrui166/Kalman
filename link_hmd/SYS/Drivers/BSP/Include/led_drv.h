#ifndef _LED_DRV_H_
#define _LED_DRV_H_

#include "led_hal.h"

#define LED_NUMS 				2
#define LP5562_DIRECT_PWM_MODE 	0

#define LED_AFTER_IOEXP     	0
#define HTC_LOG_LED             1
#if HTC_LOG_LED
	#define led_log_d(args...)				hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_LED, ##args)
	#define led_log_i(args...)				hlog_printf(HLOG_LVL_INFO, HLOG_TAG_LED, ##args)
	#define led_log_e(args...)				hlog_printf(HLOG_LVL_ERR, HLOG_TAG_LED, ##args)
#else
	#define led_log_d(args...)				printf("[LED]" args)
	#define led_log_i(args...)				printf("[LED]" args)
	#define led_log_e(args...)				printf("[LED][ERROR]" args)
#endif

typedef enum {
	LED_R,
	LED_G,
	LED_B,
	LED_W,
	LED_A
}LedType;

/* #if LP5562_DIRECT_PWM_MODE */
typedef enum {
	PWM_DUTY_0 = 0x0,
	PWM_DUTY_25 = 0x20,
	PWM_DUTY_50 = 0x80,
	PWM_DUTY_75 = 0xC0,
	PWM_DUTY_100 = 0xFF
}PwmDuty;
/* #endif */

int8_t led_set_led_state(uint8_t led, LED_STATE s);
void led_event_deal(LED_EVENT evt);
int8_t led_drv_suspend(void);
int8_t led_drv_resume(void);
int8_t led_drv_init(void);
int8_t led_drv_deinit(void);
int8_t led_reg_write(uint8_t reg, uint8_t val);
int8_t led_reg_read(uint8_t reg, uint8_t *val);


#endif /*_LEDCTRL_DRV_H_*/
