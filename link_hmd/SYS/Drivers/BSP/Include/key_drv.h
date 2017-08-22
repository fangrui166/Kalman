#ifndef __KEY_DRV_H__
#define __KEY_DRV_H__
#ifdef __cplusplus
   extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

#define BTN_NUM                  (1)
#define BTN_INDEX_POWER          0

#define BTN_PRESSED_BIT_POWER    (1 << BTN_INDEX_POWER)
#define BTN_POWER_OFF_FLAG       (1 << 6)

#define DEBOUNCE_TIME_MS		 40
#define BLINK_LED_TIME_MS		 1000
#define LONG_PRESS_TIME_MS		 1000

#define HTC_LOG_KEY              1

#if HTC_LOG_KEY
	#define key_log_d(args...)				hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_BUTTON, ##args)
	#define key_log_i(args...)				hlog_printf(HLOG_LVL_INFO, HLOG_TAG_BUTTON, ##args)
	#define key_log_e(args...)				hlog_printf(HLOG_LVL_ERR, HLOG_TAG_BUTTON, ##args)
#else
	#define key_log_d(args...)				printf("[KEY]" args)
	#define key_log_i(args...)				printf("[KEY]" args)
	#define key_log_e(args...)				printf("[KEY][ERROR]" args)
#endif

typedef enum{
    KeyRelease=0,
    KeyPress
}KeyAction;

/* Send Button data: */
struct btn_send_t{
    uint8_t KeyCode;
    uint8_t KeyPress;
};

/* Button data struct */
typedef struct{
	char *keyTimerName;
	uint16_t keyGpioPin;
	uint32_t keyGpioPullMode;
	GPIO_TypeDef *keyGpioPort;
	GPIO_PinState PinState;
	uint8_t index;
	uint32_t debounce_time_new;
	uint32_t debounce_time_old;
	struct btn_send_t s;
} btn_data_t;
extern btn_data_t btn_info[BTN_NUM];

int key_drv_init(void);
void key_btn_isr(uint8_t key_index);


#ifdef __cplusplus
   }
#endif

#endif /*__KEY_DRV_H__*/
