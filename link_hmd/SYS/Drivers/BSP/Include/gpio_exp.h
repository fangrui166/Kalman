/*!
@file gpio_exp.h
@brief This file includes API's used for Gpio Expander.
@details This provides to toggle the GPIO Expander pins
@version :
*/

#ifndef _GPIOEXP_DRV_H_
#define _GPIOEXP_DRV_H_

#include "cmsis_os.h"
#include "gpio.h"
#include "hlog_api.h"

//#define GPIOEXP6418E_SLAVE_ADDRESS	0x68
/* 0/1: input/output */
#define TCA6418E_Reg_GPIO_DAT_STAT1	0x14  //(H)GPIO0~GPIO7(L)
#define TCA6418E_Reg_GPIO_DAT_STAT2	0x15  //(H)GPIO15~GPIO8(L)
#define TCA6418E_Reg_GPIO_DAT_STAT3	0x16  //(H)GPIO17~GPIO16(L)
/* 0/1: low/high */
#define TCA6418E_Reg_GPIO_DAT_OUT1	0x17  //(H)GPIO0~GPIO7(L)
#define TCA6418E_Reg_GPIO_DAT_OUT2	0x18  //(H)GPIO15~GPIO8(L)
#define TCA6418E_Reg_GPIO_DAT_OUT3	0x19  //(H)GPIO17~GPIO16(L)
/* 0/1: input/output */
#define TCA6418E_Reg_GPIO_DIR1		0x23  //(H)GPIO0~GPIO7(L)
#define TCA6418E_Reg_GPIO_DIR2		0x24  //(H)GPIO15~GPIO8(L)
#define TCA6418E_Reg_GPIO_DIR3		0x25  //(H)GPIO17~GPIO16(L)
/* 0/1: enable/disable */
#define TCA6418E_Reg_GPIO_PULLDOWN1 0x2C  //(H)GPIO0~GPIO7(L)
#define TCA6418E_Reg_GPIO_PULLDOWN2 0x2D  //(H)GPIO15~GPIO8(L)
#define TCA6418E_Reg_GPIO_PULLDOWN3 0x2E  //(H)GPIO17~GPIO16(L)
#define IOEXP_GPIO_NUM              18

#define IOEXP_USE_TASK              0
#define IOEXP_USE_QUEUE             0
#define HTC_LOG_IOEXP               1

#if HTC_LOG_IOEXP
	#define ioexp_log_d(args...)				hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_IOEXP, ##args)
	#define ioexp_log_e(args...)				hlog_printf(HLOG_LVL_ERR, HLOG_TAG_IOEXP, ##args)
#else
	#define ioexp_log_d(args...)				printf("[IOEXP]" args)
	#define ioexp_log_e(args...)				printf("[IOEXP][ERROR]" args)
#endif

enum {
	IOEXP_REG_STAT,
	IOEXP_REG_OUT,
	IOEXP_REG_DIR,
	IOEXP_REG_PULL
};

/* IOEXP_REG_OUT, */
enum {
	IOEXP_GPIO_LOW,
	IOEXP_GPIO_HIGH
};

/* IOEXP_REG_DIR, */
enum {
	IOEXP_GPIO_INPUT,
	IOEXP_GPIO_OUTPUT
};

/* IOEXP_REG_PULL */
enum {
	IOEXP_GPIO_PULLDOWN,
	IOEXP_GPIO_NOPULL
};

/* enum all the ioexp gpio numbers */
/* (pcb_id == XB2) */
enum {
	/* pin on io expendor1 */
	IOEXP_I_V_VDDI_L_EN,
	IOEXP_I_V_VDDI_R_EN,
	IOEXP_I_UNUSED_GPIO02,
	IOEXP_I_R_GI_FLM_1_IO,
	IOEXP_I_R_GI_FLM_2_IO,
	IOEXP_I_ERR_FG_R_IO,
	IOEXP_I_V_BOOST_5V_EN,
	IOEXP_I_V_DP_PWM_EN,
	IOEXP_I_ALC4040_GPIO_AL1_1,
	IOEXP_I_AUD_CDC_ALC5665_GPIO6,
	IOEXP_I_AUD_CDC_ALC5665_GPIO7,
	IOEXP_I_AUD_CDC_ALC5665_GPIO8,
	IOEXP_I_LED_DRV_EN,
	IOEXP_I_V_USB30_5V_EN,
	IOEXP_I_AUD_HP_EN,
	IOEXP_I_V_VDD_1V2_PWM_EN,
	IOEXP_I_AUD_CDC_LDO1_EN,
	IOEXP_I_V_VDD_3V3_PWM_EN,
	/* pin on io expendor2 */
    IOEXP_II_V_HDMI_EN,
    IOEXP_II_USBHUB_RST_N,
    IOEXP_II_V_DP_PS_EN,
    IOEXP_II_UNUSED_GPIO04,
    IOEXP_II_CCG4_XRES,
    IOEXP_II_BB_RSTN,
    IOEXP_II_MUX_EN,
    IOEXP_II_UNUSED_GPIO07,
    IOEXP_II_UNUSED_GPIO08,
    IOEXP_II_RST_DP_N,
    IOEXP_II_PS176_LANE_MODE,
    IOEXP_II_PS176_PD_N,
    IOEXP_II_UNUSED_GPIO12,
    IOEXP_II_V_PX_1V1_EN,
    IOEXP_II_V_PX_PWM_EN,
    IOEXP_II_V_PX_1V8_EN,
    IOEXP_II_PX_RESET_L_LS,
    IOEXP_II_PX_RESET_R_LS,
};

enum {
	/* pin on io expendor1 */
	IOEXP_L_GI_FLM_1_IO,
	IOEXP_L_GI_FLM_2_IO,
	IOEXP_ERR_FG_L_IO,
	IOEXP_R_GI_FLM_1_IO,
	IOEXP_R_GI_FLM_2_IO,
	IOEXP_ERR_FG_R_IO,
	IOEXP_PHONE_CHG_ISET_1,
	IOEXP_V_DP_PWM_EN,
	IOEXP_ALC4040_GPIO_AL1_1,
	IOEXP_AUD_CDC_ALC5665_GPIO6,
	IOEXP_AUD_CDC_ALC5665_GPIO7,
	IOEXP_AUD_CDC_ALC5665_GPIO8,
	IOEXP_LED_DRV_EN,
	IOEXP_V_USB30_5V_EN,
	IOEXP_CHG_TEMP_FAULT,
	IOEXP_V_VDD_1V2_PWM_EN,
	IOEXP_AUD_CDC_LDO1_EN,
	IOEXP_PHONE_CHG_ISET_2,
};

#if IOEXP_USE_TASK
extern osThreadId IOExpTaskHandle;
#endif
#if IOEXP_USE_QUEUE
extern osMessageQId IoexpMsgQueueHandle;
#endif

int8_t ioexp_drv_init(void);
int8_t ioexp_gpio_set_value(uint8_t index, uint8_t gpio, uint8_t value);
int8_t ioexp_gpio_get_value(uint8_t index, uint8_t gpio);


#endif /*_GPIOEXP_DRV_H_*/
