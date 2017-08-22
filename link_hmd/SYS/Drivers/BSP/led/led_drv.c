#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *			   I N C L U D E S
 *****************************************************************************/
#include "led_drv.h"
#include "leds-lp55xx.h"
#include "gpio_exp.h"
#include "PowerManager_power.h"

/* enable the API when need */
#if 1
int8_t led_reg_read(uint8_t reg, uint8_t *val)
{
	return lp55xx_read(reg, val);
}
#endif

int8_t led_reg_write(uint8_t reg, uint8_t val)
{
	return (0 - lp55xx_write(reg, val));
}

/* percent:0x00/0% ~ 0xff/100% */
int8_t led_set_pwm_duty(LedType led, uint8_t percent)
{
	uint8_t ret = -1;
	/* change to direct PWM mode. */
	ret = led_reg_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);
	if (ret != 0)
		return ret;
	switch(led)
	{
		case LED_R:
			ret = led_reg_write(LP5562_REG_B_PWM, percent);
			/* led_reg_write(LP5562_REG_G_PWM, 0); */
			break;
		case LED_G:
			ret = led_reg_write(LP5562_REG_G_PWM, percent);
			/* led_reg_write(LP5562_REG_B_PWM, 0); */
			break;
		case LED_A: /* amber color */
			/* led_reg_write(LP5562_REG_B_PWM, percent ? 255:0); */
			/* led_reg_write(LP5562_REG_G_PWM, percent ? 191:0); */
			ret = led_reg_write(LP5562_REG_B_PWM, percent ? (255+1)/4:0);
			ret |= led_reg_write(LP5562_REG_G_PWM, percent ? (191+1)/6:0);
			break;
		default:break;
	}

	return ret;
}

#if LP5562_DIRECT_PWM_MODE
void led_get_pwm_duty(LedType led, uint8_t *percent)
{
	switch(led)
	{
		case LED_R:
			led_reg_read(LP5562_REG_B_PWM, percent);
			break;
		case LED_G:
			led_reg_read(LP5562_REG_G_PWM, percent);
			break;
		case LED_A:
			led_reg_read(LP5562_REG_B_PWM, percent);
			/* read one is enough to detemine led on/off */
			/* led_reg_read(LP5562_REG_G_PWM, percent); */
			break;
		default:break;
	}
}
#endif

/* set current to change brightness */
void led_set_brightness(LedType led, uint8_t current)
{
}

#if LED_AFTER_IOEXP
/* output high to enable chip */
int8_t led_chip_enable(int enable)
{
	int8_t ret = 0;
	if(enable)
		ret = ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_LED_DRV_EN, IOEXP_GPIO_HIGH);
	else
		ret = ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_LED_DRV_EN, IOEXP_GPIO_LOW);

	return ret;
}
#endif

#if LED_AFTER_IOEXP
/**use io expender gpio as chip enable pin */
static void led_enable_pin_init(uint8_t dir)
{
	/* set gpio as output pin */
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_LED_DRV_EN, dir);
}
#endif

uint8_t reg_enable = 0;
int8_t led_drv_suspend(void)
{
#if 1
	SendLedEventToDeal(LED_EVENT_STAND_BY | LED_EVENT_ON);
	return 0;
#else
	/* only no charger plugin state can turn off led */
	if(isChargerIn())
		return 0;
	led_reg_read(LP5562_REG_ENABLE, &reg_enable);
	/* printf("+++++%s: reg_enable(0x%0x)\r\n", __func__, reg_enable); */
	return led_reg_write(LP5562_REG_ENABLE, 0x0);
#endif
}

int8_t led_drv_resume(void)
{
	SendLedEventToDeal(LED_EVENT_POWER_OFF | LED_EVENT_OFF);
#if 1
	SendLedEventToDeal(LED_EVENT_STAND_BY | LED_EVENT_OFF);
	return 0;
#else
	if(isChargerIn())
		return 0;

	if(LED_EVENT_NORMAL == led_get_event_priority())
		led_reg_write(LP5562_REG_G_PWM, 0x1f);
	/* printf("+++++%s: reg_enable(0x%0x)\r\n", __func__, reg_enable); */
	return led_reg_write(LP5562_REG_ENABLE, LP5562_MASTER_ENABLE);
	/* return led_reg_write(LP5562_REG_ENABLE, reg_enable); */
#endif
}

int8_t led_drv_init(void)
{
	int8_t ret = 0;

	/* Direct I2C Register PWM Control Example */
	/* VDD = 3.6V -> 1.8V EN -> wait 1ms -> */
	/* 00h = 0x40 -> wait 500us */
	/* 70h = 0x00 -> */
	/* set R/G/B PWM duty */

#if LED_AFTER_IOEXP
	/* ioexp gpio set as output */
	/* code in ioexp init table or here ??? */
	/* led_enable_pin_init(IOEXP_GPIO_OUTPUT); */
	/* code in ioexp init table or here ??? */
	/* led_chip_enable(1); */
#endif
	ret = lp5562_init();

	return ret;
}

int8_t led_drv_deinit(void)
{
	int8_t ret = 0;
	ret = lp55xx_deinit_device();

	return ret;
}

int8_t led_set_led_state(uint8_t led, LED_STATE s)
{
/* #if LP5562_DIRECT_PWM_MODE */
	int ret = -1;
	LedType l;
	if(led & LED_COLOR_RED)
	{
        /* amber: both red and green are on */
		if(led & LED_COLOR_GREEN)
			l = LED_A;
		else
			l = LED_R;
	}
	else if(led & LED_COLOR_GREEN)
		l = LED_G;
	else
		;

	if(s == LED_STEADY_ON)
		ret = led_set_pwm_duty(l, PWM_DUTY_25);
	else if(s == LED_STEADY_OFF)
		ret = led_set_pwm_duty(l, PWM_DUTY_0);

	return ret;
/* #endif */
}

void led_get_led_state(uint8_t led, LED_STATE *s)
{
#if LP5562_DIRECT_PWM_MODE
	LedType l;
	if(led & LED_COLOR_AMBER)
		l = LED_A;
	else
	{
		if(led & LED_COLOR_RED)
			l = LED_R;
		else if(led & LED_COLOR_GREEN)
			l = LED_G;
		else
			;
	}

	uint8_t pecent = 0;
	*s = LED_STEADY_OFF;
	led_get_pwm_duty(l, &pecent);
	if(pecent)
		*s = LED_STEADY_ON;
#endif
}

void led_event_deal(LED_EVENT evt)
{
	if(evt == led_event)
		return ;
	else
	{
		led_event = evt;
		led_reg_write(LP5562_REG_G_PWM, 0x00);
		led_reg_write(LP5562_REG_B_PWM, 0x00);
		/* lp5562_stop_engine(); */
		led_reg_write(LP5562_REG_OP_MODE, LP5562_CMD_DISABLE);
		osDelay(1);
	}

	switch(evt)
	{
#define ENGINE_1     		0x1
#define ENGINE_2     		0x2
#define ENGINE_3     		0x3
#define SEL_R        		0x0
#define SEL_G        		0x2
#define MODE_RUN     		0x2
#define MODE_ENG_1   		0x4
#define MODE_ENG_2   		0x2
#define MODE_ENG_3   		0x0
#define ENABLE_RUN   		0x2
#define ENABLE_ENG_1 		0x4
#define ENABLE_ENG_2 		0x2
#define ENABLE_ENG_3 		0x0

		/* engine 2: blink red */
		case LED_EVENT_LOW_BAT:
			led_reg_write(LP5562_REG_ENG_SEL, (ENGINE_2<<SEL_R));
			led_reg_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_2));
			led_reg_write(LP5562_REG_ENABLE, (ENABLE_RUN<<ENABLE_ENG_2) | LP5562_MASTER_ENABLE);
			break;
		/* direct PWM: steady amber(red + green)*/
		case LED_EVENT_CHG_ING:
			led_reg_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);
			led_reg_write(LP5562_REG_G_PWM, 0x1f);
			led_reg_write(LP5562_REG_B_PWM, 0x3f);
			led_reg_write(LP5562_REG_ENABLE, LP5562_MASTER_ENABLE);
			break;
		/* engine 3: blink red */
		case LED_EVENT_CHG_FAULT:
			led_reg_write(LP5562_REG_ENG_SEL, (ENGINE_3<<SEL_R));
			led_reg_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_3));
			led_reg_write(LP5562_REG_ENABLE, (ENABLE_RUN<<ENABLE_ENG_3) | LP5562_MASTER_ENABLE);
			break;
		/* engine 1 blink red */
		case LED_EVENT_CONNECTING_FAIL:
			lp5562_load_selected_pattern(500);
			led_reg_write(LP5562_REG_ENG_SEL, (ENGINE_1<<SEL_R));
			led_reg_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_1));
			led_reg_write(LP5562_REG_ENABLE, (ENABLE_RUN<<ENABLE_ENG_1) | LP5562_MASTER_ENABLE);
			break;
		/* engine 1 blink green */
		case LED_EVENT_STAND_BY:
			lp5562_load_selected_pattern(500);
			led_reg_write(LP5562_REG_ENG_SEL, (ENGINE_1<<SEL_G));
			led_reg_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_1));
			led_reg_write(LP5562_REG_ENABLE, (ENABLE_RUN<<ENABLE_ENG_1) | LP5562_MASTER_ENABLE);
			break;
		case LED_EVENT_CONNECTED:
		case LED_EVENT_CHG_FULL:
		case LED_EVENT_NORMAL:
			led_reg_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);
			led_reg_write(LP5562_REG_G_PWM, 0x3f);
			led_reg_write(LP5562_REG_ENABLE, LP5562_MASTER_ENABLE);
			break;
		/* case LED_EVENT_KERNEL_MODE: */
			/* break; */
		case LED_EVENT_LOADING:
		case LED_EVENT_CONNECTING:
			lp5562_load_selected_pattern(100);
			led_reg_write(LP5562_REG_ENG_SEL, (ENGINE_1<<SEL_G));
			led_reg_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_1));
			led_reg_write(LP5562_REG_ENABLE, (ENABLE_RUN<<ENABLE_ENG_1) | LP5562_MASTER_ENABLE);
			break;
		/* turn off led when shutdown */
		/* enable register turn 0 when led reset pin low */
		/* ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_LED_DRV_EN, IOEXP_GPIO_LOW); */
		case LED_EVENT_POWER_OFF:
		case LED_EVENT_ALL_LEDS_OFF:
			led_reg_write(LP5562_REG_ENABLE, 0x0);
			break;
		default:break;
	}
}


#ifdef __cplusplus
}
#endif
