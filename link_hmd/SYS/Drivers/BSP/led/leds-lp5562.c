/*
 * LP5562 LED driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * Author: Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "rtos_i2c_drv.h"
#include "led_drv.h"
#include "leds-lp55xx.h"

struct lp55xx_led led_data[LED_NUMS] = {
		[LED_R] = {2, 200, 0xff, 0x3f},   //the chan_nr should be 0, but HW is 2
		[LED_G] = {1, 200, 0xff, 0x3f},   //decress green current for amber color
	};

int lp55xx_write(u8 reg, u8 val)
{
	int ret = 0;
	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_LED_CTRL_ADDR, reg, I2C_8BIT, &val, 1, I2C_SHORT_DELAY);
	if(ret > 0)
		led_log_e("%s: RTOS_I2C_WriteBuffer fail(reg:0x%x, val:0x%x)\r\n", __func__, reg, val);
	return ret;
}

int lp55xx_read(u8 reg, u8 *val)
{
	int ret = 0;

	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_LED_CTRL_ADDR, reg, I2C_8BIT, val, 1, I2C_SHORT_DELAY);

	return ret;
}

int lp55xx_update_bits(u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	ret = lp55xx_read(reg, &tmp);
	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= val & mask;

	return lp55xx_write(reg, tmp);
}

static inline void lp5562_wait_opmode_done(void)
{
	/* operation mode change needs to be longer than 153 us */
	/* usleep_range(200, 300); */
	HAL_Delay(1);
}

static inline void lp5562_wait_enable_done(void)
{
	/* it takes more 488 us to update ENABLE register */
	/* usleep_range(500, 600); */
	HAL_Delay(1);
}

void lp5562_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	u8 addr[] = {
		LP5562_REG_R_CURRENT,
		LP5562_REG_G_CURRENT,
		LP5562_REG_B_CURRENT,
		LP5562_REG_W_CURRENT,
	};

	led->led_current = led_current;
	lp55xx_write(addr[led->chan_nr], led_current);
}

static void lp5562_load_engine(enum lp55xx_engine_index idx)
{
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5562_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5562_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5562_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5562_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5562_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5562_LOAD_ENG3,
	};

	lp55xx_update_bits(LP5562_REG_OP_MODE, mask[idx], val[idx]);

	lp5562_wait_opmode_done();
}

static void lp5562_stop_engine(void)
{
	lp55xx_write(LP5562_REG_OP_MODE, LP5562_CMD_DISABLE);
	lp5562_wait_opmode_done();
}

void lp5562_run_engine(bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp55xx_write(LP5562_REG_ENABLE, LP5562_ENABLE_DEFAULT);
		lp5562_wait_enable_done();
		lp5562_stop_engine();
		lp55xx_write(LP5562_REG_G_PWM, 0);
		lp55xx_write(LP5562_REG_B_PWM, 0);
		lp55xx_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);
		lp55xx_write(LP5562_REG_OP_MODE, LP5562_CMD_DIRECT);
		lp5562_wait_opmode_done();
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(LP5562_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(LP5562_REG_ENABLE, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5562_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG1_M) | LP5562_RUN_ENG1;
		exec = (exec & ~LP5562_EXEC_ENG1_M) | LP5562_RUN_ENG1;
	}
	if (LP5562_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG2_M) | LP5562_RUN_ENG2;
		exec = (exec & ~LP5562_EXEC_ENG2_M) | LP5562_RUN_ENG2;
	}

	if (LP5562_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG3_M) | LP5562_RUN_ENG3;
		exec = (exec & ~LP5562_EXEC_ENG3_M) | LP5562_RUN_ENG3;
	}
	lp55xx_write(LP5562_REG_OP_MODE, mode);
	lp5562_wait_opmode_done();

	/* lp55xx_update_bits(LP5562_REG_ENABLE, LP5562_EXEC_M, exec); */
#define ENGINE_1     		0x1
#define SEL_G        		0x2
#define MODE_RUN     		0x2
#define MODE_ENG_1   		0x4
#define ENABLE_RUN   		0x2
#define ENABLE_ENG_1 		0x4
	/* power on loading: blink green. */
	lp55xx_write(LP5562_REG_ENG_SEL, (ENGINE_1<<SEL_G));
	/* lp55xx_write(LP5562_REG_OP_MODE, (MODE_RUN<<MODE_ENG_1)); */
	/* lp55xx_write(LP5562_REG_ENABLE, 0x60); */
	lp55xx_write(LP5562_REG_ENABLE, LP5562_MASTER_ENABLE | (ENABLE_RUN<<ENABLE_ENG_1));
	lp5562_wait_enable_done();
}
#if 0
static int lp5562_init_device(void)
{
	int ret;
	u8 cfg = LP5562_DEFAULT_CFG;

	/* Set all PWMs to direct control mode */
	ret = lp55xx_write(LP5562_REG_OP_MODE, LP5562_CMD_DIRECT);
	if (ret)
		return ret;

	lp5562_wait_opmode_done();

	/* Update configuration for the clock setting */
	/* if (!lp55xx_is_extclk_used(chip)) */
		cfg |= LP5562_CLK_INT;

	ret = lp55xx_write(LP5562_REG_CONFIG, cfg);
	if (ret)
		return ret;

	/* Initialize all channels PWM to zero -> leds off */
	lp55xx_write(LP5562_REG_R_PWM, 0);
	lp55xx_write(LP5562_REG_G_PWM, 0);
	lp55xx_write(LP5562_REG_B_PWM, 0);
	lp55xx_write(LP5562_REG_W_PWM, 0);

	/* Set LED map as register PWM by default */
	lp55xx_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);

	return 0;
}
#endif
#if LP5562_DIRECT_PWM_MODE
static void lp5562_led_brightness_work(u8 brightness)
{
	u8 addr[] = {
		LP5562_REG_R_PWM,
		LP5562_REG_G_PWM,
		LP5562_REG_B_PWM,
		LP5562_REG_W_PWM,
	};

	/* mutex_lock(&chip->lock); */
	lp55xx_write(addr[led->chan_nr], brightness);
	/* mutex_unlock(&chip->lock); */
}
#endif
static void lp5562_write_program_memory(u8 base, const u8 *rgb, int size)
{
	int i;

	if (!rgb || size <= 0)
		return;

	for (i = 0; i < size; i++)
		lp55xx_write(base + i, *(rgb + i));

	lp55xx_write(base + i, 0);
	lp55xx_write(base + i + 1, 0);
}

/* check the size of program count */
static inline bool _is_pc_overflow(struct lp55xx_predef_pattern *ptn)
{
	return (ptn->size_r >= LP5562_PROGRAM_LENGTH ||
		ptn->size_g >= LP5562_PROGRAM_LENGTH ||
		ptn->size_b >= LP5562_PROGRAM_LENGTH);
}

static int lp5562_run_predef_led_pattern(int mode)
{
	if (mode == LP5562_PATTERN_OFF) {
		/* lp5562_run_engine(chip, false); */
		return 0;
	}

	/* 0x40|xx (>30.87 = 0.49*63) */
		/* {0x40, 0x1f, 0x40|(100/15.6)}, */
	uint8_t const pdata[3][24] ={
		/* engine1: 500ms on -> 500ms off */
		/* { 0x40, 0x1f, 0x40|0x20, 0x00, 0x40, 0x00, 0x40|0x20, 0x00 }, */
		/* engine1: loading mode 100ms on -> 100ms off */
		{ 0x40, 0x1f, 0x40|0x6, 0x00, 0x40, 0x00, 0x40|0x6, 0x00 },
		/* engine2: 100ms on -> 100ms off -> 100ms on -> 1000ms off */
		{ 0x40, 0x1f, 0x40|6, 0x00, 0x40, 0x00, 0x40|6, 0x00,
		  0x40, 0x1f, 0x40|6, 0x00, 0x40, 0x00, 0x40|0x3f, 0x00 },
		/* engine3: 100ms on -> 100ms off -> 100ms on -> 100ms off -> 100ms on -> 1000ms off */
		{ 0x40, 0x1f, 0x40|6, 0x00, 0x40, 0x00, 0x40|6, 0x00,
		  0x40, 0x1f, 0x40|6, 0x00, 0x40, 0x00, 0x40|6, 0x00,
		  0x40, 0x1f, 0x40|6, 0x00, 0x40, 0x00, 0x40|0x3f, 0x00 },
	};
	struct lp55xx_predef_pattern pat = {pdata[0], pdata[1], pdata[2], 8, 16, 24};
	struct lp55xx_predef_pattern *ptn;
	ptn = &pat;

	/* ptn = chip->pdata->patterns + (mode - 1); */
	if (!ptn || _is_pc_overflow(ptn)) {
		led_log_e("invalid pattern data\n");
		return -1;
	}

	lp5562_stop_engine();

	/* Set LED map as RGB */
	/* engine 1: steady green */
	/* engine 2: blink red */
	/* engine 3: blink red and green */
	/* lp55xx_write(LP5562_REG_ENG_SEL, (0x01<<2) | (0x02)); */

	/* Load engines */
	int i = LP55XX_ENGINE_1;
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++)
	{
		lp5562_load_engine((enum lp55xx_engine_index)i);
	}

	/* Clear program registers */
	lp55xx_write(LP5562_REG_PROG_MEM_ENG1, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG1 + 1, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG2, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG2 + 1, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG3, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG3 + 1, 0);
	/* Program engines */
	lp5562_write_program_memory(LP5562_REG_PROG_MEM_ENG1,
				ptn->r, ptn->size_r);
	lp5562_write_program_memory(LP5562_REG_PROG_MEM_ENG2,
				ptn->g, ptn->size_g);
	lp5562_write_program_memory(LP5562_REG_PROG_MEM_ENG3,
				ptn->b, ptn->size_b);
	/* Run engines */
	lp5562_run_engine(true);

	return 0;
}

void lp5562_load_selected_pattern(int ms)
{
	static uint8_t index_old = 0x0f;
	uint8_t index = 0;
	if(ms == 100)
		index = 1;

	if(index_old != index)
		index_old = index;
	else
	{
		led_log_d("no need change pattern\n");
		return ;
	}

	const u8 pdata[2][8] ={
		/* engine1: 500ms on -> 500ms off */
		{ 0x40, 0x1f, 0x40|0x20, 0x00, 0x40, 0x00, 0x40|0x20, 0x00 },
		/* engine1: loading mode 100ms on -> 100ms off */
		{ 0x40, 0x1f, 0x40|0x6, 0x00, 0x40, 0x00, 0x40|0x6, 0x00 }
	};

	lp5562_stop_engine();
	lp5562_load_engine(LP55XX_ENGINE_1);
	/* Clear program registers */
	lp55xx_write(LP5562_REG_PROG_MEM_ENG1, 0);
	lp55xx_write(LP5562_REG_PROG_MEM_ENG1 + 1, 0);

	/* Program engines */
	lp5562_write_program_memory(LP5562_REG_PROG_MEM_ENG1,
				pdata[index], sizeof(pdata[0]));

}

int lp55xx_deinit_device(void)
{
	return lp55xx_write(LP5562_REG_ENABLE, 0x00);
}

int lp5562_init(void)
{
	int ret = 0;
	/* struct lp55xx_chip *chip; */
	/* chip = &led_chip; */

	/* soft reset chip */
	lp55xx_write(LP5562_REG_RESET, LP5562_RESET);
	HAL_Delay(1);

	/* chip enable */
	lp55xx_write(LP5562_REG_ENABLE, 0x40);
	HAL_Delay(1);

	lp55xx_write(LP5562_REG_CONFIG, LP5562_PWRSAVE_EN | 0x01);
	/* init led current */
	int i=0;
	for(i=0; i<LED_NUMS; i++)
		lp5562_set_led_current(&led_data[i], led_data[i].led_current);

	lp5562_run_predef_led_pattern(LP5562_CMD_RUN);

	/* power on normal: steady green */
	/* lp55xx_write(LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM); */
	/* lp55xx_write(LP5562_REG_G_PWM, 0x3f); */

	return ret;
}

