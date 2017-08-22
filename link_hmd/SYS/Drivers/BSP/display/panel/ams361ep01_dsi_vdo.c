#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "x_pmic.h"
#include "lcm_drv.h"
#include "gpio.h"
#include "display_drv.h"

/* ***************************************************************************
 * 	Local Constant
 * ***************************************************************************/
#define LCM_WIDTH											(1080)
#define LCM_HEIGHT 											(1200)
#define LCM_DEFAULT_BACKLIGHT_LEVEL 						(0x71)	/* 0x33,0x5E,0x8C,0xB2,0xD8,0xFF */
#define LCM_BACKLIGHT_CHANGE_SPEED 							(0x20)	/* 0x20(1 frame) - 0x28(32 frame) */

//#define LCM_DSI_CMD_MODE
//#define LCM_MONITOR_TASK

#define LCM_L_RST_PIN										(GPIO_L_AMO_LCD_RST)
#define LCM_L_RST_PIN_PORT									(GPIO_PORT_L_AMO_LCD_RST)
#define LCM_XB_L_RST_PIN										(GPIO_XB_L_AMO_LCD_RST)
#define LCM_XB_L_RST_PIN_PORT									(GPIO_XB_PORT_L_AMO_LCD_RST)

#define LCM_R_RST_PIN										(GPIO_R_AMO_LCD_RST)
#define LCM_R_RST_PIN_PORT									(GPIO_PORT_R_AMO_LCD_RST)
#define LCM_XB_R_RST_PIN										(GPIO_XB_R_AMO_LCD_RST)
#define LCM_XB_R_RST_PIN_PORT									(GPIO_XB_PORT_R_AMO_LCD_RST)



/* ***************************************************************************
 * 	Local Variables
 * ***************************************************************************/
extern pcbid_t pcb_id;
static dsi_funcs *lcm_dsi_funcs						= NULL;
#ifdef LCM_MONITOR_TASK
osThreadId lcm_task_handle = NULL;
#endif

#define DSI_CMD_RAW_BUILDER(name) {name, sizeof(name) - 3}

static const unsigned char level2_key_unlock[] = {0x39, 0xF0,  2, 0x5A, 0x5A};
static const unsigned char resolution_setting[] = {0x39, 0xF2,  9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x38, 0x04, 0xB0};
static unsigned char AID_ctrl[] = {0x39, 0xB2,  4, 0x04, 0xC3, 0x04, 0xC3};//fixme for debug purpose.
static const unsigned char source_ctrl[] = {0x39, 0xB9,  2, 0x36, 0x15};
static const unsigned char pentile_ctrl[] = {0x39, 0xC1,  2, 0x00, 0x20};
static const unsigned char power_ctrl[] = {0x39, 0xF4,  2, 0x02, 0x10};
static const unsigned char VLIN1_ctrl[] = {0x39, 0xB8,  7, 0x00, 0x5A, 0x74, 0x03, 0x00, 0x00, 0x19};
static const unsigned char LTPS_update[] = {0x15, 0xF7,  1, 0x01};
static const unsigned char level2_key_lock[] = {0x39, 0xF0,  2, 0xA5, 0xA5};
static const unsigned char dimming_speed_setting[] = {0x15, 0x53,  1, LCM_BACKLIGHT_CHANGE_SPEED};
static const unsigned char luminance_setting[] = {0x15, 0x51,  1, LCM_DEFAULT_BACKLIGHT_LEVEL};
static const unsigned char level3_key_unlock[] = {0x39, 0xFC,  2, 0x5A, 0x5A};
static const unsigned char global_para[] = {0x15, 0xB0,  1, 0x05};
static const unsigned char OTP_loading[] = {0x15, 0xD1,  1, 0x80};
static const unsigned char Sleep_Out[] = {0x05, 0x11,  0};
static const unsigned char wait_150ms[] = {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 1, 150};
static const unsigned char level3_key_lock [] = {0x39, 0xFC,  2, 0xA5, 0xA5};
static const unsigned char set_max_ret_size[] = {0x37, 0x25,  0};
static const unsigned char read_Gamma[] = {0x06, 0xC8,  0};
static const unsigned char OTP_key_unlock[] = {0x39, 0xF1,  2, 0x5A, 0x5A};
static const unsigned char TE_pin_normal_low[] = {0x15, 0xCC,  1, 0x0A};
static const unsigned char OTP_key_lock[] = {0x39, 0xF1,  2, 0xA5, 0xA5};

/*
 * Note :
 *
 * Structure Format :
 *
 * {Data_ID, DCS command, count of parameters, {parameter list}},
 *
 * {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, milliseconds of time, {}},
 *
 * ...
 *
 */
static const dsi_cmd_raw lcm_initialization_setting[] = {

    //step 10: common setting & Brightness Control
	/* level2 key unlock */
    DSI_CMD_RAW_BUILDER(level2_key_unlock),

	/* level3 key unlock */
    DSI_CMD_RAW_BUILDER(level3_key_unlock),

	/* resolution setting */
    DSI_CMD_RAW_BUILDER(resolution_setting),

	/* AID ctrl */
    DSI_CMD_RAW_BUILDER(AID_ctrl),

	/* source ctrl */
    DSI_CMD_RAW_BUILDER(source_ctrl),

	/* pentile ctrl */
    DSI_CMD_RAW_BUILDER(pentile_ctrl),

	/* power ctrl */
    DSI_CMD_RAW_BUILDER(power_ctrl),

	/* VLIN1 ctrl */
    DSI_CMD_RAW_BUILDER(VLIN1_ctrl),

	/* LTPS update */
    DSI_CMD_RAW_BUILDER(LTPS_update),

	/* level2 key lock */
    DSI_CMD_RAW_BUILDER(level2_key_lock),

	/* level3 key lock */
	//{0x39, 0xFC,  2, {0xA5, 0xA5}},

	/* dimming speed setting */
    DSI_CMD_RAW_BUILDER(dimming_speed_setting),

	/* luminance setting */
    DSI_CMD_RAW_BUILDER(luminance_setting),

    //step 11: OTP Loading set control
	/* level3 key unlock */
    //DSI_CMD_RAW_BUILDER(level3_key_unlock),

    /*global para*/
    DSI_CMD_RAW_BUILDER(global_para),

    /*OTP loading*/
    DSI_CMD_RAW_BUILDER(OTP_loading),

	//{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 50, {0x00}},

	//step 12: Sleep Out
    DSI_CMD_RAW_BUILDER(Sleep_Out),

	//step 13: wait > 120ms --> rgb data */
    DSI_CMD_RAW_BUILDER(wait_150ms),

    //step 14: Frame(RGB data), before 1 frame.
    //fixme.

    //step15: level 3 lock key set
    //fixme.
	/* level3 key lock */
    DSI_CMD_RAW_BUILDER(level3_key_lock),

    //read Gamma offset (C8h)
    DSI_CMD_RAW_BUILDER(set_max_ret_size),
    DSI_CMD_RAW_BUILDER(read_Gamma),

    //step16: ERR FLAG Control
    /*OTP key unlock*/
    DSI_CMD_RAW_BUILDER(OTP_key_unlock),
    /*Global para*/
    DSI_CMD_RAW_BUILDER(global_para),
    /*TE pin normal low*/
    DSI_CMD_RAW_BUILDER(TE_pin_normal_low),
    /*OTP key lock*/
    DSI_CMD_RAW_BUILDER(OTP_key_lock),

    //step17: Setting for porch change(optional)
    //fixme

    //step18: Display ON
	//{0x05, 0x29,  0, {0x00}},

    //step19: Display On Status.
	//{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 5, {0x00}},
};

static const unsigned char Display_ON[] = {0x05, 0x29,  0};
static const unsigned char wait_5ms[] = {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 1, 5};

static const dsi_cmd_raw lcm_display_on_setting[] = {
    //step18: Display ON
    DSI_CMD_RAW_BUILDER(Display_ON),

    //step19: Display On Status.
    DSI_CMD_RAW_BUILDER(wait_5ms),
};

static const unsigned char Display_off[] = {0x05, 0x28,  0};
static const unsigned char wait_50ms[] = {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 1, 50};
static const unsigned char sleep_in[] = {0x05, 0x10,  0};

static const dsi_cmd_raw lcm_sleep_in_sequence_setting[] = {
	/* step1: Display off */
    DSI_CMD_RAW_BUILDER(Display_off),

	/* step2: wait > 35ms */
    DSI_CMD_RAW_BUILDER(wait_50ms),

	/* step3: sleep in */
    DSI_CMD_RAW_BUILDER(sleep_in),

	/* step4: wait > 120ms */
    DSI_CMD_RAW_BUILDER(wait_150ms),

    /* step5: sync pkt stop(HS - LP11) */
    //fixme
};

static unsigned char dimming_speed[] = {0x15, 0x53,  1, LCM_BACKLIGHT_CHANGE_SPEED};
static unsigned char luminance[] = {0x15, 0x51,  1, LCM_DEFAULT_BACKLIGHT_LEVEL};

static const dsi_cmd_raw lcm_backlight_level_setting[] = {
	/* dimming speed setting */
    DSI_CMD_RAW_BUILDER(dimming_speed),

	/* luminance setting */
    DSI_CMD_RAW_BUILDER(luminance)
};

static const unsigned char wait_40ms[] = {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 1, 40};
static const unsigned char wait_30ms[] = {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS, 1, 30};

static const dsi_cmd_raw lcm_display_off_sequence_setting[] = {
    //step2: display off
    DSI_CMD_RAW_BUILDER(Display_off),
    //step3: wait 2 frame
    //fixme
    DSI_CMD_RAW_BUILDER(wait_40ms),

    //step4: sleep in
    DSI_CMD_RAW_BUILDER(sleep_in),

    //step5: wait > 120ms
    DSI_CMD_RAW_BUILDER(wait_150ms),
    //step6: sync packet stop (HS-->LP11-->Off)//waiting these cmds will be sent finish?
    //fixme
    DSI_CMD_RAW_BUILDER(wait_30ms),
};

static const dsi_cmd_raw lcm_wait_50ms_setting[] = {
    DSI_CMD_RAW_BUILDER(wait_50ms),
};

static int8_t gamma_compensation_setting[33] = {
    0, 0, 0,
    0, 0, 0,
    0, 1, 0,
    0, -1, 0,
    -1, -1, -1,
    0, -1, 0,
    -1, -2, -1,
    -2, -2, -2,
    -9, -9, -10,
    0, 3, 3,
    0, 0, 0
};

static unsigned char gamma_setting[3 + sizeof(gamma_compensation_setting)] = {0x39, 0xC8,  0};//if read gamma during initialization, paralist count will be 33.

static const dsi_cmd_raw write_gamma_setting[] = {
    /*OTP key unlock*/
    DSI_CMD_RAW_BUILDER(OTP_key_unlock),
    DSI_CMD_RAW_BUILDER(gamma_setting),
    DSI_CMD_RAW_BUILDER(LTPS_update),
    /*OTP key lock*/
    DSI_CMD_RAW_BUILDER(OTP_key_lock)
};

/* ***************************************************************************
 * 	Local Functions
 * ***************************************************************************/
static void lcm_mdelay(uint32_t ms)
{
	HAL_Delay(ms);
}

#define lcm_cmdq_push(pdata, queue_size)					do {\
	if (lcm_dsi_funcs && lcm_dsi_funcs->dsi_set_cmdq)			\
		lcm_dsi_funcs->dsi_set_cmdq((pdata), (queue_size));		\
} while(0)

static unsigned int lcm_read_reg_v2(unsigned char cmd, unsigned char *buffer, unsigned char buffer_size)
{
	if (lcm_dsi_funcs && lcm_dsi_funcs->dsi_dcs_read_reg_v2)
		return lcm_dsi_funcs->dsi_dcs_read_reg_v2(cmd, buffer, buffer_size);
	else
		return -1;
}

static void lcm_set_reset_pin(int set)
{
    if(XA0n == pcb_id){
		if (!set) {
			HAL_GPIO_WritePin(LCM_L_RST_PIN_PORT, LCM_L_RST_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCM_R_RST_PIN_PORT, LCM_R_RST_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCM_L_RST_PIN_PORT, LCM_L_RST_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LCM_R_RST_PIN_PORT, LCM_R_RST_PIN, GPIO_PIN_RESET);
		}
	}
    else{
		if (!set) {
			HAL_GPIO_WritePin(LCM_XB_L_RST_PIN_PORT, LCM_XB_L_RST_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCM_XB_R_RST_PIN_PORT, LCM_XB_R_RST_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCM_XB_L_RST_PIN_PORT, LCM_XB_L_RST_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LCM_XB_R_RST_PIN_PORT, LCM_XB_R_RST_PIN, GPIO_PIN_RESET);
		}
    }
}

static void lcm_set_power_en(int enable)
{
	if (enable) {
		/* turn on VDDI 1.8V(from PMIC BUCK2 1.8V) --> VCI 3.3V(from PMIC LDO3/4 3.3V) --> VBAT 3.8V(IC to DC/DC AVDD_EN& EL_EN, then DC/DC to IC VBAT) */
		bsp_pmic_power_enable(V_VDDI_L_EN, 1);	/* PMIC turn on BUCK2 and enable V_VDDI_L_EN for L panel VDDI power on */
		bsp_pmic_power_enable(V_VDDI_R_EN, 1);	/* PMIC turn on BUCK2 and enable V_VDDI_R_EN for R panel VDDI power on */
		bsp_pmic_power_enable(V_VCI_3V3_L, 1);	/* PMIC turn on LDO3 for L panel VCI power on */
		bsp_pmic_power_enable(V_VCI_3V3_R, 1);	/* PMIC turn on LDO4 for R panel VCI power on */
	} else {
		/* turn off VCI 3.3V(from PMIC LDO3/4 3.3V) --> VDDI 1.8V(from PMIC BUCK2 1.8V) */
		bsp_pmic_power_enable(V_VCI_3V3_L, 0);	/* PMIC turn off LDO3 for L panel VCI power off */
		bsp_pmic_power_enable(V_VCI_3V3_R, 0);	/* PMIC turn off LDO4 for R panel VCI power off */
		bsp_pmic_power_enable(V_VDDI_L_EN, 0);	/* PMIC turn off BUCK2 and disable V_VDDI_L_EN for L panel VDDI power off */
		bsp_pmic_power_enable(V_VDDI_R_EN, 0);	/* PMIC turn off BUCK2 and disable V_VDDI_R_EN for R panel VDDI power off */
	}
}

static void lcm_gpio_configure(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    if(XA0n == pcb_id){
		/* config left panel reset pin direction */
		GPIO_InitStruct.Pin = LCM_L_RST_PIN;
		HAL_GPIO_Init(LCM_L_RST_PIN_PORT, &GPIO_InitStruct);
		/* config right panel reset pin direction */
		GPIO_InitStruct.Pin = LCM_R_RST_PIN;
		HAL_GPIO_Init(LCM_R_RST_PIN_PORT, &GPIO_InitStruct);
	}
    else{
		GPIO_InitStruct.Pin = LCM_XB_L_RST_PIN;
		HAL_GPIO_Init(LCM_XB_L_RST_PIN_PORT, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = LCM_XB_R_RST_PIN;
		HAL_GPIO_Init(LCM_XB_R_RST_PIN_PORT, &GPIO_InitStruct);
    }

    lcm_set_reset_pin(1);

    lcm_set_power_en(0);
}

/* lcm task function, implement for ESD recover & CMD_MODE & test command in the future */
#ifdef LCM_MONITOR_TASK
void lcm_task_entry(void const * argument)
{
	/* Infinite loop */
	for(;;) {
		osDelay(1000);
	}
}
#endif

/* ***************************************************************************
 *	LCM Driver Implementations
 * ***************************************************************************/
static void lcm_power_on(void)
{
	//lcm power on sequence
	//step1:power off status
	//step2:power on
	lcm_set_power_en(1);
	lcm_mdelay(50); // > 25ms + 10ms

	lcm_set_reset_pin(0);
	/* wait > 10us */
	lcm_mdelay(5);
	lcm_set_reset_pin(1);
	/* wait > 10ms --> sync pkt start(HS) --> wait 1 frame(1/fps)*/
	lcm_mdelay(50);
}

static void lcm_power_off(void)
{
    //step1: display on status
    lcm_cmdq_push(lcm_display_off_sequence_setting, ARY_SIZE(lcm_display_off_sequence_setting));
    //step7: reset off (H-->L)
    lcm_set_reset_pin(0);
    lcm_mdelay(150);
    //step8: system power off
    lcm_set_power_en(0);
    lcm_mdelay(150);
}

static void lcm_sleep_out_sequence(void)
{
    /* step1: sync pkt start(LP11 - HS) */
    //fixme
    /* step2: wait > 20 ms */
    lcm_cmdq_push(lcm_wait_50ms_setting, ARY_SIZE(lcm_wait_50ms_setting));
    /* step3: common settings & brightness ctrl */
    /* step4: OTP Loading set control */
    /* step5: Sleep out */
    /* step6: wait > 120 ms */
    /* step7: Frame(RGB data), before 1 frame. */
    /* step8: level 3 lock key set */
    /* step9: ERR FLAG Control */
    /* step10: Setting for porch change(optional) */
    lcm_cmdq_push(lcm_initialization_setting, ARY_SIZE(lcm_initialization_setting));

    /* step11: display on */
    lcm_cmdq_push(lcm_display_on_setting, ARY_SIZE(lcm_display_on_setting));
}

static void lcm_set_dsi_funcs(dsi_funcs *funcs)
{
	if (NULL == funcs)
		lcm_log(HLOG_LVL_ERR, "%s: funcs is null\n", __func__);
	lcm_dsi_funcs = funcs;
}

static void lcm_get_params(lcm_params *params)
{
		memset(params, 0, sizeof(lcm_params));

		params->width  = LCM_WIDTH;
		params->height = LCM_HEIGHT;

		params->vertical_sync_active				= 2;
		params->vertical_backporch					= 52;
		params->vertical_frontporch					= 364;
		params->vertical_active_line				= LCM_HEIGHT;

		params->horizontal_sync_active				= 10;
		params->horizontal_backporch				= 58;
		params->horizontal_frontporch				= 310;
		params->horizontal_active_pixel				= LCM_WIDTH;

		params->format								= LCM_DSI_FORMAT_RGB888;
		params->lane_num							= LCM_DSI_FOUR_LANE;

		params->fps									= 90;
}

static void lcm_init(void)
{
#ifdef LCM_MONITOR_TASK
	/* definition and creation of touch_task */
	osThreadDef(lcm_task, lcm_task_entry, osPriorityNormal, 0, 256);
	lcm_task_handle = osThreadCreate(osThread(lcm_task), NULL);
#endif
}

static void lcm_suspend(void)
{
	lcm_cmdq_push(lcm_sleep_in_sequence_setting, ARY_SIZE(lcm_sleep_in_sequence_setting));
}

static void lcm_resume(void)
{
    lcm_sleep_out_sequence();
}

static void lcm_set_backlight(unsigned char level)
{
    luminance[3] = level;

	/* Refresh value of backlight level. */
	lcm_cmdq_push(lcm_backlight_level_setting, ARY_SIZE(lcm_backlight_level_setting));
}

static void lcm_initialize(void)
{
	/* common&brightness setting --> otp loading --> Sleep out sequence */
	lcm_cmdq_push(lcm_initialization_setting, ARY_SIZE(lcm_initialization_setting));
}

static void lcm_display_on(void)
{
	lcm_cmdq_push(lcm_display_on_setting, ARY_SIZE(lcm_display_on_setting));
}

static void lcm_get_id(void)
{
	unsigned char ID[3] = {0};

	lcm_read_reg_v2(0xDA, ID, 3);
	lcm_log(HLOG_LVL_INFO, "%s: ID1 = %#x, ID2 = %#x, ID3 = %#x\n", __func__, ID[0], ID[1], ID[2]);
}

void disp_read_comp(void)
{
    int i = 0;

    for (i = 0; i < sizeof(gamma_compensation_setting)/3; i++) {
        disp_warning("%d ", gamma_compensation_setting[i*3]);
        printf("%d ", gamma_compensation_setting[i*3+1]);
        printf("%d.\n", gamma_compensation_setting[i*3+2]);
    }
}

static int check_args(char *args)
{
    int i = 0, j = 0;
    int cnt = 0;
    char seg[8];

    while (1) {
        if(args[i] == '#' || args[i] == '/' || args[i] == 0) {
            cnt++;
            j = 0;
        } else {
            j++;
            if (j > sizeof(seg) - 1) {
                disp_err("invalid args.\n");
                return 0;
            }
        }

        if (0 == args[i]) {
            break;
        }

        i++;
    }

    return cnt;
}

static void write_args(char * args, int8_t *setting)
{
    int i = 0, j = 0;
    int cnt = 0;
    char seg[8];

    while (1) {
        seg[j] = args[i];
        if(args[i] == '#' || args[i] == '/' || args[i] == 0) {
            seg[j] = 0;
            j = 0;
            setting[cnt] = strtol(seg, 0, 0);
            cnt++;
        } else {
            j++;
        }

        if (0 == args[i]) {
            break;
        }

        i++;
    }
}

void disp_write_comp(char * comps)
{
    int cnt = 0;

    disp_debug("disp_write_comp %s.\n", comps);
    //check args
    cnt = check_args(comps);
    if (cnt != sizeof(gamma_compensation_setting)) {
        disp_err("invalid compensation offset num %d", cnt);
        return;
    }
    write_args(comps, gamma_compensation_setting);
}

void disp_read_aid(void)
{
    disp_warning("AID_ctrl 0x%02x 0x%02x 0x%02x 0x%02x \n", AID_ctrl[3], AID_ctrl[4], AID_ctrl[5], AID_ctrl[6]);
}

void disp_write_aid(char * aids)
{
    int cnt = 0;

    disp_debug("disp_write_AID_ctrl %s.\n", aids);
    //check args
    cnt = check_args(aids);
    if (cnt != sizeof(AID_ctrl) - 3) {
        disp_err("invalid AID_ctrl args num %d", cnt);
        return;
    }
    write_args(aids, (int8_t*)&AID_ctrl[3]);
}

static void init_gamma_setting(int x4idx, uint32_t value)
{
    int i = 0;

    dsi_cmd_t *dsi_cmd = (dsi_cmd_t *)gamma_setting;

    for (i = 0; i < sizeof(value); i++) {
        if (i + x4idx*4 >= sizeof(gamma_compensation_setting)) {
            break;
        }
        dsi_cmd->para[i + x4idx*4] = (value>>(8*i) & 0xFF) + gamma_compensation_setting[i + x4idx*4];
        dsi_cmd->count = i + x4idx*4 + 1;
    }
}

static void update_gamma_setting(void)
{
    lcm_cmdq_push(write_gamma_setting, sizeof(write_gamma_setting)/sizeof(write_gamma_setting[0]));
}

#ifdef LCM_DSI_CMD_MODE
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
}
#endif

lcm_drv_t ams361ep01_dsi_vdo_lcm_drv = {
	.name			= "ams361ep01_dsi_vdo_lcm_drv",
	.set_dsi_funcs	= lcm_set_dsi_funcs,
	.get_params		= lcm_get_params,
    .configure      = lcm_gpio_configure,
    .power_en       = lcm_set_power_en,
    .set_reset_pin  = lcm_set_reset_pin,
	.init			= lcm_init,
	.suspend		= lcm_suspend,
	.resume			= lcm_resume,
	.set_backlight	= lcm_set_backlight,
    .initialize     = lcm_initialize,
    .display_on     = lcm_display_on,
    .power_off      = lcm_power_off,
    .power_on       = lcm_power_on,
	.get_id			= lcm_get_id,
    .init_gamma     = init_gamma_setting,
    .update_gamma   = update_gamma_setting,
#ifdef LCM_DSI_CMD_MODE
	.update			= lcm_update,
#endif
};

/* ---------------------------------------------------------------------------
 * Get LCM Driver Hooks
 * ---------------------------------------------------------------------------*/
const lcm_drv_t* LCM_GetDriver(void)
{
	return &ams361ep01_dsi_vdo_lcm_drv;
}
