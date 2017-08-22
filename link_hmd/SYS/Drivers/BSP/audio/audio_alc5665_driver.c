#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "hlog_api.h"

#include "rtos_i2c_drv.h"
#include "audio_alc5665_driver.h"

#include <stdio.h>
#include <string.h>

#include "x_pmic.h"
#include "gpio_exp.h"
#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"
#include "misc_data.h"

#define ALC5665_I2C_RETRY_MAX_COUNT	(10)
#define ALC5665_I2C_TIMEOUT_MAX		0x1000

#define ALC5665_POWER_CONTROL_PIN	IOEXP_AUD_CDC_LDO1_EN

struct __alc5665_data {
	unsigned char is_initialized;
	unsigned char power_on;
	unsigned char amp_power_on;
	int (*power_control)(unsigned char);
	int (*amp_power_control)(unsigned char);
	const struct __alc5665_reg *init_reg_list;
	int init_reg_list_count;
	const struct __alc5665_reg *cal_reg_list;
	int cal_reg_list_count;
	const struct __alc5665_reg *hdmi_reg_list;
	int hdmi_reg_list_count;
	const struct __alc5665_reg *usb_reg_list;
	int usb_reg_list_count;
	const struct __alc5665_reg *usb_plug_list;
	int usb_plug_list_count;
	const struct __alc5665_reg *usb_unplug_list;
	int usb_unplug_list_count;
	const struct __alc5665_reg *loopback_list;
	int loopback_list_count;
	const struct __alc5665_reg *lout_mute_list;
	int lout_mute_list_count;
	const struct __alc5665_reg *lout_unmute_list;
	int lout_unmute_list_count;
	struct pwrmgr_notify_func_data pm_notify_data;
	enum __audio_5665_path path;
};

static struct __alc5665_data g_alc5665_data;

static const struct __alc5665_reg alc5665_evm_init_list[] = {
	/* Playback Event */
	{ ALC5665_RESET, 0x0000 },
	{ ALC5665_HP_IMP_GAIN_1, 0x5757 },
	{ ALC5665_STO1_ADC_MIXER, 0x6064 },
	{ ALC5665_AD_DA_MIXER, 0x8580 },
	{ ALC5665_GLB_CLK, 0x4200 },
	{ ALC5665_STO1_DAC_MIXER, 0x2A8A },
	{ ALC5665_MICBIAS_2, 0x0180 },
	{ ALC5665_DIG_MISC, 0x0001 },
	/* PLL setup */
	{ ALC5665_PLL_CTRL_1, 0x0F02 },
	{ ALC5665_PLL_CTRL_2, 0x0801 },
	/* Mute */
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	/* Power on */
	{ ALC5665_PWR_DIG_1, 0xA118 },
	{ ALC5665_PWR_DIG_2, 0x8400 },
	{ ALC5665_PWR_ANLG_1, 0xAA7E },
	{ ALC5665_PWR_ANLG_1, 0xFE3E },
	{ ALC5665_PWR_ANLG_2, 0x4E43 },
	{ ALC5665_PWR_ANLG_3, 0x0240 },
	/* HP fine tune */
	{ ALC5665_BIAS_CUR_CTRL_8, 0xA602 },
	{ ALC5665_PWR_ANLG_1, 0xF23F },
	{ ALC5665_CHARGE_PUMP_1, 0x0410 },
	{ ALC5665_HP_CHARGE_PUMP_1, 0x0E26 },
	{ ALC5665_PWR_ANLG_2, 0x0008 },
	{ ALC5665_RC_CLK_CTRL, 0xC000 },
	{ ALC5665_GPIO_CTRL_1, 0x8000 },
	{ ALC5665_IRQ_CTRL_1, 0x0008 },
	{ ALC5665_IRQ_CTRL_6, 0x2000 },
	/* MCLK HP */
	{ ALC5665_CHOP_DAC, 0x3030 },
	{ ALC5665_ADDA_CLK_1, 0x1100 },
	{ ALC5665_I2S1_SDP, 0x8000 },
	{ ALC5665_I2S2_SDP, 0x8000 },
	{ ALC5665_HP_LOGIC_CTRL_2, 0x0003 },
	{ ALC5665_HP_CTRL_2, 0xC000 },
	{ ALC5665_STO_NG2_CTRL_1, 0x4EFF },
	{ ALC5665_STO1_DAC_SIL_DET, 0xC1B1 },
	{ ALC5665_SIL_PSV_CTRL1, 0x8000 },
	{ ALC5665_SIL_PSV_CTRL5, 0x3000 },
	/* Reccord */
	{ ALC5665_IN1_IN2, 0x0008 },
	{ ALC5665_EJD_CTRL_1, 0x4060 },
	{ ALC5665_REC1_L2_MIXER, 0x00EF },
	{ ALC5665_REC1_R2_MIXER, 0x00EF },
	{ ALC5665_CHOP_ADC, 0x3030 },
	{ ALC5665_GPIO_CTRL_1, 0xAA80 },
	/* UnMute */
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
};

static const struct __alc5665_reg alc5665_evm_xc_init_list[] = {
	/* Playback Event */
	{ ALC5665_RESET, 0x0000 },
	{ ALC5665_LOUT, 0x4848 },
	{ ALC5665_EJD_CTRL_1, 0xD040 },
	{ ALC5665_HP_IMP_GAIN_1, 0x5757 },
	{ ALC5665_STO1_ADC_MIXER, 0xE0E4 },     // Recoder
	{ ALC5665_AD_DA_MIXER, 0x8580 },
	{ ALC5665_GLB_CLK, 0x4200 },
	{ ALC5665_STO1_DAC_MIXER, 0x2A8A },
	{ ALC5665_MONO_DAC_MIXER, 0x2A8A },
	{ ALC5665_A_DAC2_MUX, 0x0000 },
	{ ALC5665_MICBIAS_2, 0x0180 },
	{ ALC5665_DIG_MISC, 0x0001 },
	{ ALC5665_LOUT_MIXER, 0x3000 },         // unmute line out
	/* PLL setup */
	{ ALC5665_PLL_CTRL_1, 0x0F02 },
	{ ALC5665_PLL_CTRL_2, 0x0801 },
	/* Mute */
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	/* Power on */
	{ ALC5665_PWR_DIG_1, 0xA1D8 },
	{ ALC5665_PWR_DIG_2, 0x8400 },
	{ ALC5665_PWR_ANLG_1, 0xAA7E },
	{ ALC5665_PWR_ANLG_1, 0xFF3E},
	{ ALC5665_PWR_ANLG_2, 0x4E43 },
	{ ALC5665_PWR_ANLG_3, 0x0240 },
	{ ALC5665_PWR_MIXER, 0x303F },
	/* HP fine tune */
	{ ALC5665_BIAS_CUR_CTRL_8, 0xA602 },
	{ ALC5665_PWR_ANLG_1, 0xF23F },
	{ ALC5665_CHARGE_PUMP_1, 0x0410 },
	{ ALC5665_HP_CHARGE_PUMP_1, 0x0E26 },
	{ ALC5665_PWR_ANLG_2, 0x0008 },
	{ ALC5665_RC_CLK_CTRL, 0xC000 },
	{ ALC5665_GPIO_CTRL_1, 0x8000 },
	{ ALC5665_IRQ_CTRL_1, 0x0008 },
	{ ALC5665_IRQ_CTRL_6, 0x2000 },
	/* LINE OUT FINE TUNE */
	{ ALC5665_PWR_ANLG_1, 0xF33C },
	{ ALC5665_DEPOP_1, 0x0068 },
	{ ALC5665_CLK_DET, 0x0001 },
	{ ALC5665_TEST_MODE_CTRL_2, 0x001C },
	/* MCLK HP */
	{ ALC5665_CHOP_DAC, 0x3030 },
	{ ALC5665_ADDA_CLK_1, 0x1100 },
	{ ALC5665_I2S1_SDP, 0x8000 },
	{ ALC5665_I2S2_SDP, 0x8000 },
	{ ALC5665_HP_LOGIC_CTRL_2, 0x0003 },
	{ ALC5665_HP_CTRL_2, 0xC000 },
	{ ALC5665_STO_NG2_CTRL_1, 0x4EFF },
	{ ALC5665_STO1_DAC_SIL_DET, 0xC1B1 },
	{ ALC5665_SIL_PSV_CTRL1, 0x8000 },
	{ ALC5665_SIL_PSV_CTRL5, 0x3000 },
	/* Reccord */
	{ ALC5665_IN1_IN2, 0x0008 },
	{ ALC5665_EJD_CTRL_1, 0x4060 },
	{ ALC5665_REC1_L2_MIXER, 0x00EF },
	{ ALC5665_REC1_R2_MIXER, 0x00EF },
	{ ALC5665_CHOP_ADC, 0x3030 },
	{ ALC5665_GPIO_CTRL_1, 0xAA80 },
	/* UnMute */
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
};

static const struct __alc5665_reg alc5665_calibration_1_list[] = {
	/* Calibration DC */
	{ ALC5665_BIAS_CUR_CTRL_8, 0xA602 },
	{ ALC5665_HP_CHARGE_PUMP_1, 0x0C26 },
	{ ALC5665_MONOMIX_IN_GAIN, 0x021F },
	{ ALC5665_MONO_OUT, 0x480A },
	{ ALC5665_PWR_MIXER, 0x083F },
	{ ALC5665_PWR_DIG_1, 0x0180 },
	{ ALC5665_EJD_CTRL_1, 0x4040 },
	{ ALC5665_HP_LOGIC_CTRL_2, 0x0000 },
	{ ALC5665_DIG_MISC, 0x0001 },
	{ ALC5665_MICBIAS_2, 0x0380 },
	{ ALC5665_GLB_CLK, 0x8000 },
	{ ALC5665_ADDA_CLK_1, 0x1000 },
	{ ALC5665_CHOP_DAC, 0x3030 },
	{ ALC5665_CALIB_ADC_CTRL, 0x3C05 },
	{ ALC5665_PWR_ANLG_1, 0xAA3E },
};

static const struct __alc5665_reg alc5665_calibration_50_list[] = {
	/* Calibration DC delay 50ms*/
	{ ALC5665_PWR_ANLG_1, 0xFE7E },
	{ ALC5665_HP_CALIB_CTRL_2, 0x0321 },
	{ ALC5665_HP_CALIB_CTRL_1, 0xFC00 },
};

static const struct __alc5665_reg alc5665_hdmi_path_list[] = {
	/* I2S2 */
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	{ ALC5665_AD_DA_MIXER, 0x8580 },
	{ ALC5665_GLB_CLK, 0x4200 },
	/* unmute */
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
	{ ALC5665_STO1_DAC_SIL_DET, 0xC1B1 },
};

static const struct __alc5665_reg alc5665_usb_path_list[] = {
	/* I2S1 */
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	{ ALC5665_AD_DA_MIXER, 0x8080 },
	{ ALC5665_GLB_CLK, 0x4100 },
	/* unmute */
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
	{ ALC5665_STO1_DAC_SIL_DET, 0xC1B1 },
};

static const struct __alc5665_reg alc5665_usb_plug_list[] = {
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
	{ ALC5665_STO1_DAC_SIL_DET, 0xC1B1 },
};

static const struct __alc5665_reg alc5665_usb_unplug_list[] = {
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	{ ALC5665_STO1_DAC_SIL_DET, 0x8000 },
};

static const struct __alc5665_reg alc5665_loopback_list[] = {
	/* I2S1 */
	{ ALC5665_DAC1_DIG_VOL, 0x0000 },
	{ ALC5665_GLB_CLK, 0x4100 },
	/* unmute */
	{ ALC5665_DAC1_DIG_VOL, 0x9797 },
	/* loopback */
	{ ALC5665_STO1_DAC_SIL_DET, 0x41BD },
	{ ALC5665_AD_DA_MIXER, 0x0000 },
};

static const struct __alc5665_reg alc5665_lout_mute_list[] = {
	/* LOUT MUTE */
	{ ALC5665_LOUT, 0xC8C8 },
	{ ALC5665_LOUT_MIXER, 0xF000 },
};

static const struct __alc5665_reg alc5665_lout_unmute_list[] = {
	/* LOUT MUTE */
	{ ALC5665_LOUT, 0x4848 },
	{ ALC5665_LOUT_MIXER, 0x3000 },
};

int audio_alc5665_i2c_read(uint16_t reg, uint16_t *value)
{
	int retry_count = 0;
	uint8_t priv_data[2] = { 0 };
	// struct __alc5665_data *data = &g_alc5665_data;
	I2C_STATUS ret = I2C_OK;
	while (retry_count++ < ALC5665_I2C_RETRY_MAX_COUNT) {
		ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_AUDIOCODEC_ADDR, reg,
		I2C_MEMADD_SIZE_16BIT, priv_data, 2, ALC5665_I2C_TIMEOUT_MAX );
		if (ret == I2C_OK)
			break;
	}
	/* Check the communication status */
	if (ret != I2C_OK) {
		/* Execute user timeout callback */
		audio_5665_err("%s: status %d, failed\n", __func__, ret);
		return -1;
	}
	*value = (priv_data[0] << 8) | priv_data[1];
	return 0;
}

int audio_alc5665_i2c_write(uint16_t reg, uint16_t value)
{
	int retry_count = 0;
	I2C_STATUS ret = I2C_OK;
	uint8_t priv_data[2];
	priv_data[0] = value >> 8;
	priv_data[1] = value & 0xFF;
	while (retry_count++ < ALC5665_I2C_RETRY_MAX_COUNT) {
		ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_AUDIOCODEC_ADDR, reg,
		I2C_MEMADD_SIZE_16BIT, priv_data, 2, ALC5665_I2C_TIMEOUT_MAX );
		if (ret == I2C_OK)
			break;
	}

	/* Check the communication status */
	if (ret != I2C_OK) {
		/* Execute user timeout callback */
		audio_5665_err("%s: status %d, failed\r\n", __func__, ret);
		return -1;
	}
	return 0;
}

static int alt5665_regs_set(const struct __alc5665_reg *regs, int regs_count)
{
	int i;
	int ret = 0;
	for (i = 0; i < regs_count; i++) {
		ret = audio_alc5665_i2c_write(regs[i].reg, regs[i].def);
		if (ret != 0) {
			audio_5665_err("%s: reg: 0x%4.4X, "
					"value: 0x%4.4X failed\n",
					__func__, regs[i].reg, regs[i].def);
			return ret;
		}
	}
	return ret;
}

static int alt5665_audio_init_reg(void)
{
	struct __alc5665_data *alc5665_data_p = &g_alc5665_data;
	if (alt5665_regs_set(alc5665_data_p->init_reg_list,
			alc5665_data_p->init_reg_list_count) != 0) {
		audio_5665_err("%s: loading default config failed, exit\r\n",
				__func__);
		return -1;
	}
	return 0;
}

static int alt5665_audio_calibration_reg(enum __audio_5665_cal_section section)
{
	struct __alc5665_data *alc5665_data_p = &g_alc5665_data;
	switch (section) {
	case CAL_SECTION_1:
		alc5665_data_p->cal_reg_list = alc5665_calibration_1_list;
		alc5665_data_p->cal_reg_list_count =
					sizeof(alc5665_calibration_1_list) /
					sizeof(alc5665_calibration_1_list[0]);
		break;
	case CAL_SECTION_50:
		alc5665_data_p->cal_reg_list = alc5665_calibration_50_list;
		alc5665_data_p->cal_reg_list_count =
					sizeof(alc5665_calibration_50_list) /
					sizeof(alc5665_calibration_50_list[0]);
		break;
	default:
		break;
	}
	if (alt5665_regs_set(alc5665_data_p->cal_reg_list,
		alc5665_data_p->cal_reg_list_count) != 0) {
		audio_5665_err("%s: loading default config failed, exit\r\n",
				__func__);
		return -1;
	}
	audio_5665_debug("%s: loading cal_%d config success\r\n",
							__func__, section);
	return 0;
}
static int audio_alc5665_amp_power_control(unsigned char onoff);
static int audio_alc5665_power_control(unsigned char onoff);
static int __audio_alc5665_power_supply(unsigned char onoff);

int alt5665_audio_hotplug_noise(uint32_t flag, uint32_t state, void *data)
{
	struct __alc5665_data *alc5665_data_p = &g_alc5665_data;
	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT) {
			//audio_5665_debug("%s: audio mute\r\n", __func__);
			audio_alc5665_amp_power_control(0);
            audio_alc5665_power_control(POWER_DISABLE);
			__audio_alc5665_power_supply(POWER_DISABLE);
            if (alt5665_regs_set(alc5665_data_p->usb_unplug_list,
				alc5665_data_p->usb_unplug_list_count) != 0) {
				audio_5665_err("%s: loading usb_unplug_list "
						"failed\r\n", __func__);
				return -1;
			}
		} else if (state == STOP_LEAVE || state == PLUG_IN){
			//audio_5665_debug("%s: audio unmute\r\n", __func__);
			audio_alc5665_amp_power_control(1);
            audio_alc5665_power_control(POWER_ENABLE);
			__audio_alc5665_power_supply(POWER_ENABLE);
			if (alt5665_regs_set(alc5665_data_p->usb_plug_list,
				alc5665_data_p->usb_plug_list_count) != 0) {
				audio_5665_err("%s: loading usb_plug_list "
						"failed\r\n", __func__);
				return -1;
			}
		}
	} else {
		audio_5665_err("%s: unsupported flag\r\n", __func__);
	}
	return 0;
}

static void audio_alc5665_power_init(void)
{
	ioexp_gpio_set_value(IOEXP_REG_DIR,
			ALC5665_POWER_CONTROL_PIN, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_DIR,
			IOEXP_I_AUD_HP_EN, IOEXP_GPIO_OUTPUT);

}

static int audio_alc5665_amp_power_control(unsigned char onoff)
{
	uint8_t pin_highlow;
	struct __alc5665_data *data = &g_alc5665_data;
	if (onoff)
		pin_highlow = IOEXP_GPIO_HIGH;
	else
		pin_highlow = IOEXP_GPIO_LOW;

	ioexp_gpio_set_value(IOEXP_REG_OUT,
			IOEXP_I_AUD_HP_EN, pin_highlow);

	data->amp_power_on = onoff;
	return 0;

}

int audio_alc5665_audio_amp_power_set(unsigned char onoff)
{
	int ret = 0;
	struct __alc5665_data *data = &g_alc5665_data;
	if (data->amp_power_on == onoff)
		return 0;

	if (data == NULL || data->amp_power_control == NULL) {
		audio_5665_err("%s: function not ready\r\n", __func__);
		return -1;
	}

	ret = data->amp_power_control(onoff);
	if (ret != 0) {
		audio_5665_err("%s: amp power control failed\r\n", __func__);
		return -1;
	}
	data->amp_power_on = onoff;
	return 0;
}

static int audio_alc5665_power_control(unsigned char onoff)
{
	uint8_t pin_highlow;
	struct __alc5665_data *data = &g_alc5665_data;
	if (onoff)
		pin_highlow = IOEXP_GPIO_HIGH;
	else
		pin_highlow = IOEXP_GPIO_LOW;

	ioexp_gpio_set_value(IOEXP_REG_OUT,
			ALC5665_POWER_CONTROL_PIN, pin_highlow);

	if (onoff) {
		HAL_Delay(50);
		alt5665_audio_init_reg();
	}

	data->power_on = onoff;
	return 0;
}

int audio_alc5665_audio_power_set(unsigned char onoff)
{
	int ret = 0;
	struct __alc5665_data *data = &g_alc5665_data;
	if (data->power_on == onoff)
		return 0;

	if (data == NULL || data->power_control == NULL) {
		audio_5665_err("%s: function not ready\r\n", __func__);
		return -1;
	}

	ret = data->power_control(onoff);
	if (ret != 0) {
		audio_5665_err("%s: power control failed\r\n", __func__);
		return -1;
	}
	data->power_on = onoff;
	return 0;
}

void audio_alc5665_audio_power_get(unsigned char *onoff)
{
	struct __alc5665_data *data = &g_alc5665_data;
	*onoff = data->power_on;
}

void audio_alc5665_audio_amp_power_get(unsigned char *onoff)
{
	struct __alc5665_data *data = &g_alc5665_data;
	*onoff = data->amp_power_on;
}

int alt5665_audio_path_set(enum __audio_5665_path new_path)
{
	struct __alc5665_data *data = &g_alc5665_data;
	if (data->path == new_path)
		return 0;

	switch (new_path) {
	case AUDIO_5665_HDMI_PATH:
		audio_5665_warning("%s: audio path setup as HDMI\r\n", __func__);
		if (alt5665_regs_set(data->hdmi_reg_list,
					data->hdmi_reg_list_count) != 0) {
		      audio_5665_err("%s: change HDMI config failed, exit\r\n",
						__func__);
		      return -1;
		}
		break;
	case AUDIO_5665_USB_PATH:
		audio_5665_warning("%s: audio path setup as USB\r\n", __func__);
		if (alt5665_regs_set(data->usb_reg_list,
					data->usb_reg_list_count) != 0) {
		      audio_5665_err("%s: change USB config failed, exit\r\n",
						__func__);
		      return -1;
		}
		break;
	case AUDIO_5665_LOOPBACK_PATH:
		audio_5665_warning("%s: audio path setup as LOOPBACK\r\n",
								__func__);
		if (alt5665_regs_set(data->loopback_list,
					data->loopback_list_count) != 0) {
			audio_5665_err("%s: change Loopback config failed, "
							"exit\r\n", __func__);
		      return -1;
		}
		break;
	case AUDIO_5665_OUTPUT_TO_AJ:
		audio_5665_warning("%s: audio output setup as AJ\r\n",
								__func__);
		if (alt5665_regs_set(data->lout_mute_list,
					data->lout_mute_list_count) != 0) {
			audio_5665_err("%s: change output AJ config failed, "
							"exit\r\n", __func__);
		      return -1;
		}
		break;
	case AUDIO_5665_OUTPUT_TO_EAR:
		audio_5665_warning("%s: audio output setup as EAR\r\n",
								__func__);
		if (alt5665_regs_set(data->lout_unmute_list,
					data->lout_unmute_list_count) != 0) {
			audio_5665_err("%s: change output EAR config failed, "
							"exit\r\n", __func__);
		      return -1;
		}
		break;
	case AUDIO_5665_OUTPUT_TO_BOTH:
		audio_5665_warning("%s: audio output setup as BOTH\r\n",
								__func__);
		if (alt5665_regs_set(data->lout_unmute_list,
					data->lout_unmute_list_count) != 0) {
			audio_5665_err("%s: change output BOTH config failed, "
							"exit\r\n", __func__);
		      return -1;
		}
		break;
	case AUDIO_5665_UNSET_PATH:
	default:
		audio_5665_err("%s: incorrect audio path setup\r\n", __func__);
		return -1;
		break;
	};
	data->path = new_path;
	return 0;
}

static int audio_alc5665_check_pid_vid(struct __alc5665_data *data)
{
	int ret = 0;
	uint16_t temp_data = 0;
	audio_alc5665_i2c_read(ALC5665_DEVICE_ID, &temp_data);
	if (temp_data != ALC5665_6_8_DEVICE_ID)
		ret = -1;

	return ret;
}

static int __audio_alc5665_power_supply(unsigned char onoff)
{
	if (onoff) {
		if (bsp_pmic_power_enable(V_VDD_1V8, POWER_ENABLE) != 0) {
			audio_5665_err("%s: power supply V_VDD_1V8 failed\r\n",
								__func__);
			return -1;
		}
		if (bsp_pmic_power_enable(V_AUD_3V3, POWER_ENABLE) != 0) {
			audio_5665_err("%s: power supply V_AUD_3V3 failed\r\n",
								__func__);
			return -1;
		}
	} else {
		if (bsp_pmic_power_enable(V_VDD_1V8, POWER_DISABLE) != 0) {
			audio_5665_err("%s: power supply V_VDD_1V8 failed\r\n",
								__func__);
			return -1;
		}
		if (bsp_pmic_power_enable(V_AUD_3V3, POWER_DISABLE) != 0) {
			audio_5665_err("%s: power supply V_AUD_3V3 failed\r\n",
								__func__);
			return -1;
		}
	}
	return 0;
}

static int audio_pm_notify_create(struct __alc5665_data *data)
{
	struct pwrmgr_notify_func_data *pm_not_p = &data->pm_notify_data;

	pm_not_p->func_name = "audio_drv";
	pm_not_p->data = data;
	pm_not_p->callback = alt5665_audio_hotplug_noise;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_MAX_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}


int audio_alc5665_driver_init(void)
{
	int ret = 0;
	pcbid_t pcb_id;
	int time_out = 100;
	uint16_t temp_data = 0;

	if (__audio_alc5665_power_supply(POWER_ENABLE) != 0) {
		audio_5665_err("%s: power supply failed\r\n", __func__);
		return -1;
	}

	struct __alc5665_data *alc5665_data_p = &g_alc5665_data;
	if (alc5665_data_p->is_initialized == 1) {
		audio_5665_err("%s: driver already initialized, exit\r\n",
								__func__);
		return ret;
	}
	memset(&g_alc5665_data, 0x0, sizeof(g_alc5665_data));

	audio_alc5665_power_init();
	audio_alc5665_power_control(POWER_ENABLE);

	if (audio_alc5665_check_pid_vid(alc5665_data_p) != 0) {
		audio_alc5665_power_control(POWER_DISABLE);
		audio_5665_err("%s: incorrect id, exit\r\n", __func__);
		return -1;
	}

	alc5665_data_p->power_control = audio_alc5665_power_control;
	alc5665_data_p->amp_power_control = audio_alc5665_amp_power_control;

	get_pcbid(&pcb_id);
	if ((pcb_id == XC01) || (pcb_id == XC02)) {
		alc5665_data_p->init_reg_list = alc5665_evm_xc_init_list;
		alc5665_data_p->init_reg_list_count =
					sizeof(alc5665_evm_xc_init_list) /
					sizeof(alc5665_evm_xc_init_list[0]);
		alc5665_data_p->lout_mute_list = alc5665_lout_mute_list;
		alc5665_data_p->lout_mute_list_count =
					sizeof(alc5665_lout_mute_list)
					/ sizeof(alc5665_lout_mute_list[0]);

		alc5665_data_p->lout_unmute_list = alc5665_lout_unmute_list;
		alc5665_data_p->lout_unmute_list_count =
					sizeof(alc5665_lout_unmute_list)
					/ sizeof(alc5665_lout_unmute_list[0]);
	} else if ((pcb_id == XB02) || (pcb_id == XA0n)){
		alc5665_data_p->init_reg_list = alc5665_evm_init_list;
		alc5665_data_p->init_reg_list_count =
					sizeof(alc5665_evm_init_list) /
					sizeof(alc5665_evm_init_list[0]);
	} else {
		alc5665_data_p->init_reg_list = alc5665_evm_init_list;
		alc5665_data_p->init_reg_list_count =
					sizeof(alc5665_evm_init_list) /
					sizeof(alc5665_evm_init_list[0]);
	}

	alc5665_data_p->hdmi_reg_list = alc5665_hdmi_path_list;
	alc5665_data_p->hdmi_reg_list_count = sizeof(alc5665_hdmi_path_list) /
				sizeof(alc5665_hdmi_path_list[0]);

	alc5665_data_p->usb_reg_list = alc5665_usb_path_list;
	alc5665_data_p->usb_reg_list_count = sizeof(alc5665_usb_path_list) /
				sizeof(alc5665_usb_path_list[0]);

	alc5665_data_p->usb_plug_list = alc5665_usb_plug_list;
	alc5665_data_p->usb_plug_list_count = sizeof(alc5665_usb_plug_list) /
				sizeof(alc5665_usb_plug_list[0]);

	alc5665_data_p->usb_unplug_list = alc5665_usb_unplug_list;
	alc5665_data_p->usb_unplug_list_count = sizeof(alc5665_usb_unplug_list)
					/ sizeof(alc5665_usb_unplug_list[0]);

	alc5665_data_p->loopback_list = alc5665_loopback_list;
	alc5665_data_p->loopback_list_count =
					sizeof(alc5665_loopback_list)
					/ sizeof(alc5665_loopback_list[0]);

	/* DC calibration */
	alt5665_audio_calibration_reg(CAL_SECTION_1);
	HAL_Delay(50);
	alt5665_audio_calibration_reg(CAL_SECTION_50);

	audio_alc5665_i2c_read(ALC5665_HP_CALIB_STA_1, &temp_data);
	while (time_out > 0 && ((temp_data >> 15) & 0x1) != 0) {
		HAL_Delay(10);
		audio_alc5665_i2c_read(ALC5665_HP_CALIB_STA_1, &temp_data);
		time_out--;
	}

	audio_5665_info("%s: calibrating timeout:(%d)\r\n", __func__, time_out);

	if (((temp_data >> 15) & 0x1) != 0) {
		audio_5665_err("%s: DC calibration failed, exit\r\n", __func__);
		return -1;
	}

	if (alt5665_audio_init_reg() != 0) {
		audio_5665_err("%s: initial registers failed, exit\r\n",
			       __func__);
		return -1;
	}

	if (audio_pm_notify_create(alc5665_data_p) != 0)
		audio_5665_err("%s: create pm notifier failed\n", __func__);

	alc5665_data_p->is_initialized = 1;
	audio_5665_debug("%s: driver initial successfully\n", __func__);
	return ret;
}
