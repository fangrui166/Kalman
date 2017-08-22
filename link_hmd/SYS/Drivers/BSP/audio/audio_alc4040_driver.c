#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "hlog_api.h"

#include "audio_alc4040_driver.h"

#include "gpio_exp.h"

/* EVM1 board path switch setting:
 * IOEXP GPIO08: for audio ALC4040 path switch, high is USB, low is HDMI
 * IOEXP GPIO16: for audio ALC5665 power controll, should be always HIGH
 */
#define ALC4040_EVM_AUDIO_PATH_SWITCH_PIN	IOEXP_ALC4040_GPIO_AL1_1


struct audio_alc4040_data {
	unsigned char is_initialized;
	enum __audio_4040_path path;
	int (*path_switch_control)(enum __audio_4040_path);
};
static struct audio_alc4040_data g_alc4040_data = { 0 };

static void evm_path_switch_control_init(void)
{
	ioexp_gpio_set_value(IOEXP_REG_DIR,
			ALC4040_EVM_AUDIO_PATH_SWITCH_PIN,
			IOEXP_GPIO_OUTPUT);
}


static int evm_path_switch_control(enum __audio_4040_path n_path)
{
	unsigned char pin_highlow;
	struct audio_alc4040_data *data = &g_alc4040_data;

	if (n_path != AUDIO_4040_HDMI_PATH && n_path != AUDIO_4040_USB_PATH) {
		audio_4040_warning("%s: incorrect path\r\n", __func__);
		return -1;
	}
	if (n_path == AUDIO_4040_HDMI_PATH)
		pin_highlow = IOEXP_GPIO_HIGH;
	else /* n_path == AUDIO_4040_USB_PATH */
		pin_highlow = IOEXP_GPIO_LOW;

	ioexp_gpio_set_value(IOEXP_REG_OUT,
			ALC4040_EVM_AUDIO_PATH_SWITCH_PIN, pin_highlow);

	data->path = n_path;
	return 0;
}


int audio_alc4040_audio_path_set(enum __audio_4040_path new_path)
{
	struct audio_alc4040_data *data = &g_alc4040_data;

	if (data->is_initialized == 0)
		return 0;

	if (data->path == new_path)
		return 0;

	if (data == NULL || data->path_switch_control == NULL) {
		audio_4040_err("%s: function not ready\r\n", __func__);
		return -1;
	}

	switch (new_path) {
	case AUDIO_4040_HDMI_PATH:
	case AUDIO_4040_USB_PATH:
		if (data->path_switch_control(new_path) == 0) {
			audio_4040_warning("%s: audio path setup as %s\r\n",
				__func__, (new_path == AUDIO_4040_HDMI_PATH) ?
				"HDMI" : "USB");
		} else {
			audio_4040_err("%s: audio path setup failed\r\n",
								__func__);
			return -2;
		}
		break;
	case AUDIO_4040_UNSET_PATH:
	default:
		audio_4040_err("%s: incorrect audio path setup\r\n", __func__);
		return -3;
		break;
	};
	data->path = new_path;
	return 0;
};



int audio_alc4040_driver_init()
{
	int ret = 0;
	struct audio_alc4040_data *alc4040_data = &g_alc4040_data;
	if (alc4040_data->is_initialized == 1) {
		audio_4040_warning("%s: driver already initialized, exit\r\n",
								__func__);
		return ret;
	}
	memset(&g_alc4040_data, 0x0, sizeof(g_alc4040_data));
	evm_path_switch_control_init();
	alc4040_data->path_switch_control = evm_path_switch_control;
	alc4040_data->is_initialized = 1;
	audio_4040_debug("%s: driver initial successfully\n", __func__);

	return ret;
}
