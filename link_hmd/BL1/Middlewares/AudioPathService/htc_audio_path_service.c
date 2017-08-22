#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "misc_data.h"
#include "htc_audio_path_service.h"

#include "audio_alc4040_driver.h"
#include "audio_alc5665_driver.h"

struct audio_path_service_data {
	unsigned char is_initialized;
	enum __audio_srv_path path;
};

static struct audio_path_service_data g_audio_srv_data;

static int __audio_path_setup(enum __audio_srv_path new_path)
{
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	switch (new_path) {
	case AUDIO_SRV_HDMI_PATH:
		audio_alc4040_audio_path_set(AUDIO_4040_HDMI_PATH);
		alt5665_audio_path_set(AUDIO_5665_HDMI_PATH);
		break;
	case AUDIO_SRV_USB_PATH:
		audio_alc4040_audio_path_set(AUDIO_4040_USB_PATH);
		alt5665_audio_path_set(AUDIO_5665_USB_PATH);
		break;
	case AUDIO_SRV_LOOPBACK_PATH:
		audio_alc4040_audio_path_set(AUDIO_4040_USB_PATH);
		alt5665_audio_path_set(AUDIO_5665_LOOPBACK_PATH);
		break;
	case AUDIO_SRV_OUTPUT_TO_AJ:
		audio_alc5665_audio_amp_power_set(AMP_POWER_DISABLE);
		alt5665_audio_path_set(AUDIO_5665_OUTPUT_TO_AJ);
		break;
	case AUDIO_SRV_OUTPUT_TO_EAR:
		audio_alc5665_audio_amp_power_set(AMP_POWER_ENABLE);
		alt5665_audio_path_set(AUDIO_5665_OUTPUT_TO_EAR);
		break;
	case AUDIO_SRV_OUTPUT_TO_BOTH:
		audio_alc5665_audio_amp_power_set(AMP_POWER_ENABLE);
		alt5665_audio_path_set(AUDIO_5665_OUTPUT_TO_BOTH);
		break;
	case AUDIO_SRV_UNSET_PATH:
		break;
	}
	srv_data->path = new_path;
	return 0;
}

int audio_path_set(enum __audio_srv_path new_path)
{
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	if (srv_data->path == new_path) {
		audio_srv_warning("%s: incorrect config, exit\r\n",
				__func__);
		return -1;
	}
        switch (new_path) {
		case AUDIO_SRV_HDMI_PATH:
		case AUDIO_SRV_USB_PATH:
		case AUDIO_SRV_LOOPBACK_PATH:
		case AUDIO_SRV_OUTPUT_TO_AJ:
		case AUDIO_SRV_OUTPUT_TO_EAR:
		case AUDIO_SRV_OUTPUT_TO_BOTH:
			__audio_path_setup(new_path);
			break;
		case AUDIO_SRV_UNSET_PATH:
		default:
			audio_srv_warning("%s: incorrect config, exit\r\n",
				__func__);
			break;
        }
	return 0;
}

int audio_path_get(enum __audio_srv_path *my_path)
{
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	*my_path = srv_data->path;
	return 0;
}

int audio_path_service_init()
{
	pcbid_t pcb_id;
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	if (srv_data->is_initialized == 1) {
		audio_srv_info("%s: service already initialized, exit\r\n",
								__func__);
		return 0;
	}
	memset(&g_audio_srv_data, 0x0, sizeof(g_audio_srv_data));
        get_pcbid(&pcb_id);
	if (pcb_id == XA0n)
		__audio_path_setup(AUDIO_SRV_USB_PATH);
	else /* XB02 , XC01 , XC02 */
		__audio_path_setup(AUDIO_SRV_HDMI_PATH);
	srv_data->is_initialized = 1;
	audio_srv_info("%s: service initial successfully\n", __func__);
	return 0;
}
