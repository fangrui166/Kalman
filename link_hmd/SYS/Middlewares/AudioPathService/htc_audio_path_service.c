#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "hlog_api.h"
#include "misc_data.h"
#include "htc_audio_path_service.h"

#include "audio_alc4040_driver.h"
#include "audio_alc5665_driver.h"

struct audio_path_service_data {
	unsigned char is_initialized;
	enum __audio_srv_path path;
	osThreadId srv_action_thread;
	osMessageQId srv_action_queue;
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

void audio_path_service_func(void const * argument)
{
	osEvent notify_data;
	enum __audio_srv_path new_path;
	struct audio_path_service_data *srv_data =
				(struct audio_path_service_data *)argument;
	do {
		notify_data = osMessageGet(srv_data->srv_action_queue,
							osWaitForever);

		new_path = (enum __audio_srv_path)notify_data.value.signals;
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
	} while (1);
}

int audio_output_type_set(enum __audio_srv_path new_path)
{
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	if (srv_data->path == new_path) {
		audio_srv_warning("%s: incorrect type, exit\r\n",
				__func__);
		return -1;
	}

	if (osMessagePut(srv_data->srv_action_queue, (uint32_t)new_path,
				portTICK_PERIOD_MS * 500) != osOK) {
		audio_srv_warning("%s: sending request failed, exit\r\n",
				__func__);
		return -2;
	}
	return 0;
}

int audio_aj_detect(void)
{
	uint16_t reg = 0x00F0;
	uint16_t data = 0;
	uint16_t aj_in;
	audio_alc5665_i2c_read(reg, &data);
	aj_in = data >> 4 & 0x0001;
	if (aj_in == 0)
		audio_output_type_set(AUDIO_SRV_OUTPUT_TO_AJ);
	else
		audio_output_type_set(AUDIO_SRV_OUTPUT_TO_EAR);
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

    	if (osMessagePut(srv_data->srv_action_queue, (uint32_t)new_path,
    				portTICK_PERIOD_MS * 500) != osOK) {
    		audio_srv_warning("%s: sending request failed, exit\r\n",
				__func__);
    		return -2;
    	}
	return 0;
}

int audio_path_get(enum __audio_srv_path *my_path)
{
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	*my_path = srv_data->path;
	return 0;
}

static int audio_path_service_create_msgq(
				struct audio_path_service_data *srv_data)
{
	osMessageQDef(aps_queue_t, 8, uint32_t);
	srv_data->srv_action_queue = osMessageCreate(osMessageQ(aps_queue_t),
					srv_data->srv_action_thread);
	if (srv_data->srv_action_queue == NULL)
		return -1;

	return 0;
}

static int audio_path_service_create_thread(
				struct audio_path_service_data *srv_data)
{
	osThreadDef(aps_thread_t, audio_path_service_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 4);
	srv_data->srv_action_thread =
			osThreadCreate(osThread(aps_thread_t), srv_data);
	if (srv_data->srv_action_thread == NULL)
		return -1;

	return 0;
}

int audio_path_service_init()
{
	pcbid_t pcb_id;
        //uint8_t output_type;
	struct audio_path_service_data *srv_data = &g_audio_srv_data;
	if (srv_data->is_initialized == 1) {
		audio_srv_warning("%s: service already initialized, exit\r\n",
								__func__);
		return 0;
	}
	memset(&g_audio_srv_data, 0x0, sizeof(g_audio_srv_data));
	if (audio_path_service_create_thread(srv_data) != 0) {
		audio_srv_err("%s: create thread failed, exit\r\n", __func__);
		return -1;
	}
	if (audio_path_service_create_msgq(srv_data) != 0) {
		audio_srv_err("%s: create msgq failed, exit\r\n", __func__);
		return -2;
	}
	get_pcbid(&pcb_id);
	if (pcb_id == XA0n)
		__audio_path_setup(AUDIO_SRV_USB_PATH);
	else if (pcb_id == XB02)
                __audio_path_setup(AUDIO_SRV_HDMI_PATH);
	else if ((pcb_id == XC01) || (pcb_id == XC02)){
                __audio_path_setup(AUDIO_SRV_HDMI_PATH);
		audio_aj_detect();
                /* dedault setting, wait for XC version confirm
                output_type = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) << 1) |
                                (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11));
                output_type = 3;
                __audio_path_setup((enum __audio_srv_path)
                                (output_type + AUDIO_SRV_OUTPUT_OFFSET));
                */
        }

	srv_data->is_initialized = 1;
	audio_srv_debug("%s: service initial successfully\n", __func__);
	return 0;
}
