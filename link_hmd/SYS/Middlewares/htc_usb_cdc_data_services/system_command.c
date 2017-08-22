#include "FreeRTOS.h"
#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Fusion/HeadTracker.h"
#include "cmsis_os.h"

#include <hlog_api.h>
#include <usbd_comp.h>
#include <system_command.h>
#include "htc_audio_path_service.h"
#include "htc_usb_cdc_data_service.h"
#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"
#include "display_drv.h"
#include "sensor_task.h"
#include "hlog_main.h"
#include "led_hal.h"
#include "led_drv.h"
#include "bq2589x_charger.h"
#include "htc_3dof_transfer_service.h"
#include "fota_command.h"


#define SYSTEM_COMMAND_COMMAND_TIMEOUT		2000

struct __system_command_set {
	uint8_t cmd_id;
	int32_t (*cmd_func)(int, uint8_t *, uint8_t);
};

struct __system_command_data {
	uint8_t is_initialized;
	osThreadId work_thread;
	uint8_t recv_data[COMP_CDC_DATA_FS_MAX_PACKET_SIZE];
	uint8_t recv_data_length;
	osSemaphoreId recv_data_semaphore;
	struct pwrmgr_notify_func_data pm_notify_data;
	enum sys_cmd_service_state service_state;
	const struct __system_command_set *cmd_set;
	int32_t cmd_set_count;
};

static struct __system_command_data g_system_command_data = { 0 };

static int32_t sys_sensor_fusion_cmd(int action, uint8_t *buf, uint8_t len)
{
	if (action == USB_SYSCMD_ACTION_SET) {
		setTrackingReset(&buf[1]);
		return 0;
	}
	else if (action == USB_SYSCMD_ACTION_GET) {

	}

	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return -1;
}

static int32_t sys_audio_cmd(int action, uint8_t *buf, uint8_t len)
{
	uint8_t resp_str[COMP_CDC_DATA_FS_MAX_PACKET_SIZE] = { 0 };
	enum __audio_srv_path audio_path = AUDIO_SRV_UNSET_PATH;
	if (action == USB_SYSCMD_ACTION_SET) {
		if (buf[1] == AUDIO_CMD_ID_PATH) {
			if (buf[2] == AUDIO_SRV_HDMI_PATH)
				audio_path = AUDIO_SRV_HDMI_PATH;
			else if (buf[2] == AUDIO_SRV_USB_PATH)
				audio_path = AUDIO_SRV_USB_PATH;
			else {
				htc_sys_cmd_err("%s: incorrect path!!\r\n",
								__func__);
				return -1;
			}
			return audio_path_set(audio_path);
		}
	} else if (action == USB_SYSCMD_ACTION_GET) {
		if (buf[1] == AUDIO_CMD_ID_PATH) {
			audio_path_get(&audio_path);
			if (audio_path == AUDIO_SRV_HDMI_PATH)
				sprintf((char *)resp_str, "hdmi");
			else if (audio_path == AUDIO_SRV_USB_PATH)
				sprintf((char *)resp_str, "usb");
			else
				sprintf((char *)resp_str, "unknown");

			return usb_cdc_transmit_data(resp_str,
					strlen((char const *)resp_str));
		}
	}
	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return -1;
}

static int32_t sys_display_cmd(int action, uint8_t *buf, uint8_t len)
{
	int ret = -1;
	if (action == USB_SYSCMD_ACTION_SET) {
		if (DISPLAY_FUNC_SET00 == buf[1]) {
			if (DISPLAY_FUNC_SET00_PHONE_10 == buf[2])
				ret = disp_phone_type_set(DISPLAY_FUNC_SET00_PHONE_10);
			else if (DISPLAY_FUNC_SET00_PHONE_U_ULTRA == buf[2])
				ret = disp_phone_type_set(DISPLAY_FUNC_SET00_PHONE_U_ULTRA);
			else if (DISPLAY_FUNC_SET00_PHONE_OCEAN == buf[2])
				ret = disp_phone_type_set(DISPLAY_FUNC_SET00_PHONE_OCEAN);

			if (ret >= 0)
				ret = 0;
			else
				htc_sys_cmd_err(
					"%s: incorrect fps config: %d, "
					"ret: %d[0x%2.2X, 0x%2.2X]\r\n",
					__func__, buf[1], ret, buf[1], buf[2]);

			return ret;
		}
	} else if (action == USB_SYSCMD_ACTION_GET) {
	}
	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return ret;
}

static int32_t sys_ecompass_cmd(int action, uint8_t *buf, uint8_t len)
{
	uint8_t *tmp;
	uint8_t ecompass_info[4];
	uint8_t ecompass_str[COMP_CDC_DATA_FS_MAX_PACKET_SIZE] = { 0 };
	if (action == USB_SYSCMD_ACTION_SET) {
		if (buf[1] == MAG_SET_SUB_FUNC00) {
			if(buf[2] == MAG_CALIBRATION_CANCEL){
				magneticSetUserCalibration(0);
			}
			else if (buf[2] == MAG_CALIBRATION_START){
				magneticSetUserCalibration(1);
			} else {
				htc_sys_cmd_err("%s: incorrect command!!\r\n",
								__func__);
				return -1;
			}
			return 0;
		}
		htc_sys_cmd_err("%s: unknown set command\r\n", __func__);
		return -1;
	} else
	if (action == USB_SYSCMD_ACTION_GET) {
		if(buf[1] == MAG_GET_SUB_FUNC00){
			tmp = ecompass_str;
			memset(ecompass_info, 0xff, sizeof(ecompass_info));
			magneticGetUserCalibration(ecompass_info,sizeof(ecompass_info));
			if(ecompass_info[0] == 'p'){
				sprintf((char *)tmp, "calibration=pass,");
				tmp += strlen("calibration=pass,");
			}
			else if(ecompass_info[0] == 'f'){
				sprintf((char *)tmp, "calibration=fail,");
				tmp += strlen("calibration=fail,");
			}

			if(ecompass_info[1] == 'c'){
				sprintf((char *)tmp, "status=calibrating,");
				tmp += strlen("status=calibrating,");
			}
			else if(ecompass_info[1] == 'd'){
				sprintf((char *)tmp, "status=calibrated,");
				tmp += strlen("status=calibrated,");
			}
			if(ecompass_info[2] <=3){
				sprintf((char *)tmp, "level=%1.1d,",ecompass_info[2]);
				tmp = ecompass_str + strlen((char const *)ecompass_str);
			}
			if(ecompass_info[3] == 't'){
				sprintf((char *)tmp, "support=true");
				tmp += strlen("support=true");
			}
			else if(ecompass_info[3] == 'f'){
				sprintf((char *)tmp, "support=false");
				tmp += strlen("support=false");
			}
			htc_sys_cmd_info("%s:%s\r\n", __func__,ecompass_str);
			return usb_cdc_transmit_data(ecompass_str,
					strlen((char const *)ecompass_str));
		}
		htc_sys_cmd_err("%s: unknown get command\r\n", __func__);
		return -1;
	}

	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return -1;
}

/*
 * buf[1]: 0-start/stop estress test
 * buf[1]: 1-led on/off test
 */
static int32_t sys_led_cmd(int action, uint8_t *buf, uint8_t len)
{
	int ret = -1;
	uint8_t led_str[COMP_CDC_DATA_FS_MAX_PACKET_SIZE] = { 0 };
	sprintf((char *)led_str, "success");

	if (action == USB_SYSCMD_ACTION_SET) {
		if (0 == buf[1]) {
			/* estress test stop */
			if (buf[2] == 0x00) {
				SendLedEventToDeal(LED_EVENT_ESTRESS_TEST |
								LED_EVENT_OFF);
			}
			/* estress test start */
			else if (buf[2] == 0x01) {
				/* turn off green and red led first */
				ret = led_set_led_state(LED_COLOR_AMBER,
							LED_STEADY_OFF);
				SendLedEventToDeal(LED_EVENT_ESTRESS_TEST |
							LED_EVENT_ON);
			}
		} else if (1 == buf[1]) {
			htc_sys_cmd_info("%s: set led:%d status:%d\r\n",
						__func__, buf[2], buf[3]);
			if ((LED_STEADY_OFF != buf[3]) &&
						(LED_STEADY_ON != buf[3])) {
				htc_sys_cmd_err("%s: unknown led command\r\n",
								__func__);
				sprintf((char *)led_str, "unknown");
				usb_cdc_transmit_data(led_str, strlen((char const *)led_str));
				return -5;
			}
			if ((LED_COLOR_RED == buf[2]) ||
						(LED_COLOR_GREEN == buf[2]) ||
						(LED_COLOR_AMBER == buf[2])) {
				ret = led_set_led_state(buf[2],
							(LED_STATE)buf[3]);

				if (ret >= 0) {
					ret = 0;
					sprintf((char *)led_str, "success");
				}
				else {
					sprintf((char *)led_str, "fail");
					htc_sys_cmd_err(
						"%s: led command: "
						"%d set fail, ret: %d\r\n",
						__func__, buf[2], ret);
				}

				usb_cdc_transmit_data(led_str, strlen((char const *)led_str));
				return ret;
			}
		}
	} else
	if (action == USB_SYSCMD_ACTION_GET) {
	}
	sprintf((char *)led_str, "unknown");
	htc_sys_cmd_err("%s: unknown command\r\n", __func__);

	usb_cdc_transmit_data(led_str, strlen((char const *)led_str));
	return ret;
}

static int32_t sys_log_cmd(int action, uint8_t *buf, uint8_t len)
{
	if (action == USB_SYSCMD_ACTION_SET) {
		/* clean log buffer */
		if (buf[1] == 0x00) {
			hlog_cdc_clear_logbuf();
		}else if (buf[1] == 0x01) {
			hlog_cdc_clear_flash_log();
			htc_3dof_update_flashlog_4_dump();
			usb_cdc_printf("done.\r\n");
		}

		/* TODO: XXXXX */
		return 0;
	} else
	if (action == USB_SYSCMD_ACTION_GET) {
		/* dump all log */
		if (buf[1] == 0x00) {
			hlog_cdc_dump_logbuf();
		}
		if (buf[1] == 0x01) {
			hlog_cdc_dump_flash_log();
		}
		return 0;
	}

	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return -1;
}

static int32_t sys_power_cmd(int action, uint8_t *buf, uint8_t len)
{
	int chg_stat = 0;
	uint8_t chg_str[COMP_CDC_DATA_FS_MAX_PACKET_SIZE] = { 0 };
	if (action == USB_SYSCMD_ACTION_SET) {
		return 0;
	} else if (action == USB_SYSCMD_ACTION_GET) {
		if (buf[1] == 0x00) {
			chg_stat = htc_get_charger_state();
			if (chg_stat == 0) {
				sprintf((char *)chg_str, "no");
			} else if ((chg_stat == 1) || (chg_stat == 2) || (chg_stat == 3)) {
				sprintf((char *)chg_str, "yes");
			} else
				sprintf((char *)chg_str, "error");
			return usb_cdc_transmit_data(chg_str,
						strlen((char const *)chg_str));
		} else
			htc_sys_cmd_err("%s: other power cmd not set.\n", __func__);
	} else {
		htc_sys_cmd_err("%s: unknown command.\n", __func__);
	}

	return 0;
}


static int32_t sys_system_cmd(int action, uint8_t *buf, uint8_t len)
{
	if (action == USB_SYSCMD_ACTION_SET) {
		/* FOTA service */
		if (buf[1] == 0x00) {
			return fota_ProcessRcvData(buf, len);
		}
	}
	/* else if (action == USB_SYSCMD_ACTION_GET) {

	}
	*/
	htc_sys_cmd_err("%s: unknown command\r\n", __func__);
	return -1;
}


static int32_t system_command_get(uint8_t * buf, uint8_t len)
{
	uint32_t i;
	struct __system_command_data *sc_data = &g_system_command_data;
	for (i = 0; i < sc_data->cmd_set_count; i++)
		if (buf[0] == sc_data->cmd_set[i].cmd_id)
			return sc_data->cmd_set[i].cmd_func(
				USB_SYSCMD_ACTION_GET, buf, len);

	return -1;
}

static int32_t system_command_set(uint8_t * buf, uint8_t len)
{
	uint32_t i;

	struct __system_command_data *sc_data = &g_system_command_data;
	for (i = 0; i < sc_data->cmd_set_count; i++)
		if (buf[0] == sc_data->cmd_set[i].cmd_id)
			return sc_data->cmd_set[i].cmd_func(
				USB_SYSCMD_ACTION_SET, buf, len);

	return -1;
}

void __system_command_work_task_func(void const * argument)
{
	struct __system_command_data *sc_data =
			(struct __system_command_data *)argument;
	do {
		osSignalWait(0xFFFFFFFF, osWaitForever);

		if (sc_data->recv_data_length == 0)
			continue;

		if (sc_data->recv_data[1] == USB_SYSCMD_ACTION_GET)
			system_command_get(&sc_data->recv_data[2],
					sc_data->recv_data_length - 2);

		if (sc_data->recv_data[1] == USB_SYSCMD_ACTION_SET)
			system_command_set(&sc_data->recv_data[2],
					sc_data->recv_data_length - 2);

		sc_data->recv_data_length = 0;
		osSemaphoreRelease(sc_data->recv_data_semaphore);
	} while (1);
}

int32_t system_command(uint8_t * buf, uint8_t len)
{
	struct __system_command_data *sc_data = &g_system_command_data;
	if (buf == NULL)
		return -1;

	if (sc_data->service_state != SYS_CMD_SERVICE_RUNNING)
		return -2;

	if (osSemaphoreWait(sc_data->recv_data_semaphore,
			SYSTEM_COMMAND_COMMAND_TIMEOUT) == osErrorOS) {
		htc_sys_cmd_err("%s: timeout!!\n", __func__);
		return -3;
	}
	if (len > COMP_CDC_DATA_FS_MAX_PACKET_SIZE)
		sc_data->recv_data_length = COMP_CDC_DATA_FS_MAX_PACKET_SIZE;
	else
		sc_data->recv_data_length = len;

	memset(&sc_data->recv_data, 0x0, sizeof(sc_data->recv_data));
	memcpy(&sc_data->recv_data, buf, sc_data->recv_data_length);
	osSignalSet(sc_data->work_thread, 1);
	return 0;
}

static int __create_system_command_work_task(
			struct __system_command_data *sc_data)
{
	osThreadDef(syscommand_work_task, __system_command_work_task_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
	sc_data->work_thread =
		osThreadCreate(osThread(syscommand_work_task), sc_data);
	if (sc_data->work_thread == NULL)
		return -1;

	sc_data->service_state = SYS_CMD_SERVICE_RUNNING;

	return 0;
}

static int __create_system_command_semaphore(
			struct __system_command_data *sc_data)
{
	osSemaphoreDef(sysproperty_work_task_sema_t);
	sc_data->recv_data_semaphore = osSemaphoreCreate(
				osSemaphore(sysproperty_work_task_sema_t), 1);
	if (sc_data->recv_data_semaphore == NULL)
		return -1;

	return 0;
}

int set_sys_cmd_serv_control(enum sys_cmd_service_state state)
{
	int ret = 0;
	struct __system_command_data *sc_data = &g_system_command_data;
	enum sys_cmd_service_state old_state = sc_data->service_state;

	if (sc_data->service_state == state) {
		htc_sys_cmd_info("%s: service state is same\r\n", __func__);
		return ret;
	}
	sc_data->service_state = state;
	if (state == SYS_CMD_SERVICE_RUNNING) {
		if (osThreadResume(sc_data->work_thread) != osOK) {
			htc_sys_cmd_err("%s: resume failed!!\n",
								__func__);
			ret = -2;
		}
	} else
	if (state == SYS_CMD_SERVICE_SUSPEND) {
		if (osThreadSuspend(sc_data->work_thread) != osOK) {
			htc_sys_cmd_err("%s: suspend failed!!\n",
								__func__);
			ret = -3;
		}
	}

	if (ret != 0)
		sc_data->service_state = old_state;

	htc_sys_cmd_debug("%s: service %s(%s)\r\n", __func__,
		(sc_data->service_state == SYS_CMD_SERVICE_SUSPEND) ?
								"SUSPEND" :
		(sc_data->service_state == SYS_CMD_SERVICE_RUNNING) ?
							"RUNNING" : "UNKNOWN",
		(ret != 0) ? "fail" : "done");

	return ret;
}

enum sys_cmd_service_state get_sys_cmd_serv_control(void)
{
	struct __system_command_data *sc_data = &g_system_command_data;

	return sc_data->service_state;
}

int sys_cmd_pm_notify_func(uint32_t flag, uint32_t state, void *data)
{
	int ret = 0;
	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT)
			ret = set_sys_cmd_serv_control(
					SYS_CMD_SERVICE_SUSPEND);
		else
		if (state == STOP_LEAVE || state == PLUG_IN)
			ret = set_sys_cmd_serv_control(
					SYS_CMD_SERVICE_RUNNING);
	} else {
		htc_sys_cmd_info("%s: unsupported flag\r\n", __func__);
	}
	return ret;
}

static int __create_system_command_pm_notify_func(
				struct __system_command_data *sc_data)
{
	struct pwrmgr_notify_func_data *pm_not_p = &sc_data->pm_notify_data;

	pm_not_p->func_name = "sys_cmd_srv";
	pm_not_p->data = sc_data;
	pm_not_p->callback = sys_cmd_pm_notify_func;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_APP_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}

const struct __system_command_set sys_cmd_set[] = {
	{
		.cmd_id = CMD_ID_SENSOR_FUSION,
		.cmd_func = sys_sensor_fusion_cmd,
	},
	{
		.cmd_id = CMD_ID_AUDIO_CONTROL,
		.cmd_func = sys_audio_cmd,
	},
	{
		.cmd_id = CMD_ID_DISPLAY_CONTROL,
		.cmd_func = sys_display_cmd,
	},
	{
		.cmd_id = CMD_ID_ECOMPASS_CALIBRATION,
		.cmd_func = sys_ecompass_cmd,
	},
	{
		.cmd_id = CMD_ID_LED_CONTROL,
		.cmd_func = sys_led_cmd,
	},
	{
		.cmd_id = CMD_ID_LOG_CONTROL,
		.cmd_func = sys_log_cmd,
	},
	{
		.cmd_id = CMD_ID_POWER_CONTROL,
		.cmd_func = sys_power_cmd,
	},
	{
		.cmd_id = CMD_ID_SYSTEM_CONTROL,
		.cmd_func = sys_system_cmd,
	},
};

void system_command_init(void)
{
	struct __system_command_data *sc_data = &g_system_command_data;
	if (sc_data->is_initialized) {
		htc_sys_cmd_info("%s: already initialized, exit\n", __func__);
		return;
	}

	sc_data->cmd_set = sys_cmd_set;
	sc_data->cmd_set_count = sizeof(sys_cmd_set) / sizeof(sys_cmd_set[0]);

	if (__create_system_command_work_task(sc_data) != 0)
		htc_sys_cmd_warning("%s: create work task failed\n",
								__func__);

	if (__create_system_command_pm_notify_func(sc_data) != 0)
		htc_sys_cmd_warning("%s: create pm notifier failed\n",
								__func__);

	if (__create_system_command_semaphore(sc_data) != 0)
		htc_sys_cmd_warning("%s: create work semaphore failed\n",
								__func__);

	sc_data->is_initialized = 1;
}
