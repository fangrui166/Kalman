#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include <stdarg.h>
#include "hlog_api.h"
#include "usart_drv.h"
#include "system_property.h"
#include "system_command.h"

#include "usbd_comp.h"
#include "usbd_comp_if.h"
#include "htc_usb_cdc_data_service.h"
#include "command.h"
#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"


#define CDC_BUFFER_SIZE			COMP_CDC_DATA_FS_IN_PACKET_SIZE

enum __buffer_state {
	buffer_state_empty = 0,
	buffer_state_full = 1,
};

struct usb_cdc_cmd_func {
	uint8_t key;
	int (*cdc_cmd)(uint8_t *, uint8_t);
};

struct usb_cdc_service_data {
	unsigned char is_initialized;
	osThreadId work_thread;
	const struct usb_cdc_cmd_func *cmd_func;
	uint8_t cmd_func_count;
	uint8_t cmd_buff[CDC_BUFFER_SIZE];
	uint8_t cmd_buff_size;
	enum __buffer_state cmd_buff_is_full;
	struct pwrmgr_notify_func_data pm_notify_data;
	enum htc_cdc_service_state service_state;
};

static struct usb_cdc_service_data g_usb_cdc_data;

int usb_cdc_transmit_data(uint8_t *buf, uint8_t len)
{
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;
	int ret = -1;
	uint8_t transmit_data_fail_retry_counter = 0;
	uint32_t cur_tx_status;
	if (buf == NULL || len == 0)
		return -1;

	if (srv_data->service_state != HTC_CDC_SRV_RUNNING)
		return -2;

	if (len > CDC_BUFFER_SIZE)
		len = CDC_BUFFER_SIZE;

	do {
		cur_tx_status = COMP_CDC_Transmit_STATE();
		if( cur_tx_status == USB_CDC_TX_IDLE) {
			ret = COMP_CDC_Transmit_FS(buf, len);
			break;
		} else
			++transmit_data_fail_retry_counter;

		osDelay(100);

	} while(transmit_data_fail_retry_counter < 3);
    return ret;
}

int usb_cdc_printf(const char *fmt, ...)
{
	int str_len = 0;
	uint8_t buff[CDC_BUFFER_SIZE] = { 0 };
	va_list ap;
	va_start(ap, fmt);
	str_len = vsnprintf((char *)buff, CDC_BUFFER_SIZE, fmt, ap);
	va_end(ap);

	if (str_len <= 0)
		return str_len;

	if (usb_cdc_transmit_data(buff, str_len) == 0)
		return str_len;

	return -1;
}

static int usb_cdc_execute_command(uint8_t *cmd, uint8_t size)
{
	int loop_i;
	int ret = 0;
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;

	for (loop_i = 0; loop_i < srv_data->cmd_func_count; loop_i++) {
		if ((cmd[0] == srv_data->cmd_func[loop_i].key) &&
			srv_data->cmd_func[loop_i].cdc_cmd != NULL) {
			ret = srv_data->cmd_func[loop_i].cdc_cmd(cmd, size);
			break;
		}
	}
	if (loop_i == srv_data->cmd_func_count)
		ret = -1;

	return ret;
}

static int usb_cdc_handle_resp_string(uint8_t *buf, int len)
{
	int loop_i;
	int ret = 0;
	uint8_t cdc_response_str[CDC_BUFFER_SIZE] = { 0 };
	int cdc_resp_str_len = 0;
	uint8_t back_char[3] = { 0x08, 0x20, 0x08 };
	for (loop_i = 0; loop_i < len; loop_i++) {
		if ((buf[loop_i] >= 0x20) && (buf[loop_i] <= 0x7E)) {
			if (cdc_resp_str_len + 1 <= CDC_BUFFER_SIZE - 1) {
				cdc_response_str[cdc_resp_str_len++] =
								buf[loop_i];
			} else {
				ucdc_srv_err("%s: cdc response string "
					"lenth over limit\r\n", __func__);
				ret = -1;
			}
		}
		else if ((buf[loop_i] == 0x08) || (buf[loop_i] == 0x7F)) {
			if (cdc_resp_str_len + 3 <= CDC_BUFFER_SIZE - 1) {
				cdc_response_str[cdc_resp_str_len++] =
							back_char[0];
				cdc_response_str[cdc_resp_str_len++] =
							back_char[1];
				cdc_response_str[cdc_resp_str_len++] =
							back_char[2];
			} else {
				ucdc_srv_err("%s: cdc response string"
					"(back/del) lenth over limit\r\n",
					__func__);
				ret = -2;
			}
		}
		else if ((buf[loop_i] == 0x0D) || (buf[loop_i] == 0x0A)) {
			if (cdc_resp_str_len + 1 <= CDC_BUFFER_SIZE - 1) {
				cdc_response_str[cdc_resp_str_len++] =
								buf[loop_i];
			} else {
				ucdc_srv_err("%s: cdc response string (CR+LF)"
					"lenth over limit\r\n", __func__);
				ret = -1;
			}
		}
	}
	if (ret != 0)
		return ret;

	return usb_cdc_transmit_data(cdc_response_str, cdc_resp_str_len);

}

static int usb_cdc_handle_command(void)
{
	int ret;
	const char cmd_t[] = "\r\nCMD>";
	const char cmd_over_limit[] = "cmd string over limit\r\n";
	static uint8_t shell_cmd[CDC_BUFFER_SIZE] = { 0 };
	static uint8_t shell_string_remain = CDC_BUFFER_SIZE - 1;
	static uint8_t shell_string_offset = 0;
	static int exec_shell_cmd = 0;
	static int in_shell_mode = 0;
	int loop_i;
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;

	if ((in_shell_mode == 0 && shell_string_offset == 0) &&
				(srv_data->cmd_buff[0] == 0x0D ||
				srv_data->cmd_buff[0] == 0x0A ||
				(srv_data->cmd_buff[0] == 0x0D &&
					srv_data->cmd_buff[1] == 0x0A))) {
		usb_cdc_transmit_data((uint8_t *)cmd_t, strlen(cmd_t));
		srv_data->cmd_buff_is_full = buffer_state_empty;
		return 0;
	}

	if (in_shell_mode == 0 && shell_string_offset == 0 &&
				srv_data->cmd_buff[0] == APP_ID_SHELLCMD) {
		in_shell_mode = 1;
		memset(&shell_cmd, 0x0, sizeof(shell_cmd));
		shell_string_remain = CDC_BUFFER_SIZE - 1;
		exec_shell_cmd = 0;
	}
	if (in_shell_mode != 0) {
		if (shell_string_offset != 0)
			usb_cdc_handle_resp_string(srv_data->cmd_buff,
						srv_data->cmd_buff_size);

		if ((srv_data->cmd_buff_size + shell_string_offset) >
							shell_string_remain) {
			memset(&shell_cmd, 0x0, sizeof(shell_cmd));
			shell_string_remain = CDC_BUFFER_SIZE - 1;
			shell_string_offset = 0;
			exec_shell_cmd = 0;
			in_shell_mode = 0;
			srv_data->cmd_buff_is_full = buffer_state_empty;
			usb_cdc_transmit_data((uint8_t *)cmd_over_limit,
						strlen(cmd_over_limit));
			usb_cdc_transmit_data((uint8_t *)cmd_t,
						strlen(cmd_t));
			return 0;
		}
		for (loop_i = 0; loop_i < srv_data->cmd_buff_size; loop_i++) {
			/* printable character */
			if ((srv_data->cmd_buff[loop_i] >= 0x20) &&
				(srv_data->cmd_buff[loop_i] <= 0x7E)) {
				shell_cmd[shell_string_offset] =
						srv_data->cmd_buff[loop_i];
				shell_string_offset++;
				shell_string_remain--;
			}
			/* backspace and delete */
			else if (srv_data->cmd_buff[loop_i] == 0x08 ||
				srv_data->cmd_buff[loop_i] == 0x7F) {
				if (shell_string_offset != 0) {
					shell_string_offset--;
					shell_string_remain++;
					shell_cmd[shell_string_offset] = 0x00;
				} else {
					in_shell_mode = 0;
				}
			}
			/* \r or \n */
			else if ((srv_data->cmd_buff[loop_i] == 0x0D) ||
				(srv_data->cmd_buff[loop_i] == 0x0A)) {
				usb_cdc_transmit_data(
					srv_data->cmd_buff + loop_i, 1);
				exec_shell_cmd = 1;
			}
		}
		if (exec_shell_cmd != 0) {
			ret = usb_cdc_execute_command(
					(uint8_t *)shell_cmd,
					shell_string_offset);
			memset(&shell_cmd, 0x0, sizeof(shell_cmd));
			shell_string_remain = CDC_BUFFER_SIZE - 1;
			shell_string_offset = 0;
			exec_shell_cmd = 0;
			in_shell_mode = 0;
			usb_cdc_transmit_data((uint8_t *)cmd_t, strlen(cmd_t));
		}
	} else { /* in_shell_mode == 0 */
		ret = usb_cdc_execute_command(srv_data->cmd_buff,
						srv_data->cmd_buff_size);
	}
	srv_data->cmd_buff_is_full = buffer_state_empty;
	return ret;

}

void usb_cdc_service_func(void const * argument)
{
	struct usb_cdc_service_data *srv_data =
			(struct usb_cdc_service_data *)argument;
	do {
		osSignalWait(0xFFFFFFFF, osWaitForever);
		if (srv_data->cmd_buff_is_full == buffer_state_empty)
			continue;

		usb_cdc_handle_command();

	} while (1);
}

int usb_cdc_receive_data(uint8_t *buffer, uint8_t length)
{
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;

	if (srv_data->service_state == HTC_CDC_SRV_SUSPEND)
		return 0;

	if (srv_data->cmd_buff_is_full == buffer_state_full) {
		ucdc_srv_err("%s: buffer is full\r\n", __func__);
		return -1;
	}
	srv_data->cmd_buff_is_full = buffer_state_full;
	srv_data->cmd_buff_size = CDC_BUFFER_SIZE;
	if (srv_data->cmd_buff_size > length)
		srv_data->cmd_buff_size = length;

	memset(srv_data->cmd_buff, 0x0, CDC_BUFFER_SIZE);
	memcpy(srv_data->cmd_buff, buffer, srv_data->cmd_buff_size);
	osSignalSet(srv_data->work_thread, 1);
	return 0;
}

static int usb_cdc_service_create_thread(
				struct usb_cdc_service_data *ucdc_data)
{
	osThreadDef(ucdc_thread_t, usb_cdc_service_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 4);
	ucdc_data->work_thread =
			osThreadCreate(osThread(ucdc_thread_t), ucdc_data);
	if (ucdc_data->work_thread == NULL)
		return -1;

	ucdc_data->service_state = HTC_CDC_SRV_RUNNING;

	return 0;
}

int32_t usb_shell_data_handler(uint8_t * buf, uint8_t len)
{
	int ret = 0;
	int argn = 0;
	uint8_t shell_cmd[CDC_BUFFER_SIZE] = { 0 };
	int shell_cmd_len = len;
	unsigned char *argv[MAX_ARGS] = { 0 };

	if (buf[0] == 'E')
		shell_cmd_len--;
	if (buf[1] == ' ')
		shell_cmd_len--;

	memcpy(&shell_cmd, buf + (len - shell_cmd_len), shell_cmd_len);
	if(0 != ParseCommandAndData((uint8_t *)&shell_cmd, &argn, argv,
					(sizeof(argv) / sizeof(argv[0])))) {
	    ret = DoCommand(argn, argv, CMD_SOURCE_USBCDC);
	}

	return ret;
}

int set_htc_cdc_srv_control(enum htc_cdc_service_state state)
{
	int ret = 0;
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;
	enum htc_cdc_service_state temp_state = srv_data->service_state;
	if (srv_data->service_state == state) {
		ucdc_srv_info("%s: service state is same\r\n", __func__);
		srv_data->service_state = temp_state;
		return ret;
	}

	srv_data->service_state = state;

	if (state == HTC_CDC_SRV_RUNNING) {
		if (osThreadResume(srv_data->work_thread) != osOK) {
			ucdc_srv_err("%s: work task resume failed!!\n",
								__func__);
			ret = -1;
		}
		srv_data->cmd_buff_is_full = buffer_state_empty;
	} else
	if (state == HTC_CDC_SRV_SUSPEND) {
		if (osThreadSuspend(srv_data->work_thread) != osOK) {
			ucdc_srv_err("%s: work task suspend failed!!\n",
								__func__);
			ret = -2;
		}
		srv_data->cmd_buff_is_full = buffer_state_empty;
	}
	if (ret != 0)
		srv_data->service_state = temp_state;

	ucdc_srv_debug("%s: service %s(%s)\r\n", __func__,
		(srv_data->service_state == HTC_CDC_SRV_SUSPEND) ? "SUSPEND" :
		(srv_data->service_state == HTC_CDC_SRV_RUNNING) ? "RUNNING" :
			"UNKNOWN",
		(ret != 0) ? "fail" : "done");

	return ret;
}

enum htc_cdc_service_state get_htc_cdc_srv_control(void)
{
	struct usb_cdc_service_data *srv_data = &g_usb_cdc_data;

	return srv_data->service_state;
}

int htc_cdc_srv_pm_notify_func(uint32_t flag, uint32_t state, void *data)
{
	int ret = 0;
	//struct usb_cdc_service_data *srv_data =
	//			(struct usb_cdc_service_data *)data;

	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT)
			ret = set_htc_cdc_srv_control(
					HTC_CDC_SRV_SUSPEND);
		else
		if (state == STOP_LEAVE || state == PLUG_IN)
			ret = set_htc_cdc_srv_control(
					HTC_CDC_SRV_RUNNING);
	} else {
		ucdc_srv_warning("%s: unsupported flag\r\n", __func__);
	}
	return ret;
}

static int usb_cdc_service_pm_notify_func(
				struct usb_cdc_service_data *ucdc_data)
{
	struct pwrmgr_notify_func_data *pm_not_p = &ucdc_data->pm_notify_data;

	pm_not_p->func_name = "usb_cdc_srv";
	pm_not_p->data = ucdc_data;
	pm_not_p->callback = htc_cdc_srv_pm_notify_func;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_APP_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}

int32_t lookback_func(uint8_t * buf, uint8_t len)
{
	return usb_cdc_transmit_data(buf, len);
}

const struct usb_cdc_cmd_func evm_cmd_func[] = {
	{
		.key = APP_ID_SYSPROP,
		.cdc_cmd = system_property,
	},
	{
		.key = APP_ID_SYSCMD,
		.cdc_cmd = system_command,
	},
	{
		.key = APP_ID_SHELLCMD,
		.cdc_cmd = usb_shell_data_handler,
	},
	{
		.key = APP_ID_LOOPBACK,
		.cdc_cmd = lookback_func,
	},
};


int usb_cdc_service_init(void)
{
	struct usb_cdc_service_data *ucdc_data = &g_usb_cdc_data;
	if (ucdc_data->is_initialized == 1) {
		ucdc_srv_info("%s: service already initialized, exit\r\n",
								__func__);
		return 0;
	}
	memset(&g_usb_cdc_data, 0x0, sizeof(g_usb_cdc_data));
	if (usb_cdc_service_create_thread(ucdc_data) != 0) {
		ucdc_srv_err("%s: create thread failed, exit\r\n", __func__);
		return -1;
	}

	if (usb_cdc_service_pm_notify_func(ucdc_data) != 0)
		ucdc_srv_err("%s: create pm notifier failed\n", __func__);

	ucdc_data->cmd_func = evm_cmd_func;
	ucdc_data->cmd_func_count =
			sizeof(evm_cmd_func) / sizeof(evm_cmd_func[0]);
	ucdc_data->is_initialized = 1;
	ucdc_srv_debug("%s: service initial successfully\n", __func__);
	return 0;
}
