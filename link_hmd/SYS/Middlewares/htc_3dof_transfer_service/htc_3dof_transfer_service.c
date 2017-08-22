#include "FreeRTOS.h"
#include "timers.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "cmsis_os.h"

#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"

#include "usbd_comp_if.h"
#include "Stm32f4xx_hal.h"
#include "htc_3dof_transfer_service.h"
#include "htc_3dof_transfer_service_priv.h"
#include "hlog_main.h"

#define PSENSOR_STATE_OFFSET	0
#define PSENSOR_STATE_MASK	0x01
#define POWER_STATE_OFFSET	1
#define POWER_STATE_MASK	0x03FFF
#define VSYNC_STATE_OFFSET	16
#define VSYNC_STATE_MASK	0x0F
#define LOGFLAG_STATE_OFFSET	30
#define LOGFLAG_STATE_MASK	0x03
#define DISPLAY_STATE_OFFSET	20
#define DISPLAY_STATE_MASK	0x1

/*
 * SYSTEM STATE BIT FIELD LAYOUT
 * TOTAL SIZE SHOULD BE EQUAL 32 BIT
 * system state bit field layout
 * bit[0]: proximity is pressed or not
 * bit[1-7]: battery capacity(0-100), unit: %
 * bit[8]: battery charging state:
 *   0: no charging,
 *   1: charging
 * bit[9-10]: battery low:
 *   3: reserved
 *   2: 15% low
 *   1: 5% low
 *   0: normal
 * bit[11]: battery overheat:
 *   1: overheat,
 *   0: none
 * bit[12-13]: battery charging current is negative:
 *   3: negative(when capacity under 15% and capacity lost over 2%)
 *   2: negative(when capacity is 0 and capacity lost over 2%)
 *   1: reserved
 *   0: normal
 * bit[14]: battery over voltage protection(OVP):
 *   1: OVP,
 *   0: normal
 * bit[15]: reserved
 * bit[16-19]: vsync count(1-15)
 * bit[20]: Display output is normal or not
 * bit[21-29]: reserved
 * bit[30-31]: boot state flag:
 *   3: crash reboot
 *   1-2: reserved
 *   0: normal
 */

__packed struct __send_data {
	uint8_t sensor_data[HTC_3DOF_MAX_SENSOR_DATA_SIZE];
	uint32_t system_state;
};

struct __3dof_transfer_service_data {
	uint8_t is_initialized;
	uint8_t is_enabled;
	osMessageQId work_queue;
	osThreadId send_work_thread;
	struct pwrmgr_notify_func_data pm_notify_data;
	uint8_t send_work_thread_is_running;
	uint8_t send_data_is_full;
	struct __send_data send_data;
#ifdef HTC_3DOF_POLLING_TEST_EN
	osTimerId timer;
	osTimerDef_t timer_def;
	uint32_t timer_interval;
	unsigned char timer_is_running;
#endif /* HTC_3DOF_POLLING_TEST_EN */
};

static struct __3dof_transfer_service_data g_3dof_service_data = { 0 };

uint8_t __recv_data[HTC_3DOF_MAX_PACKET_SIZE];

void __htc_3dof_send_work_task_func(void const * argument)
{
	struct __3dof_transfer_service_data *p_3dof_data =
			(struct __3dof_transfer_service_data *)argument;
	do {
		osSignalWait(0xFFFFFFFF, osWaitForever);
#ifdef COMP_USE_VENDOR_SPEC_1
		COMP_VendorSpec_Transmit((uint8_t *)&p_3dof_data->send_data,
						HTC_3DOF_MAX_PACKET_SIZE);
#endif
		p_3dof_data->send_data_is_full = 0;
	} while (1);
}

int htc_3dof_tran_service_send_data(uint8_t *data, uint32_t length)
{
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;
	if (p_3dof_data->send_work_thread_is_running == 0) {
		/* htc_3dof_tran_err("%s: work task not running!!\n", __func__); */
		return -1;
	}
	if (length > HTC_3DOF_MAX_SENSOR_DATA_SIZE)
		return -2;

	if (p_3dof_data->send_data_is_full) {
		/* htc_3dof_tran_debug("3dof data dropped\n"); */
		return -3;
	}

	p_3dof_data->send_data_is_full = 1;
	memcpy(&p_3dof_data->send_data.sensor_data, data, length);
	osSignalSet(p_3dof_data->send_work_thread, 1);

	return 0;
}


void htc_3dof_update_vsync_status(uint8_t value)
{
	struct __3dof_transfer_service_data *p_3dof_data =
							&g_3dof_service_data;
	uint32_t new_state = p_3dof_data->send_data.system_state;
	uint8_t count = ((new_state >> VSYNC_STATE_OFFSET) &
							VSYNC_STATE_MASK) + 1;
	if (count >= 16)
		count = 1;

	new_state &= ~(VSYNC_STATE_MASK << VSYNC_STATE_OFFSET);
	new_state |= (count << VSYNC_STATE_OFFSET);
	p_3dof_data->send_data.system_state = new_state;
}

void htc_3dof_update_display_status(uint8_t value)
{
	struct __3dof_transfer_service_data *p_3dof_data =
							&g_3dof_service_data;
	uint32_t new_state = p_3dof_data->send_data.system_state;

	new_state &= ~(DISPLAY_STATE_MASK << DISPLAY_STATE_OFFSET);
	new_state |= (!!value << DISPLAY_STATE_OFFSET);
	p_3dof_data->send_data.system_state = new_state;
}

void htc_3dof_update_flash_log_flag(uint8_t value)
{
	struct __3dof_transfer_service_data *p_3dof_data =
							&g_3dof_service_data;
	uint32_t new_state = p_3dof_data->send_data.system_state;
	uint16_t new_value = (value & LOGFLAG_STATE_MASK);
	new_state &= ~(LOGFLAG_STATE_MASK << LOGFLAG_STATE_OFFSET);
	new_state |= (new_value << LOGFLAG_STATE_OFFSET);
	p_3dof_data->send_data.system_state = new_state;
}

void htc_3dof_update_flashlog_4_dump(){
	flash_log_head_t  log_head={0};
	memset(&log_head, 0xff, sizeof(log_head));
	hlog_get_flashlog_head(&log_head);
	if(log_head.log_exit_flag == 0xff){
		htc_3dof_tran_info("%s: mcu normal reboot!\n", __func__);
		htc_3dof_update_flash_log_flag(0x00);
	}else{
		htc_3dof_tran_info("%s: mcu crash,dump romlog to phone! "
			"flag = %d\n",
			__func__, log_head.log_exit_flag);
		htc_3dof_update_flash_log_flag(0x03);
	}
}

void htc_3dof_update_psensor_key(uint8_t value)
{
	struct __3dof_transfer_service_data *p_3dof_data =
							&g_3dof_service_data;
	uint32_t new_state = p_3dof_data->send_data.system_state;
	uint16_t new_value = !!value;
	new_state &= ~(PSENSOR_STATE_MASK << PSENSOR_STATE_OFFSET);
	new_state |= (new_value << PSENSOR_STATE_OFFSET);
	p_3dof_data->send_data.system_state = new_state;
}

/*
 * htc_3dof_update_battery_state argvnment:
 *   batt_ca: battery capacity
 *   charg_st: battery charging state
 *   batt_lo: battery low
 *   batt_oh: battery overheat
 *   batt_curr_n: battery charging current is negative
 *   batt_ovp: battery over voltage protection(OVP)
 */
void htc_3dof_update_battery_state(uint8_t batt_ca,
						uint8_t charg_st,
						uint8_t batt_lo,
						uint8_t batt_oh,
						uint8_t batt_curr_n,
						uint8_t batt_ovp)
{
	struct __3dof_transfer_service_data *p_3dof_data =
							&g_3dof_service_data;
	uint32_t new_state = p_3dof_data->send_data.system_state;
	uint16_t value = (!!(batt_ovp) << 13) |
		((batt_curr_n & 0x3) << 11) |
		(!!batt_oh << 10) |
		((batt_lo & 0x3) << 8) |
		(!!charg_st << 7) |
		(batt_ca & 0x7F);
	new_state &= ~(POWER_STATE_MASK << POWER_STATE_OFFSET);
	new_state |= (value << POWER_STATE_OFFSET);
	p_3dof_data->send_data.system_state = new_state;
/*
	printf("%s: batt_ca: %d, charg_st: %d, batt_lo: %d, batt_oh: %d, "
		"batt_curr_n: %d, batt_ovp: %d(0x%4.4X, 0x%8.8X)\r\n",
		__func__, batt_ca, charg_st, batt_lo, batt_oh, batt_curr_n,
		batt_ovp, value, p_3dof_data->send_data.system_state);
*/
}

#ifdef HTC_3DOF_POLLING_TEST_EN
int htc_3dof_tran_service_set_timer_interval(uint32_t interval)
{
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;

	if (interval != HTC_3DOF_TEST_TIMER_INTERVAL)
		return -1;

	p_3dof_data->timer_interval = interval;
	return 0;
}

static int __htc_3dof_tran_service_timer_enable(u16_t enable)
{
	int ret = 0;
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;
	if (p_3dof_data->timer == NULL)
		return -1;

	if ((enable != 0 && p_3dof_data->timer_is_running != 0) ||
		enable == 0 && p_3dof_data->timer_is_running == 0)
		return -2;

	if (enable) {
		if (osTimerStart(p_3dof_data->timer,
				p_3dof_data->timer_interval) != osOK)
			ret = -3;
	} else {
		if (osTimerStop(p_3dof_data->timer) != osOK)
			ret = -4;
	}
	if (ret == 0)
		p_3dof_data->timer_is_running = !!enable;
	return ret;
}


int htc_3dof_tran_service_set_timer_enable(u8_t enable)
{
	return __htc_3dof_tran_service_timer_enable(enable);
}
#endif /* HTC_3DOF_POLLING_TEST_EN */

int htc_3dof_tran_service_set_send_work_task_en(uint8_t enable)
{
	int ret = 0;
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;
	uint8_t temp_running_state = p_3dof_data->send_work_thread_is_running;
	if ((enable && temp_running_state) ||
		(!enable && temp_running_state == 0)) {
		htc_3dof_tran_info("%s: service state is same\n", __func__);
		return 0;
	}

	p_3dof_data->send_work_thread_is_running = (uint8_t)!!enable;

	if (enable) {
		if (osThreadResume(p_3dof_data->send_work_thread) != osOK) {
			htc_3dof_tran_err("%s: work task resume failed!!\n",
								__func__);
			ret = -2;
		}
#ifdef HTC_3DOF_POLLING_TEST_EN
		htc_3dof_tran_service_set_timer_enable(1);
#endif /* HTC_3DOF_POLLING_TEST_EN */
		HAL_Delay(3);
	} else {
		HAL_Delay(3);
#ifdef HTC_3DOF_POLLING_TEST_EN
		htc_3dof_tran_service_set_timer_enable(0);
#endif /* HTC_3DOF_POLLING_TEST_EN */
		if (osThreadSuspend(p_3dof_data->send_work_thread) != osOK) {
			htc_3dof_tran_err("%s: work task suspend failed!!\n",
								__func__);
			ret = -3;
		}
	}

	if (ret != 0)
		p_3dof_data->send_work_thread_is_running = temp_running_state;

	htc_3dof_tran_debug("%s: service %s(%s)\r\n", __func__,
		(p_3dof_data->send_work_thread_is_running) ?
					"start" : " stop",
		(ret != 0) ? "fail" : "done");

	return ret;
}

void htc_3dof_tran_service_get_send_work_task_en(uint8_t *enable)
{
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;

	*enable = p_3dof_data->send_work_thread_is_running;
}

int htc_3dof_tran_srv_pm_notify_func(uint32_t flag, uint32_t state,
								void *data)
{
	int ret = 0;
	//struct __3dof_transfer_service_data *p_3dof_data =
	//						&g_3dof_service_data;

	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT) {
#ifdef HTC_3DOF_POLLING_TEST_EN
			htc_3dof_tran_service_set_timer_enable(0);
#endif /* HTC_3DOF_POLLING_TEST_EN */
			ret = htc_3dof_tran_service_set_send_work_task_en(0);
		} else
		if (state == STOP_LEAVE || state == PLUG_IN) {
			ret = htc_3dof_tran_service_set_send_work_task_en(1);
#ifdef HTC_3DOF_POLLING_TEST_EN
			htc_3dof_tran_service_set_timer_enable(1);
#endif /* HTC_3DOF_POLLING_TEST_EN */
		}
	} else {
		htc_3dof_tran_warning("%s: unsupported flag\r\n", __func__);
	}
	return ret;
}

static int __create_pm_notify(
			struct __3dof_transfer_service_data *p_3dof_data)
{
	struct pwrmgr_notify_func_data *pm_not_p =
					&p_3dof_data->pm_notify_data;

	pm_not_p->func_name = "3dof_tran_srv";
	pm_not_p->data = p_3dof_data;
	pm_not_p->callback = htc_3dof_tran_srv_pm_notify_func;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_APP_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}

#ifdef HTC_3DOF_POLLING_TEST_EN
static void __htc_3dof_tran_service_timer(void const *argument)
{
	int loop_i;
	uint8_t data[HTC_3DOF_MAX_SENSOR_DATA_SIZE];
	static uint8_t tmp = 0x0;

	for (loop_i = 0; loop_i < HTC_3DOF_MAX_SENSOR_DATA_SIZE; loop_i++)
		data[loop_i] = tmp;

	if (tmp == 0xFF)
		tmp = 0;
	else
		tmp++;

	htc_3dof_tran_service_send_data((uint8_t *)&data,
					HTC_3DOF_MAX_SENSOR_DATA_SIZE);
}

static int __create_test_timer(struct __3dof_transfer_service_data
							*p_3dof_data)
{
	p_3dof_data->timer_def.ptimer = __htc_3dof_tran_service_timer;
	p_3dof_data->timer = osTimerCreate(
		&p_3dof_data->timer_def,
		osTimerPeriodic,
		p_3dof_data);
	if (p_3dof_data->timer == NULL)
		return -1;

	return 0;
}
#endif /* HTC_3DOF_POLLING_TEST_EN */

static int __create_send_work_task(
			struct __3dof_transfer_service_data *p_3dof_data)
{
	osThreadDef(s_work_task, __htc_3dof_send_work_task_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
	p_3dof_data->send_work_thread =
		osThreadCreate(osThread(s_work_task), p_3dof_data);
	if (p_3dof_data->send_work_thread == NULL)
		return -1;

	p_3dof_data->send_work_thread_is_running = 1;

	return 0;
}

static int __htc_3dof_tran_service_initial(void)
{
	struct __3dof_transfer_service_data *p_3dof_data =
						&g_3dof_service_data;
	if (p_3dof_data->is_initialized) {
		htc_3dof_tran_info("%s: already initialized, exit\n", __func__);
		return 0;
	}

	if (__create_send_work_task(p_3dof_data) != 0)
		htc_3dof_tran_warning("%s: create send work task failed\n",
								__func__);

	if (__create_pm_notify(p_3dof_data) != 0)
		htc_3dof_tran_warning("%s: create pm notifier failed\n",
								__func__);

#ifdef HTC_3DOF_POLLING_TEST_EN
	if (__create_test_timer(p_3dof_data) != 0)
		htc_3dof_tran_warning("%s: create test timer failed\n",
								__func__);
#endif /* HTC_3DOF_POLLING_TEST_EN */

	if (sizeof(p_3dof_data->send_data) != HTC_3DOF_MAX_PACKET_SIZE)
		htc_3dof_tran_warning("%s: WARNING: data buffer not equal "
					"max packet size\n", __func__);

	memset(&p_3dof_data->send_data.sensor_data, 0xFF,
				sizeof(p_3dof_data->send_data.sensor_data));
	p_3dof_data->send_data.system_state = 0;

	p_3dof_data->is_initialized = 1;
	/* setup default value and action */
#ifdef HTC_3DOF_POLLING_TEST_EN
	htc_3dof_tran_service_set_timer_interval(HTC_3DOF_TEST_TIMER_INTERVAL);
	htc_3dof_tran_service_set_timer_enable(1);
#endif /* HTC_3DOF_POLLING_TEST_EN */

	return 0;
}

int htc_3dof_transfer_service_initial()
{
	int ret = 0;
	ret = __htc_3dof_tran_service_initial();
	if (ret != 0)
		htc_3dof_tran_info("%s: initial failed!!\n", __func__);
	else
		htc_3dof_tran_info("%s: initial successfully\n", __func__);

	htc_3dof_update_flashlog_4_dump();
	return ret;
}



