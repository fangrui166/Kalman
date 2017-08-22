#include "FreeRTOS.h"
#include "string.h"
#include "stdio.h"
#include "component.h"
#include "cmsis_os.h"

#include <usbd_comp.h>
#include <system_property.h>
#include <htc_usb_cdc_data_service.h>
#include <misc_data.h>

#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"

#define SYSTEM_PROPERTY_COMMAND_TIMEOUT		2000

struct __system_property_data {
	uint8_t is_initialized;
	osMessageQId sys_property_work_queue;
	osThreadId sys_prop_thread;
	uint8_t recv_data[COMP_CDC_DATA_FS_MAX_PACKET_SIZE];
	uint8_t recv_data_length;
	osSemaphoreId recv_data_semaphore;
	struct pwrmgr_notify_func_data pm_notify_data;
	enum sys_prop_service_state service_state;
};

static struct __system_property_data g_system_property_data = { 0 };

static int32_t system_property_get(uint8_t * buf, uint8_t len)
{
	char temp_buf[COMP_CDC_DATA_FS_MAX_PACKET_SIZE] = { 0 };
	uint32_t temp_bool;
	float temp_float;
	uint64_t temp_uint64;
	int32_t temp_int32;
	float RealScreenSize[2] = { 0 };
	float poly_value[8] = { 0 };
	double RecomRenTargetSize[2] = { 0 };
	switch (*buf) {
	case PROP_TRACKING_SYSTEM_NAME_STRING:
		getTrackingSystemName(temp_buf);
		if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_MODEL_NUMBER_STRING:
	    getModelName(temp_buf);
	    if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_SERIAL_NUMBER_STRING:
		getSerialNumber(temp_buf);
		if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_RENDER_MODEL_NAME_STRING:
	    sprintf((char *)&temp_buf, "property unsupported!!");
	    break;
	case PROP_MANUFACTURER_NAME_STRING:
		getManufacturerName(temp_buf);
		if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_TRACKING_FIRMWARE_VERSION_STRING:
		getTrackingFirmwareVersion(temp_buf);
		if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_HARDWARE_REVISION_STRING:
		getHardwareRevision_string(temp_buf);
		if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_DEVICE_IS_WIRELESS_BOOL:
		temp_bool = getDeviceIsWireless();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_DEVICE_IS_CHARGING_BOOL:
		temp_bool = getDeviceIsCharging();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_DEVICE_BATTERY_PERCENTAGE_FLOAT:
		temp_float = getDeviceBatteryPercentage();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_FIRMWARE_UPDATE_AVAILABLE_BOOL:
		temp_bool = getFirmware_UpdateAvailable();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_HARDWARE_REVISION_UINT64:
		temp_uint64 = getHardwareRevision_int();
		sprintf((char *)&temp_buf, "%llu", temp_uint64);
		break;
	case PROP_FIRMWARE_VERSION_UINT64:
		temp_uint64 = getFirmwareVersion();
		sprintf((char *)&temp_buf, "%llu", temp_uint64);
		break;
	case PROP_CONTAINS_PROXIMITY_SENSOR_BOOL:
		temp_bool = getContainsProximitySensor();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_PROVIDES_BATTERY_STATUS_BOOL:
		temp_bool = getDeviceProvidesBatteryStatus();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_DEVICE_CAN_POWEROFF_BOOL:
		temp_bool = getDeviceCanPowerOff();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_HAS_CAMERA_BOOL:
		temp_bool = getHasCamera();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_PROJECT_ID:
	    getProjectID(temp_buf);
	    if (strlen(temp_buf) == 0)
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_ENGINEERING_ID_PARAM_TYPE_INT32:
	    if(getEngineerID(&temp_int32) == 0)
	       sprintf((char *)&temp_buf, "%d", temp_int32);
	    else
	       sprintf((char *)&temp_buf, "Undefined");
	    break;
	/*************** Tracking Device Class - HMD  **************/
	case PROP_USER_IPD_METERS_FLOAT:
		temp_float = getUserIpdMeters();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_LENS_CENTER_LEFT_U_FLOAT:
		temp_float = getLensCenterLeftU();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_LENS_CENTER_LEFT_V_FLOAT:
		temp_float = getLensCenterLeftV();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_LENS_CENTER_RIGHT_U_FLOAT:
		temp_float = getLensCenterRightU();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_LENS_CENTER_RIGHT_V_FLOAT:
		temp_float = getLensCenterRightV();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_USER_HEAD_TO_EYE_DEPTH_METERS_FLOAT:
		temp_float = getUserHeadToEyeDepthMeters();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_DISPLAY_FREQUENCY_FLOAT:
	    temp_float = getDisplayFrequency();
	    if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_SCREENSHOT_HORIZONTAL_FIELD_OF_VIEW_DEGREES_FLOAT:
	    temp_float = getScreenshotHorizontalFOV();
	    if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_SCREENSHOT_VERTICAL_FIELD_OF_VIEW_DEGREES_FLOAT:
	    temp_float = getScreenshotVerticalFOV();
	    if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_DEVICE_CLASS_INT32:
		if(getDeviceClass(&temp_int32) == 0)
			sprintf((char *)&temp_buf, "0x%02X", temp_int32);
		else
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_CONTAIN_RECENTER_BOOL:
		/* return true */
		temp_bool = getContainsRecenter();
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
		break;
	case PROP_CONTAINS6DOF_BOOL:
	    temp_bool = 0;
		sprintf((char *)&temp_buf, "0x%02X", temp_bool);
	    break;
	case PROP_TRACKING_MODEL_TYPE_INT32:
		temp_int32 = 0;
		sprintf((char *)&temp_buf, "0x%02X", temp_int32);
	    break;

	/*************** Vendor Specific Calss  **************/
	case PROP_VENDOR_DISPALY_RENDER_OVER_FILL_FLOAT:
		temp_float = getRenderOverfill();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_VENDOR_DISPALY_DISTORTION_DISTANCE_EYE_TO_LENS_FLOAT:
		temp_float = getDistanceEyeToLens();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_VENDOR_DISPALY_DISTORTION_DISTANCE_LENS_TO_SCREEN_FLOAT:
		temp_float = getDistanceLensToScreen();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_VENDOR_DISPALY_DISTORTION_LENS_FOCAL_LENGTH_FLOAT:
		temp_float = getLensFocalLength();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
/*
	case PROP_VENDOR_DISPALY_DISTORTION_VENDOR_PART_NUMBER_STRING:
 		FIXME: add new one
		break;
*/
	case PROP_VENDOR_DISPALY_DISTORTION_DISTANCE_SCALE_X_FLOAT:
		temp_float = getDistanceScaleX();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_VENDOR_DISPALY_DISTORTION_DISTANCE_SCALE_Y_FLOAT:
		temp_float = getDistanceScaleY();
		if (temp_float < 0)
			sprintf((char *)&temp_buf, "Undefined");
		else
			sprintf((char *)&temp_buf, "%f", temp_float);
		break;
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_RED_FLOAT_ARRAY_PART1:
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_RED_FLOAT_ARRAY_PART2:
		if (getPolynomialCoeffsRed((float *)&poly_value) != 0) {
			sprintf((char *)&temp_buf, "Undefined");
		} else {
			if (*buf == PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_RED_FLOAT_ARRAY_PART1)
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[0], poly_value[1],
					poly_value[2], poly_value[3]);
			else // part2
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[4], poly_value[5],
					poly_value[6], poly_value[7]);
		}
		break;
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_GREEN_FLOAT_ARRAY_PART1:
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_GREEN_FLOAT_ARRAY_PART2:
		if (getPolynomialCoeffsGreen((float *)&poly_value) != 0) {
			sprintf((char *)&temp_buf, "Undefined");
		} else {
			if (*buf == PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_GREEN_FLOAT_ARRAY_PART1)
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[0], poly_value[1],
					poly_value[2], poly_value[3]);
			else // part2
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[4], poly_value[5],
					poly_value[6], poly_value[7]);
		}
		break;
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_BLUE_FLOAT_ARRAY_PART1:
	case PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_BLUE_FLOAT_ARRAY_PART2:
		if (getPolynomialCoeffsBlue((float *)&poly_value) != 0) {
			sprintf((char *)&temp_buf, "Undefined");
		} else {
			if (*buf == PROP_VENDOR_DISPALY_POLYNOMIAL_COEFF_BLUE_FLOAT_ARRAY_PART1)
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[0], poly_value[1],
					poly_value[2], poly_value[3]);
			else // part2
				sprintf((char *)&temp_buf,
					"%f,%f,%f,%f",
					poly_value[4], poly_value[5],
					poly_value[6], poly_value[7]);
		}
		break;
	case PROP_VENDOR_DISPALY_GET_RECOMM_RENDER_TARGET_SIZE_DOUBLE_ARRAY:
		if (getRecommendedRenderTargetSize((double *)&RecomRenTargetSize) == 0)
			sprintf((char *)&temp_buf, "%f,%f",
				RecomRenTargetSize[0], RecomRenTargetSize[1]);
		else
			sprintf((char *)&temp_buf, "Undefined");
		break;
	case PROP_VENDOR_DISPALY_GET_REAL_SCREEN_SIZE_FLOAT_ARRAY:
		if (getRealScreenSize((float *)&RealScreenSize) == 0)
			sprintf((char *)&temp_buf, "%f,%f",
					RealScreenSize[0], RealScreenSize[1]);
		else
			sprintf((char *)&temp_buf, "Undefined");
		break;

	default:
		sprintf(temp_buf, "property unsupported!!");
		break;
	}
	usb_cdc_transmit_data((uint8_t *)&temp_buf, strlen(temp_buf));
	return 0;
}

static int32_t system_property_set(uint8_t * buf, uint8_t len)
{
	return 0;
}

void __system_property_work_task_func(void const * argument)
{
	struct __system_property_data *sp_data =
			(struct __system_property_data *)argument;
	do {
		osSignalWait(0xFFFFFFFF, osWaitForever);

		if (sp_data->recv_data_length == 0)
			continue;

		if (sp_data->recv_data[1] == USB_PROPERTY_ACTION_GET)
			system_property_get(&sp_data->recv_data[2],
					sp_data->recv_data_length - 2);

		if (sp_data->recv_data[1] == USB_PROPERTY_ACTION_SET)
			system_property_set(&sp_data->recv_data[2],
					sp_data->recv_data_length - 2);

		sp_data->recv_data_length = 0;
		osSemaphoreRelease(sp_data->recv_data_semaphore);
	} while (1);
}


int32_t system_property(uint8_t * buf, uint8_t len)
{
	struct __system_property_data *sp_data = &g_system_property_data;

	if (buf == NULL)
		return -1;

	if (sp_data->service_state != SYS_PROP_SERVICE_RUNNING)
		return -2;

	if (osSemaphoreWait(sp_data->recv_data_semaphore,
			SYSTEM_PROPERTY_COMMAND_TIMEOUT) == osErrorOS) {
		htc_sys_property_err("%s: timeout!!\n", __func__);
		return -3;
	}
	if (len > COMP_CDC_DATA_FS_MAX_PACKET_SIZE)
		sp_data->recv_data_length = COMP_CDC_DATA_FS_MAX_PACKET_SIZE;
	else
		sp_data->recv_data_length = len;

	memset(&sp_data->recv_data, 0x0, sizeof(sp_data->recv_data));
	memcpy(&sp_data->recv_data, buf, sp_data->recv_data_length);
	osSignalSet(sp_data->sys_prop_thread, 1);
	return 0;
}

static int __create_system_property_work_task(
			struct __system_property_data *sp_data)
{
	osThreadDef(sysproperty_work_task, __system_property_work_task_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
	sp_data->sys_prop_thread =
		osThreadCreate(osThread(sysproperty_work_task), sp_data);
	if (sp_data->sys_prop_thread == NULL)
		return -1;

	sp_data->service_state = SYS_PROP_SERVICE_RUNNING;

	return 0;
}

/*
static int __terminate_system_property_work_task(
			struct __system_property_data *sp_data)
{
	if (sp_data->sys_prop_thread == NULL)
		return 0;

	osThreadTerminate(sp_data->sys_prop_thread);

	sp_data->sys_prop_thread = NULL;
	return 0;
}
*/

int set_sys_prop_serv_control(enum sys_prop_service_state state)
{
	int ret = 0;
	struct __system_property_data *sp_data = &g_system_property_data;
	enum sys_prop_service_state old_state = sp_data->service_state;

	if (sp_data->service_state == state) {
		htc_sys_property_info("%s: service state is same\r\n", __func__);
		return ret;
	}
	sp_data->service_state = state;
	if (state == SYS_PROP_SERVICE_RUNNING) {
		if (osThreadResume(sp_data->sys_prop_thread) != osOK) {
			htc_sys_property_err("%s: resume failed!!\n",
								__func__);
			ret = -2;
		}
	} else
	if (state == SYS_PROP_SERVICE_SUSPEND) {
		if (osThreadSuspend(sp_data->sys_prop_thread) != osOK) {
			htc_sys_property_err("%s: suspend failed!!\n",
								__func__);
			ret = -3;
		}
	}

	if (ret != 0)
		sp_data->service_state = old_state;

	htc_sys_property_debug("%s: service %s(%s)\r\n", __func__,
		(sp_data->service_state == SYS_PROP_SERVICE_SUSPEND) ? "SUSPEND" :
		(sp_data->service_state == SYS_PROP_SERVICE_RUNNING) ? "RUNNING" :
			"UNKNOWN",
		(ret != 0) ? "fail" : "done");

	return ret;
}

enum sys_prop_service_state get_sys_prop_serv_control(void)
{
	struct __system_property_data *sp_data = &g_system_property_data;

	return sp_data->service_state;
}

int sys_prop_pm_notify_func(uint32_t flag, uint32_t state, void *data)
{
	int ret = 0;
	//struct __system_property_data *sp_data =
	//			(struct __system_property_data *)data;
	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT)
			ret = set_sys_prop_serv_control(
					SYS_PROP_SERVICE_SUSPEND);
		else
		if (state == STOP_LEAVE || state == PLUG_IN)
			ret = set_sys_prop_serv_control(
					SYS_PROP_SERVICE_RUNNING);
	} else {
		htc_sys_property_info("%s: unsupported flag\r\n", __func__);
	}
	return ret;
}

static int __create_system_property_pm_notify_func(
			struct __system_property_data *sp_data)
{
	struct pwrmgr_notify_func_data *pm_not_p = &sp_data->pm_notify_data;

	pm_not_p->func_name = "sys_prop_srv";
	pm_not_p->data = sp_data;
	pm_not_p->callback = sys_prop_pm_notify_func;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_APP_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}

static int __create_system_property_semaphore(
			struct __system_property_data *sp_data)
{
	osSemaphoreDef(sysproperty_work_task_sema_t);
	sp_data->recv_data_semaphore = osSemaphoreCreate(
				osSemaphore(sysproperty_work_task_sema_t), 1);
	if (sp_data->recv_data_semaphore == NULL)
		return -1;

	return 0;
}

void system_property_init(void)
{
	struct __system_property_data *sp_data = &g_system_property_data;
	if (sp_data->is_initialized) {
		htc_sys_property_info("%s: already initialized, exit\n", __func__);
		return;
	}

	if (__create_system_property_work_task(sp_data) != 0)
		htc_sys_property_warning("%s: create work task failed\n",
								__func__);

	if (__create_system_property_pm_notify_func(sp_data) != 0)
		htc_sys_property_warning("%s: create pm notifier failed\n",
								__func__);

	if (__create_system_property_semaphore(sp_data) != 0)
		htc_sys_property_warning("%s: create work semaphore failed\n",
								__func__);

	sp_data->is_initialized = 1;
}
