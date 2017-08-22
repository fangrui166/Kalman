#include "FreeRTOS.h"
#include "timers.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"

#include "PowerManager.h"
#include "PowerManager_battery.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_system.h"
#include "x_pmic.h"
#include "periph_multiple_drv.h"
#include "max17050_fuel_gauge.h"
#include "bq2589x_charger.h"
#include "htc_adc_func.h"
#include "led_hal.h"
#include "gpio.h"
#include "rtc_drv.h"
#include "misc_data.h"
#include "htc_3dof_transfer_service.h"

struct __battery_service_data {
    osTimerId battery_stat_timer;
    osTimerDef_t battery_stat_timer_def;
    PmicDrvTypeDef *pmic_handle;
    struct pwrmgr_battery_info battery_info;
    unsigned char battery_stat_timer_is_running;
    uint8_t charge_fault_state;
    uint8_t ui_soc;
    uint8_t last_ui_soc;
    uint8_t last_raw_soc;
};

struct battery_update_data {
	uint8_t soc;
	uint8_t charger_stat;
	uint8_t battery_low_stat;
	uint8_t batt_overheat_stat;
	uint8_t batt_curr_stat;
	uint8_t ovp_stat;

};

__packed struct __charging_fault_stat {
	uint8_t charger_fault : 1;
	uint8_t overheat_fault : 1;
	uint8_t ovp_fault : 1;
};

uint8_t pa_test = 0;
static struct __battery_service_data batt_data = { 0 };
static struct pwrmgr_notify_func_data PmBattNotifyData = { 0 };
static struct battery_update_data batt_update_data = { 0 };
static struct __charging_fault_stat charging_fault_stat = { 0 };

extern pcbid_t pcb_id;
extern int bsp_pmic_get_charger_para(PmicDrvTypeDef * handle, chargerPara *chg_para);


TimerHandle_t get_battery_timer_handle()
{
    return batt_data.battery_stat_timer;
}
static int PWRMGR_battery_get_info(struct __battery_service_data *data)
{
    int ret = 0;
    ret = bsp_pmic_get_battery_info(data->pmic_handle, &(data->battery_info));

    return ret;
}

static int PWRMGR_get_battery_online(uint8_t *online)
{
	int ret = 0;
	float volt;

	ret = htc_get_batt_average_voltage(&volt);
	if (ret != 0) {
		pwrmgr_err("%s: get battery id voltage error\n", __func__);
		return ret;
	}

	if (volt > 2)
		*online = 0;
	else
		*online = 1;

	return ret;
}

int PRMGR_battery_get_uiSoc()
{
	uint8_t online;
	PWRMGR_get_battery_online(&online);
	if (online == 1)
		return batt_data.ui_soc;
	else
		return -1;
}

int PWRMGR_update_charging_fault_state()
{
	struct __charging_fault_stat *state = &charging_fault_stat;

	batt_data.charge_fault_state = ((!!state->charger_fault) |
					((!!state->overheat_fault) << 1) |
					((!!state->ovp_fault) << 2));

	return 0;
}

int PRMGR_update_battery_uiSoC(uint8_t soc)
{
	struct battery_update_data  *data = &batt_update_data;

	if (data->soc != soc) {
		data->soc = soc;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update battery soc.\n", __func__);
	} else
		pwrmgr_info("%s: UI_SoC no need to update.\n", __func__);

	return 0;
}

int PRMGR_update_battery_low_state(uint8_t batt_low_stat)
{
	struct battery_update_data  *data = &batt_update_data;
	if (data->battery_low_stat != batt_low_stat) {
		data->battery_low_stat = batt_low_stat;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update low battery state.\n", __func__);
	} else
		pwrmgr_info("%s: low battery state no need to update.\n", __func__);

	return 0;
}


int PRMGR_update_chg_state(uint8_t chg_state)
{
	struct battery_update_data  *data = &batt_update_data;

	if (data->charger_stat != chg_state) {
		data->charger_stat = chg_state;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update battery state.\n", __func__);
	} else
		pwrmgr_info("%s: charge state no need to update.\n", __func__);

	return 0;
}

int PRMGR_update_chg_ovp_state(uint8_t ovp_state)
{
	struct battery_update_data  *data = &batt_update_data;

	if (data->ovp_stat != ovp_state) {
		data->ovp_stat = ovp_state;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update charger OVP/UVP state.\n", __func__);
	} else
		pwrmgr_info("%s: OVP/UVP state no need to update.\n", __func__);

	return 0;
}

int PRMGR_update_battery_overheat_state(uint8_t batt_overheat_state)
{
	struct battery_update_data  *data = &batt_update_data;

	if (data->batt_overheat_stat != batt_overheat_state) {
		data->batt_overheat_stat = batt_overheat_state;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update battery overheat state.\n", __func__);
	} else
		pwrmgr_info("%s: overheat state no need to update.\n", __func__);

	return 0;
}


int PRMGR_update_battery_curr_state(uint8_t battery_curr_state)
{
	struct battery_update_data  *data = &batt_update_data;

	if (data->batt_curr_stat != battery_curr_state) {
		data->batt_curr_stat = battery_curr_state;
		htc_3dof_update_battery_state(data->soc, data->charger_stat, data->battery_low_stat,
			data->batt_overheat_stat, data->batt_curr_stat, data->ovp_stat);
		pwrmgr_info("%s: update battery curr state.\n", __func__);
	} else
		pwrmgr_info("%s: battery curr state no need to update.\n", __func__);

	return 0;
}

int PRMGR_battery_update_uiSoc()
{
	chargerPara chg_para;
	BOOT_MODE mode;
	uint8_t pg_stat = 0, battery_low_stat = 0, battery_current_stat = 0;
	int temperature = 0;

	struct __battery_service_data *data = &batt_data;
	PWRMGR_battery_get_info(data);
	mode = get_bootmode();
	bsp_pmic_get_charger_para(data->pmic_handle, &chg_para);
	pwrmgr_info("update_uiSoc: raw_soc = %d, last_ui_soc = %d, chg_stat = %d.\n",
		data->battery_info.battery_raw_soc, data->last_ui_soc, chg_para.chg_stat);
	/*pwrmgr_info("%s: data->battery_info.battery_raw_soc = %d, data->last_ui_soc = %d, chg_stat = %d,\
INLIM = %d, ICHG = %d, VREG = %d, IINDPM_STAT = %d, IDPM_LIM = %d,  ICO_OPTIMIZED = %d.\n",
	__func__, data->battery_info.battery_raw_soc, data->last_ui_soc, chg_para.chg_stat, chg_para.INLIM,
	chg_para.ICHG, chg_para.VREG, chg_para.IINDPM_STAT, chg_para.IDPM_LIM, chg_para.ICO_OPTIMIZED);*/
	if ((data->battery_info.battery_raw_soc >= 50) && mode == SYS_MFG_MODE &&
		(enum charging_status)chg_para.chg_stat != CHARGER_STATE_DISCHARGING) {
		pwrmgr_info("%s: in system MFG mode, soc > 50, stop charging.\n", __func__);
		bsp_pmic_set_charge_hizMode(data->pmic_handle, 1);
	}

	bsp_pmic_get_charger_para(data->pmic_handle, &chg_para);
	if (((enum charging_status)chg_para.chg_stat == CHARGER_STATE_PRECHARGING) ||
		((enum charging_status)chg_para.chg_stat == CHARGER_STATE_FASTCHARGING)) {
		if (data->battery_info.battery_raw_soc > 100) {
			data->ui_soc = 100;
			data->last_ui_soc = data->ui_soc;
			pwrmgr_info("%s: battery is too full, show green light!\n", __func__);
			/* SendLedEventToDeal(LED_EVENT_CHG_ING | LED_EVENT_OFF); */
			SendLedEventToDeal(LED_EVENT_CHG_FULL | LED_EVENT_ON);
			SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
			return 0;
		}

		if (data->battery_info.battery_current >= 0) {
			if (data->battery_info.battery_raw_soc - data->last_ui_soc >= 2) {
				data->ui_soc = data->last_ui_soc + 2;
			} else if (data->battery_info.battery_raw_soc - data->last_ui_soc == 1) {
				data->ui_soc = data->last_ui_soc + 1;
			} else if (data->battery_info.battery_raw_soc <= data->last_ui_soc) {
				data->ui_soc = data->last_ui_soc;
			} else
				pwrmgr_info("%s: ui_soc update error!\n", __func__);
		} else {
			if (data->last_ui_soc - data->battery_info.battery_raw_soc >= 2) {
				data->ui_soc = data->last_ui_soc - 2;
			} else if (data->last_ui_soc - data->battery_info.battery_raw_soc == 1) {
				data->ui_soc = data->last_ui_soc - 1;
			} else if (data->battery_info.battery_raw_soc >= data->last_ui_soc) {
				data->ui_soc = data->last_ui_soc;
			} else
				pwrmgr_info("%s: ui_soc update error!\n", __func__);
		}

		if (((data->battery_info.battery_current / 1000) <= 280) && (data->battery_info.battery_current > 0) &&
			((data->battery_info.battery_voltage / 1000) >= 4350) &&(data->last_ui_soc < 100))
			data->ui_soc = data->last_ui_soc + 1;

		if ((data->last_ui_soc <=15) &&((data->last_ui_soc -data->ui_soc) >= 2)) {
			battery_current_stat = 2;
			PRMGR_update_battery_curr_state(battery_current_stat);
		}

		if ((data->ui_soc == 0) && (data->battery_info.battery_current < 0)) {
			battery_current_stat = 1;
			PRMGR_update_battery_curr_state(battery_current_stat);
			pwrmgr_system_send_message(PM_SYS_CMD_LOW_BATTERY, 0);
		}

		if (data->ui_soc >= 100) {
			pwrmgr_debug("%s: battery is full, show green light!\n", __func__);
			/* SendLedEventToDeal(LED_EVENT_CHG_ING | LED_EVENT_OFF); */
			SendLedEventToDeal(LED_EVENT_CHG_FULL | LED_EVENT_ON);
			SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
			//add show green light API
		} else if (data->ui_soc < 100) {
			pwrmgr_debug("%s: battery is not full, show amber light!\n", __func__);
			/* SendLedEventToDeal(LED_EVENT_CHG_FULL | LED_EVENT_OFF); */
			SendLedEventToDeal(LED_EVENT_CHG_ING | LED_EVENT_ON);
			SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
			//add show red light API
		} else
			pwrmgr_err("%s: battery soc status UNKOWN!\n", __func__);
	} else if ((enum charging_status)chg_para.chg_stat == CHARGER_STATE_DONE) {
		if (data->battery_info.battery_raw_soc <= 99) {
			pwrmgr_info("%s: set EN_HIZ to 0 to restart charging!\n", __func__);
			bsp_pmic_set_charge_hizMode(data->pmic_handle, 0);
			data->ui_soc = 100;
		}
		pwrmgr_info("%s: charging done!\n", __func__);
	} else if ((enum charging_status)chg_para.chg_stat == CHARGER_STATE_DISCHARGING) {
		SendLedEventToDeal(LED_EVENT_CHG_ING | LED_EVENT_OFF);
		SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		if ((data->battery_info.battery_voltage / 1000) <= 3100) {
			data->ui_soc = 0;
			data->last_ui_soc = data->ui_soc;
			pwrmgr_info("%s: battery is empty, going to shut down!\n", __func__);
			pwrmgr_system_send_message(PM_SYS_CMD_LOW_BATTERY, 0);
		}

		if (data->last_ui_soc - data->battery_info.battery_raw_soc >= 2)
			data->ui_soc = data->last_ui_soc - 2;
		else if (data->last_ui_soc - data->battery_info.battery_raw_soc == 1)
			data->ui_soc = data->last_ui_soc - 1;
		else if (data->battery_info.battery_raw_soc >= data->last_ui_soc)
			data->ui_soc = data->last_ui_soc;
		else
			pwrmgr_info("%s: output current is small!\n", __func__);

		bsp_pmic_get_pg_status(data->pmic_handle, &pg_stat);
		if ( pg_stat && (data->battery_info.battery_raw_soc <= 99)) {
			if ((mode == SYS_MFG_MODE) && data->battery_info.battery_raw_soc >= 50) {
				pwrmgr_info("%s: in system MFG mode, set EN_HIZ to 1 to stop charging.\n", __func__);
				bsp_pmic_set_charge_hizMode(data->pmic_handle, 1);
			} else {
				temperature = (data->battery_info.battery_temperature) / 10;
				pwrmgr_info("%s: set EN_HIZ to 0 to restart charging!\n", __func__);
				bsp_pmic_set_charge_hizMode(data->pmic_handle, 0);
				if ((temperature >= 2) && (temperature < 50))
					data->ui_soc = 100;
			}
		}

		if ((data->ui_soc <= 15) && (data->ui_soc > 5)) {
			battery_low_stat = 2;
			PRMGR_update_battery_low_state(battery_low_stat);
		} else if (data->ui_soc <= 5) {
			battery_low_stat = 1;
			PRMGR_update_battery_low_state(battery_low_stat);
		} else
			pwrmgr_info("%s: low battery state is not appear.\n", __func__);

		if ((data->ui_soc == 0) && (data->battery_info.battery_voltage / 1000) <= 3400)
		{
			pwrmgr_info("%s: battery voltage is low, going to shut down!\n", __func__);
			pwrmgr_system_send_message(PM_SYS_CMD_LOW_BATTERY, 0);
		}

		if (data->ui_soc <= 10) {
			pwrmgr_info("%s: UI_SoC is under 10 percent, current SOC = %d!\n",
					__func__, data->ui_soc);
			SendLedEventToDeal(LED_EVENT_LOW_BAT | LED_EVENT_ON);	//add red LED blinking API
			SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		} else if ((data->ui_soc > 10) && (data->ui_soc <= 100)) {
			pwrmgr_info("%s: device discharging, UI_SOC normal, show green light\n", __func__);
			SendLedEventToDeal(LED_EVENT_LOW_BAT | LED_EVENT_OFF);
			SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		} else
			pwrmgr_info("%s: device discharging, UI_SOC is abnormal\n", __func__);
	}else
		pwrmgr_err("%s: get charging status ERROR!\n", __func__);
	data->last_raw_soc = data->battery_info.battery_raw_soc;
	data->last_ui_soc = data->ui_soc;
	return 0;
}

static void PWRMGR_battery_stat_func(void const *argument)
{
	uint8_t online;
	chargerPara chg_para;
	struct __battery_service_data *data = &batt_data;
	PmicData_t *pmic_data = (PmicData_t*)data->pmic_handle->pData;
	PWRMGR_get_battery_online(&online);
	if (online == 1) {
		bsp_pmic_get_charger_para(data->pmic_handle, &chg_para);
		PRMGR_battery_update_uiSoc();
		//PRMGR_update_chg_state(chg_para.chg_stat);
		PRMGR_update_battery_uiSoC(data->ui_soc);
		if (data->battery_info.battery_voltage <= pmic_data->weak_battery_voltage) {
			PWRMGR_SendNotify(PWRMGR_NOTIFY_BATTERY_STATE,
			data->battery_info.battery_voltage, LEVER_ASCEND_ORDER);
		}

		if (data->battery_info.battery_temperature > pmic_data->hot_temperature) {
			PWRMGR_SendNotify(PWRMGR_NOTIFY_OVERHEAT_STATE,
				(uint32_t)data->battery_info.battery_temperature, LEVER_ASCEND_ORDER);
		}
	} else
		pwrmgr_err("%s: battery is not exist!\n", __func__);
}

static int PWRMGR_update_charging_stat(CHG_STAT *charging_status)
{
	int ret = 0;
	uint8_t pg_stat;
	chargerPara chg_para;
	struct __battery_service_data *data = &batt_data;

	PWRMGR_get_battery_online(&charging_status->online);
	if (charging_status->online == 1) {
		bsp_pmic_get_pg_status(data->pmic_handle, &pg_stat);
		if (pg_stat == 0) {
			SendLedEventToDeal(LED_EVENT_CHG_FULL | LED_EVENT_OFF);
			pwrmgr_warning("%s: charger plug out, turn off the LED.\n", __func__);
		}
		PRMGR_battery_update_uiSoc();
		bsp_pmic_get_charger_para(data->pmic_handle, &chg_para);
		//PRMGR_update_chg_state(chg_para.chg_stat);
		PRMGR_update_chg_state(pg_stat);
		PRMGR_update_battery_uiSoC(data->ui_soc);
		charging_status->batt_temp = (data->battery_info.battery_temperature) / 10;
	} else
		pwrmgr_err("%s: battery is not exist\n", __func__);

	return ret;
}

static int PWRMGR_charging_fault_callback(int event)
{
	struct __battery_service_data *data = &batt_data;
	enum chg_fault_event_type event_type = (enum chg_fault_event_type) event;

	switch(event_type) {
	case CHG_FALUT_EVENT_UNKOWN:
		pwrmgr_err("%s: charger falut code unkown!\n", __func__);
		break;
	case CHG_FALUT_EVENT_NORMAL:
		charging_fault_stat.charger_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		pwrmgr_info("%s: charger no falut!\n", __func__);
		break;
	case CHG_FALUT_EVENT_INPUT:
		charging_fault_stat.charger_fault = 1;
		PWRMGR_update_charging_fault_state();
		SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_ON);
		SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		pwrmgr_err("%s: charger input fault!\n", __func__);
		break;
	case CHG_FALUT_EVENT_THERMAL:
		charging_fault_stat.charger_fault = 1;
		PWRMGR_update_charging_fault_state();
		SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_ON);
		SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		pwrmgr_err("%s: charger thernal fault!\n", __func__);
		break;
	case CHG_FALUT_EVENT_TIMER:
		charging_fault_stat.charger_fault = 1;
		PWRMGR_update_charging_fault_state();
		bsp_pmic_set_charger_enableDisable(data->pmic_handle, 0);
		SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_ON);
		SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_OFF);
		pwrmgr_err("%s: charger timer expired!\n", __func__);
		break;
	}

	return 0;
}

static int PWRMGR_charging_ovp_callback()
{
	uint8_t pg_stat;
	bool pin_stat;
	uint8_t ovp_state;

	struct __battery_service_data *data = &batt_data;

    if(XA0n == pcb_id){
		pin_stat = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0) ? 1 : 0;
    }
    else{
		pin_stat = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) ? 1 : 0;
    }
	osDelay(350);
	bsp_pmic_get_pg_status(data->pmic_handle, &pg_stat);
	if ((pg_stat == 0) && (pin_stat == 1)) {
		pwrmgr_err("%s: charger OVP/UVP!\n", __func__);
		charging_fault_stat.ovp_fault = 1;
		PWRMGR_update_charging_fault_state();
		SendLedEventToDeal(LED_EVENT_CHG_FAULT| LED_EVENT_ON);
		ovp_state = 1;
		PRMGR_update_chg_ovp_state(ovp_state);
	} else if ((pg_stat == 0) && (pin_stat == 0)) {
		pwrmgr_err("%s: charge voltage is low!\n", __func__);
		charging_fault_stat.ovp_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		ovp_state = 0;
		PRMGR_update_chg_ovp_state(ovp_state);
	} else if ((pg_stat == 1) && (pin_stat == 1)) {
		pwrmgr_err("%s: charge voltage is OK!\n", __func__);
		charging_fault_stat.ovp_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		ovp_state = 0;
		PRMGR_update_chg_ovp_state(ovp_state);
	} else
		pwrmgr_err("%s: AC plug in!\n", __func__);

	return 0;
}

static int PWRMGR_charging_led_callback()
{
	uint8_t pin_stat;
	if(XA0n == pcb_id){
		pin_stat = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0) ? 1 : 0;
	}
    else{
		pin_stat = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) ? 1 : 0;
    }

	if(pin_stat == 1)
	{
		SendLedEventToDeal(LED_EVENT_ALL_LEDS_OFF | LED_EVENT_ON);
		osDelay(1000);
	}

	return 0;
}

static int PWRMGR_battery_state_notify(int event)
{
	int temperature = 0;
	uint8_t batt_overheat_stat;
	struct __battery_service_data *data = &batt_data;
	enum batt_intr_event_type event_type = (enum batt_intr_event_type)event;
	pwrmgr_info("%s: event type = %d\n", __func__, event_type);

	PWRMGR_battery_get_info(data);
	temperature = (data->battery_info.battery_temperature) / 10;

	switch (event_type) {
	case BATT_INT_EVENT_UNKONW:
		pwrmgr_info("%s: battery status unkown!\n", __func__);
		break;
	case BATT_INT_EVENT_SOC_EMPTY:
		pwrmgr_info("%s:battery soc is empty, going to shutdown!\n", __func__);
		break;
	case BATT_INT_EVENT_SOC_LOW:
		break;
	case BATT_INT_EVENT_SOC_FULL:
		break;
	case BATT_INT_EVENT_TEMP_COLD:
		charging_fault_stat.overheat_fault = 1;
		PWRMGR_update_charging_fault_state();
		SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_ON);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		if (pa_test  == 0)
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 0);
		else
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
		break;
	case BATT_INT_EVENT_TEMP_2C:
		pwrmgr_info("%s: current temperature = %d.\n", __func__, temperature);
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		break;
	case BATT_INT_EVENT_TEMP_COOL:
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		if ((temperature >= 2) &&(temperature < 10)) {
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
			bsp_pmic_set_charge_voltageLimit(data->pmic_handle, 4400);

			if (data->battery_info.battery_voltage <= 4100)
				bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
			else
				bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
		}
		break;
	case BATT_INT_EVENT_TEMP_NORMAL:
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
		bsp_pmic_set_charge_voltageLimit(data->pmic_handle, 4400);

		if (data->battery_info.battery_voltage <= 4100)
			bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
		else
			bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
		break;
	case BATT_INT_EVENT_TEMP_WARM:
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
		bsp_pmic_set_charge_voltageLimit(data->pmic_handle, 4400);
		bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
		break;
	case BATT_INT_EVENT_TEMP_HOT:
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 0;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		if ((temperature < 55) && (temperature >=  50)) {
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
			bsp_pmic_set_charge_voltageLimit(data->pmic_handle, 4096);
			bsp_pmic_set_charge_currentLimit(data->pmic_handle, 2800);
		}
		break;
	case BATT_INT_EVENT_TEMP_55C:
		pwrmgr_info("%s: current temperature = %d.\n", temperature);
		charging_fault_stat.overheat_fault = 0;
		PWRMGR_update_charging_fault_state();
		if (data->charge_fault_state == 0)
			SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_OFF);
		batt_overheat_stat = 1;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		break;
	case BATT_INT_EVENT_TEMP_COOL_DOWN:
		charging_fault_stat.overheat_fault = 1;
		PWRMGR_update_charging_fault_state();
		SendLedEventToDeal(LED_EVENT_CHG_FAULT | LED_EVENT_ON);
		batt_overheat_stat = 1;
		PRMGR_update_battery_overheat_state(batt_overheat_stat);
		if (pa_test == 0)
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 0);
		else
			bsp_pmic_set_charger_enableDisable(data->pmic_handle, 1);
		break;
	default:
		break;
	}

	return 0;
}

static int PWRMGR_battery_service_enable(
        struct __battery_service_data *data, unsigned char enable)
{
	if (data->battery_stat_timer_is_running == 0 && enable != 0) {
		data->battery_stat_timer_def.ptimer = PWRMGR_battery_stat_func;
		data->battery_stat_timer = osTimerCreate(
				&data->battery_stat_timer_def,
				osTimerPeriodic,
				NULL);

		if (data->battery_stat_timer == NULL)
			return -1;

		if (osTimerStart(data->battery_stat_timer,
			PWRMGR_BATTERY_NORMAL_POLLING_CYCLE) != osOK)
			return -2;

		data->battery_stat_timer_is_running = 1;
		return 0;
	} else if (data->battery_stat_timer_is_running == 1 && enable == 0) {
		if (osTimerStop(data->battery_stat_timer) != osOK)
			return -3;

		if (osTimerDelete(data->battery_stat_timer) != osOK)
			return -4;

		data->battery_stat_timer_is_running = 0;
		return 0;
	}
	return -5;
}

static int PWRMGR_batt_notify_callback(uint32_t _notify_flag, uint32_t _state, void *pdata)
{
	uint8_t hour = 0, min = 0, sec = 0;
	bool pg_stat;
    if(XA0n == pcb_id){
		pg_stat = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0) ? 1 : 0;
	}
    else{
		pg_stat = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0) ? 1 : 0;
    }
	switch(_notify_flag) {
	case PWRMGR_NOTIFY_STOP_STATE:
		if (_state == STOP_ENTER) {
			rtc_drv_getTime(&hour, &min, &sec);
			pwrmgr_debug("%s: current time is %02d:%02d:%02d.\n", __func__, hour, min, sec);
			if (pg_stat == 0) {
				if (hour == 23)
					hour = 0;
				else
					hour += 1;
			} else {
				if (min == 59) {
					min = 0;
					if (hour != 23)
						hour += 1;
					else
						hour = 0;
				} else
					min += 1;
			}
			pwrmgr_debug("%s: set alarmA, %02d:%02d:%02d.\n", __func__, hour, min, sec);
			rtc_drv_setAlarmA(hour, min, sec);
		} else if (_state == STOP_LEAVE) {
			pwrmgr_debug("%s: leave stop mode, disable Alarm timer.\n", __func__);
			mcu_rtc_alarm_disable(RTC_ALARM_A);
		} else
			pwrmgr_err("%s: state ERROR.\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

static int PWRMGR_system_register_notify(void)
{
    PmBattNotifyData.func_name = "pm_batt";
    PmBattNotifyData.data = (void *)&batt_data;
    PmBattNotifyData.callback= PWRMGR_batt_notify_callback;
    PmBattNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE;
    PmBattNotifyData.func_level = PWRMGR_FUNC_APP_LEVEL;
    return PWRMGR_register_notify_func(&PmBattNotifyData);
}

int PWRMGR_battery_initial_service(PmicDrvTypeDef *handle)
{
	chargerPara chg_para;
	batt_data.pmic_handle = handle;
	if (PWRMGR_battery_service_enable(&batt_data, 1) != 0) {
	    pwrmgr_err("%s: enable battery stat service failed\n",
	                            __func__);
	    return -2;
	}

	PWRMGR_battery_get_info(&batt_data);
	bsp_pmic_get_charger_para(batt_data.pmic_handle, &chg_para);
	if (batt_data.battery_info.battery_raw_soc < 1) {
		if ((enum charging_status)chg_para.chg_stat == CHARGER_STATE_DISCHARGING) {
			pwrmgr_err("%s: battery is empty, cannot boot up!\n", __func__);
			SendLedEventToDeal(LED_EVENT_LOW_BAT | LED_EVENT_ON);
		} else if (((enum charging_status)chg_para.chg_stat == CHARGER_STATE_PRECHARGING) ||
				((enum charging_status)chg_para.chg_stat == CHARGER_STATE_FASTCHARGING)) {
			if (batt_data.battery_info.battery_current < 0) {
				pwrmgr_err("%s: battery is empty, Enter stop mode!\n", __func__);
				SendLedEventToDeal(LED_EVENT_CHG_ING| LED_EVENT_ON);
			}
		} else
			pwrmgr_err("%s: charging state ERROR.\n", __func__);

		pwrmgr_system_send_message(PM_SYS_CMD_LOW_BATTERY, 0);
	}
	batt_data.last_ui_soc = batt_data.battery_info.battery_raw_soc;
	batt_data.ui_soc = batt_data.battery_info.battery_raw_soc;

	max17050_register_batt_status_notify(PWRMGR_battery_state_notify);
	max17050_register_get_batt_online(PWRMGR_get_battery_online);
	bq2589x_register_update_charging_stat(PWRMGR_update_charging_stat);
	bq2589x_register_charge_fault_callback(PWRMGR_charging_fault_callback);
	bq2589x_register_charge_ovp_callback(PWRMGR_charging_ovp_callback);
	bq2589x_register_charge_led_callback(PWRMGR_charging_led_callback);
	PWRMGR_system_register_notify();

	return 0;
}
