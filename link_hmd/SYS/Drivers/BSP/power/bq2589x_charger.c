#include "integer.h"
#include "cmsis_os.h"
#include "rtos_i2c_drv.h"
#include <string.h>
#include "bq2589x_charger.h"
#include "gpio.h"
#include "max17050_fuel_gauge.h"


typedef int (*UPDATE_CHARGING_STAT)(CHG_STAT *);
typedef int (*CHARGING_FAULT_CALLBACK)(int);
typedef int (*CHARGING_OVP_CALLBACK)(void);
typedef int (*CHARGING_LED_CALLBACK)(void);


extern pcbid_t pcb_id;

enum bq2589x_part_no {
	BQ25890 = 0x03,
	BQ25896 = 0x00,
	BQ25895 = 0x07,
};

struct bq2589x_config {
	bool	enable_auto_dpdm;
	bool	enable_term;
	bool	enable_ico;
	bool use_absolute_vindpm;
/*	bool	enable_12v;*/

	int charge_volt;
	int charge_curr;
	int term_curr;
	int prechg_volt_threshould;
	int prechg_curr;
	int input_volt;
};


struct bq2589x_device {
	struct bq2589x_config cfg;
	PmicDrvTypeDef *handle;
	int revision;
	//unsigned int status;
	int vbus_type;
	//bool	enabled;
	int vbus_volt;
	int vbat_volt;
	int rsoc;
	float usb_temp;

	osMutexId bq2589x_i2c_lock;
	osThreadId intr_handler;
	BaseType_t intr_handler_token;

	osThreadId charger_work_handler;
	osMessageQId chargerMsgQueueHandle;

	enum bq2589x_part_no part_no;

	UPDATE_CHARGING_STAT update_charging_stat;
	CHARGING_FAULT_CALLBACK charging_fault_callback;
	CHARGING_OVP_CALLBACK charging_ovp_callback;
	CHARGING_LED_CALLBACK charging_led_callback;
};

static struct bq2589x_device bq2589x_dev = { 0 };

static int bq2589x_read_byte(struct bq2589x_device *bq, uint8_t *data, uint8_t reg)
{
	I2C_STATUS ret = I2C_OK;

	osMutexWait(bq->bq2589x_i2c_lock, portMAX_DELAY);
	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CHARGER_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, BQ2589x_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		chg_err("%s I2C write error ret=%d\n",__func__,ret);
		osMutexRelease(bq->bq2589x_i2c_lock);
		return -1;
	}
	osMutexRelease(bq->bq2589x_i2c_lock);

	return 0;
}

static int bq2589x_write_byte(struct bq2589x_device *bq, uint8_t reg, uint8_t *data)
{
	I2C_STATUS ret = I2C_OK;

	osMutexWait(bq->bq2589x_i2c_lock, portMAX_DELAY);
	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_CHARGER_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, BQ2589x_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		chg_err("%s I2C write error ret=%d\n",__func__,ret);
		osMutexRelease(bq->bq2589x_i2c_lock);
		return -1;
	}
	osMutexRelease(bq->bq2589x_i2c_lock);
	return 0;
}

static int bq2589x_update_bits(struct bq2589x_device *bq, uint8_t reg, uint8_t mask, uint8_t data)
{
	int ret;
	uint8_t tmp;

	ret = bq2589x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, &tmp);
}

static int bq2589x_enable_otg(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}

static int bq2589x_disable_otg(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}

int bq2589x_set_otg_volt(struct bq2589x_device *bq, int volt)
{
	uint8_t val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);

}

int bq2589x_set_otg_curr(struct bq2589x_device *bq, int curr)
{
	uint8_t temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 750)
		temp = BQ2589X_BOOST_LIM_750MA;
	else if (curr == 1200)
		temp = BQ2589X_BOOST_LIM_1200MA;
	else if (curr == 1400)
		temp = BQ2589X_BOOST_LIM_1400MA;
	else if (curr == 1650)
		temp = BQ2589X_BOOST_LIM_1650MA;
	else if (curr == 1875)
		temp = BQ2589X_BOOST_LIM_1875MA;
	else if (curr == 2150)
		temp = BQ2589X_BOOST_LIM_2150MA;
	else
		temp = BQ2589X_BOOST_LIM_1400MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}

static int bq2589x_enableDisable_charger(struct bq2589x_device *bq, bool enable)
{
	int ret;
	uint8_t val;
	if (enable) {
		chg_info("%s: enable charger!\n", __func__);
		val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	} else {
		chg_info("%s: disable charger!\n", __func__);
		val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	}
	return ret;
}

static int bq2589x_set_precharge_volt_threshould(struct bq2589x_device *bq, int volt)
{
	uint8_t val;

	if (volt == 2800)
		val = BQ2589X_BATLOWV_2800MV;
	else if (volt == 3000)
		val = BQ2589X_BATLOWV_3000MV;
	else
		chg_info("%s: invalid precherge to fastcharge threshould!\n", __func__);

	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_BATLOWV_MASK, val << BQ2589X_BATLOWV_SHIFT);
}

static int bq2589x_set_precharge_curr(struct bq2589x_device *bq, int curr)
{
	uint8_t val = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, val << BQ2589X_IPRECHG_SHIFT);

}

/* interfaces that can be called by other module */
int bq2589x_adc_start(struct bq2589x_device *bq, bool oneshot)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		chg_err("%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) ==
									BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot) {
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
								BQ2589X_CONV_START_MASK,
								BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
		chg_err("%s, start one shot ADC conversion!\n", __func__);
	}
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
								BQ2589X_CONV_RATE_MASK,
								BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);

	return ret;
}

int bq2589x_adc_stop(struct bq2589x_device *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02,
							BQ2589X_CONV_RATE_MASK,
							BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}

int bq2589x_adc_read_battery_volt(struct bq2589x_device *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		chg_err("read battery volt failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}


int bq2589x_adc_read_sys_volt(struct bq2589x_device *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		chg_err("read system volt failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}


uint32_t bq2589x_adc_read_vbus_volt(struct bq2589x_device *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		chg_err("read vbus volt failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}

int bq2589x_adc_read_temperature(struct bq2589x_device *bq)
{
	uint8_t val;
	int temp;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		chg_err("read temperature failed :%d\n", ret);
		return ret;
	} else{
		temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		return temp;
	}
}

int bq2589x_adc_read_charge_curr(struct bq2589x_device *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		chg_err("read charge curr failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}

int bq2589x_set_chargecurr(struct bq2589x_device *bq, int curr)
{
	uint8_t ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}

int bq2589x_get_chargecurr(struct bq2589x_device *bq, int *curr)
{
	uint8_t val;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_04);
	if (ret < 0) {
		chg_err("read charge curr failed :%d\n", ret);
		return ret;
	}

	*curr = (((val & BQ2589X_ICHG_MASK) * BQ2589X_ICHG_LSB) + BQ2589X_ICHG_BASE);
	return 0;

}

int bq2589x_set_term_curr(struct bq2589x_device *bq, int curr)
{
	uint8_t iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}


int bq2589x_set_prechg_curr(struct bq2589x_device *bq, int curr)
{
	uint8_t iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}

int bq2589x_set_chargevolt(struct bq2589x_device *bq, int volt)
{
	uint8_t val;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}

int bq2589x_get_chargevolt(struct bq2589x_device *bq, int *volt)
{
	uint8_t val;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_06);
	if (ret < 0) {
		chg_err("read charge curr failed :%d\n", ret);
		return ret;
	}

	*volt = ((((val & BQ2589X_VREG_MASK) >> BQ2589X_VREG_SHIFT)* BQ2589X_VREG_LSB) + BQ2589X_VREG_BASE);
	return 0;
}

int bq2589x_set_vrecharge(struct bq2589x_device *bq, int volt)
{
	uint8_t val;

	if (volt == 100)
		val = BQ2589X_VRECHG_100MV;
	else if (volt == 200)
		val = BQ2589X_VRECHG_200MV;
	else
		chg_err("%s: recharge volt set ERROR!\n", __func__);

	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VRECHG_MASK, val << BQ2589X_VRECHG_SHIFT);
}


int bq2589x_set_input_volt_limit(struct bq2589x_device *bq, int volt)
{
	uint8_t val;
	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}

int bq2589x_set_input_curr_limit(struct bq2589x_device *bq, uint32_t curr)
{
	uint8_t val;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}

int bq2589x_get_input_curr_limit(struct bq2589x_device *bq, uint32_t *curr)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret < 0) {
		chg_err("%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	*curr = (((val & BQ2589X_IINLIM_MASK) * BQ2589X_IINLIM_LSB) + BQ2589X_IINLIM_BASE);

	return 0;
}


int bq2589x_get_IINDPM_state(struct bq2589x_device *bq, uint8_t* state)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		chg_err("%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	val &= BQ2589X_IDPM_STAT_MASK;
	val >>= BQ2589X_IDPM_STAT_SHIFT;
	*state = val;

	return 0;
}


int bq2589x_set_vindpm_offset(struct bq2589x_device *bq, int offset)
{
	uint8_t val;

	val = (offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}

int bq2589x_get_charging_status(struct bq2589x_device *bq)
{
	uint8_t val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		chg_err("%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}

int htc_get_charger_state(void)
{
	struct bq2589x_device *bq = &bq2589x_dev;
	return bq2589x_get_charging_status(bq);
}

int bq2589x_get_pg_status(struct bq2589x_device *bq)
{
	uint8_t val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		chg_err("%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}

	val &= BQ2589X_PG_STAT_MASK;
	val >>= BQ2589X_PG_STAT_SHIFT;

	return val;
}

void bq2589x_set_otg(struct bq2589x_device *bq, int enable)
{
	int ret;

	if (enable) {
		ret = bq2589x_enable_otg(bq);
		if (ret < 0) {
			chg_err("%s:Failed to enable otg-%d\n", __func__, ret);
			return;
		}
	} else{
		ret = bq2589x_disable_otg(bq);
		if (ret < 0)
			chg_err("%s:Failed to disable otg-%d\n", __func__, ret);
	}
}

int bq2589x_set_watchdog_timer(struct bq2589x_device *bq, uint8_t timeout)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK,
			(uint8_t)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}

int bq2589x_disable_watchdog_timer(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}

int bq2589x_disable_stat_pin(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_CHG_STAT_PIN_DISABLE << BQ2589X_EN_STAT_PIN_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_STAT_PIN_MASK, val);
}

int bq2589x_enable_stat_pin(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_CHG_STAT_PIN_ENABLE << BQ2589X_EN_STAT_PIN_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_STAT_PIN_MASK, val);
}

int bq2589x_reset_watchdog_timer(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

int bq2589x_force_dpdm(struct bq2589x_device *bq)
{
	int ret;
	uint8_t val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if (ret)
		return ret;

	osDelay(20);/*TODO: how much time needed to finish dpdm detect?*/
	return 0;

}

int bq25896_set_tmr2x(struct bq2589x_device *bq, uint8_t enable)
{
	int ret;
	uint8_t val = enable << BQ2589X_TMR2X_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_TMR2X_EN_MASK, val);
	return ret;
}

int bq2589x_reset_chip(struct bq2589x_device *bq)
{
	int ret;
	uint8_t val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	return ret;
}

int bq2589x_enter_ship_mode(struct bq2589x_device *bq)
{
	int ret;
	uint8_t val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	return ret;

}

int bq2589x_enter_hiz_mode(struct bq2589x_device *bq)
{
	uint8_t val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}

int bq2589x_exit_hiz_mode(struct bq2589x_device *bq)
{

	uint8_t val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}

int bq2589x_get_hiz_mode(struct bq2589x_device *bq, uint8_t *state)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}


int bq2589x_pumpx_enable(struct bq2589x_device *bq, int enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}


int bq2589x_pumpx_increase_volt(struct bq2589x_device *bq)
{
	uint8_t val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}

int bq2589x_pumpx_increase_volt_done(struct bq2589x_device *bq)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}

int bq2589x_pumpx_decrease_volt(struct bq2589x_device *bq)
{
	uint8_t val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}

int bq2589x_pumpx_decrease_volt_done(struct bq2589x_device *bq)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}

static int bq2589x_check_ico_done(struct bq2589x_device *bq)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}

static int bq2589x_enable_term(struct bq2589x_device* bq, bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);

	return ret;
}

static int bq2589x_enable_auto_dpdm(struct bq2589x_device* bq, bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}

static int bq2589x_use_absolute_vindpm(struct bq2589x_device* bq, bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}

static int bq2589x_enable_ico(struct bq2589x_device* bq, bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}


int bq2589x_read_idpm_limit(struct bq2589x_device *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		chg_err("read vbus volt failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
		return curr;
	}
}

bool bq2589x_is_charge_done(struct bq2589x_device *bq)
{
	int ret;
	uint8_t val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		chg_err("%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}

static int bq2589x_init_device(struct bq2589x_device *bq)
{
	int ret;

    /*common initialization*/

	bq2589x_disable_watchdog_timer(bq);	//Disable watchdog timer
	bq2589x_disable_stat_pin(bq);

	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	bq2589x_enable_ico(bq, bq->cfg.enable_ico);
	/*force use absolute vindpm if auto_dpdm not enabled*/
	if (!bq->cfg.enable_auto_dpdm)
		bq->cfg.use_absolute_vindpm = true;
	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);


	ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		chg_err("%s:Failed to set vindpm offset:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_precharge_volt_threshould(bq, bq->cfg.prechg_volt_threshould);
	if (ret < 0) {
		chg_err("%s: Failed to set prechg volt threshould:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_precharge_curr(bq, bq->cfg.prechg_curr);
	if (ret < 0) {
		chg_err("%s: Failed to set prechg curr:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_term_curr(bq, bq->cfg.term_curr);
	if (ret < 0) {
		chg_err("%s:Failed to set termination curr:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargevolt(bq, bq->cfg.charge_volt);
	if (ret < 0) {
		chg_err("%s:Failed to set charge volt:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_input_volt_limit(bq, bq->cfg.input_volt);
	if (ret < 0) {
		chg_err("%s:Failed to set input volt:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurr(bq, bq->cfg.charge_curr);
	if (ret < 0) {
		chg_err("%s:Failed to set charge curr:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enableDisable_charger(bq, 1);
	if (ret < 0) {
		chg_err("%s:Failed to enable charger:%d\n", __func__, ret);
		return ret;
	}

	bq2589x_adc_start(bq, false);

	ret = bq2589x_pumpx_enable(bq, 1);
	if (ret) {
		chg_err("%s:Failed to enable pumpx:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int bq2589x_show_registers(struct bq2589x_device *dev)
{
	uint8_t addr;
	uint8_t reg[20];
	int ret ;
	struct bq2589x_device *bq = dev;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &reg[addr], addr);
		if (ret == 0) {
			chg_debug("reg[%d] = 0x%x\n", addr, reg[addr]);
		} else {
			chg_err("%s, I2C read error!\n", __func__);
			return -1;
		}
	}

	return 0;
}

static int bq2589x_get_device_data(struct bq2589x_device *bq)
{
	bq->cfg.enable_auto_dpdm = 1;
	bq->cfg.enable_term = 0;
	bq->cfg.enable_ico = 1;
	bq->cfg.use_absolute_vindpm = 0;
	bq->cfg.charge_volt = 4400;
	bq->cfg.input_volt = 4500;
	bq->cfg.prechg_volt_threshould = 3000;
	bq->cfg.charge_curr = 2800;
	bq->cfg.prechg_curr = 64;
	//bq->cfg.charge_volt = 4208;
	bq->cfg.term_curr = 64;

	return 0;
}

static int bq2589x_detect_device(struct bq2589x_device *bq)
{
	int ret;
	uint8_t data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no = (enum bq2589x_part_no)((data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT);
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}

int bq2589x_register_charge_fault_callback(int (*callback)(int))
{
	int ret = 0;

	if (callback)
		bq2589x_dev.charging_fault_callback = callback;
	else
		return -1;

	return ret;
}

int bq2589x_register_charge_ovp_callback(int (*callback)(void))
{
	int ret = 0;

	if (callback)
		bq2589x_dev.charging_ovp_callback = callback;
	else
		return -1;

	return ret;
}

int bq2589x_register_charge_led_callback(int (*callback)(void))
{
	int ret = 0;

	if (callback)
		bq2589x_dev.charging_led_callback = callback;
	else
		return -1;

	return ret;
}

int bq2589x_register_update_charging_stat(int (*callback)(CHG_STAT *))
{
	int ret = 0;

	if (callback)
		bq2589x_dev.update_charging_stat = callback;
	else
		return -1;

	return ret;
}

static void bq2589x_charger_irq_workfunc(struct bq2589x_device *dev)
{
	uint8_t status = 0;
	uint8_t fault = 0;
	uint8_t charge_status = 0;
	uint8_t fault_status = 0;
	uint8_t idpm_status = 0;
	uint8_t IINDPM_STAT = 0;
	uint32_t INLIM = 0;
	CHG_STAT charging_stat;
	int idpm;
	int ret;

	struct bq2589x_device * bq = dev;
	enum chg_fault_event_type event_type = CHG_FALUT_EVENT_UNKOWN;
	osDelay(5);

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret)
		return;
	chg_info("%s: fault REG = %#x.\n", __func__, fault);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret)
		return;
	chg_info("%s: REG0B val = %#x\n", __func__, status);
	if ((status & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT)
		chg_info("%s: Power Good source detected!\n", __func__);
	else
		chg_info("%s: Power Good source removed!\n", __func__);
	if (bq->update_charging_stat != NULL)
		bq->update_charging_stat(&charging_stat);
	if (charging_stat.online == 1) {
		//osDelay(1000);
		bq2589x_get_IINDPM_state(&bq2589x_dev, &IINDPM_STAT);
		ret = bq2589x_check_ico_done(bq);
		chg_info("%s: ICO_OPTIMIZED = %d, IINDPM_STAT = %d.\n", __func__, ret, IINDPM_STAT);
		if ((ret == 1) && (IINDPM_STAT == 1)) {/*ico done*/
			ret = bq2589x_read_byte(bq, &idpm_status, BQ2589X_REG_13);
			if (ret == 0) {
				idpm = ((idpm_status & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) *
					BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
				chg_info("%s:ICO done, result is:%d mA\n", __func__, idpm);

				if (idpm >=500 && idpm < 1000)
					bq2589x_set_input_curr_limit(bq, 500);
				else if (idpm >=1000 && idpm < 1500)
					bq2589x_set_input_curr_limit(bq, 1000);
				else if (idpm >=1500 && idpm < 2000)
					bq2589x_set_input_curr_limit(bq, 1500);
				else if (idpm >=2000)
					bq2589x_set_input_curr_limit(bq, 2000);
				/*else if (idpm >=2500)
					bq2589x_set_input_curr_limit(bq, 2500);*/
				else
					chg_info("%s: idpm = %d ERROR\n", __func__, idpm);
			}
		}else {
			chg_info("%s: set charger input current to 2000mA.\n", __func__);
			bq2589x_set_input_curr_limit(bq, 2000);
		}

		bq2589x_get_input_curr_limit(&bq2589x_dev, &INLIM);
		chg_info("%s: charger INLIM = %d.\n", __func__, INLIM);
		if (INLIM > 1000)
			bq25896_set_tmr2x(bq, 0);
		else
			bq25896_set_tmr2x(bq, 1);
		/* Read STATUS and FAULT registers */

		charge_status = (status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
		if (charge_status == BQ2589X_CHRG_STAT_IDLE)
			chg_info("%s:not charging\n", __func__);
		else if (charge_status == BQ2589X_CHRG_STAT_PRECHG)
			chg_info("%s:precharging\n", __func__);
		else if (charge_status == BQ2589X_CHRG_STAT_FASTCHG)
			chg_info("%s:fast charging\n", __func__);
		else if (charge_status == BQ2589X_CHRG_STAT_CHGDONE) {
			bq2589x_enter_hiz_mode(bq);
			chg_info("%s:charge done!\n", __func__);
		}

		fault_status = (fault & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
		if (fault_status == BQ2589X_FAULT_CHRG_NORMAL)
			event_type = CHG_FALUT_EVENT_NORMAL;
		else if (fault_status == BQ2589X_FAULT_CHRG_INPUT)
			event_type = CHG_FALUT_EVENT_INPUT;
		else if (fault_status == BQ2589X_FAULT_CHRG_THERMAL)
			event_type = CHG_FALUT_EVENT_THERMAL;
		else if (fault_status == BQ2589X_FAULT_CHRG_TIMER)
			event_type = CHG_FALUT_EVENT_TIMER;

		if (bq->charging_fault_callback !=NULL)
			bq->charging_fault_callback(event_type);
	}
}

static void bq2589x_intr_work(void const * argument)
{
	CHG_STAT chg_stat;
	struct bq2589x_device *dev = &bq2589x_dev;
	uint16_t int_pin;
	uint16_t VBUS_GPIO_PIN_INT;;

	dev->update_charging_stat(&chg_stat);
	if (chg_stat.online == 1) {
		dev->cfg.enable_term = 1;
		bq2589x_enable_term(dev, dev->cfg.enable_term);
		bq2589x_charger_irq_workfunc(dev);
		chg_info("%s: battery exist, enable termination config\n", __func__);
	} else
		chg_info("%s: battery is not exist, disable termination config\n", __func__);

	do {
		xQueueReceive( dev->chargerMsgQueueHandle, &int_pin, portMAX_DELAY );
		dev->intr_handler_token = pdFALSE;
		chg_info("%s: interrupt occured. %#x\n", __func__,int_pin);
        if(XA0n == pcb_id){
			VBUS_GPIO_PIN_INT =  GPIO_PIN_6;
		}
        else{
			VBUS_GPIO_PIN_INT = GPIO_PIN_12;
        }
		if ((int_pin & VBUS_GPIO_PIN_INT) == VBUS_GPIO_PIN_INT) {
			if (dev->update_charging_stat != NULL)
				dev->update_charging_stat(&chg_stat);
			if (chg_stat.online ==1) {
				if (dev->charging_led_callback != NULL)
					dev->charging_led_callback();
				if (dev->charging_ovp_callback != NULL)
					dev->charging_ovp_callback();
			}
			chg_info("%s: PG state changed.\n", __func__);
		}

		if ((int_pin & GPIO_PIN_3) == GPIO_PIN_3){
			bq2589x_charger_irq_workfunc(dev);
		}
	} while(1);
}

void bq2589x_intr_handler(uint16_t GPIO_Pin)
{
	struct bq2589x_device *dev = &bq2589x_dev;
	if (dev->intr_handler == NULL)
		return;
    	dev->intr_handler_token = pdFALSE;
	if (dev->chargerMsgQueueHandle != NULL) {
	        xQueueSendFromISR(dev->chargerMsgQueueHandle, &GPIO_Pin, &dev->intr_handler_token);
	}
	portYIELD_FROM_ISR(dev->intr_handler_token);
}

static int bq2589x_create_intr_thread(struct bq2589x_device *dev)
{
	dev->intr_handler_token = pdFALSE;
	osThreadDef(bq2589x_intr_thread, bq2589x_intr_work, osPriorityNormal, 0,
						configMINIMAL_STACK_SIZE * 4);
	dev->intr_handler = osThreadCreate(osThread(bq2589x_intr_thread), dev);
	if (dev->intr_handler == NULL)
		return -1;

	return 0;
}

static int bq25896_get_charging_para(PmicDrvTypeDef *handle, chargerPara *chg_para)

{
	chg_para->chg_stat = bq2589x_get_charging_status(&bq2589x_dev);
	bq2589x_get_input_curr_limit(&bq2589x_dev, &chg_para->INLIM);
	bq2589x_get_chargecurr(&bq2589x_dev, &chg_para->ICHG);
	bq2589x_get_chargevolt(&bq2589x_dev, &chg_para->VREG);
	bq2589x_get_IINDPM_state(&bq2589x_dev, &chg_para->IINDPM_STAT);
	chg_para->IDPM_LIM = bq2589x_read_idpm_limit(&bq2589x_dev);
	chg_para->ICO_OPTIMIZED = bq2589x_check_ico_done(&bq2589x_dev);

	return 0;
}

static int bq25896_get_pg_state(PmicDrvTypeDef *handle, uint8_t *state)
{
	*state = bq2589x_get_pg_status(&bq2589x_dev);
	return 0;
}

static int bq2589x_get_vbus_volt(PmicDrvTypeDef *handle, uint32_t *volt)
{
	*volt = bq2589x_adc_read_vbus_volt(&bq2589x_dev);
	return 0;
}

static int bq2589x_charge_enable_or_disable(PmicDrvTypeDef *handle, bool enable)
{
	bq2589x_enableDisable_charger(&bq2589x_dev, enable);
	return 0;
}

static int bq2589x_set_in_curr_limit(PmicDrvTypeDef *handle, uint32_t curr)
{
	bq2589x_set_input_curr_limit(&bq2589x_dev, curr);
	return 0;
}

static int bq2589x_set_charge_volt_limit(PmicDrvTypeDef *handle, uint32_t volt)
{
	bq2589x_set_chargevolt(&bq2589x_dev, volt);
	return 0;
}

static int bq2589x_set_charge_curr_limit(PmicDrvTypeDef *handle, uint32_t curr)
{
	bq2589x_set_chargecurr(&bq2589x_dev, curr);
	return 0;
}

static int bq25896_set_hiz_mode(PmicDrvTypeDef *handle, bool state)
{
	if (state)
		bq2589x_enter_hiz_mode(&bq2589x_dev);
	else
		bq2589x_exit_hiz_mode(&bq2589x_dev);

	return 0;
}

static int bq25896_power_off(PmicDrvTypeDef *handle)
{
    return bq2589x_enter_ship_mode(&bq2589x_dev);

}
static int bq25896_power_reset(PmicDrvTypeDef *handle)
{
    return bq2589x_reset_chip(&bq2589x_dev);
}

static int bq2589x_init(PmicDrvTypeDef *handle)
{
	int ret;
	memset(&bq2589x_dev, 0, sizeof(bq2589x_dev));
	bq2589x_dev.handle = handle;

	osMutexDef(bq2589x_i2c_mutex);
	bq2589x_dev.bq2589x_i2c_lock =
		osMutexCreate(osMutex(bq2589x_i2c_mutex));

	ret = bq2589x_detect_device(&bq2589x_dev);
	if (!ret && bq2589x_dev.part_no == BQ25896) {
		chg_info("%s: charger device bq25890 detected, revision:%d\n",
				__func__, bq2589x_dev.revision);
	} else {
		chg_info("%s: no bq25890 charger device found:%d\n", __func__, ret);
		return -1;
	}

	bq2589x_get_device_data(&bq2589x_dev);

	ret = bq2589x_init_device(&bq2589x_dev);
	if (ret) {
		chg_err("device init failure: %d\n", ret);
		return -1;
	}

	if (bq2589x_create_intr_thread(&bq2589x_dev) != 0) {
		chg_err("%s create thread ERROR\n!", __func__);
		return -1;
	}

	osMessageQDef(chg_int_queue, 4, uint16_t);
	bq2589x_dev.chargerMsgQueueHandle = osMessageCreate(osMessageQ(chg_int_queue), NULL);
	chg_info("charger initialized DONE!\n");

	return 0;
}

static int bq2589x_deinit(PmicDrvTypeDef *handle)
{
	bq2589x_enableDisable_charger(&bq2589x_dev, 0);
	bq2589x_reset_chip(&bq2589x_dev);
	chg_info("charger has unregistered!\n");

	return 0;
}

CHARGER_DRV_t BQ25896_Drv = {
	bq2589x_init,
	bq2589x_deinit,
	bq2589x_set_in_curr_limit,
	bq2589x_charge_enable_or_disable,
	bq2589x_get_vbus_volt,
	bq25896_get_charging_para,
	bq25896_get_pg_state,
	bq2589x_set_charge_volt_limit,
	bq2589x_set_charge_curr_limit,
	bq25896_power_off,
	bq25896_power_reset,
	bq25896_set_hiz_mode
};
