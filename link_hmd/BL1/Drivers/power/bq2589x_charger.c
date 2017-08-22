#include "integer.h"
#include "i2c_drv.h"
#include <string.h>
#include "bq2589x_charger.h"
#include "gpio.h"

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

	//osMutexId bq2589x_i2c_lock;
	//osThreadId intr_handler;
	//BaseType_t intr_handler_token;

	//osTimerId timer_id;
	//osTimerDef_t timer_def;

	//osThreadId charger_work_handler;
	//osMessageQId chargerMsgQueueHandle;

	enum bq2589x_part_no part_no;
};

static struct bq2589x_device bq2589x_dev = { 0 };

static int bq2589x_read_byte(struct bq2589x_device *bq, uint8_t *data, uint8_t reg)
{
	I2C_STATUS ret = I2C_OK;

	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_CHARGER_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, BQ2589x_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		printf("%s I2C write error ret=%d\n",__func__,ret);
		return -1;
	}

	return 0;
}

static int bq2589x_write_byte(struct bq2589x_device *bq, uint8_t reg, uint8_t *data)
{
	I2C_STATUS ret = I2C_OK;

	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_CHARGER_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, BQ2589x_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		printf("%s I2C write error ret=%d\n",__func__,ret);
		return -1;
	}
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

int bq2589x_get_pg_status(struct bq2589x_device *bq)
{
	uint8_t val = 0;
	int ret;

    if(bq == NULL) return val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		chg_err("%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}

	val &= BQ2589X_PG_STAT_MASK;
	val >>= BQ2589X_PG_STAT_SHIFT;

	return val;
}

static int bq2589x_enableDisable_charger(struct bq2589x_device *bq, bool enable)
{
	int ret;
	uint8_t val;
	if (enable) {
		//chg_info("%s: enable charger!\n", __func__);
		val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	} else {
		//chg_info("%s: disable charger!\n", __func__);
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
		printf("%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) ==
									BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot) {
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
								BQ2589X_CONV_START_MASK,
								BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
		printf("%s, start one shot ADC conversion!\n", __func__);
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
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >>
			BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
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
		temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >>
				BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
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
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >>
				BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}


int bq2589x_set_chargecurr(struct bq2589x_device *bq, int curr)
{
	uint8_t ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

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
	bq->cfg.enable_auto_dpdm = 0;
	bq->cfg.enable_term = 0;
	bq->cfg.enable_ico = 1;
	bq->cfg.use_absolute_vindpm = 0;
	bq->cfg.charge_volt = 4400;
	bq->cfg.prechg_volt_threshould = 3000;
	bq->cfg.charge_curr = 2800;
	bq->cfg.prechg_curr = 64;
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

static int bq25896_get_charging_state(PmicDrvTypeDef *handle, uint8_t *state)
{
	*state = bq2589x_get_charging_status(&bq2589x_dev);
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
static int bq25896_get_pg_state(PmicDrvTypeDef *handle, uint8_t *state)
{
	*state = ((bq2589x_get_pg_status(&bq2589x_dev) > 0) ? 1 : 0);
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
	bq25896_get_charging_state,
	bq2589x_set_charge_volt_limit,
	bq2589x_set_charge_curr_limit,
	bq25896_power_off,
	bq25896_power_reset,
	bq25896_set_hiz_mode,
	bq25896_get_pg_state
};
