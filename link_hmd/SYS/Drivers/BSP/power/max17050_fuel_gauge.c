#include <stdlib.h>
#include <string.h>
#include "rtos_i2c_drv.h"
#include "max17050_fuel_gauge.h"
#include "PowerManager_notify_func.h"


typedef void (*battery_soc_limites)(void *, bool);
typedef int (*BATT_STATUS_DET)(int);
typedef int (*GET_BATT_ONLINE)(uint8_t *);
struct max17050_config {
	 //Sense resistor
	uint32_t r_sns;

	uint16_t param_version;

	//Application specific settings
	uint16_t config;
	uint16_t relaxcfg;
	uint16_t filtercfg;
	uint16_t learncfg;
	uint16_t fullsocthr;
	uint16_t misccfg;
	uint16_t tempnom;
	//Thermistor attributes
	uint16_t tgain;
	uint16_t toff;

	//Battery attributes
	uint16_t qrtable00;
	uint16_t qrtable10;
	uint16_t qrtable20;
	uint16_t qrtable30;
	uint16_t rcomp0;
	uint16_t tempco;
	uint16_t capacity;
	uint16_t vf_fullcap;
	uint16_t ichgterm;
	uint16_t iavg_empty;
	uint16_t vempty;

	uint16_t temperature;
	uint16_t dpacc;

	/* model characterisation data */
	uint16_t model[MODEL_SIZE];
};

/* Modelgauge M3 save/restore values */
struct max17050_learned_params {
	uint16_t rcomp0;
	uint16_t tempco;
	uint16_t fullcap;
	uint16_t fullcapnom;
	uint16_t iavg_empty;
	uint16_t qrtable00;
	uint16_t qrtable10;
	uint16_t qrtable20;
	uint16_t qrtable30;
	uint16_t cycles;
	uint16_t param_version;
};

struct max17050_device {
	struct max17050_config *config;
	struct max17050_learned_params learned;
	PmicDrvTypeDef *handle;
	osMutexId mutex;
	bool use_ext_temp;
	bool init_done;
	bool power_on_reset;
	osThreadId intr_handler;
	osMessageQId fgMsgQueueHandle;
	BaseType_t intr_handler_token;
	osTimerId timer_id;
	osTimerDef_t timer_def;
	uint32_t next_update_time;
	int ext_temp;
	/*alert value*/
	//uint8_t min_salrt;
	//uint8_t max_salrt;
	//int8_t min_talrt;
	//int8_t max_talrt;
	uint8_t isInitialized;
	bool max_soc_alrt;
	enum batt_soc_level soc_level;
	enum batt_temp_level temp_level;
	/* values read from chip */
	uint16_t status;
	uint16_t vcell;
	uint16_t repcap;
	uint16_t repsoc;
	uint16_t temp;
	uint16_t current_now;
	uint16_t tte;
	BATT_STATUS_DET batt_status_callback;
	GET_BATT_ONLINE get_batt_online;
};

/* External battery power supply poll times */
#define EXT_BATT_FAST_PERIOD		100
#define EXT_BATT_SLOW_PERIOD	10000

static void max17050_update(struct max17050_device *);
static int max17050_cfg_init(struct max17050_device *);

static struct max17050_device max17050_dev = { 0 };
static struct max17050_config max17050_cfg = {
	.param_version = 0x2,
	.r_sns = 10000,
	.relaxcfg = 0x203B,
	.config = 0x2210,
	.filtercfg = 0x87A4,
	.learncfg = 0x2606,
	.misccfg = 0x0870,
	.fullsocthr = 0x1501,
	.rcomp0 = 0x0041,
	.tempco = 0x2029,
	.tempnom = 0x1400,
	.tgain = 0xDCB2,
	.toff = 0x28F5,
	.ichgterm = 0x03C0,
	.iavg_empty = 0x0E8B,
	.vempty = 0xACE5,
	.qrtable00 = 0x8800,
	.qrtable10 = 0x4100,
	.qrtable20 = 0x1c80,
	.qrtable30 = 0x1403,
	.capacity = 0x161C,
	.vf_fullcap = 0x161C,
	.temperature = 0x1400,
	.dpacc = 0x0C80,
	.model = {
			0x9ce0, 0xb640, 0xb8d0, 0xba60,
			0xbbb0, 0xbd30, 0xbe00, 0xbea0,
			0xc0c0, 0xc2a0, 0xc5a0, 0xc8a0,
			0xcb80, 0xce60, 0xd4a0, 0xdab0,
			0x00d0, 0x0b10, 0x0f10, 0x1050,
			0x1780, 0x1b20, 0x1fc0, 0x0df0,
			0x0eb0, 0x07f0, 0x07c0, 0x0730,
			0x0880, 0x06d0, 0x0610, 0x0610,
			0x0200, 0x0200, 0x0200, 0x0200,
			0x0200, 0x0200, 0x0200, 0x0200,
			0x0200, 0x0200, 0x0200, 0x0200,
			0x0200, 0x0200, 0x0200, 0x0200
		},
};

static int max17050_read_reg(uint8_t addr)
{
	uint8_t val[2] = { 0 };
	uint16_t read_val = 0;
	I2C_STATUS ret = I2C_OK;

	ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_GAUGE_ADDR, addr, I2C_MEMADD_SIZE_8BIT, val, 2, MAX17050_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		fg_err("%s I2C write error ret=%d\n",__func__,ret);
		return -1;
	}
	else {
			read_val = ((val[1] << 8) | val[0]);
			return read_val;
	}
}

static int max17050_write_reg(uint8_t addr, uint16_t buf)
{
	uint8_t val[2] = { 0 };
	I2C_STATUS ret = I2C_OK;

	val[0] = buf & 0xFF;
	val[1] = buf >> 8;
	ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_GAUGE_ADDR, addr, I2C_MEMADD_SIZE_8BIT, val, 2, MAX17050_I2C_TIMEOUT_MAX);
	if (ret != I2C_OK ) {
		fg_err("%s I2C write error ret=%d\n",__func__,ret);
		return -1;
	}
	else
		return 0;
}

static int max17050_write_verify_reg(uint8_t reg, uint16_t value)
{
	int ret;
	int read = 0;
	ret = max17050_write_reg(reg, value);
	if (ret < 0) {
		fg_err("%s: err %d\n", __func__, ret);
		return ret;
	}
	read = max17050_read_reg(reg);
	if (read < 0) {
		fg_err("%s: err %d\n", __func__, read);
		return read;
	}
	if (read != value) {
		fg_err("%s: expected 0x%x, got 0x%x\n",
			__func__, value, read);
		return 1;
	}
	//fg_info("%s: [0x%x]=0x%x\n", __func__, reg, value);
	return ret;
}

static int max17050_write_verify_block(uint8_t reg, uint16_t *values, uint8_t len)
{
	int ret;
	int read;
	uint8_t i;
	for (i = 0; i < len; i++) {
		ret = max17050_write_reg(reg + i, values[i]);
		if (ret != 0) {
			fg_err( "%s: write err %d\n", __func__, ret);
			return ret;
		}
		read = max17050_read_reg(reg + i);
		if (read < 0) {
			fg_err("%s: read err %d\n", __func__, read);
			return read;
		}
		if (read != values[i]) {
			fg_err("%s: addr 0x%x expected 0x%x, got 0x%x\n",
				__func__, reg + i, values[i], read);
			return 1;
		} /*else
			fg_info("%s: [0x%x]=0x%x\n", __func__, reg + i, read);*/
	}
	return ret;
}

static int max17050_batt_online(PmicDrvTypeDef *handle, bool *status)
{
	int ret;

	ret = max17050_read_reg(MAX17050_STATUS);
	if (ret < 0)
	    return -1;

	// BST is 0 when battery is present and 1 when battery is removed
	*status = ((ret & MAX17050_STATUS_BST) != MAX17050_STATUS_BST);

	return 0;
}

static int max17050_current(struct max17050_device *dev, bool average, int *current)
{
	uint8_t reg = average ? MAX17050_AVGCURRENT : MAX17050_CURRENT;
	int ret;

	ret = max17050_read_reg(reg);
	if (ret < 0)
		return -1;

	*current = ret;
	if (*current & 0x8000) {
		/* Negative */
		*current = ~*current & 0x7fff;
		*current += 1;
		*current *= -1;
	}
	*current *= 1562500 / max17050_cfg.r_sns;

	return 0;
}

static int max17050_voltage(struct max17050_device *dev, uint32_t *voltage)
{
	int ret;

	ret = max17050_read_reg(MAX17050_VCELL);
	if (ret < 0)
		return -1;

	*voltage = ret * 625 / 8;

	return 0;
}

static int max17050_soc(struct max17050_device *dev, int *soc)
{
	int ret;
	int val;

	ret = max17050_read_reg(MAX17050_REPSOC);
	if (ret < 0) {
		fg_info("%s: read MAX17050_REPSOC ERROR\n", __func__);
		return -1;
	}
	/* high byte with a resolution of 1.0% */
	val = ret >> 8;

	/* count in low byte, round to the nearest integer */
	if ((ret & 0xff) > 0)
		*soc = val + 1;
	else
		*soc = val;

	if (*soc >= 100)
		*soc = 100;

	return 0;
}

static int max17050_temperature(struct max17050_device *dev, int *temperature)
{
	int ret;

	ret = max17050_read_reg(MAX17050_TEMP);
	if (ret < 0)
		return -1;

	*temperature = ret;
	/* The value is signed. */
	if (*temperature & 0x8000) {
		*temperature = (0x7fff & ~*temperature) + 1;
		*temperature *= -1;
	}

	/* The value is converted into deci-centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temperature = *temperature * 10 / 256;

	return 0;
}

static int max17050_full_capacity(struct max17050_device *dev, uint32_t *capacity)
{
	int ret;

	ret = max17050_read_reg(MAX17050_FULLCAP);
	if (ret < 0)
		return -1;

	*capacity = ret * (5000000 / max17050_cfg.r_sns);

	return 0;
}

static int max17050_get_current(PmicDrvTypeDef *handle, bool average, int *current)
{
	int ret;
	if (average) {
		ret = max17050_current(&max17050_dev, 1, current);
		//fg_info("%s: Current average current = %d\n", __func__, (*current / 1000));
	} else {
		ret = max17050_current(&max17050_dev, 0, current);
		//fg_info("%s: Current current = %d\n", __func__, (*current / 1000));
	}
	if (ret < 0)
		return -1;

	return 0;
}

static int max17050_get_voltage(PmicDrvTypeDef *handle, uint32_t *voltage)
{
	int ret;

	ret = max17050_voltage(&max17050_dev, voltage);
	if (ret < 0)
		return -1;
	//fg_info("%s: Current voltage = %d\n", __func__, *voltage);

	return 0;
}

static int max17050_get_soc(PmicDrvTypeDef *handle, int *soc)
{
	int ret;

	ret = max17050_soc(&max17050_dev, soc);
	if (ret < 0)
		return -1;
	//fg_info("%s: Current soc = %d\n",__func__, *soc);
	return 0;
}
static int max17050_get_temperature(PmicDrvTypeDef *handle, int *temperature)
{
	int ret;

	ret = max17050_temperature(&max17050_dev, temperature);
	if (ret < 0)
		return -1;
	//fg_info("%s: Current temperature = %d\n",__func__, (*temperature / 10));
	return 0;
}

static int max17050_get_full_capacity(PmicDrvTypeDef *handle, uint32_t *capacity)
{
	int ret;

	ret = max17050_full_capacity(&max17050_dev, capacity);
	if (ret < 0)
		return -1;

	return 0;
}

bool htc_batt_is_online(void)
{
	bool status;

	max17050_batt_online(max17050_dev.handle, &status);

	return status;
}

int htc_batt_get_soc(void)
{
	int soc = 0;
	int ret;

	ret = max17050_soc(&max17050_dev, &soc);
	if (ret < 0)
		return -1;

	return soc;
}

int htc_batt_get_temperature(void)
{
	int temperature;
	int ret;

	ret = max17050_temperature(&max17050_dev, &temperature);
	if (ret < 0)
		return -1;

	return temperature;
}

int htc_batt_get_current(bool average)
{
	int ret;
	int current;
	if (average)
		ret = max17050_current(&max17050_dev, 1, &current);
	else
		ret = max17050_current(&max17050_dev, 0, &current);
	if (ret < 0)
		return -1;

	return current;
}

int htc_batt_get_voltage(void)
{
	int ret;
	uint32_t voltage;

	ret = max17050_voltage(&max17050_dev, &voltage);
	if (ret < 0)
		return -1;

	return voltage;
}

int htc_batt_get_full_capacity(void)
{
	int ret;
	uint32_t capacity;

	ret = max17050_full_capacity(&max17050_dev, &capacity);
	if (ret < 0)
		return -1;

	return capacity;
}

static int max17050_get_property(PmicDrvTypeDef *handle,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	uint32_t now;
	int16_t int16_t_value;
	struct max17050_device *dev = &max17050_dev;

	now = (osKernelSysTick() * portTICK_PERIOD_MS);
	if ((int)(now - dev->next_update_time) >= 0) {
		osMutexWait(dev->mutex, portMAX_DELAY);
		max17050_update(dev);
		osMutexRelease(dev->mutex);
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (dev->status & MAX17050_STATUS_BATTABSENT)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = dev->learned.cycles / 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = dev->vcell * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		max17050_soc(dev, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (dev->use_ext_temp) {
			val->intval = dev->ext_temp;
		} else {
			/* cast to int16_t */
			int16_t_value = (int16_t)dev->temp;
			/* sign extend to int32_t */
			val->intval = (int32_t)int16_t_value;
			/* units are 1/256, and temp is reported in C * 10 */
			val->intval = (val->intval * 10) / 256;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* cast to int16_t */
		int16_t_value = (int16_t)dev->current_now;
		/* sign extend to int32_t */
		val->intval = (int32_t)int16_t_value;
		val->intval *= 1562500 / dev->config->r_sns;
		/* negate (charging is negative by convention) */
		val->intval *= -1;
		break;
	default:
		return -1;
	}
	return 0;
}

static int max17050_set_soc_limits(struct max17050_device *dev, int min, int max)
{
	uint16_t salrt;
	uint16_t salrt_min;
	uint16_t salrt_max;

	if (min != INT_MIN && min < DEVICE_BATTERY_LEVEL_EMPTY)
		return -1;

	if (max != INT_MAX && max > DEVICE_BATTERY_LEVEL_FULL)
		return -1;

	/* Use end of charging detection for DEVICE_BATTERY_LEVEL_FULL */
	//dev->max_soc_alrt = (max == DEVICE_BATTERY_LEVEL_FULL);
	//if (dev->max_soc_alrt)
		//max = INT_MAX;

	//if (min == DEVICE_BATTERY_LEVEL_EMPTY)
		//min = MIN_SOC_ALRT;

	salrt_min = (min == INT_MIN) ? 0x00 : min;
	salrt_max = (max == INT_MAX) ? 0xff : max;

	salrt = salrt_min | salrt_max << 8;
	fg_info("%s: soc min alrt = %d, soc max alrt = %d\n", __func__, salrt_min, salrt_max);

	//dev->min_salrt = min;
	//dev->max_salrt = max;

	return max17050_write_reg(MAX17050_SALRT_TH, salrt);

}

static int max17050_set_temperature_limits(struct max17050_device *dev, int min, int max)
{
	int16_t talrt_min;
	int16_t talrt_max;
	int16_t talrt;

	if (min == INT_MIN)
		 min = TALRT_MIN_DIS;
	else if (min < INT_MIN || min > INT_MAX)
		return -1;

	if (min >= 0)
		talrt_min = min;
	else {
		talrt_min = -min;
		talrt_min = (~talrt_min + 1) & 0x00FF;
	}

	if (max == INT_MAX)
		max = TALRT_MAX_DIS;
	else if (max < INT_MIN || max > INT_MAX)
		return -1;

	if (max >= 0)
		talrt_max = max;
	else {
		talrt_max = -max;
        	talrt_max = (~talrt_max + 1) & 0x00FF;
	}

	//dev->min_talrt = min;
	//dev->max_talrt = max;

	talrt = talrt_min | talrt_max << 8;
	fg_info("%s: temp min alrt = %d, temp max alrt = %d\n", __func__, talrt_min, talrt_max);

	return max17050_write_reg(MAX17050_TALRT_TH, talrt);
}

static void max17050_set_soc_thresholds(struct max17050_device *dev,
                              enum batt_soc_level level)
{
    int min, max; /* new capacity limits */

    /* Determine new capacity limits */
    switch (level) {
    case BATTERY_LEVEL_EMPTY:
        min = INT_MIN;
        max = DEVICE_BATTERY_LEVEL_EMPTY + CONFIG_BATTERY_LEVEL_EMPTY_HYST;
        break;
    case BATTERY_LEVEL_LOW:
        min = DEVICE_BATTERY_LEVEL_EMPTY + CONFIG_BATTERY_LEVEL_EMPTY_HYST;
        max = CONFIG_BATTERY_LEVEL_LOW;
        break;
    case BATTERY_LEVEL_NORMAL:
        min = CONFIG_BATTERY_LEVEL_LOW;
        max = DEVICE_BATTERY_LEVEL_FULL - CONFIG_BATTERY_LEVEL_FULL_HYST;
        break;
    case BATTERY_LEVEL_FULL:
        min = DEVICE_BATTERY_LEVEL_FULL - CONFIG_BATTERY_LEVEL_FULL_HYST;
        max = INT_MAX;
        break;
     default:
        /* Should never be here! */
        return;
    }

    dev->soc_level = level;
    max17050_set_soc_limits(dev, min, max);
}

static void max17050_set_temperature_thresholds(struct max17050_device *dev,
                              						enum batt_temp_level level)
{
	int min,max;

	switch (level) {
	case BATTERY_COLD:
		min = INT_MIN;
		max = CONFIG_BATTERY_TEMP_COLD;
		break;
	case BATTERY_2C:
		min = CONFIG_BATTERY_TEMP_COLD;
		max = CONFIG_BATTERY_TEMP_2C;
		break;
	case BATTERY_COOL:
		min = CONFIG_BATTERY_TEMP_2C;
		max = CONFIG_BATTERY_TEMP_COOL;
		break;
	case BATTERY_NORMAL:
        	min = CONFIG_BATTERY_TEMP_COOL;
		max = CONFIG_BATTERY_TEMP_WARM;
        	break;
	case BATTERY_WARM:
		min = CONFIG_BATTERY_TEMP_WARM;
        	max = CONFIG_BATTERY_TEMP_HOT;
        	break;
	case BATTERY_HOT:
		min = CONFIG_BATTERY_TEMP_HOT;
		max = CONFIG_BATTERY_TEMP_55C;
		break;
	case BATTERY_55C:
		min = CONFIG_BATTERY_TEMP_55C;
        	max = CONFIG_BATTERY_TEMP_COOL_DOWN;
        	break;
	case BATTERY_COOL_DOWN:
		min = CONFIG_BATTERY_TEMP_COOL_DOWN;
        	max = INT_MAX;
        	break;
	default:
		break;
		}
	dev->temp_level = level;
	max17050_set_temperature_limits(dev, min, max);
	fg_info("%s: dev->temp_level = %d, min = %d, max = %d.\n", __func__, dev->temp_level, min, max);
}

static void max17050_soc_thresholds_init(struct max17050_device *dev)
{
	int soc = 0;
	int ret;

	ret = max17050_get_soc(dev->handle, &soc);
	if (ret < 0) {
		fg_info("%s: get current soc failed.\n", __func__);
		max17050_set_soc_thresholds(dev, BATTERY_LEVEL_EMPTY);
		return;
	}

	if (soc <= (DEVICE_BATTERY_LEVEL_EMPTY + CONFIG_BATTERY_LEVEL_EMPTY_HYST))
		max17050_set_soc_thresholds(dev, BATTERY_LEVEL_EMPTY);
	else if ((soc <= CONFIG_BATTERY_LEVEL_LOW) &&
		(soc > (DEVICE_BATTERY_LEVEL_EMPTY + CONFIG_BATTERY_LEVEL_EMPTY_HYST)))
		max17050_set_soc_thresholds(dev, BATTERY_LEVEL_LOW);
	else if ((soc < DEVICE_BATTERY_LEVEL_FULL) && (soc > CONFIG_BATTERY_LEVEL_LOW))
		max17050_set_soc_thresholds(dev, BATTERY_LEVEL_NORMAL);
	else
		max17050_set_soc_thresholds(dev, BATTERY_LEVEL_FULL);
}

static void max17050_temperature_thresholds_init(struct max17050_device *dev)
{
	int temperature = 0, temp = 0;
	int ret;

	ret = max17050_get_temperature(dev->handle, &temp);
	if (ret < 0) {
		fg_err("%s: get batt temperature ERROR!\n", __func__);
		max17050_set_temperature_thresholds(dev, BATTERY_COOL_DOWN);
		return;
	} else
		temperature = (temp / 10);

	if (temperature < CONFIG_BATTERY_TEMP_COLD)
		max17050_set_temperature_thresholds(dev, BATTERY_COLD);
	else if ((temperature <= CONFIG_BATTERY_TEMP_2C) && (temperature >= CONFIG_BATTERY_TEMP_COLD))
		max17050_set_temperature_thresholds(dev, BATTERY_2C);
	else if ((temperature  < CONFIG_BATTERY_TEMP_COOL) && (temperature >= CONFIG_BATTERY_TEMP_2C))
		max17050_set_temperature_thresholds(dev, BATTERY_COOL);
	else if ((temperature < CONFIG_BATTERY_TEMP_WARM) && (temperature >= CONFIG_BATTERY_TEMP_COOL))
		max17050_set_temperature_thresholds(dev, BATTERY_NORMAL);
	else if ((temperature < CONFIG_BATTERY_TEMP_HOT) && (temperature >= CONFIG_BATTERY_TEMP_WARM))
		max17050_set_temperature_thresholds(dev, BATTERY_WARM);
	else if ((temperature < CONFIG_BATTERY_TEMP_55C) && (temperature >= CONFIG_BATTERY_TEMP_HOT))
		max17050_set_temperature_thresholds(dev, BATTERY_HOT);
	else if ((temperature < CONFIG_BATTERY_TEMP_COOL_DOWN) && (temperature >= CONFIG_BATTERY_TEMP_55C))
		max17050_set_temperature_thresholds(dev, BATTERY_55C);
	else
		max17050_set_temperature_thresholds(dev, BATTERY_COOL_DOWN);
}


static void max17050_soc_exceed_handler(struct max17050_device *dev, bool min)
{
	enum batt_soc_level new_level;

	/* Transition to a new level */
	switch (dev->soc_level) {
	case BATTERY_LEVEL_EMPTY:
	    if (min) /* no change */
	        return;
	    new_level = BATTERY_LEVEL_LOW;
	    break;
	case BATTERY_LEVEL_LOW :
	    new_level = min ?  BATTERY_LEVEL_EMPTY : BATTERY_LEVEL_NORMAL;
	    break;
	case BATTERY_LEVEL_NORMAL:
	    new_level = min ? BATTERY_LEVEL_LOW : BATTERY_LEVEL_FULL;
	    break;
	case BATTERY_LEVEL_FULL:
	    if (!min) /* no change */
	        return;
	    new_level = BATTERY_LEVEL_NORMAL;
	    break;
	default:
	    /* Should never be here !*/
	    return;
	}
	max17050_set_soc_thresholds(dev, new_level);
}

static void max17050_temperature_exceed_handler(struct max17050_device *dev, bool min)
{
	enum batt_temp_level new_level;

	switch (dev->temp_level) {
	case BATTERY_COLD:
		new_level = min ? BATTERY_COLD : BATTERY_2C;
		break;
	case BATTERY_2C:
		new_level = min ? BATTERY_COLD : BATTERY_COOL;
		break;
	case BATTERY_COOL:
		new_level = min ? BATTERY_2C : BATTERY_NORMAL;
		break;
	case BATTERY_NORMAL:
		new_level = min ? BATTERY_COOL : BATTERY_WARM;
		break;
	case BATTERY_WARM:
		new_level = min ? BATTERY_NORMAL : BATTERY_HOT;
        	break;
	case BATTERY_HOT:
		new_level = min ? BATTERY_WARM : BATTERY_55C;
		break;
	case BATTERY_55C:
		new_level = min ? BATTERY_HOT : BATTERY_COOL_DOWN;
		break;
	case BATTERY_COOL_DOWN:
		//new_level = BATTERY_HOT;
		new_level = min ? BATTERY_55C : BATTERY_COOL_DOWN;
        	break;
	default:
		break;
	}
	fg_info("%s: dev->temp_level = %d, new_level = %d.\n", __func__, dev->temp_level, new_level);
	max17050_set_temperature_thresholds(dev, new_level);
}

#if 0
/* Must call with mutex locked */
static void max17050_save_learned_params(struct max17050_device *dev)
{
	dev->learned.rcomp0 = max17050_read_reg(MAX17050_RCOMP0);
	dev->learned.tempco = max17050_read_reg(MAX17050_TEMPCO);
	dev->learned.fullcap = max17050_read_reg(MAX17050_FULLCAP);
	dev->learned.cycles = max17050_read_reg(MAX17050_CYCLES);
	dev->learned.fullcapnom = max17050_read_reg(MAX17050_FULLCAPNOM);
	dev->learned.iavg_empty = max17050_read_reg(MAX17050_IAVG_EMPTY);
	dev->learned.qrtable00 = max17050_read_reg(MAX17050_QRTABLE00);
	dev->learned.qrtable10 = max17050_read_reg(MAX17050_QRTABLE10);
	dev->learned.qrtable20 = max17050_read_reg(MAX17050_QRTABLE20);
	dev->learned.qrtable30 = max17050_read_reg(MAX17050_QRTABLE30);
	dev->learned.param_version = max17050_read_reg(MAX17050_CUSTOMVER);
}
/* Must call with mutex locked */
static void max17050_restore_learned_params(struct max17050_device *dev)
{
	uint16_t remcap;
	/* Skip restore when custom param version has changed */
	if (dev->learned.param_version !=
			max17050_read_reg( MAX17050_CUSTOMVER))
		return;
	max17050_write_verify_reg( MAX17050_RCOMP0,
						dev->learned.rcomp0);
	max17050_write_verify_reg( MAX17050_TEMPCO,
						dev->learned.tempco);
	max17050_write_verify_reg( MAX17050_IAVG_EMPTY,
						dev->learned.iavg_empty);
	max17050_write_verify_reg( MAX17050_FULLCAPNOM,
						dev->learned.fullcapnom);
	max17050_write_verify_reg( MAX17050_QRTABLE00,
						dev->learned.qrtable00);
	max17050_write_verify_reg( MAX17050_QRTABLE10,
						dev->learned.qrtable10);
	max17050_write_verify_reg( MAX17050_QRTABLE20,
						dev->learned.qrtable20);
	max17050_write_verify_reg( MAX17050_QRTABLE30,
						dev->learned.qrtable30);
	/* Wait for VFSOC to be calculated */
	osDelay(350);
	remcap = (max17050_read_reg( MAX17050_SOC) *
		max17050_read_reg( MAX17050_FULLCAP0)) / 25600;
	max17050_write_verify_reg( MAX17050_REMCAP, remcap);
	max17050_write_verify_reg( MAX17050_FULLCAP,
						dev->learned.fullcap);
	max17050_write_verify_reg( MAX17050_DQACC,
						dev->learned.fullcap / 16);
	max17050_write_verify_reg( MAX17050_DPACC, dev->config->dpacc);
	osDelay(350);
	max17050_write_verify_reg( MAX17050_CYCLES,
							dev->learned.cycles);
	if (dev->learned.cycles > 0xff) {
		/* advance to learnstage 7 */
		max17050_write_verify_reg( MAX17050_LEARNCFG,
									0x0676);
	}
}

static int32_t max17050_learned_read(char *buf, uint32_t count)
{
	struct max17050_device *dev = &max17050_dev;

	if (count == 0)
		return 0;
	else if (count < sizeof(struct max17050_learned_params))
		return -1;
	osMutexWait(dev->mutex, portMAX_DELAY);
	max17050_save_learned_params(dev);
	memcpy(buf, &dev->learned, sizeof(struct max17050_learned_params));
	osMutexRelease(dev->mutex);

	return count;
}
static int32_t max17050_learned_write(char *buf, uint32_t count)
{
	struct max17050_device *dev = &max17050_dev;

	if (count == 0)
		return 0;
	else if (count < sizeof(struct max17050_learned_params))
		return -1;
	osMutexWait(dev->mutex, portMAX_DELAY);
	memcpy(&dev->learned, buf, sizeof(struct max17050_learned_params));
	max17050_restore_learned_params(dev);
	osMutexRelease(dev->mutex);

	return count;
}

#endif

static void max17050_update_ext_temp(void const *argument)
{
	struct max17050_device *dev =&max17050_dev;
	int val, temp_value;

	if (!dev->use_ext_temp) {
		/* Power supply not ready - try again soon */
		if (xTimerChangePeriod(dev->timer_id, EXT_BATT_FAST_PERIOD, 100)) {
			fg_info("%s: timer period change succesfully!\n", __func__);
			return;
		}
		else {
			fg_err("%s: timer period change ERROR!\n", __func__);
			return;
		}
	}
	/* Get the temperature from the external power supply */
	//dev->ext_battery->get_property(dev->ext_battery,
					//POWER_SUPPLY_PROP_TEMP, &propval);

	val = 0;	//add temperature test API
	if (dev->ext_temp != val) {
		dev->ext_temp = val;
		/* Write the temperature to the MAX17050 */
		temp_value = (uint16_t)(dev->ext_temp * 256 / 10);
		osMutexWait(dev->mutex, portMAX_DELAY);
		max17050_write_verify_reg(MAX17050_TEMP,
								temp_value);
		osMutexRelease(dev->mutex);
	}
	/* Update again after some time */
	if (xTimerChangePeriod(dev->timer_id, EXT_BATT_SLOW_PERIOD, 100)) {
		fg_info("%s: timer period change succesfully!\n", __func__);
		return;
	}
	else {
		fg_err("%s: timer period change ERROR!\n", __func__);
		return;
	}
}
/* Must call with mutex locked */

static void max17050_update(struct max17050_device *dev)
{
	/* Check for power-on-reset or battery insertion */
	dev->status = max17050_read_reg( MAX17050_STATUS);
	if (dev->status & (MAX17050_STATUS_POR | MAX17050_STATUS_BI))
		max17050_cfg_init(dev);
	dev->repcap = max17050_read_reg( MAX17050_REPCAP);
	dev->repsoc = max17050_read_reg( MAX17050_REPSOC);
	dev->tte = max17050_read_reg( MAX17050_TTE);
	dev->current_now = max17050_read_reg( MAX17050_CURRENT);
	if (!dev->use_ext_temp)
		dev->temp = max17050_read_reg( MAX17050_TEMP);
	dev->vcell = max17050_read_reg( MAX17050_VCELL);
	dev->learned.cycles = max17050_read_reg( MAX17050_CYCLES);
	/* next update must be at least 1 second later */
	dev->next_update_time = ((osKernelSysTick() * portTICK_PERIOD_MS) + 1000);
}

static enum batt_intr_event_type max17050_batt_soc_changed(struct max17050_device *dev)
{
	enum batt_intr_event_type event_type = BATT_INT_EVENT_UNKONW;
	int soc;
	max17050_get_soc(dev->handle, &soc);
	if (soc == 0)
		event_type = BATT_INT_EVENT_SOC_EMPTY;
	else if ((soc <=20) && (soc >0))
		event_type = BATT_INT_EVENT_SOC_LOW;
	else if (soc >= 100)
		event_type = BATT_INT_EVENT_SOC_FULL;
	else {
		event_type = BATT_INT_EVENT_SOC_NORMAL;
		fg_info("%s: Current battery capacity = %d%, battery status is normal!\n", __func__, soc);
	}
	return event_type;
}

static enum batt_intr_event_type max17050_batt_temp_changed(struct max17050_device *dev)
{
	int ret;
	int temp = 0, temperature = 0;

	enum batt_intr_event_type event_type = BATT_INT_EVENT_UNKONW;
	ret = max17050_get_temperature(dev->handle, &temp);
	if (ret < 0) {
		fg_err("%s: get temperature ERROR.\n", __func__);
		return BATT_INT_EVENT_UNKONW;
	}

	temperature = (temp / 10);
	if (temperature <= 0)
		event_type = BATT_INT_EVENT_TEMP_COLD;
	else if ((temperature > 0) && (temperature < 2)) {
		event_type = BATT_INT_EVENT_TEMP_2C;
		fg_info("Current battery temperature = %d, battery status is 2C!\n", temperature);
	} else if ((temperature >= 2) && (temperature < 10)) {
		event_type = BATT_INT_EVENT_TEMP_COOL;
		fg_info("Current battery temperature = %d, battery status is COOL!\n", temperature);
	} else if ((temperature >= 10) && (temperature < 20)) {
		event_type = BATT_INT_EVENT_TEMP_NORMAL;
		fg_info("Current battery temperature = %d, battery status is NORMAL!\n", temperature);
	} else if ((temperature >= 20) && (temperature < 50)) {
		event_type = BATT_INT_EVENT_TEMP_WARM;
		fg_info("Current battery temperature = %d, battery status is WARM!\n", temperature);
	} else if ((temperature >= 50) && (temperature < 55)) {
		event_type = BATT_INT_EVENT_TEMP_HOT;
		fg_info("Current battery temperature = %d, battery status is HOT!\n", temperature);
	} else if ((temperature >= 55) && (temperature < 58)) {
		event_type = BATT_INT_EVENT_TEMP_55C;
		fg_info("Current battery temperature = %d, battery status is 55C!\n", temperature);
	}else if (temperature >= 58) {
		event_type = BATT_INT_EVENT_TEMP_COOL_DOWN;
		fg_info("Current battery temperature = %d, battery status is COOLDOWN!\n", temperature);
	}

	return event_type;
}

static int max17050_por_application_setting(struct max17050_device *dev)
{
	max17050_write_verify_reg(MAX17050_CONFIG, dev->config->config);
	max17050_write_verify_reg(MAX17050_FILTERCFG, dev->config->filtercfg);	//may needhw provide
	max17050_write_verify_reg(MAX17050_RELAXCFG, dev->config->relaxcfg);	//may needhw provide
	max17050_write_verify_reg(MAX17050_LEARNCFG, dev->config->learncfg);	//may needhw provide
	max17050_write_verify_reg(MAX17050_FULLSOCTHR, dev->config->fullsocthr);

	return 0;
}

static int max17050_por_load_model(struct max17050_device *dev)
{
	int i;
	/* Unlock model using magic lock numbers from Maxim */
	max17050_write_verify_reg(MAX17050_MODEL_LOCK1, 0x59);
	max17050_write_verify_reg(MAX17050_MODEL_LOCK2, 0xc4);
	/* Write characterisation data */
	max17050_write_verify_block( MAX17050_MODEL_TABLE,
						dev->config->model, MODEL_SIZE);
	/* Lock the model */
	max17050_write_verify_reg(MAX17050_MODEL_LOCK1, 0);
	max17050_write_verify_reg(MAX17050_MODEL_LOCK2, 0);
	/* Verify that we can't read the model any more */
	for (i = 0; i < MODEL_SIZE; i++)
		if (max17050_read_reg(MAX17050_MODEL_TABLE + i))
			fg_info("model is non-zero");

	return 0;
}

static int max17050_por_write_custom_params(struct max17050_device *dev)
{
	max17050_write_verify_reg(MAX17050_RCOMP0, dev->config->rcomp0);
	max17050_write_verify_reg(MAX17050_TEMPCO, dev->config->tempco);
	max17050_write_verify_reg(MAX17050_ICHGTERM, dev->config->ichgterm);
	//max17050_write_verify_reg(MAX17050_TEMPNOM, dev->config->tempnom);
	max17050_write_verify_reg(MAX17050_TGAIN, dev->config->tgain);
	max17050_write_verify_reg(MAX17050_TOFF, dev->config->toff);
	max17050_write_verify_reg(MAX17050_V_EMPTY, dev->config->vempty);
	max17050_write_verify_reg(MAX17050_QRTABLE00, dev->config->qrtable00);
	max17050_write_verify_reg(MAX17050_QRTABLE10, dev->config->qrtable10);
	max17050_write_verify_reg(MAX17050_QRTABLE20, dev->config->qrtable20);
	max17050_write_verify_reg(MAX17050_QRTABLE30, dev->config->qrtable30);
	//max17050_write_verify_reg(MAX17050_IAVG_EMPTY, dev->config->iavg_empty);

	return 0;
}

static int max17050_por_update_full_capacity(struct max17050_device *dev)
{
	max17050_write_verify_reg(MAX17050_FULLCAP, dev->config->capacity);
	max17050_write_verify_reg(MAX17050_DESIGNCAP, dev->config->vf_fullcap);
	max17050_write_verify_reg(MAX17050_FULLCAPNOM, dev->config->vf_fullcap);

	return 0;
}


static int max17050_por_write_vfsoc_and_qh0(struct max17050_device *dev)
{
	int vfsoc, qh;

	vfsoc = max17050_read_reg(MAX17050_VFSOC);
	max17050_write_verify_reg(MAX17050_VFSOC0_LOCK, 0x0080);
	max17050_write_verify_reg(MAX17050_VFSOC0, vfsoc);
	max17050_write_verify_reg(MAX17050_VFSOC0_LOCK, 0);

	qh = max17050_read_reg(MAX17050_QH);
	max17050_write_verify_reg(MAX17050_QH0, qh);

	return 0;
}

static int max17050_por_load_new_capacity_params(struct max17050_device *dev)
{
	int remcap, repcap, vfsoc;

	vfsoc = max17050_read_reg(MAX17050_VFSOC);
	if (vfsoc < 0)
		return -1;

	remcap = (vfsoc * dev->config->vf_fullcap) / 25600;
	max17050_write_verify_reg(MAX17050_REMCAP, remcap);

	repcap = remcap * (dev->config->capacity / dev->config->vf_fullcap);
	max17050_write_verify_reg(MAX17050_REPCAP, repcap);

	max17050_write_verify_reg(MAX17050_DPACC, dev->config->dpacc);
	max17050_write_verify_reg(MAX17050_DQACC, (dev->config->vf_fullcap) / 16);
	max17050_write_verify_reg(MAX17050_FULLCAP, dev->config->capacity);
	max17050_write_verify_reg(MAX17050_DESIGNCAP, dev->config->vf_fullcap);
	max17050_write_verify_reg(MAX17050_FULLCAPNOM, dev->config->vf_fullcap);
	max17050_write_verify_reg(MAX17050_REPSOC, vfsoc);

	return 0;
}

int max17050_register_batt_status_notify(int (*callback)(int))
{
	int ret = 0;
	if(callback){
		max17050_dev.batt_status_callback = callback;
	} else {
		ret = -1;
	}

	return ret;
}

int max17050_register_get_batt_online(int (*callback)(uint8_t *))
{
	int ret = 0;
	if(callback){
		max17050_dev.get_batt_online = callback;
	} else {
		ret = -1;
	}

	return ret;
}

static int max17050_cfg_init(struct max17050_device *dev)
{
	/* Wait 500ms after power up */
	HAL_Delay(500);
	/*Set COFF and MiscRate register*/
	//max17050_write_reg(MAX17050_CGAIN, 0x4000);	//whether need this config
	//max17050_write_reg(MAX17050_COFF, 0);
	max17050_write_reg(MAX17050_MISCCFG, dev->config->misccfg);	//may need hw provide
	/*Application specific settings	*/
	if (max17050_por_application_setting(dev))
		return -1;

	/*load model table*/
	if (max17050_por_load_model(dev))
		return -1;

	/* Write custom parameters */
	if (max17050_por_write_custom_params(dev))
		return -1;

	/* Update full capacity registers */
	if (max17050_por_update_full_capacity(dev))
		return -1;
	/* Wait for VFSOC to be calculated */
	HAL_Delay(360);
	/* Write VFSOC value to VFSOC0, write QH to QH0 */

	if (max17050_por_write_vfsoc_and_qh0(dev))
		return -1;

	//max17050_write_verify_reg(MAX17050_CYCLES, 96);		//whether need this config

	/* Write temperature */
	//max17050_write_verify_reg(MAX17050_TEMP, dev->config->temperature);

	/* Load new capacity params */
	if (max17050_por_load_new_capacity_params(dev))
		return -1;

	/* clear POR, BI and BR bit */
	dev->status = max17050_read_reg(MAX17050_STATUS);
	max17050_write_verify_reg(MAX17050_STATUS,
		dev->status & ~(MAX17050_STATUS_POR | MAX17050_STATUS_BR |
							MAX17050_STATUS_BI));
	/* Write custom parameter version */
	max17050_write_verify_reg(MAX17050_CUSTOMVER,
					dev->config->param_version);

	dev->status = max17050_read_reg(MAX17050_STATUS);
	/* Set the parameter init status */
	if (dev->status & (MAX17050_STATUS_POR | MAX17050_STATUS_BI))
		dev->power_on_reset = true;

	return 0;
}

static void max17050_complete_init(struct max17050_device *dev)
{
	int val;
	max17050_soc_thresholds_init(dev);
	max17050_temperature_thresholds_init(dev);
	/* Enable capacity interrupts */
	val = max17050_read_reg( MAX17050_CONFIG);
	max17050_write_reg( MAX17050_CONFIG,
					(val | MAX17050_CONFIG_AEN |
					MAX17050_CONFIG_TS |
					MAX17050_CONFIG_SS));
	if (dev->use_ext_temp && dev->timer_id != NULL)
		osTimerStart(dev->timer_id, 0);
	dev->init_done = true;
}

static void max17050_intr_work(void const * argument)
{
	uint16_t val;
	//uint32_t ulNotificationValue;
	uint16_t intr_pin;
	int event_type = 0;

	struct max17050_device *dev = &max17050_dev;

	do {
		int soc = 0;
		xQueueReceive( dev->fgMsgQueueHandle, &intr_pin, portMAX_DELAY );
		//xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);
		dev->intr_handler_token = pdFALSE;
		fg_info("%s: fuel gauge interrupt uccoured!\n", __func__);
		if ((intr_pin & GPIO_PIN_7) == GPIO_PIN_7) {
			val = max17050_read_reg(MAX17050_STATUS);
			fg_info("%s: val = %#x\n", __func__, val);
			if (val & MAX17050_STATUS_SMN) {
				/* Reset capacity thresholds */
				max17050_soc_exceed_handler(dev, true);
				event_type = (int)max17050_batt_soc_changed(dev);

			} else if (val & MAX17050_STATUS_SMX) {
				/* Reset capacity thresholds */
				max17050_soc_exceed_handler(dev, false);
				max17050_get_soc(dev->handle, &soc);
				event_type = (int)max17050_batt_soc_changed(dev);
			}

			if (val & MAX17050_STATUS_TMN) {
				max17050_temperature_exceed_handler(dev, true);
				event_type = (int)max17050_batt_temp_changed(dev);
			} else if (val & MAX17050_STATUS_TMX) {
				max17050_temperature_exceed_handler(dev, false);
				event_type = (int)max17050_batt_temp_changed(dev);
			}
			/*clear interrupt bit*/
			max17050_write_reg(MAX17050_STATUS, 0);
			if (dev->batt_status_callback != NULL)
				dev->batt_status_callback(event_type);
		}
	} while(1);
}

void max17050_intr_handler(uint16_t GPIO_Pin)
{
	struct max17050_device *dev = &max17050_dev;
	if (dev->intr_handler == NULL || dev->isInitialized == 0)
		return;

	dev->intr_handler_token = pdFALSE;
	if (dev->intr_handler == NULL)  return ;
	xQueueSendFromISR(dev->fgMsgQueueHandle, &GPIO_Pin, &dev->intr_handler_token);
	//xTaskNotifyFromISR(dev->intr_handler, GPIO_Pin, eSetBits, &dev->intr_handler_token);
	portYIELD_FROM_ISR(dev->intr_handler_token);
}

static int max17050_create_intr_thread(struct max17050_device *dev)
{
	dev->intr_handler_token = pdFALSE;
	osThreadDef(max17050_intr_thread, max17050_intr_work, osPriorityNormal, 0,
						configMINIMAL_STACK_SIZE * 2);
	dev->intr_handler = osThreadCreate(osThread(max17050_intr_thread), dev);
	if (dev->intr_handler == NULL)
		return -1;

	return 0;
}

static int max17050_init(PmicDrvTypeDef *handle)
{
	int ret = 0;
	uint16_t version, val;
	bool new_custom_param = false;

	memset(&max17050_dev, 0, sizeof(max17050_dev));
	max17050_dev.config = &max17050_cfg;
	max17050_dev.handle = handle;

	if (!(max17050_dev.config->r_sns))
		return -1;

	max17050_dev.use_ext_temp = !!(max17050_dev.config->config & MAX17050_CONFIG_TEX);
	if (max17050_dev.use_ext_temp)
		max17050_dev.timer_def.ptimer = max17050_update_ext_temp;
		max17050_dev.timer_id = osTimerCreate(&max17050_dev.timer_def,
									osTimerPeriodic, &max17050_dev);
	osMessageQDef(fg_intr_queue, 4, uint16_t);
	max17050_dev.fgMsgQueueHandle = osMessageCreate(osMessageQ(fg_intr_queue), NULL);

	if (max17050_create_intr_thread(&max17050_dev) != 0) {
		fg_err("%s create thread ERROR\n!", __func__);
		return -1;
	}

	osMutexDef(fuel_gauge_mutex);
	max17050_dev.mutex = osMutexCreate(osMutex(fuel_gauge_mutex));
	if (max17050_dev.mutex == NULL) {
		fg_err("%s create mutex failed!\n", __func__);
		return -1;
	}
	/*
	 * If the version of custom parameters is changed, the init dev
	 * should be called even if POR doesn't happen.
	 * For checking parameter change, the reserved register, 0x20,
	 * is defined as custom version register.
	 */
	version = max17050_read_reg( MAX17050_VERSION);
		fg_info("%s: chip version = 0x%x\n", __func__, version);

	if (max17050_read_reg( MAX17050_CUSTOMVER) != max17050_dev.config->param_version)
		new_custom_param = true;
	val = max17050_read_reg( MAX17050_STATUS);
	fg_info("%s: status REG = %#x, new_custom_param = %#x\n", __func__, val, new_custom_param);
	if ((val & (MAX17050_STATUS_POR | MAX17050_STATUS_BI)) || new_custom_param) {
		max17050_cfg_init(&max17050_dev);
		max17050_complete_init(&max17050_dev);
	} else
		max17050_complete_init(&max17050_dev);

	max17050_dev.isInitialized = 1;

	fg_info("%s initial DONE!\n", __func__);

	return ret;
}

static int max17050_deinit(PmicDrvTypeDef *handle)
{
	struct max17050_device *dev = &max17050_dev;

	osMutexDelete(dev->mutex);
	osTimerDelete(dev->timer_id);
	dev->isInitialized = 0;
	fg_info("%s: driver unregistered!\n", __func__);

	return 0;
}


GAUGE_DRV_t MAX17050_Drv = {
    max17050_init,
    max17050_deinit,
    max17050_get_temperature,
    max17050_get_soc,
    max17050_get_current,
    max17050_get_voltage,
    max17050_batt_online,
    max17050_get_property,
    max17050_get_full_capacity,
};
