#include <stdlib.h>
#include <string.h>
#include "i2c_drv.h"
#include "max17050_fuel_gauge.h"


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
	//osMutexId mutex;
	bool use_ext_temp;
	bool init_done;
	bool power_on_reset;
	//osThreadId intr_handler;
	//BaseType_t intr_handler_token;
	//osTimerId timer_id;
	//osTimerDef_t timer_def;
	uint32_t next_update_time;
	int ext_temp;
	/*alert value*/
	uint8_t min_salrt;
	uint8_t max_salrt;
	int8_t min_talrt;
	int8_t max_talrt;
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
	//BATT_STATUS_DET batt_status_callback;
	//GET_BATT_ONLINE get_batt_online;
};

/* External battery power supply poll times */
#define EXT_BATT_FAST_PERIOD		100
#define EXT_BATT_SLOW_PERIOD	10000

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
	if (ret & 0x80)
		*soc = val + 1;
	else
		*soc = val;

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
		fg_info("%s: Current average current = %d\n", __func__, (*current / 1000));
	} else {
		ret = max17050_current(&max17050_dev, 0, current);
		fg_info("%s: Current current = %d\n", __func__, (*current / 1000));
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
	fg_info("%s: Current voltage = %d\n", __func__, *voltage);

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
	fg_info("%s: Current temperature = %d\n",__func__, *temperature);
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
	}

	max17050_dev.isInitialized = 1;

	fg_info("%s initial DONE!\n", __func__);

	return ret;
}

static int max17050_deinit(PmicDrvTypeDef *handle)
{
	struct max17050_device *dev = &max17050_dev;
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
    max17050_get_full_capacity,
};
