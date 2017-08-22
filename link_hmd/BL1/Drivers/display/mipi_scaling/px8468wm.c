#include "FreeRTOS.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
#include "component.h"
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "rtos_i2c_drv.h"
#include "x_pmic.h"
#include "gpio.h"
#include "gpio_exp.h"
#include "PowerManager_notify_func.h"

#include "display_drv.h"
#include "hdmi2dsi_driver.h"
#include "dp2hdmi_anx7737.h"
#include "dp2hdmi_ps176.h"
#include "lcm_drv.h"

#include "px8468wm.h"


/* other function init need according this value */
static enum mipi_input_timing input_timing = timing_no_define;
//static enum mipi_output_timing output_timing = timing_default;
static bool px8468wm_is_initialized = 0;
static const lcm_drv_t* lcm_drv;

//static void px8468wm_dsi_set_cmdq(const dsi_setting_table *para_list, const unsigned int size);
//static unsigned int px8468wm_dsi_dcs_read_reg_v2(unsigned char cmd, unsigned char *buffer, unsigned char buffer_size);

void px8468wm_delay_ms(uint32_t ms)
{
	//disp_info("wait_ms(%d)\n", ms);
    HAL_Delay(ms);
}

void px8468wm_delay_us(uint32_t us)
{
	//disp_info("wait_us(%d)\n", us);
    HAL_Delay(1);
}

static dsi_funcs px8468wm_dsi_fns = {
	NULL
	//.dsi_set_cmdq = px8468wm_dsi_set_cmdq,
	//.dsi_dcs_read_reg_v2 = dsi_dcs_read_reg_v2,
};

void usleep_range(uint32_t m, uint32_t n)
{
	//TODO
	px8468wm_delay_ms(m/1000);
}

#if USE_HAL_I2C
extern I2C_HandleTypeDef hi2c1;
#if HTC_USE_I2C2
extern I2C_HandleTypeDef hi2c2;
#endif
#endif
static int px8468wm_platform_i2c_read(uint8_t Instance, uint8_t *buff, int len)
{
	int ret;
#if USE_HAL_I2C
	ret = HAL_I2C_Master_Receive(&hi2c1, PX8468WM_SLAVE_ADDR, buff, len, 100);
#else
	ret = RTOS_I2C_Master_Receive(PX8468WM_SLAVE_ADDR, buff, len, 100);
#endif
	return ret;
}

/* TODO: have same two chip on i2c master1&2, at the same time write hi2c1 and hi2c2 */
static int px8468wm_platform_i2c_write(uint8_t Instance, uint8_t *buff, int len)
{
	int ret;
#if USE_HAL_I2C
	ret = HAL_I2C_Master_Transmit(&hi2c1, PX8468WM_SLAVE_ADDR, buff, len, 100);
#if HTC_USE_I2C2
	ret |= HAL_I2C_Master_Transmit(&hi2c2, PX8468WM_SLAVE_ADDR, buff, len, 100);
#endif
#else
	ret = RTOS_I2C_Master_Transmit(PX8468WM_SLAVE_ADDR, buff, len, 100);
#endif
	return ret;
}

static int iris_i2c_cmd_four_read(uint8_t Instance, struct addr_val * val, int len)
{
	int i = 0;
	int ret = -1;
	int pos = 0;
	const int reg_len = 5;
	const int ret_len = 4;
	uint8_t cmd = 0xcc;
	uint8_t *data = NULL;
	uint8_t * ret_data = NULL;

	/*for ret value need to be N * len
		 * N is cmd + val+ ret (1+1+1,1+2+2,1+4+4)*/
	uint8_t nine_data_list[9 * MAX_READ_MSG_LEN] = {0,};

	for (i = 0; i < len; i++) {
		pos = 9 * i;
		nine_data_list[pos] = cmd;
		nine_data_list[pos + 1] = (val[i].addr & 0xff);
		nine_data_list[pos + 2] = ((val[i].addr >> 8) & 0xff);
		nine_data_list[pos + 3] = ((val[i].addr >> 16) & 0xff);
		nine_data_list[pos + 4] = ((val[i].addr >> 24) & 0xff);
		data = &nine_data_list[pos];
		ret_data = &nine_data_list[pos + reg_len];

		ret = px8468wm_platform_i2c_write(Instance, data, reg_len);
		if (ret) {
			return ret;
		}
		ret = px8468wm_platform_i2c_read(Instance, ret_data, ret_len);
		if (ret) {
			return ret;
		}
	}

	for (i = 0; i < len; i++) {
		pos = 9 * i + 5;
		val[i].data = (nine_data_list[pos] << 24) |(nine_data_list[pos + 1] << 16)
						| (nine_data_list[pos + 2] << 8) | nine_data_list[pos + 3];
	}

	return 0;
}

static int iris_i2c_cmd_four_write(uint8_t Instance, struct addr_val * val, int len)
{
	int i = 0;
	int ret = -1;
	int pos = 0;
	const int reg_len = 9;
	const uint8_t cmd = 0xcc;
	uint8_t *data = NULL;

	/*for ret value need to be N * len
		 * N is cmd + addr+ val (1+1+1,1+2+2,1+4+4)*/
	uint8_t nine_data_list[9 * MAX_WRITE_MSG_LEN] = {0,};

	for (i = 0; i < len; i++) {
		pos = reg_len * i;
		nine_data_list[pos] = cmd;
		nine_data_list[pos + 1] = (val[i].addr & 0xff);
		nine_data_list[pos + 2] = ((val[i].addr >> 8) & 0xff);
		nine_data_list[pos + 3] = ((val[i].addr >> 16) & 0xff);
		nine_data_list[pos + 4] = ((val[i].addr >> 24) & 0xff);
		nine_data_list[pos + 5] = ((val[i].data >> 24) &0xff);
		nine_data_list[pos + 6] = ((val[i].data >> 16) &0xff);
		nine_data_list[pos + 7] = ((val[i].data >> 8) &0xff);
		nine_data_list[pos + 8] = (val[i].data & 0xff);

		data = &nine_data_list[pos];

		ret = px8468wm_platform_i2c_write(Instance, data, reg_len);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/* now we support four byte read and write */
int iris_i2c_read(uint8_t Instance, struct addr_val *val, int len)
{
	if (!val || len == 0) {
		disp_err("the return buf = %p or len = %d \n",
				val, len);
		return -1;
	}

	if (len > MAX_READ_MSG_LEN) {
		disp_err("the len is too long for read len = %d "
				"MAX_READ_MSG_LEN = %d \n", len, MAX_READ_MSG_LEN);
		return -1;
	}

	return iris_i2c_cmd_four_read(Instance, val, len);
}

int iris_i2c_write(uint8_t Instance, struct addr_val *val, int len)
{
	if (!val || len == 0) {
		disp_err("the return buf = %p or len = %d \n",
				val, len);
		return -1;
	}

	if (len > MAX_WRITE_MSG_LEN) {
		disp_err("the len is too long for write len = %d "
				"MAX_WRITE_MSG_LEN = %d \n", len, MAX_WRITE_MSG_LEN);
		return -1;
	}

	return iris_i2c_cmd_four_write(Instance, val, len);
}

#if 0
static void px8468wm_dsi_set_cmdq(const dsi_setting_table *cmd_list, const unsigned int size)
{
    int i = 0, j = 0;

    disp_info("dsi_set_cmdq start.\n");

    for (i = 0; i < size; i++) {
        switch (cmd_list[i].id) {
            case REGFLAG_ESCAPE_ID:
                if (REGFLAG_DELAY_MS == cmd_list[i].cmd) {
                    px8468wm_delay_ms(cmd_list[i].count);
                } else {
                    disp_err("unknow cmd:%d\n", cmd_list[i].cmd);
                }
                break;
            case 0x05:
                if (0 == cmd_list[i].count) {
                    tc358870_i2c_write16(0x0504,0x0005);
                    tc358870_i2c_write16(0x0504,cmd_list[i].cmd&0xFF);
                } else {
                    disp_err("count should be 0, for DI:0x05 cmd:%d\n", cmd_list[i].cmd);
                }
                break;
            case 0x15:
                if (1 == cmd_list[i].count) {
                    tc358870_i2c_write16(0x0504,0x0015);
                    tc358870_i2c_write16(0x0504,(cmd_list[i].para_list[0] << 8) | (cmd_list[i].cmd&0xFF));
                } else {
                    disp_err("count should be 1, for DI:0x05 cmd:%d\n", cmd_list[i].cmd);
                }
                break;
            default:
                if (0 == cmd_list[i].count) {
                    tc358870_i2c_write16(0x0504,cmd_list[i].id&0xFF);
                    tc358870_i2c_write16(0x0504,cmd_list[i].cmd&0xFF);
                } else {
                    uint16_t payloadCnt = cmd_list[i].count + 1; //should include cmd.
                    //packet header
                    tc358870_i2c_write16(0x0504,(0x80<<8) | cmd_list[i].id&0xFF);
                    tc358870_i2c_write16(0x0504,payloadCnt);
                    //payload
                    tc358870_i2c_write16(0x0504,(cmd_list[i].para_list[0] << 8) | (cmd_list[i].cmd&0xFF));
                    for (j = 0; j < (cmd_list[i].count - 1)/2; j++) {
                        tc358870_i2c_write16(0x0504,(cmd_list[i].para_list[j*2+1+1]<<8) | cmd_list[i].para_list[j*2 + 1]&0xFF);
                    }
                    if (1 == (cmd_list[i].count + 1)%2) {
                        tc358870_i2c_write16(0x0504,cmd_list[i].para_list[cmd_list[i].count - 1]&0xFF);
                    }
                }
                break;
        }
    }

    disp_info("dsi_set_cmdq end.\n");
}
#endif

//this function don't modify for customer.
int iris_i2c_rewrite_efuse(uint8_t Instance)
{
	int len = 0;
	int ret0 = -1, ret1 = -1, ret2 = -1;

	struct addr_val addr_list[3];

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf0010004);
	addr_list[0].data = cpu_to_le32(0x00000005);
	addr_list[1].addr = cpu_to_le32(0xf0010000);
	addr_list[1].data = cpu_to_le32(0x81000100);
	addr_list[2].addr = cpu_to_le32(0xf0000028);
	addr_list[2].data = cpu_to_le32(0x00000001);
	len = 3;
	ret0 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,100000);
	addr_list[0].addr = cpu_to_le32(0xf0000028);
	addr_list[0].data = cpu_to_le32(0x00000003);
	len = 1;
	ret1 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,100000);
	addr_list[0].addr = cpu_to_le32(0xf0000028);
	addr_list[0].data = cpu_to_le32(0x00000002);
	len = 1;
	ret2 = iris_i2c_write(Instance, addr_list, len);

	return (ret0 || ret1 || ret2);
}

//this function sets the MIPI TX setting, if panel timing doesn't change, the function doesn't need to modify.
int iris_i2c_mipi_tx_init(uint8_t Instance)
{
	int len = 0;
	int ret = -1;
					//dsi-tx-ctrl hs-tx-timer bta-lp-timer reset-timer dphy-timing-margin lp-timing-para data-lane-timing-para clock-lane-timing-para dphy-pll-para dphy-trim-1
#ifdef PANEL_1080_1200
	uint32_t mipi_tx_data[10] = {   0x0a00c038,   0x752fff,   0xffff17,    0xa8c0780,   0x40401,          0xe010006,      0x120b0f05,           0x120a2707,            0x780,       0xedb5380c   };
#else
	uint32_t mipi_tx_data[10] = {	0x0a00c038,   0x7247ff,   0xffff17,    0xa8c0750,	0x40401,		  0xe010006,	  0x110a0f05,			0x110a2607, 		   0x780,		0xedb5380c	 };
#endif
	struct addr_val addr_list[12];

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf0180000);
	addr_list[0].data = cpu_to_le32(mipi_tx_data[0]);
	addr_list[1].addr = cpu_to_le32(0xf0180040);
	addr_list[1].data = cpu_to_le32(mipi_tx_data[4]);
	addr_list[2].addr = cpu_to_le32(0xf0180050);
	addr_list[2].data = cpu_to_le32(mipi_tx_data[5]);

	addr_list[3].addr = cpu_to_le32(0xf0180054);
	addr_list[3].data = cpu_to_le32(mipi_tx_data[6]);
	addr_list[4].addr = cpu_to_le32(0xf0180058);
	addr_list[4].data = cpu_to_le32(mipi_tx_data[7]);
	addr_list[5].addr = cpu_to_le32(0xf0180064);
	addr_list[5].data = cpu_to_le32(mipi_tx_data[9]);

	addr_list[6].addr = cpu_to_le32(0xf0180004);
	addr_list[6].data = cpu_to_le32(0x00000001);
	addr_list[7].addr = cpu_to_le32(0xf0180024);
	addr_list[7].data = cpu_to_le32(mipi_tx_data[1]);
	addr_list[8].addr = cpu_to_le32(0xf0180028);
	addr_list[8].data = cpu_to_le32(mipi_tx_data[2]);
	addr_list[9].addr = cpu_to_le32(0xf018002c); //0xf0180002c
	addr_list[9].data = cpu_to_le32(mipi_tx_data[3]);
	addr_list[10].addr = cpu_to_le32(0xf0180030);
	addr_list[10].data = cpu_to_le32(0x00000004);
	addr_list[11].addr = cpu_to_le32(0xf018005c);
	addr_list[11].data = cpu_to_le32(mipi_tx_data[8]);

	len = 12;
	ret = iris_i2c_write(Instance, addr_list, len);

	return ret;
}

//this function sets MIPI RX setting, when input timing changes, the MIPI RX need to modify.
int iris_i2c_mipi_rx_init(uint8_t Instance)
{
	int len = 0;
	int ret0 = -1, ret1 = -1;

	struct addr_val addr_list[11];

	memset(addr_list , 0x00, sizeof(addr_list));

	//mipi rx reset
	addr_list[0].addr = cpu_to_le32(0xf0120000);
	addr_list[0].data = cpu_to_le32(0x00000000);
	addr_list[1].addr = cpu_to_le32(0xf0120030);
	addr_list[1].data = cpu_to_le32(0x00000000);
	len = 2;
	ret0 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,10000);

	addr_list[0].addr = cpu_to_le32(0xf0120030);
	addr_list[0].data = cpu_to_le32(0x00000001);
	addr_list[1].addr = cpu_to_le32(0xf010000c);
	addr_list[1].data = cpu_to_le32(0x000f0000);
	addr_list[2].addr = cpu_to_le32(0xf0100018);
	addr_list[2].data = cpu_to_le32(0x04370000);

	addr_list[3].addr = cpu_to_le32(0xf010001c);
	addr_list[3].data = cpu_to_le32(0xffffffff);
	addr_list[4].addr = cpu_to_le32(0xf011ffe8);
	addr_list[4].data = cpu_to_le32(0x00000003);
	addr_list[5].addr = cpu_to_le32(0xf0120008);
	addr_list[5].data = cpu_to_le32(0x00000000);

	addr_list[6].addr = cpu_to_le32(0xf012000c);
	addr_list[6].data = cpu_to_le32(0x00000064);
	addr_list[7].addr = cpu_to_le32(0xf0120024);
	addr_list[7].data = cpu_to_le32(0x00000001);
	addr_list[8].addr = cpu_to_le32(0xf012002c);
	addr_list[8].data = cpu_to_le32(0x0000ff07);

	addr_list[9].addr = cpu_to_le32(0xf0120058);
	addr_list[9].data = cpu_to_le32(0x00020013);
	addr_list[10].addr = cpu_to_le32(0xf0120000);
	addr_list[10].data = cpu_to_le32(0x00000001);

	len = 11;
	ret1 = iris_i2c_write(Instance, addr_list, len);

	return (ret0 || ret1);
}

//this function sets the PLL and clock source select. when panel timing changes, only the DPLL need to modify.
//DPLL: htotalxVtotalxframe rate = 1316x2053x90=243.157320MHz.
int iris_i2c_sys_init(uint8_t Instance)
{
	int len = 0;
	int ret0 = -1, ret1 = -1, ret2 = -1;

	struct addr_val addr_list[10];
					                  //DCLK_SRC_SEL  INCLK_SRC_SEL  MCUCLK_SRC_SEL  PCLK_SRC_SEL  MCLK_SRC_SEL
	uint32_t clock_select_setting[5] = {   0x1,             0x1,             0x1009,         0x502,     0x509      };

	uint32_t ppll_b_setting[3] = { 0x00000002, 0x003f0901, 0x00480000 };
#ifdef PANEL_1080_1200
	uint32_t dpll_b_setting[3] = { 0x00002002, 0x00321101, 0x00a863f1 };
#else
	uint32_t dpll_b_setting[3] = { 0x00002002, 0x001f1101, 0x00529c78 };
#endif
	uint32_t mpll_b_setting[3] = { 0x0000003f, 0x003e0901, 0x00800000 };

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf000000c);
	addr_list[0].data = cpu_to_le32(0x42180102);
	addr_list[1].addr = cpu_to_le32(0xf0000010);
	addr_list[1].data = cpu_to_le32(0x00000008);
	//set clock source select
	addr_list[2].addr = cpu_to_le32(0xf0000210);
	addr_list[2].data = cpu_to_le32(clock_select_setting[0]);
	addr_list[3].addr = cpu_to_le32(0xf0000214);
	addr_list[3].data = cpu_to_le32(clock_select_setting[1]);
	addr_list[4].addr = cpu_to_le32(0xf0000218);
	addr_list[4].data = cpu_to_le32(clock_select_setting[2]);
	addr_list[5].addr = cpu_to_le32(0xf000021c);
	addr_list[5].data = cpu_to_le32(clock_select_setting[3]);
	addr_list[6].addr = cpu_to_le32(0xf0000228);
	addr_list[6].data = cpu_to_le32(clock_select_setting[4]);

	len = 7;
	ret0 = iris_i2c_write(Instance, addr_list, len);

	//xPLL_B_CTRL pll setting
	addr_list[0].addr = cpu_to_le32(0xf0000140);
	addr_list[0].data = cpu_to_le32(ppll_b_setting[0]);
	addr_list[1].addr = cpu_to_le32(0xf0000144);
	addr_list[1].data = cpu_to_le32(ppll_b_setting[1]);
	addr_list[2].addr = cpu_to_le32(0xf0000148);
	addr_list[2].data = cpu_to_le32(ppll_b_setting[2]);
	addr_list[3].addr = cpu_to_le32(0xf0000150);
	addr_list[3].data = cpu_to_le32(dpll_b_setting[0]);
	addr_list[4].addr = cpu_to_le32(0xf0000154);
	addr_list[4].data = cpu_to_le32(dpll_b_setting[1]);
	addr_list[5].addr = cpu_to_le32(0xf0000158);
	addr_list[5].data = cpu_to_le32(dpll_b_setting[2]);
	addr_list[6].addr = cpu_to_le32(0xf0000160);
	addr_list[6].data = cpu_to_le32(mpll_b_setting[0]);
	addr_list[7].addr = cpu_to_le32(0xf0000164);
	addr_list[7].data = cpu_to_le32(mpll_b_setting[1]);
	addr_list[8].addr = cpu_to_le32(0xf0000168);
	addr_list[8].data = cpu_to_le32(mpll_b_setting[2]);
	//xPLL_PROG_EN enable pll setting
	addr_list[9].addr = cpu_to_le32(0xf0000200);
	addr_list[9].data = cpu_to_le32(0x0001800e);

	len = 10;
	ret1 = iris_i2c_write(Instance, addr_list, len);

	usleep_range(10000,10000);
	addr_list[0].addr = cpu_to_le32(0xf0000200);
	addr_list[0].data = cpu_to_le32(0x00018000);
	len = 1;
	ret2 = iris_i2c_write(Instance, addr_list, len);

	return (ret0 || ret1 || ret2);
}

//this function sets MIPI RX work mode
int iris_i2c_mipi_rx_mode_set(uint8_t Instance, enum mipi_rx_work_mode eMode)
{
	uint32_t temp = 0;
	int ret = -1;
	struct addr_val addr_list[1];

	memset(addr_list , 0x00, sizeof(addr_list));

	temp = 0x000f0000 + (1 << (eMode + 2));

	addr_list[0].addr = cpu_to_le32(0xf010000c);
	addr_list[0].data = cpu_to_le32(temp);
	ret = iris_i2c_write(Instance, addr_list, 1);
	return ret;
}

//this function sets work timing, for example, input timing and output timing.
/*output timing HS  20
                HBP 46
                HFP 40
                VS  2
                VBP 234
                VFP 28 */
int iris_i2c_work_mode_set(uint8_t Instance, uint32_t *pInputTiming)
{
	int len = 0, i;
	int ret = -1;
	uint32_t *pOutputTiming = NULL;

	struct addr_val addr_list[9];
				               // HRES+HFP    HSW+HBP      VRES+VFP      VSW_VBP    INPUT DEN, HS, VS POL
#ifdef PANEL_1080_1200
	uint32_t output_timing[5] = { 0x04380028, 0x0014002e,   0x04b0001c,  0x002000ea,  0x000031cf };
#else
	uint32_t output_timing[5] = { 0x04380078, 0x00130046,   0x07800012,  0x00020004,  0x000031cf };
#endif
	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf0212c00);
	addr_list[0].data = cpu_to_le32(0x00080000);
	for(i=0;i<5;i++)
	{
		addr_list[i+1].addr = cpu_to_le32(0xf0212c04+i*4);
		addr_list[i+1].data = cpu_to_le32(*pInputTiming);
		pInputTiming++;
	}

	len = 6;
	ret = iris_i2c_write(Instance, addr_list, len);

	pOutputTiming = &output_timing[0];
	for(i=0;i<5;i++)
	{
		addr_list[i].addr = cpu_to_le32(0xf0212c18+i*4);
		addr_list[i].data = cpu_to_le32(*pOutputTiming);
		pOutputTiming++;
	}
	addr_list[5].addr = cpu_to_le32(0xf0212c30);
	addr_list[5].data = cpu_to_le32(0x00100008);//INPUT DSC SILCE H AND NUMBER
	addr_list[6].addr = cpu_to_le32(0xf0212c34);
	addr_list[6].data = cpu_to_le32(0x00000080);//DSCbpp
	addr_list[7].addr = cpu_to_le32(0xf0212c38);
	addr_list[7].data = cpu_to_le32(0x00100008);//OUTPUT DSC SILCE H AND NUMBER
	addr_list[8].addr = cpu_to_le32(0xf0212c3c);
	addr_list[8].data = cpu_to_le32(0x00000080);//DSCbpp

	len = 9;
	ret = iris_i2c_write(Instance, addr_list, len);

	return ret;
}

//this function set data path.
int iris_i2c_data_path_set(uint8_t Instance, enum ap_to_romcode_cmd eCmd)
{
	uint32_t temp = 0;
	int ret = -1;

	struct addr_val addr_list[2];

	memset(addr_list , 0x00, sizeof(addr_list));

	if(eCmd == config_data_path)
	{
		temp = 1;
	}
	else if(eCmd == enable_dport)
	{
		temp = 2;
	}

	addr_list[0].addr = cpu_to_le32(0xf0040008);
	addr_list[0].data = cpu_to_le32(temp);
	addr_list[1].addr = cpu_to_le32(0xf0040038);
	addr_list[1].data = cpu_to_le32(0x00040000);

	ret = iris_i2c_write(Instance, addr_list, 2);

	return ret;
}

//this function disable dport output.
int iris_i2c_dport_disable(uint8_t Instance)
{
	int ret = -1;

	struct addr_val addr_list[2];

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf1220004);
	addr_list[0].data = cpu_to_le32(0x21008800);

	ret = iris_i2c_write(Instance, addr_list, 1);

	return ret;
}

//this function sets the panel init command.
int iris_i2c_panel_init(uint8_t Instance)
{
	int len = 0;
	int ret0 = -1, ret1 = -1, ret2 = -1, ret3 = -1;
	#ifdef PANEL_1080_1200
	int ret4 = -1, ret5 = -1, ret6 = -1;
	#endif

	struct addr_val addr_list[12];

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf0180004);
	addr_list[0].data = cpu_to_le32(0x00000000);
	addr_list[1].addr = cpu_to_le32(0xf1220004);
	addr_list[1].data = cpu_to_le32(0x21008800);
	len = 2;
	ret0 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,10000);
	addr_list[0].addr = cpu_to_le32(0xf0180004);
	addr_list[0].data = cpu_to_le32(0x00000001);//DPHY reset
	addr_list[1].addr = cpu_to_le32(0xf0180000);
	addr_list[1].data = cpu_to_le32(0x0a00c039);
	len = 2;
	ret1 = iris_i2c_write(Instance, addr_list, len);
	#ifdef PANEL_1080_1200
    //panel common setting
	/* level2 key unlock */
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x005a5af0);
    /* resolution setting */
	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000a29);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x000000f2);
	addr_list[4].addr = cpu_to_le32(0xf019c014);
	addr_list[4].data = cpu_to_le32(0x38040000);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x0000b004);
    /* AID control GI 16.6 setting */
	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000529);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x04c304b2);
	addr_list[8].addr = cpu_to_le32(0xf019c014);
	addr_list[8].data = cpu_to_le32(0x000000c3);
	len = 8;
	ret2 = iris_i2c_write(Instance, addr_list, len);
	
	/* source control */
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x001536b9);
	/* pentile control */
	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000329);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x002000c1);
	/* power control */
	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000329);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x001002f4);
	/* VLIN1 control */
	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000829);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x745a00b8);
	addr_list[8].addr = cpu_to_le32(0xf019c014);
	addr_list[8].data = cpu_to_le32(0x19000003);
	len = 9;
	ret3 = iris_i2c_write(Instance, addr_list, len);
	
	/* LTPS update */
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000229);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x000001f7);
	/* level2 key lock */
	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000329);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x00a5a5f0);
	/* brightness control 
	   DIMMING control   0x20 ----------1 frame
	                     0x28 ----------32 frame */
	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000229);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x00002053);
    /* 0x00 -- 0xff luminance setting
	   0xff    250 nit
	   0xd8    212 nit
	   0xb2    175 nit
	   0x8c    137 nit
	   0x5e    92  nit
	   0x33    50  nit */
	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000229);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x0000d851);
	len = 8;
	ret4 = iris_i2c_write(Instance, addr_list, len);
	
	//OTP loading set control
	/* level3 key unlock */
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x005a5afc);
	/* Global para */
	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000229);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x000005b0);
	/* OTP loading */
	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000229);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x000080d1);
	/* sleep out */
	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000229);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x00000011);
	len = 8;
	ret5 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(120000,120000);

	/* level3 lock key set */
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x00a5a5fc);

	//ERR_FLAG Control
    /* OTP key unlock */
	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000329);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x005a5af1);
	/* global para */
	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000229);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x000001b0);
	/* TE pin normally low */
	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000229);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x00000acc);
	/* OTP key lock */
	addr_list[8].addr = cpu_to_le32(0xf019c010);
	addr_list[8].data = cpu_to_le32(0x07000329);
	addr_list[9].addr = cpu_to_le32(0xf019c014);
	addr_list[9].data = cpu_to_le32(0x00a5a5f1);
	
	addr_list[10].addr = cpu_to_le32(0xf019c010);
	addr_list[10].data = cpu_to_le32(0x07000229);
	addr_list[11].addr = cpu_to_le32(0xf019c014);
	addr_list[11].data = cpu_to_le32(0x00000029);
	len = 12;
	ret6 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,10000);
	#else
	//panel command init
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000229);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x00000011);
	len = 2;
	ret2 = iris_i2c_write(Instance, addr_list, len);
	usleep_range(50000,50000);
	
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x005a5af0);

	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000429);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x0a0001f2);

	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000329);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x005a5af0);

	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000229);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x00001eb0);

	addr_list[8].addr = cpu_to_le32(0xf019c010);
	addr_list[8].data = cpu_to_le32(0x07000229);
	addr_list[9].addr = cpu_to_le32(0xf019c014);
	addr_list[9].data = cpu_to_le32(0x00009efd);

	len = 10;
	ret2 = iris_i2c_write(Instance, addr_list, len);

	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000329);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x00a5a5fc);

	addr_list[2].addr = cpu_to_le32(0xf019c010);
	addr_list[2].data = cpu_to_le32(0x07000229);
	addr_list[3].addr = cpu_to_le32(0xf019c014);
	addr_list[3].data = cpu_to_le32(0x00002053);

	addr_list[4].addr = cpu_to_le32(0xf019c010);
	addr_list[4].data = cpu_to_le32(0x07000329);
	addr_list[5].addr = cpu_to_le32(0xf019c014);
	addr_list[5].data = cpu_to_le32(0x0000ff51);

	addr_list[6].addr = cpu_to_le32(0xf019c010);
	addr_list[6].data = cpu_to_le32(0x07000229);
	addr_list[7].addr = cpu_to_le32(0xf019c014);
	addr_list[7].data = cpu_to_le32(0x00000029);

	addr_list[8].addr = cpu_to_le32(0xf019c010);
	addr_list[8].data = cpu_to_le32(0x07000239);
	addr_list[9].addr = cpu_to_le32(0xf019c014);
	addr_list[9].data = cpu_to_le32(0x00000036);

	addr_list[10].addr = cpu_to_le32(0xf019c010);
	addr_list[10].data = cpu_to_le32(0x07000239);
	addr_list[11].addr = cpu_to_le32(0xf019c014);
	addr_list[11].data = cpu_to_le32(0x0000773a);

	len = 12;
	ret3 = iris_i2c_write(Instance, addr_list, len);	
	usleep_range(10000,10000);
	#endif

#ifdef PANEL_1080_1200
	return (ret0 || ret1 || ret2 || ret3 || ret4 || ret5 || ret6);
#else
	return (ret0 || ret1 || ret2 || ret3);
#endif
}

//this function initialize DTG block
int iris_i2c_dtg_init(uint8_t Instance)
{
	int len = 0;
	int ret = -1;
	struct addr_val addr_list[9];

	memset(addr_list , 0x00, sizeof(addr_list));
	//panel command init
	addr_list[0].addr = cpu_to_le32(0xf1200034);
	addr_list[0].data = cpu_to_le32(0x00000003);
	addr_list[1].addr = cpu_to_le32(0xf1200020);
	addr_list[1].data = cpu_to_le32(0x54020219);

	addr_list[2].addr = cpu_to_le32(0xf120002c);
	addr_list[2].data = cpu_to_le32(0x00000006);
	addr_list[3].addr = cpu_to_le32(0xf1200030);
	addr_list[3].data = cpu_to_le32(0x00000001);

	addr_list[4].addr = cpu_to_le32(0xf1200090);
	addr_list[4].data = cpu_to_le32(0x0004c800);
	addr_list[5].addr = cpu_to_le32(0xf120007c);
	addr_list[5].data = cpu_to_le32(0x01000010);

	addr_list[6].addr = cpu_to_le32(0xf1200080);
	addr_list[6].data = cpu_to_le32(0x00000020);
	addr_list[7].addr = cpu_to_le32(0xf12000a8);
	addr_list[7].data = cpu_to_le32(0x1000000b);

	addr_list[8].addr = cpu_to_le32(0xf1210000);
	addr_list[8].data = cpu_to_le32(0x00000001);

	len = 9;
	ret = iris_i2c_write(Instance, addr_list, len);
	usleep_range(10000,10000);
	return ret;
}
//this function initialize PWIL block
int iris_i2c_pwil_init(uint8_t Instance)
{
	int len = 0;
	int ret = -1;
	struct addr_val addr_list[12];

	memset(addr_list , 0x00, sizeof(addr_list));

	addr_list[0].addr = cpu_to_le32(0xf1241024);
	addr_list[0].data = cpu_to_le32(0x00010003);
	addr_list[1].addr = cpu_to_le32(0xf124133c);
	addr_list[1].data = cpu_to_le32(0x07800000);

	addr_list[2].addr = cpu_to_le32(0xf1241000);
	addr_list[2].data = cpu_to_le32(0xe4100020);
	addr_list[3].addr = cpu_to_le32(0xf1241024);
	addr_list[3].data = cpu_to_le32(0x00010003);

	addr_list[4].addr = cpu_to_le32(0xf1250000);
	addr_list[4].data = cpu_to_le32(0x00000300);
	addr_list[5].addr = cpu_to_le32(0xf1240274);
	addr_list[5].data = cpu_to_le32(0x00010014);

	addr_list[6].addr = cpu_to_le32(0xf1240278);
	addr_list[6].data = cpu_to_le32(0x002fc014);
	addr_list[7].addr = cpu_to_le32(0xf1240280);
	addr_list[7].data = cpu_to_le32(0x00008014);

	addr_list[8].addr = cpu_to_le32(0xf1240284);
	addr_list[8].data = cpu_to_le32(0x0004000f);
	addr_list[9].addr = cpu_to_le32(0xf124028c);
	addr_list[9].data = cpu_to_le32(0x10080010);

	addr_list[10].addr = cpu_to_le32(0xf1240294);
	addr_list[10].data = cpu_to_le32(0x00066692);
	addr_list[11].addr = cpu_to_le32(0xf1250000);
	addr_list[11].data = cpu_to_le32(0x00000300);

	len = 12;
	ret = iris_i2c_write(Instance, addr_list, len);

	return ret;
}

int iris2_init(uint8_t Instance)
{
	int ret0 = -1, ret1 = -1, ret2 = -1, ret3 = -1;
	int ret4 = -1, ret5 = -1;

	ret0 = iris_i2c_rewrite_efuse(Instance);
	ret1 = iris_i2c_mipi_tx_init(Instance);
	ret2 = iris_i2c_mipi_rx_init(Instance);
	ret3 = iris_i2c_sys_init(Instance);
	ret4 = iris_i2c_dtg_init(Instance);
	ret5 = iris_i2c_pwil_init(Instance);

	return (ret0 || ret1 || ret2 || ret3 || ret4 || ret5);
}

int iris2_panel_on(uint8_t Instance)
{
	int ret0 = -1, ret1 = -1;

	ret0 = iris_i2c_panel_init(Instance);
	usleep_range(10000,10000);
	ret1 = iris_i2c_data_path_set(Instance, enable_dport);
	usleep_range(10000,10000);

	return (ret0 || ret1);
}

int iris2_panel_off(uint8_t Instance)
{
	int ret = -1;

	struct addr_val addr_list[2];

	memset(addr_list , 0x00, sizeof(addr_list));

	//disable dport
	addr_list[0].addr = cpu_to_le32(0xf1220004);
	addr_list[0].data = cpu_to_le32(0x21008800);
	ret = iris_i2c_write(Instance, addr_list, 1);
	usleep_range(10000,10000);
	//display off
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000229);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x00000028);
	ret = iris_i2c_write(Instance, addr_list, 1);
	usleep_range(50000,50000);
	//sleep in
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(0x07000229);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(0x00000010);
	ret = iris_i2c_write(Instance, addr_list, 1);
	usleep_range(120000,120000);

	return ret;
}

int iris2_input_timing_select(uint8_t Instance, enum mipi_input_timing timing_mode)
{
	int ret0 = -1, ret1 = -1, ret2 = -1, ret3 = -1;
					         // HRES+HFP    HSW+HBP	   VRES+VFP     VSW_VBP    INPUT DEN, HS, VS POL
	uint32_t input_timing_tbl_960_1080[5]  = { 0x03c00078, 0x001300be, 0x04380012, 0x0002034c, 0x000031cf };
	uint32_t input_timing_tbl_667_750[5]   = { 0x029b0078, 0x001300be, 0x02ee0012, 0x0002034c, 0x000031cf };
	uint32_t input_timing_tbl_1280_1440[5] = { 0x05000078, 0x001300be, 0x05a00012, 0x0002034c, 0x000031cf };

	input_timing = timing_mode;
	ret0 = iris_i2c_mipi_rx_mode_set(Instance, mcu_video);
	usleep_range(10000,10000);
	if (timing_960_1080 == input_timing) {
		ret1 = iris_i2c_work_mode_set(Instance, input_timing_tbl_960_1080);
	} else if (timing_667_750 == input_timing) {
		ret1 = iris_i2c_work_mode_set(Instance, input_timing_tbl_667_750);
	} else if (timing_1280_1440 == input_timing) {
		ret1 = iris_i2c_work_mode_set(Instance, input_timing_tbl_1280_1440);
	}
	usleep_range(10000,10000);
	ret2 = iris_i2c_mipi_rx_mode_set(Instance, pwil_video);
	usleep_range(10000,10000);
	ret3 = iris_i2c_data_path_set(Instance, config_data_path);
	usleep_range(10000,10000);

	return (ret0 || ret1 || ret2 || ret3);
}

int iris2_panel_command_write(uint8_t Instance, uint32_t cmd_header, uint32_t cmd_payload)
{
	int ret = -1;
	struct addr_val addr_list[2];

	memset(addr_list , 0x00, sizeof(addr_list));
	addr_list[0].addr = cpu_to_le32(0xf019c010);
	addr_list[0].data = cpu_to_le32(cmd_header);
	addr_list[1].addr = cpu_to_le32(0xf019c014);
	addr_list[1].data = cpu_to_le32(cmd_payload);
	ret = iris_i2c_write(Instance, addr_list, 2);

	return ret;
}

int iris2_check_status(uint8_t Instance)
{
	int ret = -1;
	struct addr_val addr_list[1];

	memset(addr_list , 0x00, sizeof(addr_list));
	addr_list[0].addr = cpu_to_le32(0xf0000230);
	addr_list[0].data = cpu_to_le32(0);
	ret = iris_i2c_read(Instance, addr_list, 1);

	if(!ret)
	{
		if(addr_list[0].data == CHIP_VERSION)
		{
			disp_info("Iris2 chip check status is ok!");
		}
		else
		{
			disp_info("Iris2 chip check status is fail!");
		}
	}

	return ret;
}

void px8468wm_gpio_config(void)
{
#if 0 // TODO config interrupt pin
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Pin = GPIO_XB_INT_H_R_LS;
	HAL_GPIO_Init(GPIO_XB_PORT_INT_H_R_LS, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_XB_INT_H_L_LS;
	HAL_GPIO_Init(GPIO_XB_PORT_INT_H_L_LS, &GPIO_InitStruct);
#endif

	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_PX_RESET_L_LS, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_L_LS, 0);
	ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_PX_RESET_R_LS, IOEXP_GPIO_OUTPUT);
	ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_R_LS, 0);

}

void px8468wm_reset_pin_set(int val)
{
	if (val) {
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_L_LS, 1);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_R_LS, 1);
	} else {
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_L_LS, 0);
		ioexp_gpio_set_value(IOEXP_REG_OUT, IOEXP_II_PX_RESET_R_LS, 0);
	}
}

void px8468wm_power_en(int enable)
{
	if (enable) {
		px8468wm_reset_pin_set(0);
		px8468wm_delay_ms(1);
		bsp_pmic_power_enable(V_PX_1V1_EN, 1);
		bsp_pmic_power_enable(V_PX_1V8_EN, 1);
		px8468wm_delay_ms(5);	// > 1ms
		px8468wm_reset_pin_set(1);
		px8468wm_delay_ms(5);
	} else {
		px8468wm_reset_pin_set(0);
		px8468wm_delay_ms(5);	// > 0ms
		bsp_pmic_power_enable(V_PX_1V1_EN, 0);
		bsp_pmic_power_enable(V_PX_1V8_EN, 0);
		px8468wm_delay_ms(5);	// > 500us
	}
}

int px8468wm_init(enum mipi_input_timing timing_mode)
{
	if (true == px8468wm_is_initialized) {
		disp_info("%s: px8468wm is initialized already.\n", __func__);	
		return -1;
	}
	
	/* other function init need according this value */
	input_timing = timing_mode;
    lcm_drv = LCM_GetDriver();
    lcm_drv->set_dsi_funcs(&px8468wm_dsi_fns);

	px8468wm_gpio_config();
	px8468wm_power_en(1);
	iris2_init(0); // don't need to care about instance, the L panel is always same to R panel

	px8468wm_is_initialized = true;
	disp_info("%s: px8468wm initialize \"%s\" mode success.\n", __func__, mipi_input_timing_str[input_timing]);

	return 0;
}

int px8468wm_exit(void)
{
	if (true != px8468wm_is_initialized) {
		disp_info("%s: px8468wm has not been initialized.\n", __func__);	
		return -1;
	}

    lcm_drv->set_dsi_funcs(NULL);
	px8468wm_power_en(0);
	input_timing = timing_no_define;

	px8468wm_is_initialized = false;
	disp_info("%s: px8468wm exit \"%s\" mode success.\n", __func__, mipi_input_timing_str[input_timing]);

	return 0;
}




