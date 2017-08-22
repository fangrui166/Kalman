#define OS_ENABLE                       (0)
#if OS_ENABLE
#include "FreeRTOS.h"

#include "cmsis_os.h"
#endif

#include "string.h"
#include "stdio.h"
#if OS_ENABLE
#include "component.h"
#endif
#include "Stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#if OS_ENABLE
#include "rtos_i2c_drv.h"
#else
#include "i2c_drv.h"
#endif
#include "x_pmic.h"
#include "gpio.h"
#if OS_ENABLE
#include "PowerManager_notify_func.h"
#include "PowerManager_power.h"
#include "htc_3dof_transfer_service.h"
#endif

#include "display_drv.h"
#include "hdmi2dsi_driver.h"
#include "dp2hdmi_anx7737.h"
#include "dp2hdmi_ps176.h"
#include "lcm_drv.h"
#include "htc_memory_define.h"

//below is for debug purpose, can be removed when release.
#define COLOR_BAR_TEST                  (1)
#define CHECK_DCSCMD_ST_ENABLE          (0)
#define INT_STATUS_CHECK_ENABLE         (0)
#if OS_ENABLE
#define HDMI_SYNC_CHECK_ENABLE          (1)
#endif
#define ENABLE_GET_REGS_FROM_PARTION    (0)
#define CHECK_GAMMA_SETTING             (1) //it's depend on DP_SOURCE_VIDEO_TIMING


#define HOTPLUG_NOTIFY_ENABLE           (0)
#define DP_SOURCE_VIDEO_TIMING          (58) // 57, 58, 90 HZ
#if OS_ENABLE
#define GI_FLM_ENABLE                   (1)
#endif



#define TC358870_I2C_TIMEOUT  0x1000

#define H2D_EXTI_IRQn                        (EXTI15_10_IRQn)
#define H2D_XB_EXTI_IRQn                     (EXTI15_10_IRQn)

#define H2D_RST_PIN                         (GPIO_H2D_RST_N)
#define H2D_RST_PIN_PORT                    (GPIO_PORT_H2D_RST_N)
#define H2D_XB_RST_PIN                      (GPIO_XB_H2D_RST_N)
#define H2D_XB_RST_PIN_PORT                 (GPIO_XB_PORT_H2D_RST_N)

#define BIT(nr)             (0x1 << (nr))
#define ARRAY_SIZE(x)						(sizeof((x)) / sizeof((x[0])))

#define HDMI_SYNC_EVENT_BIT                 BIT(0)
#define DP_SRC_PLUGIN_EVENT_BIT             BIT(1)
#define DP_SRC_PLUGOUT_EVENT_BIT            BIT(2)
#define REQUEST_POWER_OFF_EVENT_BIT         BIT(3)
#define REQUEST_POWER_ON_EVENT_BIT          BIT(4)
#define REQUEST_POWER_RESET_EVENT_BIT       BIT(5)
#define EVENTS_MASK                         (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5))
//init status transfer: H2D_STATUS_UNINITIALIZE-->H2D_STATUS_RESET-->H2D_STATUS_INITIALIZED
//status transfer: H2D_STATUS_INITIALIZED-->H2D_STATUS_RESUME
//status transfer: H2D_STATUS_RESUME-->H2D_STATUS_UNINITIALIZE


typedef enum {
    INIT_STATUS_UNINITIALIZE = 0,
    INIT_STATUS_RESET,
    INIT_STATUS_INITIALIZED,
    INIT_STATUS_ERROR
} h2d_init_status_e;

typedef enum {
    H2D_STATUS_DISCONNECT = 0,
    H2D_STATUS_SLEEP,
    H2D_STATUS_RESUEM,
    H2D_STATUS_ERROR
} h2d_status_e;

struct tc358870xbg_context {
    h2d_init_status_e init_status;
    h2d_status_e status;
    const lcm_drv_t* lcm_drv;
    int is_regs_from_partion;
};

static struct tc358870xbg_context context = {
            .init_status = INIT_STATUS_UNINITIALIZE,
            .status = H2D_STATUS_DISCONNECT,
            .lcm_drv = 0,
            .is_regs_from_partion = 0
        };

typedef struct {
    uint32_t reg_type;
    uint32_t addr;
    uint32_t value;
} reg_item;

#if ENABLE_GET_REGS_FROM_PARTION
typedef struct {
    uint32_t magic_num;
    float fps;
    uint32_t startup_p1_tbl_size;
    reg_item *startup_p1_tbl;
    uint32_t startup_p2_tbl_size;
    reg_item *startup_p2_tbl;
    uint32_t colorbar_p1_tbl_size;
    reg_item *colorbar_p1_tbl;
    uint32_t colorbar_p2_tbl_size;
    reg_item *colorbar_p2_tbl;
} h2d_tbl_t;

static h2d_tbl_t h2d_tbl;

#define PARTION_ADDR   REGION_FLASH_SECTOR5
#define H2D_MAGIC      (0x68326473)
#endif

static void dsi_set_cmdq(const dsi_cmd_raw *para_list, const unsigned int size);
static unsigned int dsi_dcs_read_reg_v2(unsigned char cmd, unsigned char *buffer, unsigned char buffer_size);
static int sync_event_handle(void);


static dsi_funcs dsi_fns = {
            .dsi_set_cmdq = dsi_set_cmdq,
			.dsi_dcs_read_reg_v2 = dsi_dcs_read_reg_v2,
        };

#if OS_ENABLE
static osThreadId h2d_task_handle = NULL;
static osMutexId ops_mutex = NULL;
static osMessageQId h2dMsgQueueHandle = NULL;
static volatile uint32_t ops_flag = 0;

#if HDMI_SYNC_CHECK_ENABLE
static TaskHandle_t hdmi_sync_check_task = NULL;
#endif
static struct pwrmgr_notify_func_data h2dPmNotifyData = {0};
#endif //OS_ENABLE

#if GI_FLM_ENABLE
static uint32_t l_gi_flm_1_int_cnt = 0;
static uint32_t r_gi_flm_1_int_cnt = 0;
static uint32_t gi_change_cnt_expected = DP_SOURCE_VIDEO_TIMING;
static bool gi_flm_err_rst_en = false;
#endif

extern pcbid_t pcb_id;
static uint32_t g_disp_fps = DP_SOURCE_VIDEO_TIMING;
static uint32_t disp_power_off_last_tick;


#if CHECK_DCSCMD_ST_ENABLE
static void check_dcscmd_st(void);
#endif
static void lcd_and_h2d_reset(void);
static void start_up_init(void);
static void h2d_power_en(int enable);
static void dp2h_power_en(int enable);
#if COLOR_BAR_TEST
static void start_up_init_colorbar(void);
#endif

static void h2d_reset_pin(int val);
static int disp_fps_set(uint8_t fps);

static int tc358870_i2c_write8(uint16_t reg, uint8_t value)
{
    I2C_STATUS ret = I2C_OK;

    //disp_info("tc358870_i2c_write8(0x%04X,0x%02X);\n", reg, value);

    ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &value, 1, TC358870_I2C_TIMEOUT);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int tc358870_i2c_read8(uint16_t reg, uint8_t *value)
{
    I2C_STATUS ret = I2C_OK;

    ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, TC358870_I2C_TIMEOUT);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int tc358870_i2c_write16(uint16_t reg, uint16_t value)
{
    I2C_STATUS ret = I2C_OK;
    uint8_t data[2];

    data[0] = value & 0xFF;
    data[1] = value >> 8;

    //disp_info("tc358870_i2c_write16(0x%04X,0x%04X); (0x%02X,0x%02X)\n", reg, value, data[0], data[1]);
#if CHECK_DCSCMD_ST_ENABLE
    if (0x0504 == reg) {
        disp_info("tc358870_i2c_write16(0x%04X,0x%04X); (0x%02X,0x%02X)\n", reg, value, data[0], data[1]);
        //check_dcscmd_st();
    }
#endif

    ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 2, TC358870_I2C_TIMEOUT);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int tc358870_i2c_read16(uint16_t reg, uint16_t *value)
{
    I2C_STATUS ret = I2C_OK;
    uint8_t data[2];

    ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 2, TC358870_I2C_TIMEOUT);

    *value = (data[0]&0xFF) | (data[1]<<8);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int tc358870_i2c_write32(uint16_t reg, uint32_t value)
{
    I2C_STATUS ret = I2C_OK;
    uint8_t data[4];

    data[0] = value & 0xFF;
    data[1] = value >> 8;
    data[2] = value >> 16;
    data[3] = value >> 24;

    //disp_info("tc358870_i2c_write32(0x%04X, 0x%08X);  (0x%02X,0x%02X,0x%02X,0x%02X)\n", reg, value, data[0], data[1], data[2], data[3]);

    ret = RTOS_I2C_WriteBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 4, TC358870_I2C_TIMEOUT);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int tc358870_i2c_read32(uint16_t reg, uint32_t *value)
{
    I2C_STATUS ret = I2C_OK;
    uint8_t data[4];

    ret = RTOS_I2C_ReadBuffer(I2C_DEVICE_TC358870_ADDR, reg, I2C_MEMADD_SIZE_16BIT, data, 4, TC358870_I2C_TIMEOUT);

    *value = (data[0]&0xFF) | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);

    if(ret != I2C_OK)
    {
        disp_err("%s status=%d failed\n", __func__, ret);
        return -1;
    }

    return 0;
}

void wait_ms(int ms)
{
	//disp_info("wait_ms(%d)\n", ms);
    HAL_Delay(ms);
}

void wait_us(int us)
{
	//disp_info("wait_us(%d)\n", us);
    HAL_Delay(1);
}

#define DCS_QUEUE_WAIT_CNT (3)

#define DCS_CMD_ENTRY_MASK      (0x3F<<8) // [13:8]
#define DCS_CMD_DONE_MASK       (0x3<<3)  // [4:3]
#define DCS_CMD_OFLOW_BIT       BIT(2)
#define DCS_CMD_EMPTY_BIT       BIT(1)
#define DCS_CMD_FUL_BIT         BIT(0)
#if CHECK_DCSCMD_ST_ENABLE
static void check_dcscmd_st(void)
{
    uint16_t dcscmd_st = 0;

    tc358870_i2c_read16(0x0502, &dcscmd_st);//DCSCMD_ST

    disp_info("%s dcscmd_st %x\n", __func__, dcscmd_st);
    disp_info("%s dcs_cmd_entry=%d\n", __func__, (dcscmd_st&DCS_CMD_ENTRY_MASK)>>8);
    disp_info("%s dcs_cmd_done=%d\n", __func__, (dcscmd_st&DCS_CMD_DONE_MASK)>>3);
    disp_info("%s DCS_CMD_OFLOW_BIT=%d\n", __func__, (dcscmd_st&DCS_CMD_OFLOW_BIT)>>2);
    disp_info("%s DCS_CMD_EMPTY_BIT=%d\n", __func__, (dcscmd_st&DCS_CMD_EMPTY_BIT)>>1);
    disp_info("%s DCS_CMD_FUL_BIT=%d\n", __func__, dcscmd_st&DCS_CMD_FUL_BIT);
}
#endif

static void wait_dcs_queue(int count)
{
    uint16_t dcscmd_st = 0;
    int i = 0;

    for (i = 0; i < DCS_QUEUE_WAIT_CNT; i++) {
        tc358870_i2c_read16(0x0502, &dcscmd_st);//DCSCMD_ST
        if (((dcscmd_st&DCS_CMD_ENTRY_MASK)>>8) >= count) {
            break;
        }
        if (dcscmd_st&DCS_CMD_OFLOW_BIT) {
            disp_err("%s DCS_CMD_OFLOW_BIT.\n", __func__);
            tc358870_i2c_write16(0x0502, dcscmd_st | DCS_CMD_OFLOW_BIT);
        }
        disp_info("%s DCS CMD queue if not enough, wait 10ms\n", __func__);
        wait_ms(10);
    }
    if (DCS_QUEUE_WAIT_CNT == i) {
        disp_err("%s DCS CMD queue can't empty enough items! count=%d.\n", __func__, count);
    }
}

static void readFromDSI(int dsi_if)
{
    int j = 0, cnt = 0;
    uint32_t value = 0;
    uint16_t offset = 0x0;

    if (1 == dsi_if) {
        offset = 0x200;
    }

    tc358870_i2c_read32(0x01BC + offset, &value);
    disp_debug("check dsi%d: received header part 0x%08x.\n", dsi_if, value); //data id and data or long packet data id and wc.

    if (((value>>16)&0xFF) == 0x1C) {//long packet.
        tc358870_i2c_read32(0x01C4 + offset, &value);
        disp_debug("check dsi%d: Received 32bit count 0x%08x.\n", dsi_if, value);
        cnt = value;

        for (j = 0; j < cnt; j++) {
            tc358870_i2c_read32(0x01B8 + offset, &value); //read data
            disp_debug("check dsi%d: Received data 0x%02x, 0x%02x, 0x%02x, 0x%02x.\n", dsi_if,
                                                                ((uint8_t*)&value)[0],
                                                                ((uint8_t*)&value)[1],
                                                                ((uint8_t*)&value)[2],
                                                                ((uint8_t*)&value)[3]);
        }
    } else {
        disp_warning("check dsi%d: unknow packet type.\n", dsi_if);
    }
}

static void readDCSData(void)
{
    uint16_t cmd_sel = 0;

    tc358870_i2c_read16(0x0500, &cmd_sel);

    if (cmd_sel & BIT(2) || (cmd_sel & BIT(1)) == 0) {
        readFromDSI(0);
    }

    if (cmd_sel & BIT(2) || cmd_sel & BIT(1)) {
        readFromDSI(1);
    }
}

#if CHECK_GAMMA_SETTING
static void check_gamma_settings(void)
{
    //DSI Read Operation
    wait_dcs_queue(2);
    tc358870_i2c_write16(0x0504,0x0006);
    tc358870_i2c_write16(0x0504,0x00C8);
    wait_ms(10);

    readDCSData();
}
#endif

static void rewrite_gamma_settings(void)
{
    uint32_t value = 0;
    int j = 0, cnt = 0;
    uint16_t cmd_sel = 0;
    uint16_t offset = 0x0;

    tc358870_i2c_read16(0x0500, &cmd_sel);

    if (cmd_sel & BIT(2)) {
        disp_err("Must selec one dsi interface while write gamma settings.\n");
        return;
    }

    if (cmd_sel & BIT(1)) {
        offset = 0x200;
    }

    tc358870_i2c_read32(0x01BC + offset,&value);
    disp_debug("received header part 0x%08x.\n", value); //data id and data or long packet data id and wc.

    if (((value>>16)&0xFF) == 0x1C) {//long packet.
        tc358870_i2c_read32(0x01C4 + offset,&value);
        disp_debug("Received 32bit count 0x%08x.\n", value);
        cnt = value;
        for (j = 0; j < cnt; j++) {
            tc358870_i2c_read32(0x01B8 + offset,&value); //read data
            disp_debug("Received data 0x%02x, 0x%02x, 0x%02x, 0x%02x.\n", ((uint8_t*)&value)[0],
                                                                ((uint8_t*)&value)[1],
                                                                ((uint8_t*)&value)[2],
                                                                ((uint8_t*)&value)[3]);
        #if (DP_SOURCE_VIDEO_TIMING != 90)
            context.lcm_drv->init_gamma(j, value);
        #endif
        }
    #if (DP_SOURCE_VIDEO_TIMING != 90)
        context.lcm_drv->update_gamma();
    #endif
    } else {
        disp_warning("unknow packet type.\n");
    }
}

static void dsi_set_cmdq(const dsi_cmd_raw *cmd_list, const unsigned int size)
{
    int i = 0, j = 0;
    const dsi_cmd_t *dsi_cmd = 0;

    disp_info("dsi_set_cmdq start.\n");
#if CHECK_DCSCMD_ST_ENABLE
    check_dcscmd_st();
#endif

    for (i = 0; i < size; i++) {
        dsi_cmd = (const dsi_cmd_t *)cmd_list[i].raw;
        if (dsi_cmd->count != cmd_list[i].size) {
            disp_err("error cmd format!\n");
            return;
        }

        switch (dsi_cmd->id) {
            case REGFLAG_ESCAPE_ID:
                if (REGFLAG_DELAY_MS == dsi_cmd->cmd) {
                    if (1 == dsi_cmd->count) {
                        wait_ms(dsi_cmd->para[0]);
                    } else {
                        disp_err("count should be 1, for delay ms cmd.\n");
                    }
                } else {
                    disp_err("unknow cmd:%d\n", dsi_cmd->cmd);
                }
                break;
            case 0x05:
                if (0 == dsi_cmd->count) {
                    wait_dcs_queue(2);
                    tc358870_i2c_write16(0x0504,0x0005);
                    tc358870_i2c_write16(0x0504,dsi_cmd->cmd&0xFF);
                } else {
                    disp_err("count should be 0, for DI:0x05 cmd:%d\n", dsi_cmd->cmd);
                }
                break;
            case 0x15:
                if (1 == dsi_cmd->count) {
                    wait_dcs_queue(2);
                    tc358870_i2c_write16(0x0504,0x0015);
                    tc358870_i2c_write16(0x0504,(dsi_cmd->para[0] << 8) | (dsi_cmd->cmd&0xFF));
                } else {
                    disp_err("count should be 1, for DI:0x05 cmd:%d\n", dsi_cmd->cmd);
                }
                break;
            case 0x37:
                wait_dcs_queue(2);
                tc358870_i2c_write16(0x0504,0x0037);
                tc358870_i2c_write16(0x0504,dsi_cmd->cmd&0xFF);
                break;
            case 0x06:
                {
                    //DSI Read Operation
                    wait_dcs_queue(2);
                    tc358870_i2c_write16(0x0504,0x0006);
                    if (0 == dsi_cmd->count) {
                        tc358870_i2c_write16(0x0504, dsi_cmd->cmd&0xFF);
                    } else {
                        disp_err("count should be 0, for cmd:%d\n", dsi_cmd->cmd);
                        break;
                    }
                    wait_ms(10);
                    /*
                                tc358870_i2c_read32(0x01A0,&value);
                                disp_debug("DSI Read State 0x%x.\n", value);


                                if (0x1B == value) {
                                    disp_debug("Received done.\n");
                                } else if (0x1F == value) {
                                    disp_debug("FIFO full.\n");
                                } else {
                                    disp_err("err.\n");
                                }

                                tc358870_i2c_write32(0x01A0, 0x0000001F);//clear status bit.
                                */
                    if (0xC8 == dsi_cmd->cmd) {
                        uint16_t cmd_sel = 0;

                        tc358870_i2c_read16(0x0500, &cmd_sel);

                        tc358870_i2c_write16(0x0500, cmd_sel & (~(BIT(2)|BIT(1)))); // select dsi0.
                        rewrite_gamma_settings();
                        #if CHECK_GAMMA_SETTING
                        check_gamma_settings();
                        #endif

                        tc358870_i2c_write16(0x0500, (cmd_sel & (~(BIT(2)))|BIT(1))); // select dsi1.
                        rewrite_gamma_settings();
                        #if CHECK_GAMMA_SETTING
                        check_gamma_settings();
                        #endif

                        tc358870_i2c_write16(0x0500, cmd_sel);
                    } else {
                        readDCSData();
                    }
                }
                break;
            default:
                if (0 == dsi_cmd->count) {
                    wait_dcs_queue(2);
                    tc358870_i2c_write16(0x0504,dsi_cmd->id&0xFF);
                    tc358870_i2c_write16(0x0504,dsi_cmd->cmd&0xFF);
                } else {
                    uint16_t payloadCnt = dsi_cmd->count + 1; //should include cmd.
                    wait_dcs_queue(2 + (payloadCnt+1)/2);
                    //packet header
                    tc358870_i2c_write16(0x0504,(0x80<<8) | dsi_cmd->id&0xFF);
                    tc358870_i2c_write16(0x0504,payloadCnt);
                    //payload
                    tc358870_i2c_write16(0x0504,(dsi_cmd->para[0] << 8) | (dsi_cmd->cmd&0xFF));
                    for (j = 0; j < (dsi_cmd->count - 1)/2; j++) {
                        tc358870_i2c_write16(0x0504,(dsi_cmd->para[j*2+1+1]<<8) | dsi_cmd->para[j*2 + 1]&0xFF);
                    }
                    if (1 == (dsi_cmd->count + 1)%2) {
                        tc358870_i2c_write16(0x0504,dsi_cmd->para[dsi_cmd->count - 1]&0xFF);
                    }
                }
                break;
        }
    }

    disp_info("dsi_set_cmdq end.\n");
}

static unsigned int dsi_dcs_read_reg_v2(unsigned char cmd, unsigned char *buffer, unsigned char buffer_size)
{
	if (buffer == NULL || buffer_size == 0) {
		disp_err("dsi read fail: buffer = 0x%lx and buffer_size = %d\n", (unsigned long)buffer, (unsigned int)buffer_size);
		return 0;
	}

#if 1

	uint32_t recv_data_cnt;   /* return value */
	uint8_t packet_type;
	uint32_t reg32 = 0;
	uint8_t recv_fifo[32*4] = {0};
	int timeout = 0;
	int i = 0;

	do {
		recv_data_cnt = 0;

		/* 1. wait dsi not busy => can't read if dsi busy */

		/* 2. Check rd_rdy & cmd_done irq, enable irq and clear before irq status, if status bit has been set dump rxdata */
		tc358870_i2c_write32(0x01A0, 0x001C); // clear pkt_start, pkt_done, thresh_hit irq status.
		tc358870_i2c_write32(0x01C0, buffer_size); // set thresh_hit of dsi receive irq.

		/* 3. Send cmd */
		wait_dcs_queue(4);
		/* set max return size */
		tc358870_i2c_write16(0x0504, 0x0037);
		tc358870_i2c_write16(0x0504, buffer_size & 0xFF);
		tc358870_i2c_write16(0x0504, 0x0006); // 0x14 if cmd > 0xB0 ?
		tc358870_i2c_write16(0x0504, cmd & 0xFF);

		timeout = 0;
		do {
			timeout++;
			wait_ms(1);
			reg32 = 0;
			tc358870_i2c_read32(0x01A0, &reg32);
			if (reg32 & 0x10)
				break;
		} while (timeout < 1000); //wait pkt_start irq
		if (timeout == 1000) {
			/* wait cmd done timeout, clear rd rdy interrupt */
			disp_info("DSI wait irq timeout, line: %lu\n", __LINE__);
			tc358870_i2c_write32(0x01A0, 0x001C); // clear pkt_start, pkt_done, thresh_hit irq status.
			//tc358870_i2c_read32(0x01B8, &reg32);
			//disp_info("now rxfifo word0: %lu\n", reg32);
			return 0;
		}

		reg32 = 0;
		tc358870_i2c_read32(0x01BC, &reg32);
		packet_type = reg32 >> 16;
		disp_info("DSI read packet_type is 0x%x\n", packet_type);

		/* 0x02: acknowledge & error report */
		/* 0x11: generic short read response(1 byte return) */
		/* 0x12: generic short read response(2 byte return) */
		/* 0x1a: generic long read response */
		/* 0x1c: dcs long read response */
		/* 0x21: dcs short read response(1 byte return) */
		/* 0x22: dcs short read response(2 byte return) */
		if (packet_type == 0x11 || packet_type == 0x12 || packet_type == 0x21 || packet_type == 0x22) { /* short read response */
			tc358870_i2c_write32(0x01A0, 0x001C); // clear pkt_start, pkt_done, thresh_hit irq status.
			if (packet_type == 0x11 || packet_type == 0x21)
				recv_data_cnt = 1;
			else
				recv_data_cnt = 2;

			if (recv_data_cnt > buffer_size) {
				disp_err("DSI read short packet data exceeds buffer size: %d\n", buffer_size);
				recv_data_cnt = buffer_size;
			}

			tc358870_i2c_read32(0x01B8, &reg32);
			buffer[0] = reg32 & 0xff;
			if (2 == recv_data_cnt)
				buffer[1] = (reg32 >> 8) & 0xff;
			return recv_data_cnt;
		}

		/*not shot packet */
		timeout = 0;
		do {
			timeout++;
			wait_ms(1);
			reg32 = 0;
			tc358870_i2c_read32(0x01A0, &reg32);
			if (reg32 & 0x0C)
				break;
		} while (timeout < 1000); // wait pkt_done or thresh_hit irq
		if (timeout == 1000) {
			tc358870_i2c_write32(0x01A0, 0x001C); // clear pkt_start, pkt_done, thresh_hit irq status.
			disp_info("DSI wait irq timeout, line: %lu\n", __LINE__);
			return 0;
		}

		tc358870_i2c_write32(0x01A0, 0x001C); // clear pkt_start, pkt_done, thresh_hit irq status.
		reg32 = 0;
		tc358870_i2c_read32(0x01C4, &reg32);
		recv_data_cnt = reg32 & 0x1F;
		if (recv_data_cnt > buffer_size) {
			disp_err("DSI read short packet data exceeds buffer size: %d\n", buffer_size);
			recv_data_cnt = buffer_size;
		}

		/* packet_type should be 0x1A or 0x1C, need check packet_type TODO */
		for (i = 0; i < recv_data_cnt; i += 4)
			tc358870_i2c_read32(0x01B8, (uint32_t *)(recv_fifo + i));
		for (i = 0; i < recv_data_cnt; i++)
			buffer[i] = recv_fifo[i];
		return recv_data_cnt;

	} while(0);

#else
	return 0;
#endif

}

#if OS_ENABLE
void h2d_msg_enqueue_FromISR(uint32_t msg)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (h2dMsgQueueHandle) {
		xQueueSendFromISR(h2dMsgQueueHandle, &msg, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void h2d_msg_enqueue(uint32_t msg)
{
	if (h2dMsgQueueHandle) {
		xQueueSend(h2dMsgQueueHandle, &msg, portMAX_DELAY);
	}
}

void h2d_msg_dequeue(uint32_t *msg)
{
	if (h2dMsgQueueHandle) {
		xQueueReceive(h2dMsgQueueHandle, msg, portMAX_DELAY);
	}
}
#endif

void check_input_video_timing(void)
{
    uint8_t data = 0;

    //System clock frequency setting, X10k. 0x12C0 = 4800, 48MHz
    tc358870_i2c_write8(0x8540,0xC0); // SYS_FREQ0
    tc358870_i2c_write8(0x8541,0x12); // SYS_FREQ1
    tc358870_i2c_write8(0x8A0C,0xC0); // SYS_FREQ0(CSC)
    tc358870_i2c_write8(0x8A0D,0x12); // SYS_FREQ1(CSC)

    disp_debug("HDMI Input Video Timing Check (PCLK)\n");
    tc358870_i2c_read8(0x8405, &data); // PX_FREQ0
    disp_debug("PX_FREQ0 0x%02X\n", data);
    tc358870_i2c_read8(0x8406, &data); // PX_FREQ1
    disp_debug("PX_FREQ1 0x%02X\n", data);
    // HDMI Input Video Timing Check (Horizontal Related)
    tc358870_i2c_read8(0x858E, &data); // H_SIZE0
    disp_debug("H_SIZE0 0x%02X\n", data);
    tc358870_i2c_read8(0x858F, &data); // H_SIZE1
    disp_debug("H_SIZE1 0x%02X\n", data);
    tc358870_i2c_read8(0x8580, &data); // DE_HPOS0
    disp_debug("DE_HPOS0 0x%02X\n", data);
    tc358870_i2c_read8(0x8581, &data); // DE_HPOS1
    disp_debug("DE_HPOS1 0x%02X\n", data);
    tc358870_i2c_read8(0x8582, &data); // DE_HWID0
    disp_debug("DE_HWID0 0x%02X\n", data);
    tc358870_i2c_read8(0x8583, &data); // DE_HWID1
    disp_debug("DE_HWID1 0x%02X\n", data);
    // HDMI Input Video Timing Check (Vertical Related)
    tc358870_i2c_read8(0x8590, &data); // V_SIZE0
    disp_debug("V_SIZE0 0x%02X\n", data);
    tc358870_i2c_read8(0x8591, &data); // V_SIZE1
    disp_debug("V_SIZE1 0x%02X\n", data);
    tc358870_i2c_read8(0x8584, &data); // DE_POS_A0
    disp_debug("DE_POS_A0 0x%02X\n", data);
    tc358870_i2c_read8(0x8585, &data); // DE_POS_A1
    disp_debug("DE_POS_A1 0x%02X\n", data);
    tc358870_i2c_read8(0x8586, &data); // DE_POS_B0
    disp_debug("DE_POS_B0 0x%02X\n", data);
    tc358870_i2c_read8(0x8587, &data); // DE_POS_B1
    disp_debug("DE_POS_B1 0x%02X\n", data);
    tc358870_i2c_read8(0x858C, &data); // DE_VWID0
    disp_debug("DE_VWID0 0x%02X\n", data);
    tc358870_i2c_read8(0x858D, &data); // DE_VWID1
    disp_debug("DE_VWID1 0x%02X\n", data);
    tc358870_i2c_read8(0x8526, &data); // VI_STATUS1
    disp_debug("VI_STATUS1 0x%02X.\n", data);
}

static int start_video_tx(void)
{
    if (H2D_STATUS_RESUEM == context.status) {
        disp_warning("already start video TX. Maybe you should reinitialize.\n");
        return 0;
    }

    wait_ms(32);

    //Start Video TX
    tc358870_i2c_write16(0x0004,0x0C37); // ConfCtl0
    tc358870_i2c_write16(0x0006,0x0000); // ConfCtl1
    //Command Transmission After Video Start.
    tc358870_i2c_write32(0x0110,0x00000006); // MODE_CONFIG
    tc358870_i2c_write32(0x0310,0x00000006); // MODE_CONFIG

    wait_ms(32);
    context.lcm_drv->display_on();

    context.status = H2D_STATUS_RESUEM;

    return 1;
}

static int stop_video_tx(void)
{
    if (H2D_STATUS_RESUEM != context.status) {
        disp_warning("already stop video TX.\n");
        return 0;
    }

    //Stop Video TX
    tc358870_i2c_write16(0x0004,0x0C34); // ConfCtl0
    //Command Transmission Before Video Start
    tc358870_i2c_write32(0x0110,0x00000016); // MODE_CONFIG
    tc358870_i2c_write32(0x0310,0x00000016); // MODE_CONFIG

    wait_ms(32);

    context.status = H2D_STATUS_SLEEP;

    return 1;
}

#if INT_STATUS_CHECK_ENABLE
static void int_status_check(void)
{
    uint16_t status = 0, int_sys_status = 0, int_flags = 0;
    uint8_t sys_int = 0, sys_status = 0, err_status = 0;
    uint8_t hdmi_int0 = 0, hdmi_int1 = 0;
    uint32_t dsi_prto_int_stat = 0;

    // Check interrupt factor
    tc358870_i2c_read16(0x0014, &status); // IntStatus.
    disp_debug("IntStatus 0x%x.\n", status);

    tc358870_i2c_read16(0x0018, &int_flags); // int_flags.
    disp_debug("int_flags 0x%x.\n", int_flags);

    //Check bit5 (SYS_INT interrupt).
    if (status & BIT(5)) {
        //DDC power change detection
        disp_debug("TC358870XBG System Interrupt Status.\n");
        tc358870_i2c_read16(0x001A, &int_sys_status); // int_sys_status.
        disp_debug("int_sys_status 0x%x.\n", int_sys_status);
        if (int_sys_status & BIT(0)) {
            //fixme.
            disp_debug("wake up the host when HPDI is changing.\n");
        }
    }

    //Check bit9 (HDMI-RX interrupt).
    if (status & BIT(9)) {
        disp_debug("HDMI-RX interrupt.\n");
        tc358870_i2c_read8(0x8500, &hdmi_int0);
        disp_debug("HDMI_INT0 0x%x.\n", hdmi_int0);
        //I_KEY
        if (hdmi_int0 & BIT(7)) {
            disp_warning("KEY-EDID (address 0x85_0F) interrupt.\n");
        }
        //I_MISC
        if (hdmi_int0 & BIT(1)) {
            disp_debug("MISC (address 0x85_0B) interrupt.\n");
            err_status = 0;
            tc358870_i2c_read8(0x850B, &err_status); // MISC_INT
            if (err_status) {
                disp_debug("check_error MISC_INT err_status=%x.\n", err_status);
                //I_SYNC_CHG
                if (err_status & BIT(1)) {
                    tc358870_i2c_read8(0x8520, &sys_status); // SYS_STATUS
                    if (sys_status & BIT(7)) {
                        disp_debug("synchronizaion is established.\n");
                        check_input_video_timing();
                    } else {
                        disp_debug("synchronization is lost."); //fixme maybe need to start a timer to check. or just reinitialize.
                    }
                }
            }
            tc358870_i2c_write8(0x850B,0xFF); // clear and initialize for start monitor ERR_INT.
        }

        tc358870_i2c_read8(0x8501, &hdmi_int1);
        disp_debug("HDMI_INT1 0x%x.\n", hdmi_int1);
        //I_SYS
        if (hdmi_int1 & BIT(0)) {
            disp_debug("SYSTEM (address 0x8502) interrupt.\n");
            tc358870_i2c_read8(0x8502, &sys_int); // SYS_INT. Check bit0 (DDC interrupt). If bit0=1 then execute next step.
            disp_debug("SYS_INT 0x%x.\n", sys_int);
            if (sys_int & BIT(0)) {
                disp_debug("_DDC 0~5V change detected.\n");
                tc358870_i2c_read8(0x8520, &sys_status); // SYS_STATUS. Check bit0 (Status of 5V). If bit0=0 then execute next step.
                disp_debug("SYS_STATUS 0x%x.\n", sys_status);
                if (!(sys_status & BIT(0))) {
                    disp_debug("DDC_Power (DDC5V) no input.\n");
                    // Stop LCD display.
                } else {
                    disp_debug("DDC_Power (DDC5V) input.\n");
                    //Reset LCD
                    // Assert hardware reset (RESETN)
                    // Execute "Source_Startup" sequence again
                }
            }
            tc358870_i2c_write8(0x8502,0xFF); // clear and initialize for start monitor SYS_INT.
        }
        //I_PACKET
        if (hdmi_int1 & BIT(2)) {
            err_status = 0;
            tc358870_i2c_read8(0x8504, &err_status); // I_PACKET
            if (err_status) {
                disp_debug("check_error I_PACKET err_status=%x.\n", err_status);
                //fixme, show Info Packet here.
            }
            tc358870_i2c_write8(0x8504,0xFF); // clear and initialize for start monitor I_PACKET.
        }
        //I_ERR
        if (hdmi_int1 & BIT(5)) {
            err_status = 0;
            tc358870_i2c_read8(0x8507, &err_status); // ERR_INT
            if (err_status) {
                disp_debug("check_error ERR_INT err_status=%x.\n", err_status);
            }
            tc358870_i2c_write8(0x8507,0xFF); // clear and initialize for start monitor ERR_INT.
        }
        //I_HDCP
        if (hdmi_int1 & BIT(6)) {
            err_status = 0;
            tc358870_i2c_read8(0x8508, &err_status); // HDCP_INT
            if (err_status) {
                disp_debug("check_error HDCP_INT err_status=%x.\n", err_status);
            }
            tc358870_i2c_write8(0x8508,0xFF); // clear and initialize for start monitor HDCP_INT.
        }
        //I_GBD
        if (hdmi_int1 & BIT(7)) {
            err_status = 0;
            tc358870_i2c_read8(0x8509, &err_status); // GBD_INT
            if (err_status) {
                disp_debug("check_error GBD_INT err_status=%x.\n", err_status);
            }
            tc358870_i2c_write8(0x8509,0xFF); // clear and initialize for start monitor GBD_INT.
        }
    }

    //Check bit8 (DSI-TX0 Interrupt Status).
    if (status & BIT(8)) {
        tc358870_i2c_read32(0x0208, &dsi_prto_int_stat); // DSI_PRTO_INT_STAT. bit3:Peripheral response time out. bit1:HSTX time out.
        if (dsi_prto_int_stat & BIT(3)) {
            disp_debug("DSI-TX0 Peripheral response time out");
        }
        if (dsi_prto_int_stat & BIT(1)) {
            disp_debug("DSI-TX0 HSTX time out");
        }
        tc358870_i2c_write32(0x0208,0x0000000A);
    }
    //Check bit11 (DSI-TX1 Interrupt Status).
    if (status & BIT(11)) {
        tc358870_i2c_read32(0x0308, &dsi_prto_int_stat); // DSI_PRTO_INT_STAT. bit3:Peripheral response time out. bit1:HSTX time out.
        if (dsi_prto_int_stat & BIT(3)) {
            disp_debug("DSI-TX1 Peripheral response time out");
        }
        if (dsi_prto_int_stat & BIT(1)) {
            disp_debug("DSI-TX1 HSTX time out");
        }
        tc358870_i2c_write32(0x0308,0x0000000A);
    }

    tc358870_i2c_write16(0x0014,0x0FBF); // IntStatus
}
#endif // INT_STATUS_CHECK_ENABLE

static int h2d_sync_int_check(void)
{
    uint16_t s_status = 0;
    uint8_t b_status = 0;
    int is_sync_chg = 0;

    if (context.init_status != INIT_STATUS_INITIALIZED) {
        disp_warning("got interrupt before is_initialized! init_status=0x%x.\n", context.init_status);
        if (INIT_STATUS_RESET == context.init_status) {
            disp_warning("got interrupt just after reset.\n");
            tc358870_i2c_write8(0x850B,0xFF); // clear and initialize for start monitor MISC_INT.
            tc358870_i2c_write16(0x0014,0x0FBF); // IntStatus
        } else {
            disp_err("got interrupt before reset done!\n"); //chipset must disable interrupt before reset done.
        }
        return 0;
    }

    is_sync_chg = 0;

    // Check interrupt factor
    tc358870_i2c_read16(0x0014, &s_status); // IntStatus.
    //disp_debug("IntStatus 0x%x.\n", s_status);

    //Check bit9 (HDMI-RX interrupt).
    if (s_status & BIT(9)) {
        //disp_debug("HDMI-RX interrupt.\n");
        tc358870_i2c_read8(0x8500, &b_status);
        //disp_debug("HDMI_INT0 0x%x.\n", b_status);

        //I_MISC
        if (b_status & BIT(1)) {
            //disp_debug("MISC (address 0x85_0B) interrupt.\n");
            b_status = 0;
            tc358870_i2c_read8(0x850B, &b_status); // MISC_INT
            if (b_status) {
                //disp_debug("misc_int_status=%x.\n", b_status);
                //I_SYNC_CHG
                if (b_status & BIT(1)) {
                    is_sync_chg = 1;
                }
            }
            tc358870_i2c_write8(0x850B,0xFF); // clear and initialize for start monitor MISC_INT.
        }
    } else {
        disp_warning("other interrrupt source enanbled?\n");
    #if INT_STATUS_CHECK_ENABLE
        int_status_check();
    #endif
    }

    tc358870_i2c_write16(0x0014,0x0FBF); // IntStatus

    return is_sync_chg;
}

#if OS_ENABLE
int h2d_task_request_wait(uint32_t ops_set)
{
	if (NULL == ops_mutex) {
		disp_err("%s: ops_mutex is not be created.\n", __func__);
		return -1;
	}

	if (osOK != osMutexWait(ops_mutex, osWaitForever)) {
		disp_err("%s: take mutex failure.\n", __func__);
		return -1;
	}

	if (h2d_task_handle) {
		ops_flag = 0;
#if 0
		xTaskNotify(h2d_task_handle, ops_set, eSetValueWithOverwrite);
#else
		h2d_msg_enqueue(ops_set);
#endif
		//osSignalSet(h2d_task_handle, ops_set);
		//wait
		for (;;) {
			osDelay(10);
			if (!((ops_flag & ops_set) ^ ops_set)) {
				ops_flag &= (~ops_set);
				break;
			}
		}
	}

	osMutexRelease(ops_mutex);

	return 0;
}

int h2d_task_request_ack(uint32_t ops_set)
{
	//ack wait
	ops_flag |= ops_set;
	return 0;
}
#endif

static int sync_event_handle(void)
{
    uint8_t b_status = 0;
    disp_debug("tc358870 interrupt event.\n");
    if (0 == h2d_sync_int_check()) {
        return 0;
    }

    b_status = 0;
    tc358870_i2c_read8(0x8520, &b_status); // SYS_STATUS
    if (b_status & BIT(7)) {
        disp_debug("synchronizaion is established.\n");
        check_input_video_timing();
        start_video_tx();
        return 1;
    } else {
        disp_debug("synchronization is lost.\n"); // fixme. Maybe need to start a timer to check. or just reinitialize.
        // Operation -> SLEEP, Cable Disconnect
        // Stop LCD display. Reset LCD if needed
        // Assert hardware reset (RESETN)
        if (DP_SOURCE_VIDEO_TIMING == g_disp_fps) {
            disp_power_off();
            disp_power_on();
        } else {
            disp_fps_set(DP_SOURCE_VIDEO_TIMING);
        }
        return 2;
    }
}

#if OS_ENABLE
/* h2d task function */
void h2d_task_entry(void const * argument)
{
    uint32_t events = 0;

    disp_debug("init h2d in task.\n");

    //osDelay(2000);
	disp_power_off_last_tick = HAL_GetTick();
    //dp2h_power_en(1);
	disp_power_on();

    /* Infinite loop */
    for(;;) {
#if 0
        if (xTaskNotifyWait(EVENTS_MASK, 0, &events, portMAX_DELAY) == pdFALSE) {
            continue;
        }
#else
		h2d_msg_dequeue(&events);
#endif

        if (!(events & EVENTS_MASK)) {
            continue;
        }

        #if !HOTPLUG_NOTIFY_ENABLE
        if (events & HDMI_SYNC_EVENT_BIT) {
            sync_event_handle();
        }
        #endif

		if (events & REQUEST_POWER_RESET_EVENT_BIT) {
			disp_debug("request disp power reset.\n");
			disp_power_off();
			disp_power_on();
			h2d_task_request_ack(REQUEST_POWER_RESET_EVENT_BIT);
		}

		if (events & REQUEST_POWER_OFF_EVENT_BIT) {
			disp_debug("request disp power off.\n");
			disp_power_off();
			h2d_task_request_ack(REQUEST_POWER_OFF_EVENT_BIT);
		}

		if (events & REQUEST_POWER_ON_EVENT_BIT) {
			disp_debug("request disp power on.\n");
			disp_power_on();
			h2d_task_request_ack(REQUEST_POWER_ON_EVENT_BIT);
		}

        #if HOTPLUG_NOTIFY_ENABLE
        if (events & DP_SRC_PLUGIN_EVENT_BIT) {
            disp_debug("dp source plugin event.\n");
        }

        if (events & DP_SRC_PLUGOUT_EVENT_BIT) {
            disp_debug("dp source plugout event.\n");
            disp_power_off();
            disp_power_on();
        }
        #endif
    }
}
#endif

void h2d_int_handle(void)
{
#if OS_ENABLE
#if !HOTPLUG_NOTIFY_ENABLE
#if 0
    BaseType_t need_resched = pdFALSE;

    if(h2d_task_handle) {
        xTaskNotifyFromISR(h2d_task_handle, HDMI_SYNC_EVENT_BIT, eSetBits, &need_resched);
        if (pdTRUE == need_resched) {
            taskYIELD();
        }
    }
#else
	h2d_msg_enqueue_FromISR(HDMI_SYNC_EVENT_BIT);
#endif
#endif
#endif
}

#if GI_FLM_ENABLE
void l_gi_flm_1_int_handle(void)
{
	static uint32_t tick_start;
	static uint32_t tolerance = 3;
	uint32_t fps = gi_change_cnt_expected;

	if (!l_gi_flm_1_int_cnt) {
		tick_start = HAL_GetTick();
	}
	l_gi_flm_1_int_cnt++;
	if (HAL_GetTick() - tick_start > 1000) { // check per 1s
		if (l_gi_flm_1_int_cnt < fps - tolerance || l_gi_flm_1_int_cnt > fps + tolerance) { // 1.power off  2.power on
			disp_info("monitor L_GI pin per second error, pin level change cnt=%lu, fps=%lu.\n", l_gi_flm_1_int_cnt, fps);
			// reset display
#if 0
			BaseType_t need_resched = pdFALSE;
			if (h2d_task_handle && gi_flm_err_rst_en) {
				xTaskNotifyFromISR(h2d_task_handle, REQUEST_POWER_RESET_EVENT_BIT, eSetBits, &need_resched);
				if (pdTRUE == need_resched) {
					taskYIELD();
				}
			}
#else
			if (gi_flm_err_rst_en)
				h2d_msg_enqueue_FromISR(REQUEST_POWER_RESET_EVENT_BIT);
#endif
			// end
		}
		l_gi_flm_1_int_cnt = 0;
	}
}

void r_gi_flm_1_int_handle(void)
{
	static uint32_t tick_start;
	static uint32_t tolerance = 3;
	uint32_t fps = gi_change_cnt_expected;

	if (!r_gi_flm_1_int_cnt) {
		tick_start = HAL_GetTick();
	}
	r_gi_flm_1_int_cnt++;
	htc_3dof_update_vsync_status(0);
	if (HAL_GetTick() - tick_start > 1000) { // check per 1s
		if (r_gi_flm_1_int_cnt < fps - tolerance || r_gi_flm_1_int_cnt > fps + tolerance) { // 1.power off  2.power on
			disp_info("monitor R_GI pin per second error, pin level change cnt=%lu, fps=%lu.\n", r_gi_flm_1_int_cnt, fps);
			// reset display
#if 0
			BaseType_t need_resched = pdFALSE;
			if (h2d_task_handle && gi_flm_err_rst_en) {
				xTaskNotifyFromISR(h2d_task_handle, REQUEST_POWER_RESET_EVENT_BIT, eSetBits, &need_resched);
				if (pdTRUE == need_resched) {
					taskYIELD();
				}
			}
#else
			if (gi_flm_err_rst_en)
				h2d_msg_enqueue_FromISR(REQUEST_POWER_RESET_EVENT_BIT);
#endif
			// end
		}
		r_gi_flm_1_int_cnt = 0;
	}
}

void panel_error_monitor(uint16_t gpio_pin)
{
	if (GPIO_XB_L_GI_FLM_1_IO == gpio_pin)
		l_gi_flm_1_int_handle();
	else if (GPIO_XB_R_GI_FLM_1_IO == gpio_pin)
		r_gi_flm_1_int_handle();
}
#endif


void disp_dp_hotplug_notification(int b_plugin)
{
    disp_debug("%s b_plugin %d", __func__, b_plugin);
#if HOTPLUG_NOTIFY_ENABLE
#if 0
    if(h2d_task_handle) {
        xTaskNotify(h2d_task_handle, b_plugin ? DP_SRC_PLUGIN_EVENT_BIT : DP_SRC_PLUGOUT_EVENT_BIT, eSetValueWithOverwrite);
    }
#else
        h2d_msg_enqueue(b_plugin ? DP_SRC_PLUGIN_EVENT_BIT : DP_SRC_PLUGOUT_EVENT_BIT);
#endif
#endif
}

int disp_d2h_getid(uint8_t *id, int size)
{
	if(XA0n == pcb_id){
		return disp_d2h_getid_anx7737(id, size);
	}
    else{
		return disp_d2h_getid_ps176(id, size);
    }
}

int disp_h2d_getid(uint16_t *id)
{
    if (context.init_status != INIT_STATUS_INITIALIZED && context.init_status != INIT_STATUS_RESET) {
        lcd_and_h2d_reset();
        context.init_status = INIT_STATUS_RESET;
    }

    return tc358870_i2c_read16(0x0000, id);
}

void disp_startx(void)
{
    int i = 0;

    if (context.init_status == INIT_STATUS_UNINITIALIZE) {
        disp_power_on();
    }

    for (i = 0; i < 3; i++) {
        wait_ms(2000);
        if (sync_event_handle() == 1) {
            break;
        }
    }
}

void disp_read_gamma(void)
{
#if CHECK_GAMMA_SETTING
    check_gamma_settings();
#else
    disp_debug("read gamma is disabled.\n");
#endif
}

int disp_test_colorbar(void)
{
#if COLOR_BAR_TEST
    if (context.init_status != INIT_STATUS_UNINITIALIZE) {
        disp_debug("please power off first.\n");
        return -1;
    }

    lcd_and_h2d_reset();
    context.init_status = INIT_STATUS_RESET;

    start_up_init_colorbar();
    context.init_status = INIT_STATUS_INITIALIZED;

    start_video_tx();

    return 0;
#else
    disp_debug("color bar test is disabled.\n");
    return 0;
#endif
}

int disp_test_dummy(void)
{
    disp_debug("dummy test is disabled.\n");
    return -1;
}

int disp_test_hdcp(void)
{
    disp_debug("hdcp test is disabled.\n");
    return -1;
}

int disp_test_dummy24(void)
{
    disp_debug("dummy24 test is disabled.\n");
    return -1;
}

int disp_power_on(void)
{
    if (context.init_status != INIT_STATUS_UNINITIALIZE) {
        disp_debug("please power off first.\n");
        return -1;
    }

#if 0
	if (!isUSBDataIn())// TODO
		return -1;
#endif

	while (HAL_GetTick() - disp_power_off_last_tick < 1);	// no delay, CES: power off -> delay 2s -> power on

    lcd_and_h2d_reset();
    context.init_status = INIT_STATUS_RESET;

    start_up_init();

#if GI_FLM_ENABLE
	l_gi_flm_1_int_cnt = 0;
	r_gi_flm_1_int_cnt = 0;
	gi_flm_err_rst_en = true;
#endif

    context.init_status = INIT_STATUS_INITIALIZED;

    disp_info("disp_power_on\n");

    return 0;
}

int disp_power_off(void)
{
    if (context.init_status != INIT_STATUS_INITIALIZED) {
        return -1;
    }

    disp_info("disp_power_off\n");
#if GI_FLM_ENABLE
	gi_flm_err_rst_en = false;
#endif

    dp2h_power_en(0);
    wait_ms(20);
#if 1
    if (H2D_STATUS_RESUEM == context.status) {
        context.lcm_drv->power_off();
    } else {
        context.lcm_drv->set_reset_pin(0);
        context.lcm_drv->power_en(0);
    }
#else
    context.lcm_drv->power_off();
#endif
    h2d_power_en(0);
    h2d_reset_pin(0);

#if GI_FLM_ENABLE
	l_gi_flm_1_int_cnt = 0;
	r_gi_flm_1_int_cnt = 0;
	gi_change_cnt_expected = g_disp_fps;
#endif
	disp_power_off_last_tick = HAL_GetTick();

    context.init_status = INIT_STATUS_UNINITIALIZE;
    context.status = H2D_STATUS_DISCONNECT;

    return 0;
}

int disp_show_lcd_id(void)
{
    if (context.init_status != INIT_STATUS_INITIALIZED) {
        return -1;
    }

    context.lcm_drv->get_id();
	return 0;
}

int disp_lcd_suspend(void)
{
    if (context.init_status != INIT_STATUS_INITIALIZED) {
        return -1;
    }

    if (H2D_STATUS_RESUEM != context.status) {
        disp_warning("please light on panel, before suspend it.\n");
        return -1;
    }

    context.lcm_drv->suspend();
    stop_video_tx();
    return 0;
}

int disp_lcd_resume(void)
{
    if (context.init_status != INIT_STATUS_INITIALIZED) {
        return -1;
    }

    if (H2D_STATUS_SLEEP != context.status) {
        disp_warning("please suspend panel, before resume it.\n");
        return -1;
    }

    context.lcm_drv->resume();
    start_video_tx();
    return 0;
}

int disp_lcd_backlight(uint8_t val)
{
    if (context.init_status != INIT_STATUS_INITIALIZED) {
        return -1;
    }

    context.lcm_drv->set_backlight(val);
    return 0;
}

//V_HDMI_EN       O       O/NP(L) O/NP(H) O/NP(L)
//H2D_INT_0      I   Active High I/PD(L) I/PD(H) I/PD(L)
//H2D_RST_N       O   External pull up 10Kohm to V_HDMI_3V3   O/NP(H) O/NP(L) O/NP(H)
static void h2d_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

    if(XA0n == pcb_id){
		GPIO_InitStruct.Pin = H2D_RST_PIN;
		HAL_GPIO_Init(H2D_RST_PIN_PORT, &GPIO_InitStruct);
	}
    else{
		GPIO_InitStruct.Pin = H2D_XB_RST_PIN;
		HAL_GPIO_Init(H2D_XB_RST_PIN_PORT, &GPIO_InitStruct);
    }

    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	if (XB02 == pcb_id) {
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Pin = H2D_XB_INT_0_PIN;
		HAL_GPIO_Init(H2D_XB_INT_0_PIN_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Pin = GPIO_XB_L_GI_FLM_1_IO;
		HAL_GPIO_Init(GPIO_XB_PORT_L_GI_FLM_1_IO, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_XB_R_GI_FLM_1_IO;
		HAL_GPIO_Init(GPIO_XB_PORT_R_GI_FLM_1_IO, &GPIO_InitStruct);
	}
    else if((XC01 == pcb_id) || (XC02 == pcb_id)){
        /* it's done in gpio.c*/
	} else {
		GPIO_InitStruct.Pin = H2D_INT_0_PIN;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(H2D_INT_0_PIN_PORT, &GPIO_InitStruct);
	}

	h2d_reset_pin(0);
    h2d_power_en(0);
    dp2h_power_en(0);
}

static void h2d_reset_pin(int val)
{
    if(XA0n == pcb_id){
		if (val) {
			HAL_GPIO_WritePin(H2D_RST_PIN_PORT, H2D_RST_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(H2D_RST_PIN_PORT, H2D_RST_PIN, GPIO_PIN_RESET);
		}
	}
    else{
		if (val) {
			HAL_GPIO_WritePin(H2D_XB_RST_PIN_PORT, H2D_XB_RST_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(H2D_XB_RST_PIN_PORT, H2D_XB_RST_PIN, GPIO_PIN_RESET);
		}
    }
}

static void h2d_power_en(int enable)
{
    /*
    {
        VDDC11_enable(1);    //turn on core power (1.1V) source first,
        wait_ms(10);    //delay 10 ms
        VDDIO33_enable(1);   //turn on analog PHY and IO (1.8V) power
    }
    */
    if (enable) {
        bsp_pmic_power_enable(V_HDMI_1V15, 1);
        wait_ms(5); // max 10ms
        bsp_pmic_power_enable(V_HDMI_3V3, 1);
        wait_ms(5);
    } else {
        bsp_pmic_power_enable(V_HDMI_3V3, 0);
        wait_ms(50); //5, max 100ms
        bsp_pmic_power_enable(V_HDMI_1V15, 0);
        wait_ms(5);
    }
}

static void dp2h_power_en(int enable)
{
	if(XA0n == pcb_id){
		dp2h_power_en_anx7737(enable);
	}
    else{
		dp2h_power_en_ps176(enable);
    }
}

#if OS_ENABLE
#if HDMI_SYNC_CHECK_ENABLE
static void hdmi_sync_check_task_entry( void *pvParameters )
{
    uint8_t sys_status = 0;

    for( ;; )
    {
        osDelay(2000);	//2000
        if (INIT_STATUS_INITIALIZED == context.init_status) {
            //Sequence: Check bit7 of 0x8520
			sys_status = 0;
            tc358870_i2c_read8(0x8520, &sys_status); // SYS_STATUS
			//disp_debug("%s tc358870 sys_status r(0x8520)=0x%02x.\n", __func__, sys_status);
            if (sys_status & BIT(7)) {
                disp_debug("%s HDMI sync received. sys_status=0x%02x.\n", __func__, sys_status);
            }
        }
    }
}
#endif
#endif

static void lcd_and_h2d_reset(void)
{
#if 1
    //lcm power on sequence
    //step1:power off status
    //step2:power on
    context.lcm_drv->power_en(1);
    //step3:wait > 10ms
    h2d_reset_pin(0);
    wait_ms(15);
    //step4:data,clock:LP 00->01->11
    h2d_power_en(1);
    //step5:wait > 25ms
    wait_ms(30);
    //step6:active reset(system reset):L(wait > 10us)-->H //fixme
    context.lcm_drv->set_reset_pin(0);
    wait_us(20);
    context.lcm_drv->set_reset_pin(1); //required by optical.
    //step7:wait > 10ms
    wait_ms(15);
    //step8:sync packet start(HS)

    //reset tc358870
    h2d_reset_pin(1);
    wait_ms(2); //delay 1ms waiting core clock ready.
#else
	//tc358870 power on sequence
    h2d_reset_pin(0);
    wait_ms(5);
    h2d_power_en(1);
    wait_ms(5); // > 200ns
    h2d_reset_pin(1);
    wait_ms(5); //delay 1ms waiting core clock ready.

	//panel power on sequence
	context.lcm_drv->power_on();
#endif

    dp2h_power_en(1);

    wait_ms(20);//fixme, wait i2c ready?
}

static void write_h2d_reg_item(const reg_item *item)
{
    if (8 == item->reg_type) {
        tc358870_i2c_write8(item->addr, item->value);
    } else if (16 == item->reg_type) {
        tc358870_i2c_write16(item->addr, item->value);
    } else if (32 == item->reg_type) {
        tc358870_i2c_write32(item->addr, item->value);
    } else if (0xFF == item->reg_type) {
        wait_ms((item->addr + 999)/1000);
    } else {
        disp_err("unknow reg item write type.\n");
    }
}

#if COLOR_BAR_TEST
static const reg_item colorbar_phase1_tbl_fps57[] = {
    #include "ColorBarPhase1_57Hz.tbl"
};
static const reg_item colorbar_phase1_tbl_fps58[] = {
    #include "ColorBarPhase1_58Hz.tbl"
};
static const reg_item colorbar_phase1_tbl_fps90[] = {
    #include "ColorBarPhase1_90Hz.tbl"
};
#if (DP_SOURCE_VIDEO_TIMING == 57)
static const reg_item *colorbar_phase1_tbl = colorbar_phase1_tbl_fps57;
uint32_t colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps57);
#elif (DP_SOURCE_VIDEO_TIMING != 90)
static const reg_item *colorbar_phase1_tbl = colorbar_phase1_tbl_fps58;
uint32_t colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps58);
#else
static const reg_item *colorbar_phase1_tbl = colorbar_phase1_tbl_fps90;
uint32_t colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps90);
#endif

static const reg_item colorbar_phase2_tbl_fps57[] = {
    #include "ColorBarPhase2_57Hz.tbl"
};
static const reg_item colorbar_phase2_tbl_fps58[] = {
    #include "ColorBarPhase2_58Hz.tbl"
};
static const reg_item colorbar_phase2_tbl_fps90[] = {
    #include "ColorBarPhase2_90Hz.tbl"
};
#if (DP_SOURCE_VIDEO_TIMING == 57)
static const reg_item *colorbar_phase2_tbl = colorbar_phase2_tbl_fps57;
uint32_t colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps57);
#elif (DP_SOURCE_VIDEO_TIMING != 90)
static const reg_item *colorbar_phase2_tbl = colorbar_phase2_tbl_fps58;
uint32_t colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps58);
#else
static const reg_item *colorbar_phase2_tbl = colorbar_phase2_tbl_fps90;
uint32_t colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps90);
#endif

static void start_up_init_colorbar(void)
{
    int i = 0;

#if ENABLE_GET_REGS_FROM_PARTION
    if (context.is_regs_from_partion) {
        for (i= 0; i < h2d_tbl.colorbar_p1_tbl_size; i++) {
            write_h2d_reg_item(&h2d_tbl.colorbar_p1_tbl[i]);
        }

        context.lcm_drv->initialize();

        for (i= 0; i < h2d_tbl.colorbar_p2_tbl_size; i++) {
            write_h2d_reg_item(&h2d_tbl.colorbar_p2_tbl[i]);
        }
        return;
    }
#endif

    for (i= 0; i < colorbar_phase1_tbl_size; i++) {
        write_h2d_reg_item(&colorbar_phase1_tbl[i]);
    }

    //step10: common setting and brightness control.
    context.lcm_drv->initialize();

    for (i= 0; i < colorbar_phase2_tbl_size; i++) {
        write_h2d_reg_item(&colorbar_phase2_tbl[i]);
    }
}
#endif

/* startup_phase1_tbl */
static const reg_item startup_phase1_tbl_fps57[] = {
    #include "StartupPhase1_57Hz.tbl"
};
static const reg_item startup_phase1_tbl_fps58[] = {
    #include "StartupPhase1_58Hz.tbl"
};
static const reg_item startup_phase1_tbl_fps90[] = {
    #include "StartupPhase1_90Hz.tbl"
};
#if (DP_SOURCE_VIDEO_TIMING == 57)
static const reg_item *startup_phase1_tbl = startup_phase1_tbl_fps57;
uint32_t startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps57);
#elif (DP_SOURCE_VIDEO_TIMING != 90)
static const reg_item *startup_phase1_tbl = startup_phase1_tbl_fps58;
uint32_t startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps58);
#else
static const reg_item *startup_phase1_tbl = startup_phase1_tbl_fps90;
uint32_t startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps90);
#endif

/* startup_phase2_tbl */
static const reg_item startup_phase2_tbl_fps57[] = {
    #include "StartupPhase2_57Hz.tbl"
};
static const reg_item startup_phase2_tbl_fps58[] = {
    #include "StartupPhase2_58Hz.tbl"
    {0x08,0x8a08,0x1},//add more regs settings for M10.
};
static const reg_item startup_phase2_tbl_fps90[] = {
    #include "StartupPhase2_90Hz.tbl"
};
#if (DP_SOURCE_VIDEO_TIMING == 57)
static const reg_item *startup_phase2_tbl = startup_phase2_tbl_fps57;
uint32_t startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps57);
#elif (DP_SOURCE_VIDEO_TIMING != 90)
static const reg_item *startup_phase2_tbl = startup_phase2_tbl_fps58;
uint32_t startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps58);
#else
static const reg_item *startup_phase2_tbl = startup_phase2_tbl_fps90;
uint32_t startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps90);
#endif

int disp_fps_set(uint8_t fps)
{
	if (fps == g_disp_fps)
		return 1;

	if (57 == fps) {
		colorbar_phase1_tbl = colorbar_phase1_tbl_fps57;
		colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps57);
		colorbar_phase2_tbl = colorbar_phase2_tbl_fps57;
		colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps57);
		startup_phase1_tbl = startup_phase1_tbl_fps57;
		startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps57);
		startup_phase2_tbl = startup_phase2_tbl_fps57;
		startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps57);
	} else if (58 == fps) {
		colorbar_phase1_tbl = colorbar_phase1_tbl_fps58;
		colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps58);
		colorbar_phase2_tbl = colorbar_phase2_tbl_fps58;
		colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps58);
		startup_phase1_tbl = startup_phase1_tbl_fps58;
		startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps58);
		startup_phase2_tbl = startup_phase2_tbl_fps58;
		startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps58);
	} else if (90 == fps) {
		colorbar_phase1_tbl = colorbar_phase1_tbl_fps90;
		colorbar_phase1_tbl_size = ARRAY_SIZE(colorbar_phase1_tbl_fps90);
		colorbar_phase2_tbl = colorbar_phase2_tbl_fps90;
		colorbar_phase2_tbl_size = ARRAY_SIZE(colorbar_phase2_tbl_fps90);
		startup_phase1_tbl = startup_phase1_tbl_fps90;
		startup_phase1_tbl_size = ARRAY_SIZE(startup_phase1_tbl_fps90);
		startup_phase2_tbl = startup_phase2_tbl_fps90;
		startup_phase2_tbl_size = ARRAY_SIZE(startup_phase2_tbl_fps90);
	} else {
		return -1;
	}

	g_disp_fps = fps;

#if OS_ENABLE
	//h2d_task_request_wait(REQUEST_POWER_RESET_EVENT_BIT);
	h2d_msg_enqueue(REQUEST_POWER_RESET_EVENT_BIT);
#else
	disp_power_off();
	disp_power_on();
#endif

	return 0;
}

float disp_fps_get(void)
{
	return g_disp_fps;
}


static void start_up_init(void)
{
    int i = 0;
#if ENABLE_GET_REGS_FROM_PARTION
    if (context.is_regs_from_partion) {
        for (i= 0; i < h2d_tbl.startup_p1_tbl_size; i++) {
            write_h2d_reg_item(&h2d_tbl.startup_p1_tbl[i]);
        }

        context.lcm_drv->initialize();

        for (i= 0; i < h2d_tbl.startup_p2_tbl_size; i++) {
            write_h2d_reg_item(&h2d_tbl.startup_p2_tbl[i]);
        }

        return;
    }
#endif

    for (i= 0; i < startup_phase1_tbl_size; i++) {
        write_h2d_reg_item(&startup_phase1_tbl[i]);
    }

    //step10: common setting and brightness control.
    context.lcm_drv->initialize();

    for (i= 0; i < startup_phase2_tbl_size; i++) {
        write_h2d_reg_item(&startup_phase2_tbl[i]);
    }
}


#if OS_ENABLE
int h2d_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
	switch(_notify_flag){
		case PWRMGR_NOTIFY_POWER_OFF:
		case PWRMGR_NOTIFY_SYSTEM_RESET:
			//HAL_NVIC_DisableIRQ(H2D_EXTI_IRQn);
			//disp_power_off();
			h2d_task_request_wait(REQUEST_POWER_OFF_EVENT_BIT);
			//HAL_NVIC_EnableIRQ(H2D_EXTI_IRQn);
			PWRMGR_SendNotifyAck(&h2dPmNotifyData);
			break;
		case PWRMGR_NOTIFY_STOP_STATE:
			if (_state == STOP_ENTER) {
				h2d_task_request_wait(REQUEST_POWER_OFF_EVENT_BIT);
			} else if (_state == STOP_LEAVE) {
				h2d_task_request_wait(REQUEST_POWER_ON_EVENT_BIT);
			} else
				disp_warning("%s: state ERROR.\n", __func__);
			break;

		default:
			disp_warning("h2d don't care this flag\n");
			break;
	}
	return 0;
}
#endif

int is_h2d_init(void)
{
    return context.init_status == INIT_STATUS_INITIALIZED;
}

#if ENABLE_GET_REGS_FROM_PARTION
static int get_reg_tbl()
{
    uint32_t offset = 0;

    offset = PARTION_ADDR;
    h2d_tbl.magic_num = *((uint32_t *)offset);
    if (H2D_MAGIC != h2d_tbl.magic_num) {
        return 0;
    }

    offset += 4;
    h2d_tbl.startup_p1_tbl_size = *((uint32_t *)offset);
    offset += 4;
    h2d_tbl.startup_p1_tbl = (reg_item *)offset;
    offset += (h2d_tbl.startup_p1_tbl_size*sizeof(reg_item));
    h2d_tbl.startup_p2_tbl_size = *((uint32_t *)offset);
    offset += 4;
    h2d_tbl.startup_p2_tbl = (reg_item *)offset;
    offset += (h2d_tbl.startup_p2_tbl_size*sizeof(reg_item));
    h2d_tbl.colorbar_p1_tbl_size = *((uint32_t *)offset);
    offset += 4;
    h2d_tbl.colorbar_p1_tbl = (reg_item *)offset;
    offset += (h2d_tbl.colorbar_p1_tbl_size*sizeof(reg_item));
    h2d_tbl.colorbar_p2_tbl_size = *((uint32_t *)offset);
    offset += 4;
    h2d_tbl.colorbar_p2_tbl = (reg_item *)offset;


    disp_info("get regs table size from fota %d %d %d %d.\n", h2d_tbl.startup_p1_tbl_size,
                                                    h2d_tbl.startup_p2_tbl_size,
                                                    h2d_tbl.colorbar_p1_tbl_size,
                                                    h2d_tbl.colorbar_p2_tbl_size);

    return 1;
}
#endif

int hdmi2dsi_driver_init(void)
{
    if (context.init_status != INIT_STATUS_UNINITIALIZE) {
        disp_warning("tc358870xbg chip already be initialized.");
        return 0;
    }

    memset(&context, 0x0, sizeof(context));
    context.lcm_drv = LCM_GetDriver();
    context.lcm_drv->set_dsi_funcs(&dsi_fns);

#if OS_ENABLE
    /* definition and creation of ble_task */
    osThreadDef(h2d_task, h2d_task_entry, osPriorityNormal, 0, 256);
    h2d_task_handle = osThreadCreate(osThread(h2d_task), NULL);
	ops_mutex = osMutexCreate(NULL);
	osMessageQDef(h2d_msg_queue, 4, uint32_t);
	h2dMsgQueueHandle = osMessageCreate(osMessageQ(h2d_msg_queue), NULL);

#if HDMI_SYNC_CHECK_ENABLE
    xTaskCreate( hdmi_sync_check_task_entry, "hdmi_sync_check_task", 200, NULL, tskIDLE_PRIORITY, &hdmi_sync_check_task );
#endif
#endif

    //config gpios for voltage and clock sources
	if ((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id))
		dp2h_gpio_config_ps176();
    h2d_gpio_config();
    context.lcm_drv->configure();

    disp_debug("h2d config done.\n");

#if ENABLE_GET_REGS_FROM_PARTION
    context.is_regs_from_partion = get_reg_tbl();
#endif

#if OS_ENABLE
    h2dPmNotifyData.func_name = "h2d";
    h2dPmNotifyData.data = NULL;
    h2dPmNotifyData.callback= h2d_notify_callback;
    h2dPmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE | PWRMGR_NOTIFY_POWER_OFF | PWRMGR_NOTIFY_SYSTEM_RESET;
    h2dPmNotifyData.func_level = PWRMGR_FUNC_DRIVER_LEVEL;
    PWRMGR_register_notify_func(&h2dPmNotifyData);
#endif
    return 0;
}

