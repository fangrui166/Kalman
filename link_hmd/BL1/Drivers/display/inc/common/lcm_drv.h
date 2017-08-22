#ifndef __LCM_DRV_H__
#define __LCM_DRV_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
//#include "hlog_api.h"

#define HTC_LOG_LCM							0
#if HTC_LOG_LCM
	#define HTC_LOG_TAG_LCM					(HLOG_TAG_LCM)
	#define lcm_log(level, args...)			do { hlog_printf(level, LOG_LCM, ##args); } while(0)
#else
	#define lcm_log(level, args...)			do { printf(args); } while(0)
#endif

#ifndef ARY_SIZE
#define ARY_SIZE(x)							(sizeof((x)) / sizeof((x[0])))
#endif

#define REGFLAG_ESCAPE_ID					(0x00)
#define REGFLAG_DELAY_MS					(0xFF)

typedef enum {
	LCM_LEFT,
	LCM_RIGHT,
} lcm_index;

typedef struct {
    unsigned char id;
    unsigned char cmd;
    unsigned char count;
    unsigned char para[0];
} dsi_cmd_t;

typedef struct {
    const unsigned char * raw;
    const unsigned char size;
} dsi_cmd_raw;

typedef struct {
    void (*dsi_set_cmdq)(const dsi_cmd_raw *para_list, const unsigned int size);
    unsigned int (*dsi_dcs_read_reg_v2)(unsigned char cmd, unsigned char *buffer, unsigned char buffer_size);
#if 0
	/* to implement these funtions in the future */
    void (*dsi_dcs_write_regs)(unsigned int cmd);
    void (*dsi_dcs_write_regs_v2)(unsigned char addr, unsigned char *para, unsigned int nums);
    unsigned int (*dsi_dcs_read_reg)(unsigned char cmd);
#endif
} dsi_funcs;

typedef enum {
	LCM_DSI_FORMAT_RGB565 = 0,
	LCM_DSI_FORMAT_RGB666 = 1,
	LCM_DSI_FORMAT_RGB888 = 2
} dsi_data_format;

typedef enum {
	LCM_DSI_ONE_LANE = 1,
	LCM_DSI_TWO_LANE = 2,
	LCM_DSI_THREE_LANE = 3,
	LCM_DSI_FOUR_LANE = 4,
} dsi_lan_num;

typedef struct {
	unsigned int width;
	unsigned int height;
	unsigned int vertical_sync_active;
	unsigned int vertical_backporch;
	unsigned int vertical_frontporch;
	unsigned int vertical_active_line;
	unsigned int horizontal_sync_active;
	unsigned int horizontal_backporch;
	unsigned int horizontal_frontporch;
	unsigned int horizontal_active_pixel;
	dsi_data_format format;
	dsi_lan_num lane_num;
	unsigned int fps;
} lcm_params;

typedef struct {
    const char* name;
    void (*set_dsi_funcs)(dsi_funcs *funcs);
    void (*get_params)(lcm_params *params);		/* if need */
    void (*configure)(void);
    void (*power_en)(int enable);
    void (*set_reset_pin)(int set);
    void (*init)(void);
    void (*suspend)(void);
    void (*resume)(void);
    void (*set_backlight)(unsigned char level);
    void (*initialize)(void);
    void (*display_on)(void);
    void (*power_on)(void);
    void (*power_off)(void);
    void (*get_id)(void);		/* if need */
    void (*init_gamma)(int x4idx, uint32_t value);
    void (*update_gamma)(void);

	/* for DSI_COMMAND mode */
    void (*update)(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
#if 0
    /* for power-on sequence refinement */
    void (*init_power)(void);
    void (*suspend_power)(void);
    void (*resume_power)(void);


    /* ESD_RECOVERY */
	unsigned int (*esd_check)(void);
    unsigned int  (*esd_recover)(void);
#endif
} lcm_drv_t;

const lcm_drv_t* LCM_GetDriver(void);


#ifdef __cplusplus
}
#endif

#endif /* __LCM_DRV_H__ */
