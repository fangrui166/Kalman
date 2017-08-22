#ifndef __DISPLAY_DRV_H__
#define __DISPLAY_DRV_H__
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f4xx_hal.h"
#include "hlog_api.h"
#include "gpio.h"

#define DISPLAY_DRV_HLOG_ENABLE
#ifdef DISPLAY_DRV_HLOG_ENABLE
#define disp_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_DISP, fmt, ##__VA_ARGS__)
#define disp_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_DISP, fmt, ##__VA_ARGS__)
#define disp_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_DISP, fmt, ##__VA_ARGS__)
#define disp_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_DISP, fmt, ##__VA_ARGS__)
#define disp_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_DISP, fmt, ##__VA_ARGS__)
#else /* DISPLAY_DRV_HLOG_ENABLE */
#define disp_emerg(fmt, ...) \
	printf("[DISP][EMR] :" fmt, ##__VA_ARGS__)
#define disp_err(fmt, ...) \
	printf("[DISP][ERR] :" fmt, ##__VA_ARGS__)
#define disp_warning(fmt, ...) \
	printf("[DISP][WARN]:" fmt, ##__VA_ARGS__)
#define disp_info(fmt, ...) \
	printf("[DISP][INFO]:" fmt, ##__VA_ARGS__)
#define disp_debug(fmt, ...) \
	printf("[DISP][DBG] :" fmt, ##__VA_ARGS__)
#endif /* DISPLAY_DRV_HLOG_ENABLE */


#define H2D_INT_0_PIN                       (GPIO_H2D_INT_0)
#define H2D_INT_0_PIN_PORT                  (GPIO_PORT_H2D_INT_0)
#define H2D_XB_INT_0_PIN                    (GPIO_XB_H2D_INT_0)
#define H2D_XB_INT_0_PIN_PORT               (GPIO_XB_PORT_H2D_INT_0)

/* display define start */
enum {
	DISPLAY_FUNC_SET00 = 0,
	DISPLAY_FUNC_SET01,
};

typedef enum {
	DISPLAY_FUNC_SET00_PHONE_10 = 0,
	DISPLAY_FUNC_SET00_PHONE_U_ULTRA,
	DISPLAY_FUNC_SET00_PHONE_OCEAN,
} phone_type_e;

enum {
	DISPLAY_FUNC_GET00 = 0,
	DISPLAY_FUNC_GET01,
};
/* end display define */

void h2d_int_handle(void);
int display_drv_init(void);
int disp_h2d_getid(uint16_t *id);
int disp_power_on(void);
int disp_power_off(void);
int disp_show_lcd_id(void);
int disp_phone_type_set(int type);
float disp_fps_get(void);
int disp_lcd_suspend(void);
int disp_lcd_resume(void);
int disp_lcd_backlight(uint8_t val);
int disp_test_colorbar(void);
int disp_test_dummy(void);
int disp_test_hdcp(void);
int disp_test_dummy24(void);
void disp_dp_hotplug_notification(int b_plugin);
int disp_d2h_getid(uint8_t *id, int size);
void disp_read_comp(void);
void disp_write_comp(char * comps);
void disp_read_gamma(void);
void disp_read_aid(void);
void disp_write_aid(char * aids);
void disp_startx(void);

#ifdef __cplusplus
}
#endif

#endif //__DISPLAY_DRV_H__



