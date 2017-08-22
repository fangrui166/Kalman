#ifndef __X_PMIC_H__
#define __X_PMIC_H__

#ifdef __cplusplus
extern "C" {
#endif



/* Includes
------------------------------------------------------------------*/

#include <stdint.h>
#include "PowerManager.h"
#include "stm32f4xx_hal.h"

enum charger_t{
    CHARGER_TYPE_OFF,
    CHARGER_TYPE_UNKOW,
    CHARGER_TYPE_HOST,
    CHARGER_TYPE_ADAPTER,
};

enum {
    POWER_DISABLE,
    POWER_ENABLE
};

typedef enum{
    V_BOOST_5V_EN = 0,   /* VBAT to 5V enable for CCG4, USB HUB2.0 ...*/
    V_USB30_5V_EN,       /* VBAT to 5V for USB3.0 type A power enable*/
    V_DP_EN,             /* DP to HDMI power enable ,vdd and vddio both open*/
    //V_HDMI_EN,           /* HDMI to DSI power enable, 1V15, 3V3 both open*/
    V_VDDI_L_EN,         /* left  Display vdd 1V8 enable*/
    V_VDDI_R_EN,         /* right Display vdd 1V8 enable*/
    V_VCI_3V3_L,         /* left  Display vci 3V3 enable*/
    V_VCI_3V3_R,         /* right Display vci 3V3 enable*/
    V_AUD_3V3,           /* Audio codec alc5665 3v3*/
    V_VDD_1V8,           /* Audio codec alc5665 And display 1V8*/
    V_HDMI_1V15,         /* HDMI to DSI power enable, 1V15 open*/
    V_HDMI_3V3,          /* HDMI to DSI power enable, 3V3 open*/
    V_PX_1V1_EN,
    V_PX_1V8_EN,
    V_POWER_MAX
}power_item_t;

typedef struct{
    uint8_t *who_am_i;
    uint8_t isInitialized;
    uint8_t isEnabled;
    int power_item_count[V_POWER_MAX];

    void * pVTable;
    void * pData;

}PmicDrvTypeDef;

typedef struct{
    uint32_t charge_current_limit;
    uint32_t termination_current;
    uint32_t weak_battery_voltage;
    uint32_t hot_temperature;
    enum charger_t charger_type;
    PmicDrvTypeDef *handel;
}PmicData_t;

typedef struct{
    void *charger;
    void *gauge;
    void *ldo;
}PmicFunTable_t;


#define PM_DRV_HLOG_ENABLE 1

#if PM_DRV_HLOG_ENABLE
#define pm_emerg(fmt, ...) \
    hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pm_err(fmt, ...) \
    hlog_printf(HLOG_LVL_ERR, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pm_warning(fmt, ...) \
    hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pm_info(fmt, ...) \
    hlog_printf(HLOG_LVL_INFO, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#define pm_debug(fmt, ...) \
    hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_PWRMGR, fmt, ##__VA_ARGS__)
#else /* PM_DRV_HLOG_ENABLE */
#define pm_emerg(fmt, ...) \
    printf("[PM_DRV][EMR] :" fmt, ##__VA_ARGS__)
#define pm_err(fmt, ...) \
    printf("[PM_DRV][ERR] :" fmt, ##__VA_ARGS__)
#define pm_warning(fmt, ...) \
    printf("[PM_DRV][WARN]:" fmt, ##__VA_ARGS__)
#define pm_info(fmt, ...) \
    printf("[PM_DRV][INFO]:" fmt, ##__VA_ARGS__)
#define pm_debug(fmt, ...) \
    printf("[PM_DRV][DBG] :" fmt, ##__VA_ARGS__)
#endif /* PWRMGR_HLOG_ENABLE */

int bsp_pmic_get_battery_info(PmicDrvTypeDef * handle,
                                    struct pwrmgr_battery_info * info);
int bsp_pmic_init(void);
PmicDrvTypeDef* bsp_pmic_get_handle(void);
int bsp_pmic_power_enable(power_item_t item, uint8_t enable);
int bsp_pmic_power_state_dump(void);
//int bsp_pmic_get_charger_para(PmicDrvTypeDef * handle, chargerPara *chg_para);
int bsp_pmic_set_charge_currentLimit(PmicDrvTypeDef * handle, uint32_t current);
int bsp_pmic_set_input_currentLimit(PmicDrvTypeDef * handle, uint32_t current);
int bsp_pmic_set_charger_enableDisable(PmicDrvTypeDef * handle, uint8_t enable);
int bsp_pmic_set_charge_voltageLimit(PmicDrvTypeDef * handle, uint32_t voltage);
int bsp_pmic_set_charge_hizMode(PmicDrvTypeDef * handle, uint8_t state);
int bsp_pmic_get_pg_status(PmicDrvTypeDef * handle, uint8_t *state);
int bsp_pmic_power_off(void);
int bsp_pmic_power_reset(void);
#ifdef __cplusplus
}
#endif

#endif

