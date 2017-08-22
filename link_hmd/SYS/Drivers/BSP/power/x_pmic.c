#include <string.h>
#include <stdio.h>
#include "ncp6924.h"
#include "max17050_fuel_gauge.h"
#include "bq2589x_charger.h"
#include "gpio.h"
#include "periph_multiple_drv.h"
#include "gpio_exp.h"
#include "x_pmic.h"
#include "command.h"

extern pcbid_t pcb_id;

static PmicDrvTypeDef PMIC_Handle = {0};
static PmicData_t  PMIC_Data = {0};
static PmicFunTable_t PMIC_Fun = {0};
static char *power_item_str[]={
    "V_BOOST_5V",
    "V_USB30_5V",
    "V_DP",
    //"V_HDMI",
    "V_VDDI_L",
    "V_VDDI_R",
    "V_VCI_3V3_L",
    "V_VCI_3V3_R",
    "V_AUD_3V3",
    "V_VDD_1V8",
    "V_HDMI_1V15",
    "V_HDMI_3V3",
    "V_PX_1V1_EN",
    "V_PX_1V8_EN",
};
int bsp_mcu_gpio_enable(GPIO_TypeDef *port, uint16_t pin, uint8_t enable)
{
    int state = 0;
    if(enable)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    state = HAL_GPIO_ReadPin(port, pin);
    return !(state == (enable ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

uint8_t bsp_pmic_power_get_state(power_item_t item)
{
    uint8_t ret = 0;
    PmicDrvTypeDef *handle = &PMIC_Handle;
    PmicFunTable_t *driver = (PmicFunTable_t*)(handle->pVTable);
    LDO_DRV_t * ldo_driver = (LDO_DRV_t *)(driver->ldo);
    if(!handle->isInitialized){
        pm_err("%s pmic is not initialized!\n",__func__);
        return -1;
    }
    switch(item){
        case V_BOOST_5V_EN:
            if(XA0n == pcb_id){
                ret = HAL_GPIO_ReadPin(GPIO_PORT_V_BOOST_5V_EN,
                        GPIO_V_BOOST_5V_EN);
            }
            else{
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT,
                        IOEXP_I_V_BOOST_5V_EN);
            }
            break;
        case V_USB30_5V_EN:
            ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_V_USB30_5V_EN);
            break;
        case V_DP_EN:
            if(XA0n == pcb_id){
                ret = HAL_GPIO_ReadPin(GPIO_PORT_V_DP_EN, GPIO_V_DP_EN);
            }
            else{
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_II_V_DP_PS_EN);
            }
            break;
			/*
        case V_HDMI_EN:
            ret = HAL_GPIO_ReadPin(GPIO_PORT_V_HDMI_EN, GPIO_V_HDMI_EN);
            break;
            */
        case V_VDDI_L_EN:
            if(XA0n == pcb_id){
                ret = HAL_GPIO_ReadPin(GPIO_PORT_V_VDDI_L_EN, GPIO_V_VDDI_L_EN);
            }
            else{
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_I_V_VDDI_L_EN);
            }
            break;
        case V_VDDI_R_EN:
            if(XA0n == pcb_id){
                ret = HAL_GPIO_ReadPin(GPIO_PORT_V_VDDI_R_EN, GPIO_V_VDDI_R_EN);
            }
            else{
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_I_V_VDDI_R_EN);
            }
            break;
        case V_VCI_3V3_L:
            if(ldo_driver->LDO_GetEnableState){
                ldo_driver->LDO_GetEnableState(handle, NCP6924C_LDO3, &ret);
            }
            break;
        case V_VCI_3V3_R:
          if(ldo_driver->LDO_GetEnableState){
                ldo_driver->LDO_GetEnableState(handle, NCP6924C_LDO4, &ret);
          }
            break;
        case V_AUD_3V3:
          if(ldo_driver->LDO_GetEnableState){
                ldo_driver->LDO_GetEnableState(handle, NCP6924C_LDO2, &ret);
          }
            break;
        case V_VDD_1V8:
          if(ldo_driver->LDO_GetEnableState){
                ldo_driver->LDO_GetEnableState(handle, NCP6924C_BUCK2, &ret);
          }
            break;
		case V_HDMI_1V15:
		  if(ldo_driver->LDO_GetEnableState){
				ldo_driver->LDO_GetEnableState(handle, NCP6924C_BUCK1, &ret);
		  }
			break;
		case V_HDMI_3V3:
		  if(ldo_driver->LDO_GetEnableState){
				ldo_driver->LDO_GetEnableState(handle, NCP6924C_LDO1, &ret);
		  }
			break;
        case V_PX_1V1_EN:
            if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_II_V_PX_1V1_EN);
            }
            break;
        case V_PX_1V8_EN:
            if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
                ret = ioexp_gpio_get_value(IOEXP_REG_OUT, IOEXP_II_V_PX_1V8_EN);
            }
            break;
        default:
            break;
    }
    return ret;
}
int bsp_pmic_power_enable(power_item_t item, uint8_t enable)
{
    int ret = -1;
    PmicDrvTypeDef *handle = &PMIC_Handle;
    PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
    LDO_DRV_t * ldo_driver = (LDO_DRV_t *)(driver->ldo);
    if(!handle->isInitialized){
        pm_err("%s pmic is not initialized!\n",__func__);
        return -1;
    }
    if(handle->power_item_count[item] && enable){
        handle->power_item_count[item] ++;
        return 0;
    }
    else if((handle->power_item_count[item] >= 2) && !enable){
        handle->power_item_count[item] --;
        return 0;
    }
	else if((handle->power_item_count[item] == 0) && !enable){
		return 0;
	}
    switch(item){
        case V_BOOST_5V_EN:
            if(XA0n == pcb_id){
                ret = bsp_mcu_gpio_enable(GPIO_PORT_V_BOOST_5V_EN,
                        GPIO_V_BOOST_5V_EN, enable);
            }
            else{
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR,
                        IOEXP_I_V_BOOST_5V_EN, IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                        IOEXP_I_V_BOOST_5V_EN, enable);
            }
            break;
        case V_USB30_5V_EN:
            ret = ioexp_gpio_set_value(IOEXP_REG_DIR,
                    IOEXP_V_USB30_5V_EN, IOEXP_GPIO_OUTPUT);
            ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                    IOEXP_V_USB30_5V_EN, enable);
            break;
        case V_DP_EN:
            if(XA0n == pcb_id){
                ret = bsp_mcu_gpio_enable(GPIO_PORT_V_DP_EN, GPIO_V_DP_EN, enable);
            }
            else{
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR,
                    IOEXP_II_V_DP_PS_EN, IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                            IOEXP_II_V_DP_PS_EN, enable);
            }
            break;
			/*
        case V_HDMI_EN:
            ret = bsp_mcu_gpio_enable(GPIO_PORT_V_HDMI_EN, GPIO_V_HDMI_EN, enable);
            break;
            */
        case V_VDDI_L_EN:
            if(ldo_driver->LDO_Enable && enable &&
                (++handle->power_item_count[V_VDD_1V8] == 1)){
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK2, enable);
            }
            if(XA0n == pcb_id){
                ret = bsp_mcu_gpio_enable(GPIO_PORT_V_VDDI_L_EN,
                        GPIO_V_VDDI_L_EN, enable);
            }
            else{
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_I_V_VDDI_L_EN,
                            IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                            IOEXP_I_V_VDDI_L_EN, enable);
            }
            if(ldo_driver->LDO_Enable && !enable &&
                (--handle->power_item_count[V_VDD_1V8] <= 0)){
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK2, enable);
                handle->power_item_count[V_VDD_1V8] = 0;
            }
            break;
        case V_VDDI_R_EN:
            if(ldo_driver->LDO_Enable && enable &&
                (++handle->power_item_count[V_VDD_1V8] == 1)){
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK2, enable);
            }
            if(XA0n == pcb_id){
                ret = bsp_mcu_gpio_enable(GPIO_PORT_V_VDDI_R_EN,
                            GPIO_V_VDDI_R_EN, enable);
            }
            else{
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_I_V_VDDI_R_EN,
                            IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                            IOEXP_I_V_VDDI_R_EN, enable);
            }
            if(ldo_driver->LDO_Enable && !enable &&
                (--handle->power_item_count[V_VDD_1V8] <= 0)){
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK2, enable);
                handle->power_item_count[V_VDD_1V8] = 0;
            }
            break;
        case V_VCI_3V3_L:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_LDO3, enable);
            break;
        case V_VCI_3V3_R:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_LDO4, enable);
            break;
        case V_AUD_3V3:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_LDO2, enable);
            break;
        case V_VDD_1V8:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK2, enable);
            break;
		case V_HDMI_1V15:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_BUCK1, enable);
			break;
		case V_HDMI_3V3:
            if(ldo_driver->LDO_Enable)
                ret = ldo_driver->LDO_Enable(handle, NCP6924C_LDO1, enable);
			break;

        case V_PX_1V1_EN:
            if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR,
                        IOEXP_II_V_PX_1V1_EN, IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                        IOEXP_II_V_PX_1V1_EN, enable);
            }
            break;
        case V_PX_1V8_EN:
            if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
                ret = ioexp_gpio_set_value(IOEXP_REG_DIR,
                        IOEXP_II_V_PX_1V8_EN, IOEXP_GPIO_OUTPUT);
                ret |= ioexp_gpio_set_value(IOEXP_REG_OUT,
                        IOEXP_II_V_PX_1V8_EN, enable);
            }
            break;
    }
    if(ret ==0){
        if(enable){
            handle->power_item_count[item] ++;
        }
        else{
            handle->power_item_count[item] --;
            if(handle->power_item_count[item] < 0){
                handle->power_item_count[item] = 0;
            }
        }
    }
    else{
        pm_err("%s(%d) failed:%d\n", __func__, item, ret);
    }
    return ret;
}

int bsp_pmic_power_state_dump(void)
{
    int i=0;
    uint8_t state[V_POWER_MAX] = {0};

    for(i=0;i<V_POWER_MAX;i++){
        state[i] = bsp_pmic_power_get_state((power_item_t)i);
    }
    shell_info("Item        State   User\n");
    for(i=0;i<V_POWER_MAX;i++){
        shell_info("%-13s%-8d%-4d\n",power_item_str[i],
            state[i], PMIC_Handle.power_item_count[i]);
    }
    shell_info(" \r\n");
    return 0;
}

/* Power Up Sequencing:
   1. V_VDD_1V25 -> V_VDD_3V3 ->                   //Source from DC Input
   2. V_BOOST_5V_EN ->                             //MCU GPIO
   3. NCP6924_EN -> V_HDMI_1V15 -> V_HDMI_3V3 ->   //PMIC
   4. V_DP_EN                                      //MCU GPIO
*/
int bsp_pmic_power_up_sequence(PmicDrvTypeDef *handle)
{
    bsp_pmic_power_enable(V_BOOST_5V_EN, POWER_ENABLE);
    //bsp_pmic_power_enable(V_HDMI_EN, POWER_ENABLE);
    //bsp_pmic_power_enable(V_DP_EN, POWER_ENABLE);

    /* for bringup open all power source */
    bsp_pmic_power_enable(V_USB30_5V_EN, POWER_ENABLE);
    //bsp_pmic_power_enable(V_VDDI_L_EN, POWER_ENABLE);
    //bsp_pmic_power_enable(V_VDDI_R_EN, POWER_ENABLE);
    //bsp_pmic_power_enable(V_VCI_3V3_L, POWER_ENABLE);
    //bsp_pmic_power_enable(V_VCI_3V3_R, POWER_ENABLE);
    return 0;
}
int bsp_pmic_power_off(void)
{
    int ret = 0;
    PmicDrvTypeDef *handle = &PMIC_Handle;
    PmicFunTable_t *driver = (PmicFunTable_t*)(handle->pVTable);
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);
    if(!handle->isInitialized){
        pm_err("%s pmic is not initialized!\n",__func__);
        return -1;
    }
    if(ch_driver->PowerOff){
        ret = ch_driver->PowerOff(handle);
    }
    return ret;
}

int bsp_pmic_power_reset(void)
{
    int ret = 0;
    PmicDrvTypeDef *handle = &PMIC_Handle;
    PmicFunTable_t *driver = (PmicFunTable_t*)(handle->pVTable);
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);
    if(!handle->isInitialized){
        pm_err("%s pmic is not initialized!\n",__func__);
        return -1;
    }
    if(ch_driver->PowerReset){
        ret = ch_driver->PowerReset(handle);
    }
    return ret;
}
int bsp_pmic_set_charger_enableDisable(PmicDrvTypeDef * handle, uint8_t enable)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Set_Charger_EnableDisable == NULL) return -1;
	ch_driver->Set_Charger_EnableDisable(handle, enable);

	return 0;
}

int bsp_pmic_get_charger_para(PmicDrvTypeDef * handle, chargerPara *chg_para)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Get_chargingPara == NULL) return -1;
	ch_driver->Get_chargingPara(handle, chg_para);

	return 0;
}

int bsp_pmic_get_pg_status(PmicDrvTypeDef * handle, uint8_t *state)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Get_pgState == NULL) return -1;
	ch_driver->Get_pgState(handle, state);

	return 0;
}

int bsp_pmic_set_charge_currentLimit(PmicDrvTypeDef * handle, uint32_t current)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Set_ChargeCurrentLimit == NULL) return -1;
	ch_driver->Set_ChargeCurrentLimit(handle, current);

	return 0;
}

int bsp_pmic_set_input_currentLimit(PmicDrvTypeDef * handle, uint32_t current)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Set_InputCurrentLimit == NULL) return -1;
	ch_driver->Set_InputCurrentLimit(handle, current);

	return 0;
}

int bsp_pmic_set_charge_voltageLimit(PmicDrvTypeDef * handle, uint32_t voltage)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Set_ChargeVoltageLimit == NULL) return -1;
	ch_driver->Set_ChargeVoltageLimit(handle, voltage);

	return 0;
}

int bsp_pmic_set_charge_hizMode(PmicDrvTypeDef * handle, uint8_t state)
{
	PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
	CHARGER_DRV_t *ch_driver = (CHARGER_DRV_t*)(driver->charger);

	if (!handle->isInitialized) {
		pm_err("%s pmic is not initialized!\n",__func__);
		return -1;
	}

	if (ch_driver->Set_HizMode== NULL) return -1;
	ch_driver->Set_HizMode(handle, state);

	return 0;
}

int bsp_pmic_get_battery_info(PmicDrvTypeDef * handle,
                                    struct pwrmgr_battery_info * info)
{
    int ret = 0;
    int raw_soc = 0, volt = 0, temp = 0, curr = 0;
    PmicFunTable_t *driver = (PmicFunTable_t*)((PmicDrvTypeDef*)handle)->pVTable;
    GAUGE_DRV_t * ga_driver = (GAUGE_DRV_t*)(driver->gauge);
    if(!handle->isInitialized){
        pm_err("%s pmic is not initialized!\n",__func__);
        return -1;
    }
    if(ga_driver->Get_Voltage == NULL) return -1;
    ret |= ga_driver->Get_Voltage(handle, &info->battery_voltage);
    if(ga_driver->Get_Soc == NULL) return -2;
    ret |= ga_driver->Get_Soc(handle, &info->battery_raw_soc);
    if(ga_driver->Get_Temperature == NULL) return -3;
    ret |= ga_driver->Get_Temperature(handle, &info->battery_temperature);
    if(ga_driver->Get_Current == NULL) return -4;
    ret |= ga_driver->Get_Current(handle, 0, &info->battery_current);

    raw_soc = info->battery_raw_soc;
    volt = (info->battery_voltage / 1000);
    temp = (info->battery_temperature / 10);
    curr = (info->battery_current / 1000);
    pm_info("SoC = %d%%, voltage = %dmV, temperature = %d `C, current = %dmA.\n",
		raw_soc, volt, temp, curr);

    return ret;
}

int bsp_pmic_init(void)
{
    PmicFunTable_t *driver = NULL;
    CHARGER_DRV_t *ch_driver = NULL;
    GAUGE_DRV_t * ga_driver = NULL;
    LDO_DRV_t * ldo_driver = NULL;

    if(PMIC_Handle.isInitialized == 1){
        return -2;
    }

    PMIC_Data.charger_type = CHARGER_TYPE_OFF;
    PMIC_Data.charge_current_limit = 2000;
    PMIC_Data.termination_current = 100;
    PMIC_Data.weak_battery_voltage = 3400;
    PMIC_Data.hot_temperature = 55;
    PMIC_Data.handel = &PMIC_Handle;

    PMIC_Fun.charger = &BQ25896_Drv;
    PMIC_Fun.gauge = &MAX17050_Drv;
    PMIC_Fun.ldo = &NCP6924_Drv;

    PMIC_Handle.who_am_i = "hTC Link pmic driver";
    PMIC_Handle.isInitialized = 0;
    PMIC_Handle.isEnabled = 0;
    PMIC_Handle.pData = &PMIC_Data;
    PMIC_Handle.pVTable = &PMIC_Fun;

    driver = (PmicFunTable_t*)(PMIC_Handle.pVTable);
    ch_driver = (CHARGER_DRV_t*)(driver->charger);
    ga_driver = (GAUGE_DRV_t*)(driver->gauge);
    ldo_driver = (LDO_DRV_t *)(driver->ldo);

    if((ch_driver->Init == NULL) ||
        (ga_driver->Init == NULL) ||
        (ldo_driver->Init == NULL)){
        goto FAILED;
    }

    if(ga_driver->Init(&PMIC_Handle) != 0 ) goto FAILED;
    if(ch_driver->Init(&PMIC_Handle) != 0 ) goto FAILED;
    if(ldo_driver->Init(&PMIC_Handle) != 0 ) goto FAILED;
    PMIC_Handle.isInitialized = 1;
    memset(&PMIC_Handle.power_item_count, 0, V_POWER_MAX);
    /* platform power up sequence */
    bsp_pmic_power_up_sequence(&PMIC_Handle);

    periph_multiple_init();

    pm_info("%s OK!\n",__func__);
    return 0;
FAILED:
    memset(&PMIC_Handle, 0, sizeof(PmicDrvTypeDef));
    pm_err("%s FAIL\n",__func__);
    return -1;
}
PmicDrvTypeDef* bsp_pmic_get_handle(void)
{
    if(PMIC_Handle.isInitialized == 1){
        return &PMIC_Handle;
    }
    return NULL;
}
