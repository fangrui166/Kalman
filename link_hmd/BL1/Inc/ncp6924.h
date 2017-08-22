#ifndef __NCP6924_H__
#define __NCP6924_H__

#include <stdint.h>
#include "x_pmic.h"
#include "stm32f4xx_hal.h"

/* enable DCDC/LDO by gpio or i2c rw register */
/* priorty: gpio > register */
#define NCP6924_EN_BIT_DCDC1_PWM    0x0
#define NCP6924_EN_BIT_DCDC2_PWM    0x2
#define NCP6924_EN_REG_BIT_DCDC1    0x1
#define NCP6924_EN_REG_BIT_DCDC2    0x3
#define NCP6924_EN_REG_BIT_LDO1     0x4
#define NCP6924_EN_REG_BIT_LDO2     0x5
#define NCP6924_EN_REG_BIT_LDO3     0x6
#define NCP6924_EN_REG_BIT_LDO4     0x7

#define NCP6924_REG_PID             0x11
#define NCP6924_REG_FID             0x13
#define NCP6924_REG_ENABLE          0x14 /*default 0xFA*/
#define NCP6924_REG_DCDC1           0x20
#define NCP6924_REG_DCDC2           0x22
#define NCP6924_REG_LDO1            0x24
#define NCP6924_REG_LDO2            0x25
#define NCP6924_REG_LDO3            0x26
#define NCP6924_REG_LDO4            0x27

#define VOLT_DCDC_1V15              0x2C
#define VOLT_DCDC_1V8               0x60
#define VOLT_LDO_1V8                0x10
#define VOLT_LDO_3V3                0xFF

enum {
    NCP6924_EN_DISABLE,
    NCP6924_EN_ENABLE
};

typedef enum{
    NCP6924C_BUCK1,
    NCP6924C_BUCK2,
    NCP6924C_LDO1,
    NCP6924C_LDO2,
    NCP6924C_LDO3,
    NCP6924C_LDO4
}LDO_Index_t;

typedef struct {
    uint8_t reg_addr;
    uint8_t enable_bit;
}ncp6924_vreg;

typedef struct{
    int (* Init )(PmicDrvTypeDef*);
    int (* Deinit )(PmicDrvTypeDef*);
    int (* LDO_SetVoltage )(PmicDrvTypeDef*, LDO_Index_t, uint32_t);
    int (* LDO_Enable )(PmicDrvTypeDef*, LDO_Index_t, uint8_t);
    int (* LDO_GetEnableState )(PmicDrvTypeDef*, LDO_Index_t, uint8_t*);
}LDO_DRV_t;

extern LDO_DRV_t NCP6924_Drv;
extern ncp6924_vreg pmic_vreg[6];
int8_t ncp6924_reg_write(uint8_t reg, uint8_t value);
int8_t ncp6924_reg_read(uint8_t reg, uint8_t *value);

#endif
