#ifndef __MAX17050_FUEL_GAUGE_H_
#define __MAX17050_FUEL_GAUGE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "x_pmic.h"

#ifndef bool
#define bool uint8_t
#endif

#define MAX17050_STATUS_BATTABSENT	(1 << 3)
#define MAX17050_BATTERY_FULL		100
#define MAX17050_DEFAULT_SNS_RESISTOR	10000

/* Number of words in model characterisation data */
#define MODEL_SIZE			48


#define MAX17050_I2C_ADDR               (0x36 << 1)
#define MAX17050_I2C_TIMEOUT_MAX  0x1000

enum max17050_register {
	MAX17050_STATUS		= 0X00,
	MAX17050_VALRT_TH	= 0X01,
	MAX17050_TALRT_TH	= 0X02,
	MAX17050_SALRT_TH	= 0X03,
	MAX17050_ATRATE		= 0X04,
	MAX17050_REPCAP		= 0X05,
	MAX17050_REPSOC		= 0X06,
	MAX17050_AGE		= 0X07,
	MAX17050_TEMP		= 0X08,
	MAX17050_VCELL		= 0X09,
	MAX17050_CURRENT	= 0X0A,
	MAX17050_AVGCURRENT	= 0X0B,

	MAX17050_SOC		= 0X0D,
	MAX17050_AVSOC		= 0X0E,
	MAX17050_REMCAP		= 0X0F,
	MAX17050_FULLCAP	= 0X10,
	MAX17050_TTE		= 0X11,
	MAX17050_QRTABLE00	= 0X12,
	MAX17050_FULLSOCTHR	= 0X13,

	MAX17050_AVGTA		= 0X16,
	MAX17050_CYCLES		= 0X17,
	MAX17050_DESIGNCAP	= 0X18,
	MAX17050_AVGVCELL	= 0X19,
	MAX17050_MINMAXTEMP	= 0X1A,
	MAX17050_MINMAXVOLT	= 0X1B,
	MAX17050_MINMAXCURR	= 0X1C,
	MAX17050_CONFIG		= 0X1D,
	MAX17050_ICHGTERM	= 0X1E,
	MAX17050_AVCAP		= 0X1F,
	MAX17050_CUSTOMVER	= 0X20,
	MAX17050_VERSION	= 0X21,
	MAX17050_QRTABLE10	= 0X22,
	MAX17050_FULLCAPNOM	= 0X23,
	MAX17050_TEMPNOM	= 0X24,
	MAX17050_TEMPLIM	= 0X25,

	MAX17050_AIN		= 0X27,
	MAX17050_LEARNCFG	= 0X28,
	MAX17050_FILTERCFG	= 0X29,
	MAX17050_RELAXCFG	= 0X2A,
	MAX17050_MISCCFG	= 0X2B,
	MAX17050_TGAIN		= 0X2C,
	MAX17050_TOFF		= 0X2D,
	MAX17050_CGAIN		= 0X2E,
	MAX17050_COFF		= 0X2F,

	MAX17050_QRTABLE20	= 0X32,

	MAX17050_FULLCAP0	= 0x35,
	MAX17050_IAVG_EMPTY	= 0X36,
	MAX17050_FCTC		= 0X37,
	MAX17050_RCOMP0		= 0X38,
	MAX17050_TEMPCO		= 0X39,
	MAX17050_V_EMPTY	= 0X3A,

	MAX17050_FSTAT		= 0X3D,
	MAX17050_TIMER		= 0X3E,
	MAX17050_SHDNTIMER	= 0X3F,

	MAX17050_QRTABLE30	= 0X42,

	MAX17050_DQACC		= 0X45,
	MAX17050_DPACC		= 0X46,

	MAX17050_VFSOC0		= 0x48,

	MAX17050_QH0		= 0x4c,
	MAX17050_QH		= 0X4D,

	MAX17050_VFSOC0_LOCK	= 0x60,
	MAX17050_MODEL_LOCK1	= 0x62,
	MAX17050_MODEL_LOCK2	= 0x63,

	MAX17050_MODEL_TABLE	= 0x80,

	MAX17050_OCVINTERNAL	= 0XFB,

	MAX17050_VFSOC		= 0XFF,
};

#define BIT(nr)		(1 << (nr))

/* Status register bits */
#define MAX17050_STATUS_POR             	BIT(1)  /* Power-On Reset */
#define MAX17050_STATUS_BST        		BIT(3)  /* Battery Status */
#define MAX17050_STATUS_VMN             	BIT(8)  /* Min Valrt */
#define MAX17050_STATUS_TMN             	BIT(9)  /* Min Talrt */
#define MAX17050_STATUS_SMN             	BIT(10) /* Min SOCalrt */
#define MAX17050_STATUS_BI			BIT(11) /*Battery Insertion*/
#define MAX17050_STATUS_VMX             	BIT(12) /* Max Valrt */
#define MAX17050_STATUS_TMX             	BIT(13) /* Max Talrt */
#define MAX17050_STATUS_SMX             	BIT(14) /* Max SOCalrt */
#define MAX17050_STATUS_BR			BIT(15) /*Battery Removal*/

/* Interrupt config REG bits */
#define MAX17050_CONFIG_BER             	BIT(0)  /* Enable alert on battery insertion */
#define MAX17050_CONFIG_BEI              	BIT(1)  /* Enable alert on battery removal */
#define MAX17050_CONFIG_AEN             	BIT(2)  /* Alert enable */
#define MAX17050_CONFIG_TEX			BIT(8)  /* Temperature channel Enable*/
#define MAX17050_CONFIG_TEN			BIT(9)  /* Temperature channel Enable*/
#define MAX17050_CONFIG_TS             	BIT(13)  /* Alert enable */
#define MAX17050_CONFIG_SS            	BIT(14)  /* Alert enable */

#define DEVICE_BATTERY_LEVEL_EMPTY	(0)
#define DEVICE_BATTERY_LEVEL_FULL	(100)
#define CONFIG_BATTERY_LEVEL_LOW	(20)
#define CONFIG_BATTERY_LEVEL_LOW_HYST	(1)
#define CONFIG_BATTERY_LEVEL_EMPTY_HYST		(1)
#define CONFIG_BATTERY_LEVEL_FULL_HYST	(1)
#define MIN_SOC_ALRT	(1)

#define CONFIG_BATTERY_TEMP_ICE		(0)
#define CONFIG_BATTERY_TEMP_COLD	(0)
#define CONFIG_BATTERY_TEMP_COOL	(10)
#define CONFIG_BATTERY_TEMP_WARM	(20)
#define CONFIG_BATTERY_TEMP_HOT	(50)
#define CONFIG_BATTERY_TEMP_COOL_DOWN	(58)
#define TALRT_MIN_DIS	(0x80)
#define TALRT_MAX_DIS 	(0x7F)

typedef struct{
	int (* Init )(PmicDrvTypeDef*);
	int (* Deinit )(PmicDrvTypeDef*);
	int (* Get_Temperature )(PmicDrvTypeDef*, int *);
	int (* Get_Soc)(PmicDrvTypeDef*, int *);
	int (* Get_Current)(PmicDrvTypeDef *, bool , int *);
	int (* Get_Voltage)(PmicDrvTypeDef *, uint32_t *);
	int (* Batt_Online)(PmicDrvTypeDef *, bool *);
	int (* Get_Full_Capacity)(PmicDrvTypeDef *, uint32_t *);
}GAUGE_DRV_t;

enum batt_soc_level {
    BATTERY_LEVEL_EMPTY = 0,
    BATTERY_LEVEL_LOW,
    BATTERY_LEVEL_NORMAL,
    BATTERY_LEVEL_FULL,
};

enum batt_temp_level {
    BATTERY_COLD,
    BATTERY_COOL,
    BATTERY_NORMAL,
    BATTERY_WARM,
    BATTERY_HOT,
    BATTERY_COOL_DOWN
};

enum batt_intr_event_type {
	BATT_INT_EVENT_UNKONW,
	BATT_INT_EVENT_SOC_EMPTY,
	BATT_INT_EVENT_SOC_LOW,
	BATT_INT_EVENT_SOC_FULL,
	BATT_INT_EVENT_SOC_NORMAL,
	BATT_INT_EVENT_TEMP_COLD,
	BATT_INT_EVENT_TEMP_COOL,
	BATT_INT_EVENT_TEMP_NORMAL,
	BATT_INT_EVENT_TEMP_WARM,
	BATT_INT_EVENT_TEMP_HOT,
	BATT_INT_EVENT_TEMP_COOL_DOWN,
};

void max17050_intr_handler(uint16_t);
int htc_batt_get_soc(void);
int htc_batt_get_temperature(void);
int htc_batt_get_current(bool average);
int htc_batt_get_voltage(void);
int htc_batt_get_full_capacity(void);
bool htc_batt_is_online(void);
int max17050_register_batt_status_notify(int (*callback)(int));
int max17050_register_get_batt_online(int (*callback)(uint8_t *));

#ifdef FG_HLOG_ENABLE
#define fg_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_CHG, fmt, ##__VA_ARGS__)
#define fg_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_CHG, fmt, ##__VA_ARGS__)
#define fg_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_CHG, fmt, ##__VA_ARGS__)
#define fg_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_CHG, fmt, ##__VA_ARGS__)
#define fg_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_CHG, fmt, ##__VA_ARGS__)
#else /* FG_HLOG_ENABLE */
#define fg_emerg(fmt, ...) \
	printf("[FG][EMR] :" fmt, ##__VA_ARGS__)
#define fg_err(fmt, ...) \
	printf("[FG][ERR] :" fmt, ##__VA_ARGS__)
#define fg_warning(fmt, ...) \
	printf("[FG][WARN]:" fmt, ##__VA_ARGS__)
#define fg_info(fmt, ...) \
	printf("[FG][INFO]:" fmt, ##__VA_ARGS__)
#define fg_debug(fmt, ...) \
	printf("[FG][DBG] :" fmt, ##__VA_ARGS__)
#endif /* FG_HLOG_ENABLE */

extern GAUGE_DRV_t MAX17050_Drv;


#ifdef __cplusplus
}
#endif

#endif /* __MAX17050_BATTERY_H_ */
