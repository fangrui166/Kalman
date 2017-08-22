#ifndef __FOTA_COMMAND_H__
#define __FOTA_COMMAND_H__
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "misc_data.h"
#include "htc_usb_cdc_data_service.h"
#include "PowerManager_command.h"
#include "PowerManager.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_battery.h"

#if (__ARMCC_VERSION)
    __attribute__ ((section(".runtype"), zero_init))
#elif defined (__GNUC__)
    __attribute__ ((section(".runtype")))
#elif defined (__ICCARM__)
    #pragma location=".runtype"
#endif /* (__ARMCC_VERSION) */

#define FOTA_DRV_HLOG_ENABLE
#ifdef FOTA_DRV_HLOG_ENABLE
#define fota_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_FOTA, fmt, ##__VA_ARGS__)
#define fota_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_FOTA, fmt, ##__VA_ARGS__)
#define fota_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_FOTA, fmt, ##__VA_ARGS__)
#define fota_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FOTA, fmt, ##__VA_ARGS__)
#define fota_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_FOTA, fmt, ##__VA_ARGS__)
#else /* FOTA_DRV_HLOG_ENABLE */
#define fota_emerg(fmt, ...) \
	printf("[FOTA][EMR] :" fmt, ##__VA_ARGS__)
#define fota_err(fmt, ...) \
	printf("[FOTA][ERR] :" fmt, ##__VA_ARGS__)
#define fota_warning(fmt, ...) \
	printf("[FOTA][WARN]:" fmt, ##__VA_ARGS__)
#define fota_info(fmt, ...) \
	printf("[FOTA][INFO]:" fmt, ##__VA_ARGS__)
#define fota_debug(fmt, ...) \
	printf("[FOTA][DBG] :" fmt, ##__VA_ARGS__)
#endif /* FOTA_DRV_HLOG_ENABLE */


int32_t fota_ProcessRcvData(uint8_t * buf, uint8_t len);
void fota_reboot(uint32_t runtype);

#endif
