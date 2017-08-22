/*
 * HTC Corporation Proprietary Rights Acknowledgment
 *
 * Copyright (C) 2013 HTC Corporation
 *
 * All Rights Reserved.
 *
 * The information contained in this work is the exclusive property of HTC Corporation
 * ("HTC").  Only the user who is legally authorized by HTC ("Authorized User") has
 * right to employ this work within the scope of this statement.  Nevertheless, the
 * Authorized User shall not use this work for any purpose other than the purpose
 * agreed by HTC.  Any and all addition or modification to this work shall be
 * unconditionally granted back to HTC and such addition or modification shall be
 * solely owned by HTC.  No right is granted under this statement, including but not
 * limited to, distribution, reproduction, and transmission, except as otherwise
 * provided in this statement.  Any other usage of this work shall be subject to the
 * further written consent of HTC.
 */

// Created by Raymond Fan for MAXIM flash MAX14676C


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_DRV_H__
#define __FLASH_DRV_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//#include "hlog_api.h"
#include <stdint.h>
typedef enum
{
    FLASH_IRQ_NO = 0x0,
    FLASH_IRQ_NORMAL = 0x1,
} FLASH_IRQ_STATE;
typedef enum
{
    FLASH_WP_BL = 0x0,
    FLASH_WP_SYS = 0x6,
} FLASH_WP_AREA;

#define FLASH_HLOG_ENABLE 1

#if FLASH_HLOG_ENABLE
#define flash_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_FLASH, fmt, ##__VA_ARGS__)
#define flash_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_FLASH, fmt, ##__VA_ARGS__)
#define flash_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_FLASH, fmt, ##__VA_ARGS__)
#define flash_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FLASH, fmt, ##__VA_ARGS__)
#define flash_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_FLASH, fmt, ##__VA_ARGS__)
#else /* flash_HLOG_ENABLE */
#define flash_emerg(fmt, ...) \
	printf("[FLASH][EMR] :" fmt, ##__VA_ARGS__)
#define flash_err(fmt, ...) \
	printf("[FLASH][ERR] :" fmt, ##__VA_ARGS__)
#define flash_warning(fmt, ...) \
	printf("[FLASH][WARN]:" fmt, ##__VA_ARGS__)
#define flash_info(fmt, ...) \
	printf("[FLASH][INFO]:" fmt, ##__VA_ARGS__)
#define flash_debug(fmt, ...) \
	printf("[FLASH][DBG] :" fmt, ##__VA_ARGS__)
#endif /* flash_HLOG_ENABLE */

int flash_init(void);
uint32_t VR_flash_erase(uint32_t offset, uint32_t length);
uint32_t VR_flash_write(void * source, uint32_t offset, uint32_t length);
uint32_t VR_flash_read(uint32_t addr, void *buff, unsigned size);
uint32_t flash_ob_status(void);

uint32_t flash_ob_wp_disable(FLASH_WP_AREA wp_area);
uint32_t flash_ob_wp_enable(FLASH_WP_AREA wp_area);
uint32_t flash_ob_wp_is_BL_protected(void);
uint32_t flash_ob_rdp_level_set(uint32_t level);
uint32_t flash_ob_rdp_level_get(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ifndef __FLASH_DRV_H__ */
