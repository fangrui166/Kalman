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

// Created by Raymond Fan for MAXIM PMIC MAX14676C


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_DRV_H__
#define __FLASH_DRV_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
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


int flash_init(void);
uint32_t VR_flash_erase(uint32_t offset, uint32_t length);
uint32_t VR_flash_write(void * source, uint32_t offset, uint32_t length);
uint32_t VR_flash_read(uint32_t addr, void *buff, unsigned size);

uint32_t flash_ob_wp_disable(FLASH_WP_AREA wp_area);
uint32_t flash_ob_wp_enable(FLASH_WP_AREA wp_area);

#if 0
typedef enum {
     PART_BL0,            /* 32K     sector[0:1] */
     PART_MISC_DATA,      /* 16K     sector[2]   */
     PART_LOG,            /* 16K     sector[3]   */
     PART_BL1,            /* 128K    sector[5]   */
     PART_RTOS,           /* 256K    sector[6:7] */
     PART_FOTA,           /* 256K    sector[8:9] */
     PART_ERROR,
 }FLASH_PART;
#else
typedef enum {
     PART_BL0,              /* 16K     sector[0]   */
     PART_MISC_DATA,        /* 16K     sector[1]   */
     PART_MFG_BL1,          /* 96K     sector[2:4] */
     PART_FOTA_LOG,         /* 128K    sector[5]   */
     PART_RTOS,             /* 256K    sector[6:7] */
     PART_ERROR,
 }FLASH_PART;
 #endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ifndef __FLASH_DRV_H__ */
