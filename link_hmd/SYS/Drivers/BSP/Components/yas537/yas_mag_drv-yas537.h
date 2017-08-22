/*
 * Copyright (c) 2014-2015 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#ifndef __YAS537_MAG_DRV_YAS537_H
#define __YAS537_MAG_DRV_YAS537_H

#ifdef __cplusplus
extern "C" {
#endif

#include "yas.h"


#define YAS537_REG_DIDR			(0x80)
#define YAS537_REG_CMDR			(0x81)
#define YAS537_REG_CONFR		(0x82)
#define YAS537_REG_INTRVLR		(0x83)
#define YAS537_REG_OXR			(0x84)
#define YAS537_REG_OY1R			(0x85)
#define YAS537_REG_OY2R			(0x86)
#define YAS537_REG_AVRR			(0x87)
#define YAS537_REG_HCKR			(0x88)
#define YAS537_REG_LCKR			(0x89)
#define YAS537_REG_SRSTR		(0x90)
#define YAS537_REG_ADCCALR		(0x91)
#define YAS537_REG_MTCR			(0x93)
#define YAS537_REG_OCR			(0x9e)
#define YAS537_REG_TRMR			(0x9f)
#define YAS537_REG_DATAR		(0xb0)
#define YAS537_REG_CALR			(0xc0)

#define YAS537_DATA_UNDERFLOW		(0)
#define YAS537_DATA_OVERFLOW		(16383)
#define YAS537_DEVICE_ID		(0x07)	/* YAS537 (MS-3T) */

#define YAS_X_OVERFLOW			(0x01)
#define YAS_X_UNDERFLOW			(0x02)
#define YAS_Y1_OVERFLOW			(0x04)
#define YAS_Y1_UNDERFLOW		(0x08)
#define YAS_Y2_OVERFLOW			(0x10)
#define YAS_Y2_UNDERFLOW		(0x20)
#define YAS_OVERFLOW	(YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW	(YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

#define YAS537_MAG_STATE_NORMAL		(0)
#define YAS537_MAG_STATE_INIT_COIL	(1)
#define YAS537_MAG_INITCOIL_TIMEOUT	(1000)	/* msec */
#define YAS537_MAG_POWER_ON_RESET_TIME	(4000)	/* usec */
#define YAS537_MAG_NOTRANS_POSITION	(2)

#define YAS537_MAG_AVERAGE_8		(0)
#define YAS537_MAG_AVERAGE_16		(1)
#define YAS537_MAG_AVERAGE_32		(2)
#define YAS537_MAG_AVERAGE_64		(3)
#define YAS537_MAG_AVERAGE_128		(4)
#define YAS537_MAG_AVERAGE_256		(5)

#define YAS537_MAG_RCOIL_TIME		(65)

#define YAS537_SLAVE_ADDR		(0x5D)
#define YAS537_SOFT_RESET		(0x02)


#ifdef __cplusplus
}
#endif

#endif /* __YAS537_MAG_DRV_YAS537_H */
