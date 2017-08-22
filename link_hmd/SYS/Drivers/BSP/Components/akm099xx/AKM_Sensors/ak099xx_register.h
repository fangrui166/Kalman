/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#ifndef INCLUDE_AK099XX_REGISTER_H
#define INCLUDE_AK099XX_REGISTER_H

#include "AKM_Config.h"

#define AK099XX_REG_WIA1                 0x00
#define AK099XX_REG_WIA2                 0x01
#define AK099XX_REG_INFO1                0x02
#define AK099XX_REG_INFO2                0x03
#define AK099XX_REG_ST1                  0x10
#define AK099XX_REG_HXL                  0x11
#define AK099XX_REG_HXH                  0x12
#define AK099XX_REG_HYL                  0x13
#define AK099XX_REG_HYH                  0x14
#define AK099XX_REG_HZL                  0x15
#define AK099XX_REG_HZH                  0x16
#define AK099XX_REG_TMPS                 0x17
#define AK099XX_REG_ST2                  0x18
#define AK099XX_REG_CNTL1                0x30
#define AK099XX_REG_CNTL2                0x31
#define AK099XX_REG_CNTL3                0x32

/*only ak09911 and ak09912 use ASA*/
#define AK099XX_FUSE_ASAX                0x60
#define AK099XX_FUSE_ASAY                0x61
#define AK099XX_FUSE_ASAZ                0x62

#define AK099XX_BDATA_SIZE               9

#define AK099XX_MODE_SNG_MEASURE         0x01
#define AK099XX_MODE_CONT_MEASURE_MODE1  0x02
#define AK099XX_MODE_CONT_MEASURE_MODE2  0x04
#define AK099XX_MODE_CONT_MEASURE_MODE3  0x06
#define AK099XX_MODE_CONT_MEASURE_MODE4  0x08
#define AK099XX_MODE_CONT_MEASURE_MODE5  0x0A
#define AK099XX_MODE_CONT_MEASURE_MODE6  0x0C
#define AK099XX_MODE_SELF_TEST           0x10
#define AK099XX_MODE_FUSE_ACCESS         0x1F		/*used for ak09911 and ak09912 sensor*/
#define AK099XX_MODE_POWER_DOWN          0x00

#define AK099XX_SOFT_RESET               0x01

#define AK09911_WIA_VAL                  0x548
#define AK09912_WIA_VAL                  0x448
#define AK09913_WIA_VAL                  0x848
#define AK09915_WIA_VAL                  0x1048
#define AK09916C_WIA_VAL                 0x948
#define AK09916D_WIA_VAL                 0xB48

#if defined(AKM_MAGNETOMETER_AK09915)
#define AK0991X_NSF_VAL                  0x00  /*ak09915 NSF disable*/
#else
#define AK0991X_NSF_VAL                  0x40  /*ak09912 NSF Middle*/
#endif
#define AK09915_SET_LOWNOISE(cntl2)  ((0x40) | (cntl2))
#endif /* INCLUDE_AK099XX_REGISTER_H */
