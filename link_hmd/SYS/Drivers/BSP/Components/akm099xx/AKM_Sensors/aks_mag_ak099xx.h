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
#ifndef INCLUDE_AKS_MAG_AK099XX_H
#define INCLUDE_AKS_MAG_AK099XX_H
#include "AKM_Common.h"
#include "AKS_APIs.h"

int16_t ak099xx_init(
    const uint8_t axis_order[3],
    const uint8_t axis_sign[3]
);

int16_t ak099xx_get_info(
    struct AKS_DEVICE_INFO *info
);

int16_t ak099xx_start(
    const int32_t interval_us
);

int16_t ak099xx_stop(
    void
);

int16_t ak099xx_check_rdy(
    const int32_t timeout_us
);

int16_t ak099xx_get_data(
    struct AKM_SENSOR_DATA *data,
    uint8_t                *num
);

int16_t ak099xx_get_mode(
    uint8_t                *mode
);

int16_t ak099xx_get_whoami(
    uint16_t                *whoami
);

int16_t ak099xx_soft_reset(
    void
);

int16_t ak099xx_self_test(
    int32_t *result
);
#endif /* INCLUDE_AKS_MAG_AK099XX_H */
