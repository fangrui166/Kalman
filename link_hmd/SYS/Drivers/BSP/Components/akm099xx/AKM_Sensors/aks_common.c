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
#include "AKM_Common.h"
#include "aks_common.h"

void AKS_MyStrcpy(
    char          *dst,
    const char    *src,
    const int16_t dst_len)
{
    int i = 1;

    while (*src != '\0') {
        *dst = *src;
        dst++;
        src++;
        i++;

        if (i >= dst_len) {
            break;
        }
    }

    *dst = '\0';
}

void AKS_ConvertCoordinate(
    int32_t       vec[3],
    const uint8_t axis_order[3],
    const uint8_t axis_sign[3])
{
    int32_t val32[3];
    uint8_t i;

    /* Axis conversion */
    for (i = 0; i < 3; i++) {
        val32[i] = vec[axis_order[i]];

        if (axis_sign[i]) {
            val32[i] *= -1;
        }
    }

    /* Copy to argument */
    for (i = 0; i < 3; i++) {
        vec[i] = val32[i];
    }
}
