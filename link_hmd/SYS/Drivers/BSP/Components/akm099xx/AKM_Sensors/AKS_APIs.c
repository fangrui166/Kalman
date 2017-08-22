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
#include "AKS_APIs.h"

#if defined(AKM_MAGNETOMETER_AK8963)
#include "aks_mag_ak8963.h"
#define aks_init_mag       ak8963_init
#define aks_get_info_mag   ak8963_get_info
#define aks_start_mag      ak8963_start
#define aks_stop_mag       ak8963_stop
#define aks_check_rdy_mag  ak8963_check_rdy
#define aks_get_data_mag   ak8963_get_data
#define aks_self_test_mag  ak8963_self_test
#elif defined (AKM_MAGNETOMETER_AK099XX)
#include "aks_mag_ak099xx.h"
#define aks_init_mag       ak099xx_init
#define aks_get_info_mag   ak099xx_get_info
#define aks_start_mag      ak099xx_start
#define aks_stop_mag       ak099xx_stop
#define aks_check_rdy_mag  ak099xx_check_rdy
#define aks_get_data_mag   ak099xx_get_data
#define aks_self_test_mag  ak099xx_self_test
#define aks_get_mode_mag ak099xx_get_mode
#define aks_get_whoami_mag ak099xx_get_whoami
#else
#define aks_init_mag       no_device_init
#define aks_get_info_mag   no_device_get_info
#define aks_start_mag      no_device_start
#define aks_stop_mag       no_device_stop
#define aks_check_rdy_mag  no_device_check_rdy
#define aks_get_data_mag   no_device_get_data
#define aks_self_test_mag  no_device_self_test
#endif

#if defined(AKM_ACCELEROMETER_ADXL34X)
#include "aks_acc_adxl34x.h"
#define aks_init_acc       adxl34x_init
#define aks_get_info_acc   adxl34x_get_info
#define aks_start_acc      adxl34x_start
#define aks_stop_acc       adxl34x_stop
#define aks_check_rdy_acc  adxl34x_check_rdy
#define aks_get_data_acc   adxl34x_get_data
#define aks_self_test_acc  adxl34x_self_test
#elif defined (AKM_ACCELEROMETER_DUMMY)
#include "aks_acc_dummy.h"
#define aks_init_acc       acc_dummy_init
#define aks_get_info_acc   acc_dummy_get_info
#define aks_start_acc      acc_dummy_start
#define aks_stop_acc       acc_dummy_stop
#define aks_check_rdy_acc  acc_dummy_check_rdy
#define aks_get_data_acc   acc_dummy_get_data
#define aks_self_test_acc  acc_dummy_self_test
#else
#define aks_init_acc       no_device_init
#define aks_get_info_acc   no_device_get_info
#define aks_start_acc      no_device_start
#define aks_stop_acc       no_device_stop
#define aks_check_rdy_acc  no_device_check_rdy
#define aks_get_data_acc   no_device_get_data
#define aks_self_test_acc  no_device_self_test
#endif

#if defined(AKM_GYROSCOPE_L3G4200D)
#include "aks_gyr_l3g4200d.h"
#define aks_init_gyr       l3g4200d_init
#define aks_get_info_gyr   l3g4200d_get_info
#define aks_start_gyr      l3g4200d_start
#define aks_stop_gyr       l3g4200d_stop
#define aks_check_rdy_gyr  l3g4200d_check_rdy
#define aks_get_data_gyr   l3g4200d_get_data
#define aks_self_test_gyr  l3g4200d_self_test
#else
#define aks_init_gyr       no_device_init
#define aks_get_info_gyr   no_device_get_info
#define aks_start_gyr      no_device_start
#define aks_stop_gyr       no_device_stop
#define aks_check_rdy_gyr  no_device_check_rdy
#define aks_get_data_gyr   no_device_get_data
#define aks_self_test_gyr  no_device_self_test
#endif

static int16_t no_device_init(
    const uint8_t axis_order[3],
    const uint8_t axis_sign[3])
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_get_info(struct AKS_DEVICE_INFO *info)
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_start(const int32_t interval_us)
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_stop(void)
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_check_rdy(const int32_t timeout_us)
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_get_data(
    struct AKM_SENSOR_DATA *data,
    uint8_t                *num)
{
    return AKM_ERR_NOT_SUPPORT;
}

static int16_t no_device_self_test(int32_t *result)
{
    return AKM_ERR_NOT_SUPPORT;
}

/******************************************************************************/
/***** AKS public APIs ********************************************************/
int16_t AKS_Init(
    const AKM_SENSOR_TYPE stype,
    const uint8_t         axis_order[3],
    const uint8_t         axis_sign[3])
{
    int16_t ret = AKM_ERR_NOT_SUPPORT;

    if (stype == 0) {
        return AKM_SUCCESS;
    }

    if (stype & AKM_ST_MAG) {
        if ((ret = aks_init_mag(axis_order, axis_sign)) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_ACC) {
        if ((ret = aks_init_acc(axis_order, axis_sign)) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_GYR) {
        if ((ret = aks_init_gyr(axis_order, axis_sign)) != AKM_SUCCESS) {
            goto exit;
        }
    }

exit:
    return ret;
}

int16_t AKS_GetDeviceInfo(
    const AKM_SENSOR_TYPE  stype,
    struct AKS_DEVICE_INFO *info,
    uint8_t                *num)
{
    /* Currently only one device/call is supported */
    *num = 1;

    switch (stype) {
    case AKM_ST_MAG:
        return aks_get_info_mag(info);

    case AKM_ST_ACC:
        return aks_get_info_acc(info);

    case AKM_ST_GYR:
        return aks_get_info_gyr(info);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

int16_t AKS_Start(
    const AKM_SENSOR_TYPE stype,
    const int32_t         interval_us)
{
    int16_t ret = AKM_ERR_NOT_SUPPORT;

    if (stype == 0) {
        return AKM_SUCCESS;
    }

    if (stype & AKM_ST_MAG) {
        if ((ret = aks_start_mag(interval_us)) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_ACC) {
        if ((ret = aks_start_acc(interval_us)) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_GYR) {
        if ((ret = aks_start_gyr(interval_us)) != AKM_SUCCESS) {
            goto exit;
        }
    }

exit:
    return ret;
}

int16_t AKS_Stop(const AKM_SENSOR_TYPE stype)
{
    int16_t ret = AKM_ERR_NOT_SUPPORT;

    if (stype == 0) {
        return AKM_SUCCESS;
    }

    if (stype & AKM_ST_MAG) {
        if ((ret = aks_stop_mag()) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_ACC) {
        if ((ret = aks_stop_acc()) != AKM_SUCCESS) {
            goto exit;
        }
    }

    if (stype & AKM_ST_GYR) {
        if ((ret = aks_stop_gyr()) != AKM_SUCCESS) {
            goto exit;
        }
    }

exit:
    return ret;
}

int16_t AKS_CheckDataReady(
    const AKM_SENSOR_TYPE stype,
    const int32_t         timeout_us)
{
    switch (stype) {
    case AKM_ST_MAG:
        return aks_check_rdy_mag(timeout_us);

    case AKM_ST_ACC:
        return aks_check_rdy_acc(timeout_us);

    case AKM_ST_GYR:
        return aks_check_rdy_gyr(timeout_us);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

int16_t AKS_GetData(
    const AKM_SENSOR_TYPE  stype,
    struct AKM_SENSOR_DATA *data,
    uint8_t                *num)
{
    switch (stype) {
    case AKM_ST_MAG:
        return aks_get_data_mag(data, num);

    case AKM_ST_ACC:
        return aks_get_data_acc(data, num);

    case AKM_ST_GYR:
        return aks_get_data_gyr(data, num);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

int16_t AKS_SelfTest(
    const AKM_SENSOR_TYPE stype,
    int32_t               *result)
{
    switch (stype) {
    case AKM_ST_MAG:
        return aks_self_test_mag(result);

    case AKM_ST_ACC:
        return aks_self_test_acc(result);

    case AKM_ST_GYR:
        return aks_self_test_gyr(result);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

int16_t AKS_GetMode(
    const AKM_SENSOR_TYPE stype,
    uint8_t               *mode)
{
    switch (stype) {
    case AKM_ST_MAG:
        return aks_get_mode_mag(mode);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

int16_t AKS_GetWhoami(
    const AKM_SENSOR_TYPE stype,
    uint16_t               *whoami)
{
    switch (stype) {
    case AKM_ST_MAG:
        return aks_get_whoami_mag(whoami);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}
