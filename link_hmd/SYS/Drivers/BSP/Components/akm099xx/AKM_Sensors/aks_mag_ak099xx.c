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
#include "AKH_APIs.h"

#include "ak099xx_register.h"
#include "aks_common.h"
#include "aks_mag_ak099xx.h"

#if defined(AKM_MAGNETOMETER_AK09911)
#define DEVICE_NAME    "ak09911"
#define DEVICE_NUMBER  (9911)
#define SENS_IN_Q16    ((int32_t)(39322)) /* 0.6  in Q16 format */
#elif defined(AKM_MAGNETOMETER_AK09912)
#define DEVICE_NAME    "ak09912"
#define DEVICE_NUMBER  (9912)
#define SENS_IN_Q16    ((int32_t)(9830)) /* 0.15 in Q16 format */
#elif defined(AKM_MAGNETOMETER_AK09913)
#define DEVICE_NAME    "ak09913"
#define DEVICE_NUMBER  (9913)
#define SENS_IN_Q16    ((int32_t)(9830)) /* 0.15 in Q16 format */
#elif defined(AKM_MAGNETOMETER_AK09915)
#define DEVICE_NAME    "ak09915"
#define DEVICE_NUMBER  (9915)
#define SENS_IN_Q16    ((int32_t)(9830)) /* 0.15 in Q16 format */
#elif defined(AKM_MAGNETOMETER_AK09916)
#define DEVICE_NAME    "ak09916"
#define DEVICE_NUMBER  (9916)
#define SENS_IN_Q16    ((int32_t)(9830)) /* 0.15 in Q16 format */
#else
#define SENS_IN_Q16    (0)               /* dummy */
#endif

#define AKM_FST_ERRCODE(testno, data) \
    (int32_t)((((uint32_t)testno) << 16) | ((uint16_t)data))
#define AKM_FST(no, data, lo, hi, err) \
    if (akm_fst_test_data((no), (data), (lo), (hi), (err)) != AKM_SUCCESS) \
    { goto SELFTEST_FAIL; }

/* Global variable for ASA value. */
static uint16_t g_dev;
static uint8_t  g_asa[3];
static int32_t  g_raw_to_micro_q16[3];
static uint8_t  g_mag_axis_order[3];
static uint8_t  g_mag_axis_sign[3];

/******************************************************************************/
/***** AKS static functions ***************************************************/
static int16_t ak099xx_set_mode(const uint8_t mode)
{
    uint8_t i2cData;
    int16_t fret;

    i2cData = mode;
#if defined(AKM_MAGNETOMETER_AK09915) && defined(AK09915_USE_LOW_NOISE)
    i2cData = AK09915_SET_LOWNOISE(i2cData);
#endif
    fret = AKH_TxData(AKM_ST_MAG, AK099XX_REG_CNTL2, &i2cData, 1);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    /* When succeeded, sleep */
    AKH_DelayMicro(100);
    return AKM_SUCCESS;
}

int16_t ak099xx_soft_reset(void)
{
    uint8_t i2cData;
    int16_t fret;

    /* Soft Reset */
    i2cData = AK099XX_SOFT_RESET;
    fret = AKH_TxData(AKM_ST_MAG, AK099XX_REG_CNTL3, &i2cData, 1);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    /* When succeeded, sleep */
    AKH_DelayMicro(100);
    return AKM_SUCCESS;
}

static int16_t akm_fst_test_data(
    uint16_t testno,
    int16_t  testdata,
    int16_t  lolimit,
    int16_t  hilimit,
    int32_t  *err)
{
#ifdef DEBUG
    AKH_Print("DBG: FST 0x%08X\n", AKM_FST_ERRCODE(testno, testdata));
#endif

    if ((lolimit <= testdata) && (testdata <= hilimit)) {
        return AKM_SUCCESS;
    } else {
        *err = AKM_FST_ERRCODE(testno, testdata);
        return AKM_ERROR;
    }
}

/******************************************************************************/
/***** AKS public APIs ********************************************************/
int16_t ak099xx_init(
    const uint8_t axis_order[3],
    const uint8_t axis_sign[3])
{
    uint8_t i2cData[2];
    int16_t fret;

    fret = ak099xx_soft_reset();

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    /* Read WIA */
    fret = AKH_RxData(AKM_ST_MAG, AK099XX_REG_WIA1, i2cData, 2);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    /* Store device id (actually, it is company id.) */
    g_dev = (((uint16_t)i2cData[1] << 8) | i2cData[0]);

#if defined(AKM_MAGNETOMETER_AK09911) || defined(AKM_MAGNETOMETER_AK09912)
    /* Read FUSE ROM value */
    fret = ak099xx_set_mode(AK099XX_MODE_FUSE_ACCESS);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    fret = AKH_RxData(AKM_ST_MAG, AK099XX_FUSE_ASAX, g_asa, 3);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    fret = ak099xx_set_mode(AK099XX_MODE_POWER_DOWN);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

#else
    /* Other device does not have ASA. */
    g_asa[0] = 128;
    g_asa[1] = 128;
    g_asa[2] = 128;
#endif

    /* Calculate coeff which converts from raw to micro-tesla unit. */
#if defined(AKM_MAGNETOMETER_AK09911)
    /* The equation is: H_adj = H_raw x (ASA / 128 + 1)
     * Convert from LSB to micro tesla in Q16, multiply (SENS x 2^16)
     * Simplify the equation: coeff = ((ASA + 128) x SENS x 2^16) >> 7
     * In case of AK09911, SENS = 0.6.
     * So coeff = ((ASA + 128) x 39322) >> 7 */
    g_raw_to_micro_q16[0] = ((int32_t)(g_asa[0] + 128) * SENS_IN_Q16) >> 7;
    g_raw_to_micro_q16[1] = ((int32_t)(g_asa[1] + 128) * SENS_IN_Q16) >> 7;
    g_raw_to_micro_q16[2] = ((int32_t)(g_asa[2] + 128) * SENS_IN_Q16) >> 7;
#elif defined(AKM_MAGNETOMETER_AK09912)
    /* The equation is: H_adj = H_raw x ((ASA - 128) / 256 + 1)
     * To convert micro tesla in Q16, multiply (SENS x 2^16)
     * Simplify the equation: coeff = ((ASA + 128) x SENS x 2^16) >> 8
     * In case of AK09912, SENS = 0.15
     * So coeff = ((ASA + 128) x 9830) >> 8 */
    g_raw_to_micro_q16[0] = ((int32_t)(g_asa[0] + 128) * SENS_IN_Q16) >> 8;
    g_raw_to_micro_q16[1] = ((int32_t)(g_asa[1] + 128) * SENS_IN_Q16) >> 8;
    g_raw_to_micro_q16[2] = ((int32_t)(g_asa[2] + 128) * SENS_IN_Q16) >> 8;
#else
    /* AK09913/AK09915/AK09916 does not have ASA register. It means that user
     * does not need to write adjustment equation.
     */
    g_raw_to_micro_q16[0] = SENS_IN_Q16;
    g_raw_to_micro_q16[1] = SENS_IN_Q16;
    g_raw_to_micro_q16[2] = SENS_IN_Q16;
#endif

    /* axis conversion parameter */
    g_mag_axis_order[0] = axis_order[0];
    g_mag_axis_order[1] = axis_order[1];
    g_mag_axis_order[2] = axis_order[2];
    g_mag_axis_sign[0] = axis_sign[0];
    g_mag_axis_sign[1] = axis_sign[1];
    g_mag_axis_sign[2] = axis_sign[2];
    return AKM_SUCCESS;
}

int16_t ak099xx_get_info(struct AKS_DEVICE_INFO *info)
{
    AKS_MyStrcpy(info->name, DEVICE_NAME, AKS_INFO_NAME_SIZE);
    info->parameter[0] = DEVICE_NUMBER;
    info->parameter[1] = g_dev;
    info->parameter[2] = g_asa[0];
    info->parameter[3] = g_asa[1];
    info->parameter[4] = g_asa[2];

    return AKM_SUCCESS;
}

int16_t ak099xx_start(const int32_t interval_us)
{
    int16_t ret;

#if defined(AKM_MAGNETOMETER_AK09912) || defined(AKM_MAGNETOMETER_AK09915)
    /* Set NSF */
    uint8_t i2cData;

    i2cData = AK0991X_NSF_VAL;
    ret = AKH_TxData(AKM_ST_MAG, AK099XX_REG_CNTL1, &i2cData, 1);

    if (ret != AKM_SUCCESS) {
        return ret;
    }
#endif

    if (0 > interval_us) {
        /* Single Measurement */
        ret = ak099xx_set_mode(AK099XX_MODE_SNG_MEASURE);
    } else if (5000 > interval_us) {
        /* Out of range */
        ret = AKM_ERR_INVALID_ARG;
    } else if (10000 > interval_us) {
        /* 100 - 200 Hz */
        ret = ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE5);
    } else if (20000 > interval_us) {
        /* 50 - 100 Hz */
        ret = ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE4);
    } else if (50000 > interval_us) {
        /* 20 - 50 Hz */
        ret = ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE3);
    } else if (100000 > interval_us) {
        /* 10 - 20 Hz */
        ret = ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE2);
    } else {
        /* 10 Hz or slower */
        ret = ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE1);
    }

    return ret;
}

int16_t ak099xx_stop(void)
{
    return ak099xx_set_mode(AK099XX_MODE_POWER_DOWN);
}

int16_t ak099xx_check_rdy(const int32_t timeout_us)
{
    uint8_t i2cData;
    int16_t fret;

    /* Check DRDY bit of ST1 register */
    fret = AKH_RxData(AKM_ST_MAG, AK099XX_REG_ST1, &i2cData, 1);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    /* AK09911/09912/09913 has only one data.
     * So, return is 0 or 1. */
    return (i2cData & 0x01);
}

int16_t ak099xx_get_data(
    struct AKM_SENSOR_DATA *data,
    uint8_t                *num)
{
    uint8_t i2cData[AK099XX_BDATA_SIZE];
    int16_t tmp;
    int16_t fret;
    uint8_t i;

    /* check arg */
    if (*num < 1) {
        return AKM_ERR_INVALID_ARG;
    }

    /* Read data */
    fret = AKH_RxData(
            AKM_ST_MAG, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

    if (fret != AKM_SUCCESS) {
        return fret;
    }

    for (i = 0; i < 3; i++) {
        /* convert to int16 data */
        tmp = (int16_t)(((uint16_t)i2cData[i * 2 + 2] << 8)
                        | i2cData[i * 2 + 1]);
        /* multiply ASA and convert to micro tesla in Q16 */
        data->u.v[i] = tmp * g_raw_to_micro_q16[i];
    }

    AKS_ConvertCoordinate(data->u.v, g_mag_axis_order, g_mag_axis_sign);

    data->stype = AKM_ST_MAG;
    data->time_ns = (int64_t)HAL_GetTick() * (int64_t)1000000;
    data->status[0] = i2cData[0];
    data->status[1] = i2cData[8];
    *num = 1;
    return AKM_SUCCESS;
}

int16_t ak099xx_get_mode(
    uint8_t                *mode)
{
    uint8_t i2cData;
    int16_t fret;

    /* Read data */
    fret = AKH_RxData(
            AKM_ST_MAG, AK099XX_REG_CNTL2, &i2cData, 1);

    if (fret != AKM_SUCCESS) {
        return fret;
    }
    *mode = i2cData&0x1f;

    return AKM_SUCCESS;
}

int16_t ak099xx_get_whoami(
    uint16_t                *whoami)
{
    uint8_t i2cData[2];
    int16_t fret;

    /* Read data */
    fret = AKH_RxData(
            AKM_ST_MAG, AK099XX_REG_WIA1, i2cData, 2);

    if (fret != AKM_SUCCESS) {
        return fret;
    }
    *whoami = (i2cData[1]<<8)|i2cData[0];

    return AKM_SUCCESS;
}

/*
 * Include selftest function
 */
#if defined(AKM_MAGNETOMETER_AK09911)
#include "aks_mag_ak09911_selftest.c"
#elif defined(AKM_MAGNETOMETER_AK09912)
#include "aks_mag_ak09912_selftest.c"
#elif defined(AKM_MAGNETOMETER_AK09913)
#include "aks_mag_ak09913_selftest.c"
#elif defined(AKM_MAGNETOMETER_AK09915)
#include "aks_mag_ak09915_selftest.c"
#elif defined(AKM_MAGNETOMETER_AK09916)
#include "aks_mag_ak09916_selftest.c"
#endif
