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
#define TLIMIT_NO_RST        0x101
#define TLIMIT_NO_RST_READ   0x102

#define TLIMIT_NO_RST_WIA1   0x103
#define TLIMIT_LO_RST_WIA1   0x48
#define TLIMIT_HI_RST_WIA1   0x48
#define TLIMIT_NO_RST_WIA2   0x104
#define TLIMIT_LO_RST_WIA2   0x10
#define TLIMIT_HI_RST_WIA2   0x10

#define TLIMIT_NO_SNG_CNTL2  0x201
#define TLIMIT_NO_SNG_WAIT   0x202

#define TLIMIT_NO_SNG_ST1    0x203
#define TLIMIT_LO_SNG_ST1    1
#define TLIMIT_HI_SNG_ST1    1

#define TLIMIT_NO_SNG_HX     0x204
#define TLIMIT_LO_SNG_HX     -32751
#define TLIMIT_HI_SNG_HX     32751

#define TLIMIT_NO_SNG_HY     0x206
#define TLIMIT_LO_SNG_HY     -32751
#define TLIMIT_HI_SNG_HY     32751

#define TLIMIT_NO_SNG_HZ     0x208
#define TLIMIT_LO_SNG_HZ     -32751
#define TLIMIT_HI_SNG_HZ     32751

#define TLIMIT_NO_SNG_ST2    0x20A
#define TLIMIT_LO_SNG_ST2    0
#define TLIMIT_HI_SNG_ST2    0

#define TLIMIT_NO_SLF_CNTL2  0x20B
#define TLIMIT_NO_SLF_WAIT   0x20C

#define TLIMIT_NO_SLF_ST1    0x20D
#define TLIMIT_LO_SLF_ST1    1
#define TLIMIT_HI_SLF_ST1    1

#define TLIMIT_NO_SLF_RVHX  0x20E
#define TLIMIT_LO_SLF_RVHX  -200
#define TLIMIT_HI_SLF_RVHX  200

#define TLIMIT_NO_SLF_RVHY  0x210
#define TLIMIT_LO_SLF_RVHY  -200
#define TLIMIT_HI_SLF_RVHY  200

#define TLIMIT_NO_SLF_RVHZ  0x212
#define TLIMIT_LO_SLF_RVHZ  -800
#define TLIMIT_HI_SLF_RVHZ  -200

#define TLIMIT_NO_SLF_ST2   0x214
#define TLIMIT_LO_SLF_ST2   0
#define TLIMIT_HI_SLF_ST2   0

/*
 * \result upper_16bit test number
 * \result lower_16bit test result data.
 */
int16_t ak099xx_self_test(int32_t *result)
{
    int16_t fret;
    uint8_t i2cData[AK099XX_BDATA_SIZE];
    int16_t xval_i16, yval_i16, zval_i16;

    /* initialize arg */
    *result = 0;

    /**********************************************************************
     * Step 1
     **********************************************************************/

    /* Soft Reset */
    fret = ak099xx_soft_reset();

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_RST, fret);
        goto SELFTEST_FAIL;
    }

    /* Wait over 100 us */
    AKH_DelayMicro(100);

    /* Read values. */
    fret = AKH_RxData(AKM_ST_MAG, AK099XX_REG_WIA1, i2cData, 2);

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_RST_READ, fret);
        goto SELFTEST_FAIL;
    }

    AKM_FST(TLIMIT_NO_RST_WIA1, i2cData[0], TLIMIT_LO_RST_WIA1,
            TLIMIT_HI_RST_WIA1, result);
    AKM_FST(TLIMIT_NO_RST_WIA2, i2cData[1], TLIMIT_LO_RST_WIA2,
            TLIMIT_HI_RST_WIA2, result);

    /**********************************************************************
     * Step 2
     **********************************************************************/

    /* Set to SNG measurement pattern. */
    fret = ak099xx_set_mode(AK099XX_MODE_SNG_MEASURE);

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_SNG_CNTL2, fret);
        goto SELFTEST_FAIL;
    }

    /* Wait for single measurement. */
    AKH_DelayMilli(10);

    /*
     * Get measurement data from AK09915
     * ST1 + (HXL/H) + (HYL/H) + (HZL/H) + TMPS + ST2 = 9bytes */
    fret = AKH_RxData(
            AKM_ST_MAG, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_SNG_WAIT, fret);
        goto SELFTEST_FAIL;
    }

    /* Convert to 16-bit integer value. */
    xval_i16 = (int16_t)(((uint16_t)i2cData[1]) | ((uint16_t)i2cData[2] << 8));
    yval_i16 = (int16_t)(((uint16_t)i2cData[3]) | ((uint16_t)i2cData[4] << 8));
    zval_i16 = (int16_t)(((uint16_t)i2cData[5]) | ((uint16_t)i2cData[6] << 8));

    AKM_FST(TLIMIT_NO_SNG_ST1, i2cData[0], TLIMIT_LO_SNG_ST1,
            TLIMIT_HI_SNG_ST1, result);
    AKM_FST(TLIMIT_NO_SNG_HX, xval_i16, TLIMIT_LO_SNG_HX,
            TLIMIT_HI_SNG_HX, result);
    AKM_FST(TLIMIT_NO_SNG_HY, yval_i16, TLIMIT_LO_SNG_HY,
            TLIMIT_HI_SNG_HY, result);
    AKM_FST(TLIMIT_NO_SNG_HZ, zval_i16, TLIMIT_LO_SNG_HZ,
            TLIMIT_HI_SNG_HZ, result);
    AKM_FST(TLIMIT_NO_SNG_ST2, i2cData[8], TLIMIT_LO_SNG_ST2,
            TLIMIT_HI_SNG_ST2, result);

    /* Set to self-test mode. */
    fret = ak099xx_set_mode(AK099XX_MODE_SELF_TEST);

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_SLF_CNTL2, fret);
        goto SELFTEST_FAIL;
    }

    /* Wait for self-test measurement. */
    /* Maximum time for measurement is 8.2 ms */
    /* Refer to datasheet p.6 */
    AKH_DelayMilli(9);

    /*
     * Get measurement data from AK09915
     * ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TMPS + ST2 = 9bytes */
    fret = AKH_RxData(
            AKM_ST_MAG, AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

    if (AKM_SUCCESS != fret) {
        *result = AKM_FST_ERRCODE(TLIMIT_NO_SLF_WAIT, fret);
        goto SELFTEST_FAIL;
    }

    /* Convert to 16-bit integer value. */
    xval_i16 = (int16_t)(((uint16_t)i2cData[1]) | ((uint16_t)i2cData[2] << 8));
    yval_i16 = (int16_t)(((uint16_t)i2cData[3]) | ((uint16_t)i2cData[4] << 8));
    zval_i16 = (int16_t)(((uint16_t)i2cData[5]) | ((uint16_t)i2cData[6] << 8));

    AKM_FST(TLIMIT_NO_SLF_ST1, i2cData[0], TLIMIT_LO_SLF_ST1,
            TLIMIT_HI_SLF_ST1, result);
    AKM_FST(TLIMIT_NO_SLF_RVHX, xval_i16, TLIMIT_LO_SLF_RVHX,
            TLIMIT_HI_SLF_RVHX, result);
    AKM_FST(TLIMIT_NO_SLF_RVHY, yval_i16, TLIMIT_LO_SLF_RVHY,
            TLIMIT_HI_SLF_RVHY, result);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, zval_i16, TLIMIT_LO_SLF_RVHZ,
            TLIMIT_HI_SLF_RVHZ, result);
    AKM_FST(TLIMIT_NO_SLF_ST2, i2cData[8], TLIMIT_LO_SLF_ST2,
            TLIMIT_HI_SLF_ST2, result);

    return AKM_SUCCESS;

SELFTEST_FAIL:
    return AKM_ERROR;
}
