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
#include "AKL_APIs.h"          /* API decleration */
#include "akl_smart_compass.h" /* Definition of struct */
#include "measure.h"

/*! Identify the nv data. */
#define AKL_NV_MAGIC_NUMBER   (0xdeadbeefU)
#define AKL_INI_MAGIC_NUMBER  (0xbeefcafeU)
/*! The number of default form factor. */
#define AKL_DEFAULT_FORM_NUM  1
/*! 1G (= 9.8 m/s^2) in Q16 format. */
#define ACC_1G_IN_Q16         (9.80665f * 65536.0f)
/*! 1dps in Q16 format. */
#define GYR_1DPS_IN_Q16       (65536.0f)

/*! Convert from AKSC to micro tesla in Q16 format. */
#define MAG_AKSC_TO_Q16(x)    (((float32_t)(x) * 0.06f) * 65536.0f)
/*! Convert from AKSC to SI unit (m/s^2) in Q16 format. */
#define ACC_AKSC_TO_Q16(x)    (((float32_t)(x) * ACC_1G_IN_Q16) / 720.0f)
/*! Convert from SI unit (m/s^2) to AKSC format. */
#define ACC_Q16_TO_AKSC(x)    (((float32_t)(x) * 720.0f) / ACC_1G_IN_Q16)
/*! Convert from degree/second in Q4 (i.e. AKSC) format to Q16 format. */
#define GYR_AKSC_TO_Q16(x)    ((x) * 4096)
/*! Convert from degree/second in Q16 format to AKSC (i.e. Q4) format. */
#define GYR_Q16_TO_AKSC(x)    ((float32_t)(x) / 4096.0f)
/*! Convert from micro second to milli second in Q4 format. */
#define TIME_USEC_TO_AKSC(x)  (((x) * 16) / 1000)

/*! Maximum time [usec] in Q4 format. */
#define MAXTIME_USEC_IN_Q4  (2047000000)

/******************************************************************************/
/***** AKM static function prototype declarations *****************************/
static uint16_t byte_allign(
    const int32_t sz
);

static int16_t akl_setv_mag(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data
);

static int16_t akl_setv_acc(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data
);

static int16_t akl_setv_gyr(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data
);

static int16_t akl_getv_mag(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_acc(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_gyr(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_gravity(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_lacc(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_ori(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
);

static int16_t akl_getv_quat(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[4],
    int32_t                   *status,
    int64_t                   *timestamp
);

/******************************************************************************/
extern uint16_t set_cert(
    struct AKL_SCL_PRMS                 *mem,
    const struct AKL_CERTIFICATION_INFO *info);
/***** AKM static functions ***************************************************/
static uint16_t byte_allign(
    const int32_t sz
)
{
    if (0 >= sz) {
        return (uint16_t)0;
    }

    /* Another method.
     int32_t rem = sz % 4;
     return (rem ? (sz + (4 - rem)) : (sz));
     */
    return ((sz & 0x3) ? ((sz & ~(0x3)) + 0x4) : sz);
}

/*****************************************************************************/
static int16_t akl_setv_mag(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data
)
{
    int32_t   tmp_i32;
    float32_t tmp_f;
    int16_t   reg[3];
    int16_t   ret;
    int16     bData[AKM_BDATA_SIZE];
    int       i;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    for (i = 0; i < 3; i++) {
        tmp_f = (float32_t)(data->u.v[i]) * AKM_SENSITIVITY;
        tmp_i32 = (int32_t)tmp_f;
        /* Limit to 16-bit value */
        if ((tmp_i32 < INT16_MIN) || (INT16_MAX < tmp_i32)) {
            return AKM_ERR_INVALID_ARG;
        }
        reg[i] = (int16)tmp_i32;
    }

    /* Inverse decomp, i.e. compose */
    bData[0] = (int16)(data->status[0]);
    bData[1] = (int16)(reg[0] & 0xFF);
    bData[2] = (int16)((reg[0] >> 8) & 0xFF);
    bData[3] = (int16)(reg[1] & 0xFF);
    bData[4] = (int16)((reg[1] >> 8) & 0xFF);
    bData[5] = (int16)(reg[2] & 0xFF);
    bData[6] = (int16)((reg[2] >> 8) & 0xFF);
#if defined(AKM_MAGNETOMETER_AK8963)
    bData[7] = (int16)(data->status[1]);
#elif defined(AKM_MAGNETOMETER_AK099XX)
    bData[7] = (int16)(0x80);
    bData[8] = (int16)(data->status[1]);
#ifdef AKM_MAGNETOMETER_AK09916
    bData[9] = (int16)mem->m_pat;
#endif
#endif

    ret = GetMagneticVector(
            bData,
            mem,
            data->time_ns);

    /* Check the return value */
    if (ret != AKRET_PROC_SUCCEED) {
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}

/**************************************/
static int16_t akl_setv_acc(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
static int16_t akl_setv_gyr(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA *data)
{
    return AKM_ERR_NOT_SUPPORT;
}

/*****************************************************************************/
static int16_t akl_getv_mag(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    int i;
    float32_t tmp_f;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (status == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif
    if (mem->m_ts_hvec == 0) {
        return AKM_ERR_NOT_YET_CALCULATED;
    }

    /* Convert from SmartCompass to Q16 */
    for (i = 0; i < 3; i++) {
        tmp_f = MAG_AKSC_TO_Q16(mem->v_hvec.v[i]);
        data[i] = (int32_t)tmp_f;
        tmp_f = MAG_AKSC_TO_Q16(
                (float32_t)mem->v_ho.v[i] + (float32_t)mem->v_hbase.v[i]);
        data[i + 3] = (int32_t)tmp_f;
    }

#ifndef AKM_DISABLE_DOEPLUS
    if ((mem->m_hdst == 3) && (mem->m_doep_lv <= 2)) {
        *status = 2;
    } else if ((mem->m_hdst == 2) && (mem->m_doep_lv <= 1)) {
        *status = 1;
    } else {
        *status = (int32_t)mem->m_hdst;
    }
#else
    *status = (int32_t)mem->m_hdst;
#endif
    *timestamp = mem->m_ts_hvec;

    return AKM_SUCCESS;
}

/**************************************/
static int16_t akl_getv_acc(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
static int16_t akl_getv_gyr(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[6],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
static int16_t akl_getv_gravity(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
static int16_t akl_getv_lacc(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
static int16_t akl_getv_ori(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[3],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/**************************************/
int16_t akl_getv_quat(
    const struct AKL_SCL_PRMS *mem,
    int32_t                   data[4],
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    return AKM_ERR_NOT_SUPPORT;
}

/******************************************************************************/
/***** AKM public APIs ********************************************************

 Memory map
  |-----------------------------------|
  | AKL_SCL_PRMS                      |
  |-----------------------------------|
  | NV data (AKL_NV_PRMS) x Formation |
  |-----------------------------------|
  | DOEPlus (AKSC_DOEPVAR)            |
  |-----------------------------------|

 ******************************************************************************/
/***** Function manual is described in header file. ***************************/
uint16_t AKL_GetParameterSize(
    const uint8_t max_form
)
{
    uint8_t  num;
    uint16_t size;
#ifndef AKM_DISABLE_DOEPLUS
    uint32_t tmp_size;
#endif

    if (max_form == 0U) {
        num = AKL_DEFAULT_FORM_NUM;
    } else {
        num = max_form;
    }

    size = byte_allign(sizeof(struct AKL_SCL_PRMS));
    size += AKL_GetNVdataSize(num);
#ifndef AKM_DISABLE_DOEPLUS
    tmp_size = ((uint32_t)AKSC_GetSizeDOEPVar() * sizeof(int32));
    size += (uint16_t)tmp_size;
#endif

    return size;
}

/*****************************************************************************/
uint16_t AKL_GetNVdataSize(
    const uint8_t max_form
)
{
    uint8_t num;

    if (max_form == 0U) {
        num = AKL_DEFAULT_FORM_NUM;
    } else {
        num = max_form;
    }

    return byte_allign(sizeof(struct AKL_NV_PRMS)) * num;
}

/*****************************************************************************/
int16_t AKL_Init(
    struct AKL_SCL_PRMS                 *mem,
    const struct AKL_CERTIFICATION_INFO *cert,
    const uint8_t                       max_form
)
{
#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
    if (cert == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    /* Initialize parameter */
    InitAKSCPRMS(mem);

    /* save number of maximum form. */
    if (max_form == 0U) {
        mem->m_maxForm = AKL_DEFAULT_FORM_NUM;
    } else {
        mem->m_maxForm = max_form;
    }

    if (set_cert(mem, cert) != (uint16_t)AKM_SUCCESS) {
        return AKM_ERR_NOT_SUPPORT;
    }

    /* Set NV data pointer */
    mem->ps_nv = (struct AKL_NV_PRMS *)(
            (uint32_t)mem
            + byte_allign(sizeof(struct AKL_SCL_PRMS)));

#ifndef AKM_DISABLE_DOEPLUS
    /* Set DOEPlus data pointer */
    mem->ps_doep_var = (AKSC_DOEPVAR *)(
            (uint32_t)(mem->ps_nv)
            + AKL_GetNVdataSize(mem->m_maxForm));
#endif
    mem->init = (uint32_t)AKL_INI_MAGIC_NUMBER;

    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_StartMeasurement(
    struct AKL_SCL_PRMS *mem,
    uint8_t             *nv_data
)
{
    struct AKL_NV_PRMS *p_nv;
    struct AKL_NV_PRMS *p_pr;
    uint8_t            i;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    p_nv = (struct AKL_NV_PRMS *)nv_data;
    p_pr = mem->ps_nv;

    if (p_nv == NULL) {
        /* If parameters couldn't be got, set default value. */
        SetDefaultNV(&p_pr[0], mem->m_maxForm);
    } else {
        for (i = 0U; i < mem->m_maxForm; i++) {
            if (p_nv[i].magic == (uint32_t)AKL_NV_MAGIC_NUMBER) {
                /* Copy NV data to mem struct. */
                p_pr[i] = p_nv[i];
            } else {
                SetDefaultNV(&p_pr[i], 1U);
            }
        }
    }

    /* Init SmartCompass library functions. */
    if (InitMeasure(mem) != AKRET_PROC_SUCCEED) {
        return AKM_ERROR;
    }

    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_StopMeasurement(
    const struct AKL_SCL_PRMS *mem,
    uint8_t                   *nv_data
)
{
    struct AKL_NV_PRMS *p_nv;
    struct AKL_NV_PRMS *p_pr;
    uint8_t            i;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    p_nv = (struct AKL_NV_PRMS *)nv_data;
    p_pr = mem->ps_nv;

    if (p_nv != NULL) {
        for (i = 0; i < mem->m_maxForm; i++) {
            /* Copy mem data to NV buffer. */
            p_nv[i] = p_pr[i];
            p_nv[i].magic = (uint32_t)AKL_NV_MAGIC_NUMBER;
        }
    }

    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_SetVector(
    struct AKL_SCL_PRMS          *mem,
    const struct AKM_SENSOR_DATA data[],
    const uint8_t                num
)
{
    uint8_t i;
    int16_t ret;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    for (i = 0U; i < num; i++) {
        switch (data[i].stype) {
        case AKM_ST_MAG:
            if (mem->m_ts_hvec == data[i].time_ns) {
                return AKM_SUCCESS;
            }
            ret = akl_setv_mag(mem, &data[i]);
            break;

        case AKM_ST_ACC:
            ret = akl_setv_acc(mem, &data[i]);
            break;

        case AKM_ST_GYR:
            ret = akl_setv_gyr(mem, &data[i]);
            break;

        default:
            ret = AKM_ERR_NOT_SUPPORT;
            break;
        }

        if (ret != AKM_SUCCESS) {
            return ret;
        }
    }

    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_CalcFusion(
    struct AKL_SCL_PRMS *mem
)
{
    /* Nothing to do */
    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_GetVector(
    const AKM_VECTOR_TYPE     vtype,
    const struct AKL_SCL_PRMS *mem,
    int32_t                   *data,
    uint8_t                   size,
    int32_t                   *status,
    int64_t                   *timestamp
)
{
    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }

    switch (vtype) {
    case AKM_VT_MAG:
        if (AKM_VT_MAG_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_mag(mem, data, status, timestamp);

    case AKM_VT_ACC:
        if (AKM_VT_ACC_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_acc(mem, data, status, timestamp);

    case AKM_VT_GYR:
        if (AKM_VT_GYR_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_gyr(mem, data, status, timestamp);

    case AKM_VT_ORI:
        if (AKM_VT_ORI_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_ori(mem, data, status, timestamp);

    case AKM_VT_GRAVITY:
        if (AKM_VT_GRAVITY_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_gravity(mem, data, status, timestamp);

    case AKM_VT_LACC:
        if (AKM_VT_LACC_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_lacc(mem, data, status, timestamp);

    case AKM_VT_QUAT:
        if (AKM_VT_QUAT_SIZE > size) {
            return AKM_ERR_INVALID_ARG;
        }
        return akl_getv_quat(mem, data, status, timestamp);

    default:
        return AKM_ERR_NOT_SUPPORT;
    }
}

/*****************************************************************************/
void AKL_GetLibraryInfo(
    struct AKL_LIBRARY_INFO *info
)
{
    info->partno = AKSC_GetVersion_Device();
    info->major = AKSC_GetVersion_Major();
    info->minor = AKSC_GetVersion_Minor();
    info->variation = AKSC_GetVersion_Variation();
    info->revision = AKSC_GetVersion_Revision();
    info->datecode = AKSC_GetVersion_DateCode();
}

/*****************************************************************************/
void AKL_ForceReCalibration(
    struct AKL_SCL_PRMS *mem
)
{
    /* Check initialized */
    if (mem->init == (uint32_t)AKL_INI_MAGIC_NUMBER) {
        AKSC_SetHDOELevel(
            &mem->s_hdoev,
            &mem->v_ho, 
            AKSC_HDST_UNSOLVED,
            1
        );
        mem->m_hdst = AKSC_HDST_UNSOLVED;
    }
}

/*****************************************************************************/
int16_t AKL_ChangeFormation(
    struct AKL_SCL_PRMS *mem,
    const uint8_t       formNumber
)
{
    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }
    if (mem->m_maxForm <= formNumber) {
        return AKM_ERR_INVALID_ARG;
    }
    /* Set to struct */
    mem->m_curForm = formNumber;

    return AKM_SUCCESS;
}

/*****************************************************************************/
int16_t AKL_SetPDC(
    const struct AKL_SCL_PRMS *mem,
    const uint8_t             pdc[AKL_PDC_SIZE],
    const uint8_t             formNumber
)
{
#ifdef AKM_ENABLE_PDC
    uint8   *p_pdc;
    uint8_t i;

#ifdef AKL_ARGUMENT_CHECK
    if (mem == NULL) {
        return AKM_ERR_INVALID_ARG;
    }

    if (pdc == NULL) {
        return AKM_ERR_INVALID_ARG;
    }
#endif

    /* Check initialized */
    if (mem->init != (uint32_t)AKL_INI_MAGIC_NUMBER) {
        return AKM_ERR_INVALID_ARG;
    }
    if (mem->m_maxForm <= formNumber) {
        return AKM_ERR_INVALID_ARG;
    }

    p_pdc = mem->ps_nv[formNumber].a_pdc;

    for (i = 0U; i < (uint8_t)AKL_PDC_SIZE; i++) {
        p_pdc[i] = pdc[i];
    }

    return AKM_SUCCESS;
#else
    return AKM_ERR_NOT_SUPPORT;
#endif
}
