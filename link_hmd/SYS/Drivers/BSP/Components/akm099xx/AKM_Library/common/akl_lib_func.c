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
#include "akl_lib_func.h"
#include "AKS_APIs.h"
#include "AKH_APIs.h"
#include "AKM_CustomerSpec.h"
#include <math.h>
#include <string.h>
#include "cmsis_os.h"


/**
 * @brief A simple implementation of string copy function.
 * This is really simple, so argument check is not done in the function.
 * Wrong parameter may destroy memory area.
 *
 * @param dst A pointer to destination buffer.
 * @param src A pointer to an original string.
 * @param dst_len The size of destination buffer.
 */
static void string_copy(
    char          dst[],
    const char    src[],
    const int16_t dst_len)
{
    int16_t i = 0;

    while (src[i] != '\0') {
        dst[i] = src[i];
        i++;

        if (i >= dst_len) {
            i--;
            break;
        }
    }

    dst[i] = '\0';
}

void print_version(void)
{
    struct AKL_LIBRARY_INFO info;

    AKL_GetLibraryInfo(&info);
    AKH_Print("AKM sample program for Non-OS package.\n");
    AKH_Print("  INFO: Library for AK%d\n",
              info.partno);
    AKH_Print("  INFO: Version: %d.%d.%d.%d.%d\n",
              info.major,
              info.minor,
              info.revision,
              info.datecode,
              info.variation);
#ifdef DEBUG
    AKH_Print("  DBG : AKM_CUSTOM_NUM_FORM=%d\n", AKM_CUSTOM_NUM_FORM);
    AKH_Print("  DBG : Parameter size=%d\n",
              AKL_GetParameterSize(AKM_CUSTOM_NUM_FORM));
    AKH_Print("  DBG : NV data size=%d\n",
              AKL_GetNVdataSize(AKM_CUSTOM_NUM_FORM));
#endif
    AKH_Print("\n");
}

int16_t load_and_start(struct AKL_SCL_PRMS *prm)
{
    uint8_t  *nv_data;
    uint16_t nvSz;
    int16_t  fret;
#ifdef AKM_ENABLE_PDC
    uint8_t pdc[AKL_PDC_SIZE];
#endif

    /* alloc nv data area */
    nvSz = AKL_GetNVdataSize((uint8_t)AKM_CUSTOM_NUM_FORM);

    if (nvSz == 0U) {
        nv_data = NULL;
        fret = AKM_ERROR;
        goto LOAD_QUIT;
    }

    nv_data = (uint8_t *)pvPortMalloc((size_t)nvSz);

    /* load data from storage */
    if (nv_data != NULL) {
        fret = AKH_LoadParameter(nv_data, nvSz);

        if (fret != AKM_SUCCESS) {
            free(nv_data);
            nv_data = NULL;
        }
    }

    /* AKL can accept NULL pointer for nv_data */
    fret = AKL_StartMeasurement(prm, nv_data);

    if (fret != AKM_SUCCESS) {
        goto LOAD_QUIT;
    }

#ifdef AKM_ENABLE_PDC
    /* Use customized PDC parameter */
    load_PDC_parameter(pdc);
    fret = AKL_SetPDC(prm, pdc, 0U);
#endif

LOAD_QUIT:

    if (nv_data != NULL) {
        vPortFree(nv_data);
    }

    return fret;
}

int16_t stop_and_save(struct AKL_SCL_PRMS *prm)
{
    uint8_t  *nv_data;
    uint16_t nvSz;
    int16_t  fret;

    /* alloc nv data area */
    nvSz = AKL_GetNVdataSize((uint8_t)AKM_CUSTOM_NUM_FORM);

    if (nvSz == 0U) {
        nv_data = NULL;
        fret = AKM_ERROR;
        goto SAVE_QUIT;
    }

    nv_data = (uint8_t *)pvPortMalloc((size_t)nvSz);

    /* AKL can accept NULL pointer for nv_data */
    fret = AKL_StopMeasurement(prm, nv_data);

    if (fret != AKM_SUCCESS) {
        goto SAVE_QUIT;
    }

    /* store data to storage */
    if (nv_data != NULL) {
        fret = AKH_SaveParameter(nv_data, nvSz);
    }

SAVE_QUIT:

    if (nv_data != NULL) {
        vPortFree(nv_data);
    }

    return fret;
}

/**
 * @brief Set certification information to enable DOE function.
 * This function sets LICENSER and LICENSEE string, then
 * get hardware information from driver.
 *
 * @param info A pointer to #AKL_CERTIFICATION_INFO struct.
 *
 * @return When function succeeds, #AKM_SUCCESS is returned.
 */
static int16_t set_certification_info(struct AKL_CERTIFICATION_INFO *info)
{
    struct AKS_DEVICE_INFO dev_info;
    uint8_t                n;
    int16_t                ret;

    string_copy((char *)info->a_licenser,
                AKM_CUSTOM_LICENSER, AKL_CI_MAX_CHARSIZE + 1);
    string_copy((char *)info->a_licensee,
                AKM_CUSTOM_LICENSEE, AKL_CI_MAX_CHARSIZE + 1);

    n = 1U;
    ret = AKS_GetDeviceInfo(AKM_ST_MAG, &dev_info, &n);

    if (AKM_SUCCESS != ret) {
        return ret;
    }

    info->a_key[0] = (int16_t)dev_info.parameter[0];
    info->a_key[1] = (int16_t)dev_info.parameter[1];
    info->a_key[2] = (int16_t)dev_info.parameter[2];
    info->a_key[3] = (int16_t)dev_info.parameter[3];
    info->a_key[4] = (int16_t)dev_info.parameter[4];

    info->a_key[5] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_X;
    info->a_key[6] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_Y;
    info->a_key[7] = (int16_t)AKM_CUSTOM_MAG_AXIS_ORDER_Z;
    info->a_key[8] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_X;
    info->a_key[9] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_Y;
    info->a_key[10] = (int16_t)AKM_CUSTOM_MAG_AXIS_SIGN_Z;

    return AKM_SUCCESS;
}

int16_t library_init(struct AKL_SCL_PRMS **prm)
{
    struct   AKL_CERTIFICATION_INFO info;
    uint16_t                        prmSz;
    int16_t                         fret;

    /* alloc library data parameter */
    prmSz = AKL_GetParameterSize((uint8_t)AKM_CUSTOM_NUM_FORM);

    if (prmSz == 0U) {
        *prm = NULL;
        fret = AKM_ERROR;
        goto EXIT_LIBRARY_INIT;
    }

    *prm = (struct AKL_SCL_PRMS *)pvPortMalloc((size_t)prmSz);

    if (*prm == NULL) {
        fret = AKM_ERROR;
        goto EXIT_LIBRARY_INIT;
    }

    memset(*prm, 0, (size_t)prmSz);

    /* certification */
    fret = set_certification_info(&info);

    if (fret != AKM_SUCCESS) {
        goto EXIT_LIBRARY_INIT;
    }

    /* Initialize AKM library. */
    fret = AKL_Init(*prm, &info, (uint8_t)AKM_CUSTOM_NUM_FORM);

    if (fret != AKM_SUCCESS) {
        goto EXIT_LIBRARY_INIT;
    }

    return AKM_SUCCESS;

EXIT_LIBRARY_INIT:

    if (*prm != NULL) {
        vPortFree(*prm);
        *prm = NULL;
    }

    return fret;
}
