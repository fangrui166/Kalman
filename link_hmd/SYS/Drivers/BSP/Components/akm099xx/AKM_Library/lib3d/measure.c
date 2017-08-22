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
#include "measure.h"

/*****************************************************************************/
void InitAKSCPRMS(
    struct AKL_SCL_PRMS *prms
)
{
    /* Sensitivity */
    prms->v_hs.u.x = AKSC_HSENSE_TARGET;
    prms->v_hs.u.y = AKSC_HSENSE_TARGET;
    prms->v_hs.u.z = AKSC_HSENSE_TARGET;
    /* ASA */
    prms->v_asa.u.x = 0x80;
    prms->v_asa.u.y = 0x80;
    prms->v_asa.u.z = 0x80;

    /* HDOE */
    prms->m_hdst = AKSC_HDST_UNSOLVED;

    /* (m_hdata is initialized with AKSC_InitDecomp) */
    prms->m_hnave = AKM_DEFAULT_HNAVE;

    /* Formation */
    prms->m_form = prms->m_curForm;
}

/*****************************************************************************/
void SetDefaultNV(
    struct AKL_NV_PRMS *nv,
    const uint8        num_formation
)
{
    int16 i;
#ifndef AKM_DISABLE_DOEPLUS
    int16 j;
#endif
#ifdef AKM_ENABLE_PDC
    int16 k;
#endif

#ifdef AKM_ENABLE_PDC
    /* UNIT matrix for PDC */
    const uint8 U_PDC[AKL_PDC_SIZE] = {
        32, 82, 186, 174, 49, 255, 255, 96, 214,
        55, 231, 85, 38, 206, 255, 242, 255, 255,
        127, 154, 191, 252, 255, 255, 9, 38, 255
    };
#endif

    /* Set parameter to HDST, HO, HREF, HBASE */
    for (i = 0; i < num_formation; i++) {
        nv->a_hsuc_hdst = AKSC_HDST_UNSOLVED;
        nv->va_hsuc_ho.u.x = 0;
        nv->va_hsuc_ho.u.y = 0;
        nv->va_hsuc_ho.u.z = 0;
        nv->va_hflucv_href.u.x = 0;
        nv->va_hflucv_href.u.y = 0;
        nv->va_hflucv_href.u.z = 0;
        nv->va_hsuc_hbase.u.x = 0;
        nv->va_hsuc_hbase.u.y = 0;
        nv->va_hsuc_hbase.u.z = 0;
#ifndef AKM_DISABLE_DOEPLUS
        for (j = 0; j < AKSC_DOEP_SIZE; j++) {
            nv->a_doep_prms[j] = (AKSC_FLOAT)(0.);
        }
#endif
#ifdef AKM_ENABLE_PDC
        /* Set unit matrix */
        for (k = 0; k < AKL_PDC_SIZE; k++) {
            nv->a_pdc[k] = U_PDC[k];
        }
#endif
        nv++;
    }
}

/*****************************************************************************/
int16 InitMeasure(
    struct AKL_SCL_PRMS *prms
)
{
    prms->m_form = 0;

    /* Restore the value when succeeding in estimating of HOffset. */
    prms->v_ho = prms->ps_nv[prms->m_form].va_hsuc_ho;
    prms->v_ho32.u.x = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.x;
    prms->v_ho32.u.y = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.y;
    prms->v_ho32.u.z = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.z;
    prms->m_hdst = prms->ps_nv[prms->m_form].a_hsuc_hdst;
    prms->v_hbase = prms->ps_nv[prms->m_form].va_hsuc_hbase;

    /* Initialize the decompose parameters */
    AKSC_InitDecomp(prms->va_hdata);

    /* Initialize HDOE parameters */
    AKSC_InitHDOEProcPrmsS3(
        &prms->s_hdoev,
        1,
        &prms->v_ho,
        prms->m_hdst
    );

    AKSC_InitHFlucCheck(
        &(prms->s_hflucv),
        &(prms->ps_nv[prms->m_form].va_hflucv_href),
        HFLUCV_TH
    );

#ifndef AKM_DISABLE_DOEPLUS
    /* Initialize DOEPlus parameters */
    AKSC_InitDOEPlus(prms->ps_doep_var);
    prms->m_doep_lv = AKSC_LoadDOEPlus(
            prms->ps_nv[prms->m_form].a_doep_prms,
            prms->ps_doep_var);
    AKSC_InitDecomp(prms->va_hdata_plus);
#endif

#ifdef AKM_ENABLE_PDC
    /* Set PDC parameter pointer */
    prms->pa_pdcptr = prms->ps_nv[prms->m_form].a_pdc;
#else
    prms->pa_pdcptr = 0;
#endif

    /* Reset time stamp. */
    prms->m_ts_hvec = 0;
    prms->m_ts_ev_form = 0;
    prms->m_ts_ev_doe = 0;

    return AKRET_PROC_SUCCEED;
}

/*****************************************************************************/
int16 GetMagneticVector(
    const int16         bData[],
    struct AKL_SCL_PRMS *prms,
    const int64_t       ts_mag
)
{
    const int16vec hrefZero = {{0, 0, 0}};

    int16vec have;
    int16    temperature, dor, derr, hofl, cb, dc;
    int32vec preHbase;
    int16    overflow;
    int16    hfluc;
    int16    hdSucc;
    int16    aksc_ret;
    int16    ret;
#ifndef AKM_DISABLE_DOEPLUS
    int16 i;
    int16 doep_ret;
#endif

    have.u.x = 0;
    have.u.y = 0;
    have.u.z = 0;
    temperature = 0;
    dor = 0;
    derr = 0;
    hofl = 0;
    cb = 0;
    dc = 0;

    preHbase = prms->v_hbase;
    overflow = 0;
    ret = AKRET_PROC_SUCCEED;

    /* Subtract the formation suspend counter */
    if (prms->m_ts_ev_form > 0) {
        /* If suspended time exceeds the threshold, reset parameters. */
        if ((ts_mag - prms->m_ts_ev_form) >= AKM_INTERVAL_SUSPEND) {
            /* Restore the value when succeeding in estimating of HOffset. */
            prms->v_ho = prms->ps_nv[prms->m_form].va_hsuc_ho;
            prms->v_ho32.u.x = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.x;
            prms->v_ho32.u.y = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.y;
            prms->v_ho32.u.z = (int32)prms->ps_nv[prms->m_form].va_hsuc_ho.u.z;

            prms->m_hdst = prms->ps_nv[prms->m_form].a_hsuc_hdst;
            prms->v_hbase = prms->ps_nv[prms->m_form].va_hsuc_hbase;

            /* Initialize the decompose parameters */
            AKSC_InitDecomp(prms->va_hdata);

#ifndef AKM_DISABLE_DOEPLUS
            /* Initialize DOEPlus parameters */
            AKSC_InitDOEPlus(prms->ps_doep_var);
            prms->m_doep_lv = AKSC_LoadDOEPlus(
                    prms->ps_nv[prms->m_form].a_doep_prms,
                    prms->ps_doep_var);
            AKSC_InitDecomp(prms->va_hdata_plus);
#endif
#ifdef AKM_ENABLE_PDC
            prms->pa_pdcptr = prms->ps_nv[prms->m_form].a_pdc;
#endif

            /* Initialize HDOE parameters */
            AKSC_InitHDOEProcPrmsS3(
                &prms->s_hdoev,
                1,
                &prms->v_ho,
                prms->m_hdst
            );

            /* Initialize HFlucCheck parameters */
            AKSC_InitHFlucCheck(
                &(prms->s_hflucv),
                &(prms->ps_nv[prms->m_form].va_hflucv_href),
                HFLUCV_TH
            );
            /* Reset timestamp */
            prms->m_ts_ev_form = 0;
        }
    }

    /* Decompose one block data into each Magnetic sensor's data */
    aksc_ret = AKSC_DecompS3(
            AKM_DEVICE,
            bData,
            prms->m_hnave,
            &prms->v_asa,
            prms->pa_pdcptr,
            prms->va_hdata,
            &prms->v_hbase,
            &prms->m_hn,
            &have,
            &temperature,
            &dor,
            &derr,
            &hofl,
            &cb,
            &dc
        );

    if (aksc_ret == 0) {
        return AKRET_PROC_FAIL;
    }

    if (prms->m_form != prms->m_curForm) {
        /* Formation changed! */
        prms->m_form = prms->m_curForm;
        prms->m_ts_ev_form = ts_mag;
        ret |= AKRET_FORMATION_CHANGED;
        return ret;
    }

    /* Check derr */
    if (derr == 1) {
        ret |= AKRET_DATA_READERROR;
        return ret;
    }

    /* Check hofl */
    if (hofl == 1) {
        if (prms->m_ts_ev_form == 0) {
            /* Set a HDOE level as "HDST_UNSOLVED" */
            AKSC_SetHDOELevel(
                &prms->s_hdoev,
                &prms->v_ho,
                AKSC_HDST_UNSOLVED,
                1
            );
            prms->m_hdst = AKSC_HDST_UNSOLVED;
        }

        ret |= AKRET_DATA_OVERFLOW;
        return ret;
    }

    /* Check hbase */
    if (cb == 1) {
        /* Translate HOffset */
        AKSC_TransByHbase(
            &(preHbase),
            &(prms->v_hbase),
            &(prms->v_ho),
            &(prms->v_ho32),
            &overflow
        );

        if (overflow == 1) {
            ret |= AKRET_OFFSET_OVERFLOW;
        }

        /* Set hflucv.href to 0 */
        AKSC_InitHFlucCheck(
            &(prms->s_hflucv),
            &(hrefZero),
            HFLUCV_TH
        );

        if (prms->m_ts_ev_form == 0) {
            AKSC_SetHDOELevel(
                &prms->s_hdoev,
                &prms->v_ho,
                AKSC_HDST_UNSOLVED,
                1
            );
            prms->m_hdst = AKSC_HDST_UNSOLVED;
        }

        ret |= AKRET_HBASE_CHANGED;
        return ret;
    }

    if (prms->m_ts_ev_form == 0) {
        /* Detect a fluctuation of magnetic field. */
        hfluc = AKSC_HFlucCheck(&(prms->s_hflucv), &(prms->va_hdata[0]));

        if (hfluc == 1) {
            /* Set a HDOE level as "HDST_UNSOLVED" */
            AKSC_SetHDOELevel(
                &prms->s_hdoev,
                &prms->v_ho,
                AKSC_HDST_UNSOLVED,
                1
            );
            prms->m_hdst = AKSC_HDST_UNSOLVED;
            ret |= AKRET_HFLUC_OCCURRED;
            return ret;
        } else {
            if ((ts_mag - prms->m_ts_ev_doe) >= AKM_INTERVAL_DOE) {
                prms->m_ts_ev_doe = ts_mag;
#ifndef AKM_DISABLE_DOEPLUS
                /* Compensate Magnetic Distortion by DOEPlus */
                doep_ret = AKSC_DOEPlus(
                        prms->va_hdata,
                        prms->ps_doep_var,
                        &prms->m_doep_lv);

                /* Save DOEPlus parameters */
                if ((doep_ret == 1) && (prms->m_doep_lv == 3)) {
                    AKSC_SaveDOEPlus(
                        prms->ps_doep_var,
                        prms->ps_nv[prms->m_form].a_doep_prms
                    );
                }

                /* Calculate compensated vector for DOE */
                for (i = 0; i < prms->m_hn; i++) {
                    AKSC_DOEPlus_DistCompen(
                        &prms->va_hdata[i],
                        prms->ps_doep_var,
                        &prms->va_hdata_plus[i]);
                }

                /*Calculate Magnetic sensor's offset by DOE */
                hdSucc = AKSC_HDOEProcessS3(
                        prms->s_cert.a_licenser,
                        prms->s_cert.a_licensee,
                        prms->s_cert.a_key,
                        &prms->s_hdoev,
                        prms->va_hdata_plus,
                        prms->m_hn,
                        &prms->v_ho,
                        &prms->m_hdst);
#else
                hdSucc = AKSC_HDOEProcessS3(
                        prms->s_cert.a_licenser,
                        prms->s_cert.a_licensee,
                        prms->s_cert.a_key,
                        &prms->s_hdoev,
                        prms->va_hdata,
                        prms->m_hn,
                        &prms->v_ho,
                        &prms->m_hdst);
#endif

                if (hdSucc == AKSC_CERTIFICATION_DENIED) {
                    return AKRET_CERTIFICATION_ERROR;
                }

                if (hdSucc > 0) {
                    prms->ps_nv[prms->m_form].va_hsuc_ho = prms->v_ho;
                    prms->v_ho32.u.x = (int32)prms->v_ho.u.x;
                    prms->v_ho32.u.y = (int32)prms->v_ho.u.y;
                    prms->v_ho32.u.z = (int32)prms->v_ho.u.z;

                    prms->ps_nv[prms->m_form].a_hsuc_hdst = prms->m_hdst;
                    prms->ps_nv[prms->m_form].va_hflucv_href =
                        prms->s_hflucv.href;
                    prms->ps_nv[prms->m_form].va_hsuc_hbase = prms->v_hbase;
                }
            }
        }
    }

#ifndef AKM_DISABLE_DOEPLUS
    /* Calculate compensated vector */
    AKSC_DOEPlus_DistCompen(&have, prms->ps_doep_var, &have);
#endif

    /* Subtract offset and normalize magnetic field vector. */
    aksc_ret = AKSC_VNorm(
            &have,
            &prms->v_ho,
            &prms->v_hs,
            AKSC_HSENSE_TARGET,
            &prms->v_hvec
        );

    if (aksc_ret == 0) {
        ret |= AKRET_VNORM_ERROR;
        return ret;
    }

    /* update time stamp for hvec */
    prms->m_ts_hvec = ts_mag;

    return AKRET_PROC_SUCCEED;
}
