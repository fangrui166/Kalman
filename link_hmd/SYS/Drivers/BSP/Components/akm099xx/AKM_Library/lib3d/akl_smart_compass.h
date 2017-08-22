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
#ifndef INCLUDE_AKL_SMART_COMPASS_H
#define INCLUDE_AKL_SMART_COMPASS_H

/* Root file of AKL module. */
#include "AKL_APIs.h"

/* Headers of SmartCompass Library */
#include "libSmartCompass/AKCertification.h"
#include "libSmartCompass/AKConfigure.h"
#include "libSmartCompass/AKDecomp.h"
#include "libSmartCompass/AKDirection6D.h"
#include "libSmartCompass/AKHDOE.h"
#include "libSmartCompass/AKHFlucCheck.h"
#include "libSmartCompass/AKMDevice.h"
#include "libSmartCompass/AKManualCal.h"
#include "libSmartCompass/AKVersion.h"

#ifndef AKM_DISABLE_DOEPLUS
#include "libSmartCompass/AKDOEPlus.h"
#include "libSmartCompass/AKMDeviceF.h"
#endif

#if defined(AKM_MAGNETOMETER_AK8963)
/*! The device part number. */
#define AKM_DEVICE      8963
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE  8
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 9830.f)

#elif defined(AKM_MAGNETOMETER_AK09911)
/*! The device part number. */
#define AKM_DEVICE       9911
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE   9
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 39322.f)

#elif defined(AKM_MAGNETOMETER_AK09912)
/*! The device part number. */
#define AKM_DEVICE       9912
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE   9
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 9830.f)

#elif defined(AKM_MAGNETOMETER_AK09913)
/*! The device part number. */
#define AKM_DEVICE       9913
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE   9
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 9830.f)

#elif defined(AKM_MAGNETOMETER_AK09915)
/*! The device part number. */
#define AKM_DEVICE       9915
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE   9
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 9830.f)

#elif defined(AKM_MAGNETOMETER_AK09916)
/*! The device part number. */
#define AKM_DEVICE       9916
/*! The length of measurement and status data. */
#define AKM_BDATA_SIZE   10
/*! Conversion from uT to LSB */
#define AKM_SENSITIVITY  (1.f / 9830.f)
#endif

/*! Parameter for #AKSC_HFlucCheck function. */
#define HFLUCV_TH          2500
#define THETAFILTER_SCALE  4128

/*
 * The following prefix joined this order.
 * e.g. A pointer to a struct is: 'ps_foo'
 *      An array of struct is   : 'sa_bar'
 * p_ : pointer
 * v_ : vector (a special type of struct)
 * s_ : struct
 * a_ : array
 * m_ : integer/float/enum etc..
 */

/*!
 * A parameter struct which is saved to non-volatile area for warm start up.
 */
struct AKL_NV_PRMS {
    /*! This value is used to identify the data area is AKL_NV_PRMS.
     * This value should be #AKL_NV_MAGIC_NUMBER. */
    uint32     magic;

    int16vec   va_hsuc_ho;                  /*!< fine offset of magnetic vector */
    int16vec   va_hflucv_href;              /*!< rough value of magnetic vector */
    AKSC_HDST  a_hsuc_hdst;                 /*!< status of magnetic offset */
    int32vec   va_hsuc_hbase;               /*!< rough offset of magnetic vector */
#ifndef AKM_DISABLE_DOEPLUS
    AKSC_FLOAT a_doep_prms[AKSC_DOEP_SIZE]; /*!< a parameter for DOEPlus */
#endif
#ifdef AKM_ENABLE_PDC
    uint8      a_pdc[AKL_PDC_SIZE];         /*!< a parameter for PDC */
#endif
};

/*! A parameter structure which is needed for SmartCompass Library. */
struct AKL_SCL_PRMS {
    struct AKL_NV_PRMS            *ps_nv;
    uint32_t                      init;
    struct AKL_CERTIFICATION_INFO s_cert;

    /* Variables for magnetic sensor. */
    int32vec                      v_ho32;
    int16vec                      v_hs;
    AKSC_HFLUCVAR                 s_hflucv;
    /* base */
    int32vec                      v_hbase;

    /* Variables for DecompS3. */
    int16                         m_hn;
    int16                         m_hnave;
    int16vec                      v_asa;
    uint8                         *pa_pdcptr;

    /* Variables for HDOE. */
    AKSC_HDOEVAR                  s_hdoev;
    AKSC_HDST                     m_hdst;

    /* Variables for formation change */
    uint8                         m_form;
    uint8                         m_maxForm;
    uint8                         m_curForm;

    /* vector collection */
    /*! Magnetic vector (offset subtracted) */
    int16vec                      v_hvec;
    /*! An offset of magnetic vector */
    int16vec                      v_ho;
    /*! A buffer of raw magnetic vector (with offset) */
    int16vec                      va_hdata[AKSC_HDATA_SIZE];

    /* time stamp collection */
    /*! Time stamp for 'formation chage' event. */
    int64_t                       m_ts_ev_form;
    /*! Time stamp for 'HDOE' event. */
    int64_t                       m_ts_ev_doe;
    /*! Time stamp for v_hvec */
    int64_t                       m_ts_hvec;

#ifndef AKM_DISABLE_DOEPLUS
    /* Variables for DOEPlus. */
    AKSC_DOEPVAR                  *ps_doep_var;
    int16                         m_doep_lv;
    int16vec                      va_hdata_plus[AKSC_HDATA_SIZE];
#endif

    /*! Layout pattern  number */
    int16_t                       m_pat;
};
#endif /* INCLUDE_AKL_SMART_COMPASS_H */
