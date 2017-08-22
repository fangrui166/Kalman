/******************************************************************************
 *
 *  $Id: AKDirection9D.h 338 2015-09-18 11:32:23Z yamada.rj $
 *
 * -- Copyright Notice --
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
 * -- End Asahi Kasei Microdevices Copyright Notice --
 *
 ******************************************************************************/
#ifndef AKSC_INC_AKDIRECTION9D_H
#define AKSC_INC_AKDIRECTION9D_H

#include "AKMDevice.h"

//========================= Type declaration  ===========================//
typedef int32 *D9D_Handle;

//========================= Constant definition =========================//
#define AKSC_GDATA_SIZE         64 // Buffer size for gyro offset calculation.

//========================= Prototype of Function =======================//
AKLIB_C_API_START

int16 AKSC_GetSizeOfD9DObjectIn32BitWord(void);

int16 AKSC_GetGyroOffsetAuto(      //(o)  : Whether offset calculation is enabled(return:1) or not(return:0).
    const int16vec gdata[],        //(i)   : Buffer for angular rate vector. Index 0 is earlier data.
    const int16    gNave,          //(i)   : The number of gyro data to be averaged. gNave must be 1, 2, 4, 8, 16, 32, 64.
    int16vec       *goffset        //(i/o) : Gyro offset
);

int16 AKSC_InitDirection9D(        //(o)    : success(1), failure(0)
    const D9D_Handle hD9D          //(i)   : AKSC_Direction9D handle
);

int16 AKSC_Direction9D(            //(o)    :
    const D9D_Handle hD9D,         //(i)   : AKSC_Direction9D handle
    const uint8      licenser[],   //(i)   : Licenser
    const uint8      licensee[],   //(i)   : Licensee
    const int16      key[],        //(i)   : Key
    const int16vec   *h,           //(i)   : Geomagnetic vector (offset and sensitivity are compensated)
    const int16vec   *a,           //(i)   : Acceleration vector (offset and sensitivity are compensated)
    const int16vec   *g,           //(i)   : Angular rate vector (offset and sensitivity are compensated)
    const int16      hdt,          //(i)   : Delta time of geomagnetic data in msec (Q4)
    const int16      adt,          //(i)   : Delta time of acceleration data in msec (Q4)
    const int16      gdt,          //(i)   : Delta time of gyroscope data in msec (Q4)
    const int16vec   *dvec,        //(i)   : A vector to define reference axis of the azimuth on the terminal coordinate system
    const I16MATRIX  *hlayout,     //(i)   : Layout matrix for geomagnetic vector
    const I16MATRIX  *alayout,     //(i)   : Layout matrix for acceleration vector
    const I16MATRIX  *glayout,     //(i)   : Layout matrix for angular rate vector
    int16            *theta,       //(o)   : Azimuth direction (degree)
    int16            *delta,       //(o)   : The inclination (degree)
    int16            *hr,          //(o)   : Geomagnetic vector size
    int16            *hrhoriz,     //(o)   : Horizontal element of geomagnetic vector
    int16            *ar,          //(o)   : Acceleration vector size
    int16            *phi180,      //(o)   : Pitch angle (-180 to +180 degree)
    int16            *phi90,       //(o)   : Pitch angle (-90 to +90 degree)
    int16            *eta180,      //(o)   : Roll angle  (-180 to +180 degree)
    int16            *eta90,       //(o)   : Roll angle  (-90 to +90 degree)
    I16MATRIX        *mat,         //(o)   : Rotation matrix
    I16QUAT          *quat,        //(o)   : Rotation Quaternion
    int16vec         *gravity,     //(o)   : Gravity
    int16vec         *linacc       //(o)   : Linear acceleration
);
AKLIB_C_API_END
#endif
