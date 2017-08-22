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
#ifndef INCLUDE_AKM_CONFIG_H
#define INCLUDE_AKM_CONFIG_H

/*** Define one of the definition in your make file. ***/
/* AKM_MAGNETOMETER_AK8963  */
/* AKM_MAGNETOMETER_AK09911 */
/* AKM_MAGNETOMETER_AK09912 */
/* AKM_MAGNETOMETER_AK09913 */
//#define AKM_MAGNETOMETER_AK09915
#define AKM_MAGNETOMETER_AK09916C
/* AKM_MAGNETOMETER_AK09916D */
/* AKM_ACCELEROMETER_ADXL34X */
/* AKM_ACCELEROMETER_DUMMY */
/* AKM_GYROSCOPE_L3G4200D */
/* AKM_GYROSCOPE_DUMMY */

#if defined(AKM_MAGNETOMETER_AK09911)
#define AKM_MAGNETOMETER_AK099XX
#elif defined(AKM_MAGNETOMETER_AK09912)
#define AKM_MAGNETOMETER_AK099XX
#elif defined(AKM_MAGNETOMETER_AK09913)
#define AKM_MAGNETOMETER_AK099XX
#elif defined(AKM_MAGNETOMETER_AK09915)
#define AKM_MAGNETOMETER_AK099XX
#elif defined(AKM_MAGNETOMETER_AK09916C)
#define AKM_MAGNETOMETER_AK09916
#define AKM_MAGNETOMETER_AK099XX
#elif defined(AKM_MAGNETOMETER_AK09916D)
#define AKM_MAGNETOMETER_AK09916
#define AKM_MAGNETOMETER_AK099XX
#endif

/* use akm lib3d */
#define AKM3D

/* Enable PDC */
/* Please define AKM_ENABLE_PDC */

/* Disable DOEPlus function */
#define AKM_DISABLE_DOEPLUS

/* Disable D9D function */
#define AKM_DISABLE_D9D

/* Low noise mode for AK09915 */
/* Please define AK09915_USE_LOW_NOISE*/

/* Enable both DOE and DOEaG */
#define AKM_ENABLE_BOTH_DOE_DOEAG

#endif /* INCLUDE_AKM_CONFIG_H */
