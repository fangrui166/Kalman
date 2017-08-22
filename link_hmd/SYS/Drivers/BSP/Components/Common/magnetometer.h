/**
 ******************************************************************************
 * @file    magnetometer.h
 * @author  MEMS Application Team
 * @version V2.0.0
 * @date    10-December-2015
 * @brief   This header file contains the functions prototypes for the
 *          magnetometer driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAGNETOMETER_H
#define __MAGNETOMETER_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "hlog_api.h"
/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup MAGNETOMETER MAGNETOMETER
 * @{
 */

/** @addtogroup MAGNETOMETER_Public_Types MAGNETOMETER Public types
 * @{
 */

/**
 * @brief  MAGNETOMETER driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Axes        ) ( DrvContextTypeDef*, SensorAxes_t* );
  DrvStatusTypeDef ( *Get_AxesRaw     ) ( DrvContextTypeDef*, SensorAxesRaw_t* );
  DrvStatusTypeDef ( *Get_Sensitivity ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_FS          ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_FS          ) ( DrvContextTypeDef*, SensorFs_t );
  DrvStatusTypeDef ( *Set_FS_Value    ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Set_Sensor_Mode    ) ( DrvContextTypeDef*, SensorMode_t );
  DrvStatusTypeDef ( *Sensor_Selftest    ) ( DrvContextTypeDef*);
  DrvStatusTypeDef ( *Get_Accuracy    ) ( DrvContextTypeDef*,uint8_t*);
  DrvStatusTypeDef ( *Set_User_Calibration    ) ( DrvContextTypeDef*,uint8_t enable);
  DrvStatusTypeDef ( *Get_User_Calibration    ) ( DrvContextTypeDef*,uint8_t *,uint8_t);
} MAGNETO_Drv_t;

/**
 * @brief  MAGNETOMETER data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} MAGNETO_Data_t;

#define MAG_HLOG_ENABLE
#ifdef MAG_HLOG_ENABLE
#define mag_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_SENSOR,"[MAG] " fmt, ##__VA_ARGS__)
#define mag_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_SENSOR,"[MAG] " fmt, ##__VA_ARGS__)
#define mag_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_SENSOR,"[MAG] " fmt, ##__VA_ARGS__)
#define mag_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SENSOR,"[MAG] " fmt, ##__VA_ARGS__)
#define mag_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_SENSOR,"[MAG] " fmt, ##__VA_ARGS__)
#else /* MAG_HLOG_ENABLE */
#define mag_emerg(fmt, ...) \
	printf("<0>[SENSOR] [MAG] " fmt, ##__VA_ARGS__)
#define mag_err(fmt, ...) \
	printf("<1>[SENSOR] [MAG] " fmt, ##__VA_ARGS__)
#define mag_warning(fmt, ...) \
	printf("<2>[SENSOR] [MAG] " fmt, ##__VA_ARGS__)
#define mag_info(fmt, ...) \
	printf("<3>[SENSOR] [MAG] " fmt, ##__VA_ARGS__)
#define mag_debug(fmt, ...) \
	printf("<4>[SENSOR] [MAG] " fmt, ##__VA_ARGS__)
#endif /* MAG_HLOG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __MAGNETOMETER_H */
