/**
 ******************************************************************************
 * @file    AKM099XX_MAG_driver_HL.h
 * @author  htc BSP Team
 * @version V2.0.0
 * @date    10-November-2016
 * @brief   This file contains definitions for the AKM099XX_MAG_driver_HL.h firmware driver
 ******************************************************************************
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
#ifndef __AKM099XX_MAG_DRIVER_HL_H
#define __AKM099XX_MAG_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "magnetometer.h"
/* Include magnetic sensor component drivers. */

/*export AKM099xx magnetic sensor driver handle*/
extern MAGNETO_Drv_t AKM099XXDrv;

/*define AKM099XX magnetic sensor special data in here.
**now AKM099XX data is not use
*/
typedef struct
{
	uint8_t dummy;
} AKM099XX_Data_t;

typedef enum
{
	/*calculate in micro-seconds*/
	AKM099XX_MAG_ODR_10Hz	= 100000,
	AKM099XX_MAG_ODR_20Hz	= 50000,
	AKM099XX_MAG_ODR_50Hz	= 20000,
	AKM099XX_MAG_ODR_100Hz	= 10000,
	AKM099XX_MAG_ODR_200Hz	= 5000,
} AKM099XX_MAG_ODR_t;

#ifdef __cplusplus
}
#endif

#endif /* __AKM099XX_MAG_DRIVER_HL_H */

