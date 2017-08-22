/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_proximity.h
 * @author  hTC BSP Team
 * @version V1.0.0
 * @date    31-August-2016
 * @brief   This file contains definitions for the x_nucleo_iks01a1_proximity.c
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
#ifndef __X_NUCLEO_IKS01A1_PROXIMITY_H
#define __X_NUCLEO_IKS01A1_PROXIMITY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "x_nucleo_iks01a1.h"
#include "proximity.h"

/** X_NUCLEO_IKS01A1_PROXIMITY_Public_Defines Public defines**/

#define PROXIMITY_SENSORS_MAX_NUM 1

typedef enum
{
	PROXIMITY_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
	EPL88051,                      				/* Default on board. */
} PROXIMITY_ID_t;

/**  X_NUCLEO_IKS01A1_PROXIMITY_Public_Function_Prototypes Public function prototypes**/

DrvStatusTypeDef BSP_PROXIMITY_Init( PROXIMITY_ID_t id, void **handle );
DrvStatusTypeDef BSP_PROXIMITY_DeInit( void **handle );
DrvStatusTypeDef BSP_PROXIMITY_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_PROXIMITY_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_PROXIMITY_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROXIMITY_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROXIMITY_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_PROXIMITY_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_PROXIMITY_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_PROXIMITY_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_PROXIMITY_Get_Proximity( void *handle, uint8_t *proximity );

#ifdef __cplusplus
}
#endif

#endif /* __X_NUCLEO_IKS01A1_PROXIMITY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
