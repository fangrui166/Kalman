/**
 ******************************************************************************
 * @file    proximity.h
 * @author   hTC BSP Team
 * @version V1.0.0
 * @date    31-August-2015
 * @brief   This header file contains the functions prototypes for the
 *          proximity driver
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
#ifndef __PROXIMITY_H
#define __PROXIMITY_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "component.h"
#include "hlog_api.h"

/**
 * @brief  PROXIMITY driver structure definition
 */
typedef struct
{
	DrvStatusTypeDef ( *Init           ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *DeInit         ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *Sensor_Enable  ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *Sensor_Disable ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *Get_WhoAmI     ) ( DrvContextTypeDef*, uint8_t* );
	DrvStatusTypeDef ( *Check_WhoAmI   ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *Get_Proximity  ) ( DrvContextTypeDef*, uint8_t* );
} PROXIMITY_Drv_t;



/**
 * @brief  PROXIMITY data structure definition
 */
typedef struct
{
	uint8_t proximity;
	void *pComponentData; /* Component specific data. */
	void *pExtData;       /* Other data. */
} PROXIMITY_Data_t;

#define PROX_HLOG_ENABLE
#ifdef PROX_HLOG_ENABLE
#define prox_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_SENSOR,"[PROX] " fmt, ##__VA_ARGS__)
#define prox_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_SENSOR,"[PROX] " fmt, ##__VA_ARGS__)
#define prox_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_SENSOR,"[PROX] " fmt, ##__VA_ARGS__)
#define prox_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SENSOR,"[PROX] " fmt, ##__VA_ARGS__)
#define prox_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_SENSOR,"[PROX] " fmt, ##__VA_ARGS__)
#else /* PROX_HLOG_ENABLE */
#define prox_emerg(fmt, ...) \
	printf("<0>[SENSOR] [PROX] " fmt, ##__VA_ARGS__)
#define prox_err(fmt, ...) \
	printf("<1>[SENSOR] [PROX] " fmt, ##__VA_ARGS__)
#define prox_warning(fmt, ...) \
	printf("<2>[SENSOR] [PROX] " fmt, ##__VA_ARGS__)
#define prox_info(fmt, ...) \
	printf("<3>[SENSOR] [PROX] " fmt, ##__VA_ARGS__)
#define prox_debug(fmt, ...) \
	printf("<4>[SENSOR] [PROX] " fmt, ##__VA_ARGS__)
#endif /* PROX_HLOG_ENABLE */


#ifdef __cplusplus
}
#endif

#endif /* __PROXIMITY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
