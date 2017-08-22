/**
 ******************************************************************************
 * @file    YAS537_MAG_driver_HL.h
 * @author  MEMS Application Team
 * @version V2.0.0
 * @date    10-December-2015
 * @brief   This file contains definitions for the YAS537_MAG_driver_HL.c firmware driver
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
#ifndef __YAS537_MAG_DRIVER_HL_H
#define __YAS537_MAG_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "magnetometer.h"
/* Include magnetic sensor component drivers. */
#include "yas_mag_drv-yas537.h"

//extern uint8_t MAGNETO_I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
//extern uint8_t MAGNETO_I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
//extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
//extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
#define MAGNETO_I2C_TIMEOUT_MAX  0x1000
#define MAG_CALIB_PASS 1

typedef enum
{
  YAS537_MAG_DO_15Hz	= 0x42,
  YAS537_MAG_DO_30Hz	= 0x21,
  YAS537_MAG_DO_60Hz	= 0x10,
  YAS537_MAG_DO_80Hz	= 0xc,
  YAS537_MAG_DO_120Hz	= 0x8,
} YAS537_MAG_DO_t;

typedef struct
{
	uint8_t dummy;
} YAS537_Data_t;

extern MAGNETO_Drv_t YAS537Drv;
extern struct yas_driver_callback  YAS537Callback;

#define YAS_NV_MAGIC_NUMBER   (0x594153ffU)
struct YAS_NV_PRMS
{
	uint32_t magic;
	int32_t offset[3];
	uint8_t accuracy;
};

#ifdef __cplusplus
}
#endif

#endif /* __YAS537_MAG_DRIVER_HL_H */

