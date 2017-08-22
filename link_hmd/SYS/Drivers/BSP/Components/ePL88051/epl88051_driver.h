/**
 ******************************************************************************
 * @file    epl88051_driver.h
 * @author  hTC BSP Team
 * @version V1.0.0
 * @date    10-November-2015
 * @brief   This file contains definitions for the epl88051_driver.c  driver
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
#ifndef __EPL88051_DRIVER_H
#define __EPL88051_DRIVER_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
	// register macro
#define REG_0						0X00
#define REG_3						0X03
#define REG_4						0X04
#define REG_5						0X05
#define REG_6						0X06
#define REG_12						0X0C
#define REG_13						0X0D
#define REG_14						0X0E
#define REG_15						0X0F
#define REG_17						0X11
#define REG_27						0X1B
#define REG_28						0x1C
#define REG_29						0x1D
#define REG_30						0x1E
#define REG_31						0x1F
#define REG_33						0x21
#define REG_34						0x22
#define REG_35						0x23

//register macro alias
#define REG_PS_LOW_THRESH_LSB	REG_12
#define REG_PS_LOW_THRESH_MSB	REG_13
#define REG_PS_HIGH_THRESH_LSB	REG_14
#define REG_PS_HIGH_THRESH_MSB	REG_15
#define REG_PALS_DATA_LSB			REG_28
#define REG_PALS_DATA_MSB			REG_29
#define REG_PS_DATA_LSB			REG_30
#define REG_PS_DATA_MSB			REG_31
#define REG_REVNO					REG_33
#define REG_PS_OFFSET_LSB			REG_34
#define REG_PS_OFFSET_MSB			REG_35

//register 0x00
#define EPL_IDLE_MODE				(0<<0)
#define EPL_ALS_MODE				(1<<0)
#define EPL_PS_MODE				(2<<0)
#define EPL_PALS_MODE				(3<<0)
#define EPL_WTIME_DISABLE			(0<<4)
#define EPL_WTIME_2MS				(1<<4)
#define EPL_WTIME_4MS				(2<<4)
#define EPL_WTIME_8MS				(3<<4)
#define EPL_WTIME_12MS				(4<<4)
#define EPL_WTIME_20MS				(5<<4)
#define EPL_WTIME_30MS				(6<<4)
#define EPL_WTIME_40MS				(7<<4)
#define EPL_WTIME_50MS				(8<<4)
#define EPL_WTIME_75MS				(9<<4)
#define EPL_WTIME_100MS			(10<<4)
#define EPL_WTIME_150MS			(11<<4)
#define EPL_WTIME_200MS			(12<<4)
#define EPL_WTIME_300MS			(13<<4)
#define EPL_WTIME_400MS			(14<<4)
#define EPL_WTIME_SINGLE			(15<<4)

//register 0x03
#define EPL_PS_H_GAIN				(0<<0)
#define EPL_PS_M_GAIN				(1<<0)
#define EPL_PS_L_GAIN				(3<<0)
#define EPL_PS_INTEG_48				(5<<2)
#define EPL_PS_INTEG_80				(6<<2)
#define EPL_PS_INTEG_144			(7<<2)
#define EPL_PS_INTEG_272			(8<<2)
#define EPL_PS_INTEG_384			(9<<2)

//register 0x04
#define EPL_PS_FILTER_1ORDER 		(0<<0)
#define EPL_PS_FILTER_2ORDER 		(1<<0)
#define EPL_PS_FILTER_4ORDER 		(2<<0)
#define EPL_PS_FILTER_8ORDER 		(3<<0)
#define EPL_PS_FILTER_16ORDER 		(4<<0)
#define EPL_PS_FILTER_32ORDER 		(5<<0)
#define EPL_PS_FILTER_64ORDER 		(6<<0)
#define EPL_REG4_RESERVED 			(2<<3)

//register 0x05
#define EPL_PS_IR_DRIVE_200MA 		(0<<0)
#define EPL_PS_IR_DRIVE_100MA 		(1<<0)
#define EPL_PS_IR_DRIVE_50MA 		(2<<0)
#define EPL_PS_IR_DRIVE_10MA 		(3<<0)
#define EPL_PS_IR_MODE_CURRENT 	(0<<4)
#define EPL_PS_IR_MODE_VOLTAGE 	(1<<4)
#define EPL_PS_IR_LED_OFF			(0<<5)
#define EPL_PS_IR_LED_ON			(1<<5)

//register 0x06
#define EPL_INT_DISABLE				(0<<0)
#define EPL_INT_BINARY				(1<<0)
#define EPL_INT_ACTIVE_LOW			(2<<0)
#define EPL_INT_FRAME_ENABLE		(3<<0)
#define EPL_PS_PERSIST_1FRAME		(0<<2)
#define EPL_PS_PERSIST_4FRAME		(1<<2)
#define EPL_PS_PERSIST_8FRAME		(2<<2)
#define EPL_PS_PERSIST_16FRAME	(3<<2)
#define EPL_PALS_INT_CTRL			(0<<4)
#define EPL_ALS_INT_CTRL			(1<<4)
#define EPL_PS_INT_CTRL				(2<<4)

//register 0x11
#define EPL_ACTIVE					(0<<0)
#define EPL_SLEEP					(1<<0)
#define EPL_RESET					(0<<1)
#define EPL_START					(1<<1)

//register 0x1B
#define EPL_PS_UNLOCK				(0<<0)
#define EPL_PS_LOCK					(1<<0)
#define EPL_PS_CMP_RSTN			(0<<1)
#define EPL_PS_CMP_NORMAL			(1<<1)
#define EPL_PS_INT_MASK			0x04
#define EPL_PS_CMP_L_MASK			0x08
#define EPL_PS_CMP_H_MASK			0x10
/**/
#define EPL_SINGLE_BYTE				0x01
#define EPL_DOUBLE_BYTE			0x02
#define EPL88051_REVNO				0x81
#define EPL88051_ADDR				0x92
#define EPL_LOW_THRESH			100
#define EPL_HIGH_THRESH			300

/*calculate sensing delay time array*/
static int ps_integration_value[] = {4, 8, 16, 24, 32, 48, 80, 144, 272, 384, 520, 784, 1040, 2064, 4112, 6160};//register 0x04 bit2:bit5
static int ps_adc_value[] = {128, 256, 512, 1024};//register 0x04  bit3:bit4
static int ps_filter_value[] = {1, 2, 4, 8, 16, 32, 64}; //register 0x04 bit0:bit2


typedef enum
{
	PS_OK = 0,
	PS_ERROR = -1,
	PS_ERROR_COMMUNICATION = -2,
	PS_ERROR_NOT_IMPLEMENTED = -3
} PSDrvRet;

#define PROXIMITY_I2C_TIMEOUT_MAX  0x1000

PSDrvRet epl88051_ps_init();
PSDrvRet epl88051_ps_deinit();
PSDrvRet epl88051_ps_enable(uint8_t enable);
PSDrvRet epl88051_ps_get_revno(uint8_t *revision_number );
PSDrvRet epl88051_ps_proximity(uint8_t *ps_proximity );
#if 1
PSDrvRet epl88051_ps_adc_value(uint16_t *adc_value );
PSDrvRet epl88051_ps_register_dump( void );
PSDrvRet epl88051_ps_set_threshold(uint16_t * low_threshold,uint16_t * high_threshold );
PSDrvRet epl88051_ps_get_threshold(uint16_t * low_threshold,uint16_t * high_threshold );
PSDrvRet epl88051_cmd_ps_proximity(uint8_t *ps_proximity );
PSDrvRet  epl88051_ps_Xtalk_calibration(uint16_t * xtalk,uint16_t * low_threshold,uint16_t * high_threshold );
#endif

#ifdef __cplusplus
}
#endif

#endif /* __EPL88051_DRIVER_H */

