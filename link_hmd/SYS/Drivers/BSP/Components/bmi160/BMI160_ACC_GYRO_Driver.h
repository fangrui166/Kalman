/** \mainpage
*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* File : bmi160.h
*
* Date : 2014/10/27
*
* Revision : 2.0.6 $
*
* Usage: Sensor Driver for BMI160 sensor
*
****************************************************************************
*
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

/*! \file bmi160.h
    \brief BMI160 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMI160_H__
#define __BMI160_H__
#include "BMI160_ACC_GYRO_Driver_HL.h"
/*!
* @brief The following definition uses for define the data types
*
* @note While porting the API please consider the following
* @note Please check the version of C standard
* @note Are you using Linux platform
*/

/*!
* @brief For the Linux platform support
* Please use the types.h for your data types definitions
*/
#ifdef	__KERNEL__

#include <linux/types.h>
/* singed integer type*/
typedef	int8_t int8_ts;/**< used for signed 8bit */
typedef	int16_t int16_ts;/**< used for signed 16bit */
typedef	int32_t int32_ts;/**< used for signed 32bit */
typedef	int64_t int64_ts;/**< used for signed 64bit */


typedef	u_int8_t us8;/**< used for unsigned 8bit */
typedef	u_int16_t us16;/**< used for unsigned 16bit */
typedef	u_int32_t us32;/**< used for unsigned 32bit */
typedef	u_int64_t us64;/**< used for unsigned 64bit */



#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef	uint8_t us8;/**< used for unsigned 8bit */
typedef	uint16_t us16;/**< used for unsigned 16bit */
typedef	uint32_t us32;/**< used for unsigned 32bit */
typedef	uint64_t us64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef	int8_t int8_ts;/**< used for signed 8bit */
typedef	int16_t int16_ts;/**< used for signed 16bit */
typedef	int32_t int32_ts;/**< used for signed 32bit */
typedef	int64_t int64_ts;/**< used for signed 64bit */
/************************************************
 * compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef	uint8_t us8;/**< used for unsigned 8bit */
typedef	uint16_t us16;/**< used for unsigned 16bit */
typedef	uint32_t us32;/**< used for unsigned 32bit */
typedef	uint64_t us64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef int8_t int8_ts;/**< used for signed 8bit */
typedef	int16_t int16_ts;/**< used for signed 16bit */
typedef	int32_t int32_ts;/**< used for signed 32bit */
typedef	int64_t int64_ts;/**< used for signed 64bit */
/************************************************
 * compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */
/*!
* @brief By default it is defined as 32 bit machine configuration
*	define your data types based on your
*	machine/compiler/controller configuration
*/
//#define  MACHINE_32_BIT

/*! @brief
 *	If your machine support 16 bit
 *	define the MACHINE_16_BIT
 */
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed long int int32_ts;/**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int int64_ts;/**< used for signed 64bit */
typedef unsigned long int us64;/**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int int64_ts;/**< used for signed 64bit */
typedef unsigned long long int us64;/**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set int64_ts manually.
#endif

/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned long int us32;/**< used for unsigned 32bit */

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed int int32_ts;/**< used for signed 32bit */
typedef	signed long long int int64_ts;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned int us32;/**< used for unsigned 32bit */
typedef	unsigned long long int us64;/**< used for unsigned 64bit */

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed int int32_ts;/**< used for signed 32bit */
typedef	signed long int int64_ts;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned int us32;/**< used for unsigned 32bit */
typedef	unsigned long int us64;/**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*!
* @brief By default it is defined as 32 bit machine configuration
*	define your data types based on your
*	machine/compiler/controller configuration
*/
//#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed long int int32_ts;/**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int int64_ts;/**< used for signed 64bit */
typedef unsigned long int us64;/**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int int64_ts;/**< used for signed 64bit */
typedef unsigned long long int us64;/**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set int64_ts manually.
#endif

/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned long int us32;/**< used for unsigned 32bit */

/*! @brief If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed int int32_ts;/**< used for signed 32bit */
typedef	signed long long int int64_ts;/**< used for signed 64bit */
 
/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned int us32;/**< used for unsigned 32bit */
typedef	unsigned long long int us64;/**< used for unsigned 64bit */

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed int int32_ts;/**< used for signed 32bit */
typedef	signed long int int64_ts;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned int us32;/**< used for unsigned 32bit */
typedef	unsigned long int us64;/**< used for unsigned 64bit */

#else
//#warning The data types defined above which not supported \
//define the data types manually
#endif
#endif
#endif
/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
/*!
	@brief Define the calling convention of YOUR bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    write_my_bus_xy(us8 device_addr, us8 register_addr,
    us8 * data, us8 length);

    The BMI160_WR_FUNC_PTR would equal:

    BMI160_WR_FUNC_PTR int8_ts (* bus_write)(us32,
    us8, us8 *, us8)

    Parameters can be mixed as needed refer to the
    @ref BMI160_BUS_WRITE_FUNC  macro.


*/
#define BMI160_WR_FUNC_PTR int8_ts (*bus_write)(us32, us8,\
us8 *, us8)
/**< link macro between API function calls and bus write function
	@note The bus write function can change since this is a
	system dependant issue.

    If the bus_write parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    BMI160_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_write(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    @ref BMI160_WR_FUNC_PTR definition.

*/
#define BMI160_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
				bus_write(dev_addr, reg_addr, reg_data, wr_len)

/**< Define the calling convention of YOUR bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    read_my_bus_xy(us8 device_addr, us8 register_addr,
    us8 * data, us8 length);

    The BMI160_RD_FUNC_PTR would equal:

    BMI160_RD_FUNC_PTR int8_ts (* bus_read)(us32,
    us8, us8 *, us8)

    Parameters can be mixed as needed refer to the
    refer BMI160_BUS_READ_FUNC  macro.

*/
#define BMI160_SPI_RD_MASK (0x80)   /* for spi read transactions on SPI the
			MSB has to be set */
#define BMI160_RD_FUNC_PTR int8_ts (*bus_read)(us32,\
			us8, us8 *, us8)

#define BMI160_BRD_FUNC_PTR int8_ts \
(*burst_read)(us32, us8, us8 *, us32)

/**< link macro between API function calls and bus read function
	@note The bus write function can change since this is a
	system dependant issue.

    If the bus_read parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    BMI160_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_read(dev_addr, reg_addr, reg_data, wr_len)

    This macro lets all API functions call YOUR communication routine in a
    way that equals your definition in the
    refer BMI160_WR_FUNC_PTR definition.

    @note: this macro also includes the "MSB='1'
    for reading BMI160 addresses.

*/
#define BMI160_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
				bus_read(dev_addr, reg_addr, reg_data, r_len)

#define BMI160_BURST_READ_FUNC(device_addr, \
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)


#define BMI160_MDELAY_DATA_TYPE                 us32

/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
#define BMI160_I2C_ADDR1                   (0x68)
/**< I2C Address needs to be changed */
#define BMI160_I2C_ADDR2                    (0x69)
 /**< I2C Address needs to be changed */
#define BMI160_AUX_BMM150_I2C_ADDRESS       (0x10)
/**< I2C address of BMM150*/
#define BMI160_AUX_YAS532_I2C_ADDRESS       (0x2E)
/**< I2C address of YAS532*/
#define	BMI160_AUX_AKM09911_I2C_ADDR_1		(0x0C)
/**< I2C address of AKM09911*/
#define	BMI160_AUX_AKM09911_I2C_ADDR_2		(0x0D)
/**< I2C address of AKM09911*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_1		(0x0C)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_2		(0x0D)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_3		(0x0E)
/**< I2C address of AKM09912*/
#define	BMI160_AUX_AKM09912_I2C_ADDR_4		(0x0F)
/**< I2C address of AKM09912*/
/*******************************************/
/**\name	CONSTANTS        */
/******************************************/
#define  BMI160_INIT_VALUE					(0)
#define  BMI160_ASSIGN_DATA                 (1)
#define  BMI160_GEN_READ_WRITE_DATA_LENGTH	(1)
#define  BMI160_MAXIMUM_TIMEOUT             (10)

/* output data rate condition check*/
#define  BMI160_OUTPUT_DATA_RATE0	(0)
#define  BMI160_OUTPUT_DATA_RATE1	(1)
#define  BMI160_OUTPUT_DATA_RATE2	(2)
#define  BMI160_OUTPUT_DATA_RATE3	(3)
#define  BMI160_OUTPUT_DATA_RATE4	(4)
#define  BMI160_OUTPUT_DATA_RATE5	(5)
#define  BMI160_OUTPUT_DATA_RATE6	(14)
#define  BMI160_OUTPUT_DATA_RATE7	(15)

/* accel range check*/
#define BMI160_ACCEL_RANGE0  (3)
#define BMI160_ACCEL_RANGE1  (5)
#define BMI160_ACCEL_RANGE3  (8)
#define BMI160_ACCEL_RANGE4  (12)

/* check the status of registers*/
#define  BMI160_FOC_STAT_HIGH			(1)
#define  BMI160_SIG_MOTION_STAT_HIGH	(1)
#define  BMI160_STEP_DET_STAT_HIGH		(1)

/*condition check for reading and writing data*/
#define	BMI160_MAX_VALUE_SIGNIFICANT_MOTION      (1)
#define	BMI160_MAX_VALUE_FIFO_FILTER    (1)
#define	BMI160_MAX_VALUE_FIFO_TIME      (1)
#define	BMI160_MAX_VALUE_FIFO_INTR      (1)
#define	BMI160_MAX_VALUE_FIFO_HEADER    (1)
#define	BMI160_MAX_VALUE_FIFO_MAG       (1)
#define	BMI160_MAX_VALUE_FIFO_ACCEL     (1)
#define	BMI160_MAX_VALUE_FIFO_GYRO      (1)
#define	BMI160_MAX_VALUE_SOURCE_INTR    (1)
#define	BMI160_MAX_VALUE_LOW_G_MODE     (1)
#define	BMI160_MAX_VALUE_NO_MOTION      (1)
#define	BMI160_MAX_VALUE_TAP_SHOCK      (1)
#define	BMI160_MAX_VALUE_TAP_QUIET      (1)
#define	BMI160_MAX_VALUE_ORIENT_UD      (1)
#define	BMI160_MAX_VALUE_ORIENT_AXES    (1)
#define	BMI160_MAX_VALUE_NVM_PROG       (1)
#define	BMI160_MAX_VALUE_SPI3           (1)
#define	BMI160_MAX_VALUE_PAGE           (1)
#define	BMI160_MAX_VALUE_I2C_WDT        (1)
#define	BMI160_MAX_VALUE_SLEEP_STATE    (1)
#define	BMI160_MAX_VALUE_WAKEUP_INTR    (1)
#define	BMI160_MAX_VALUE_SELFTEST_SIGN  (1)
#define	BMI160_MAX_VALUE_SELFTEST_AMP   (1)
#define	BMI160_MAX_VALUE_SELFTEST_START (1)

#define BMI160_MAX_GYRO_WAKEUP_TRIGGER		(3)
#define BMI160_MAX_ACCEL_SELFTEST_AXIS	    (3)
#define BMI160_MAX_GYRO_STEP_COUNTER        (1)
#define BMI160_MAX_GYRO_BW                  (3)
#define BMI160_MAX_ACCEL_BW                 (7)
#define BMI160_MAX_ORIENT_MODE              (3)
#define BMI160_MAX_ORIENT_BLOCKING          (3)
#define BMI160_MAX_FLAT_HOLD                (3)
#define BMI160_MAX_ACCEL_FOC                (3)
#define BMI160_MAX_IF_MODE                  (3)
#define BMI160_MAX_TARGET_PAGE              (3)
#define BMI160_MAX_GYRO_RANGE               (4)
#define BMI160_MAX_GYRO_SLEEP_TIGGER        (7)
#define BMI160_MAX_TAP_TURN                 (7)
#define BMI160_MAX_UNDER_SAMPLING           (1)
#define BMI160_MAX_UNDER_SIG_MOTION         (3)
#define BMI160_MAX_ACCEL_OUTPUT_DATA_RATE   (12)
#define BMI160_MAX_LATCH_INTR               (15)
#define BMI160_MAX_FLAT_HYST                (15)
#define BMI160_MAX_ORIENT_THETA             (63)
#define BMI160_MAX_FLAT_THETA               (63)

/* FIFO index definitions*/
#define BMI160_FIFO_X_LSB_DATA			(0)
#define BMI160_FIFO_X_MSB_DATA			(1)
#define BMI160_FIFO_Y_LSB_DATA			(2)
#define BMI160_FIFO_Y_MSB_DATA			(3)
#define BMI160_FIFO_Z_LSB_DATA			(4)
#define BMI160_FIFO_Z_MSB_DATA			(5)
#define BMI160_FIFO_R_LSB_DATA			(6)
#define BMI160_FIFO_R_MSB_DATA			(7)
/* FIFO gyro definition*/
#define BMI160_GA_FIFO_G_X_LSB		(0)
#define BMI160_GA_FIFO_G_X_MSB		(1)
#define BMI160_GA_FIFO_G_Y_LSB		(2)
#define BMI160_GA_FIFO_G_Y_MSB		(3)
#define BMI160_GA_FIFO_G_Z_LSB		(4)
#define BMI160_GA_FIFO_G_Z_MSB		(5)
#define BMI160_GA_FIFO_A_X_LSB		(6)
#define BMI160_GA_FIFO_A_X_MSB		(7)
#define BMI160_GA_FIFO_A_Y_LSB		(8)
#define BMI160_GA_FIFO_A_Y_MSB		(9)
#define BMI160_GA_FIFO_A_Z_LSB		(10)
#define BMI160_GA_FIFO_A_Z_MSB		(11)
/* FIFO mag/gyro/accel definition*/
#define BMI160_MGA_FIFO_M_X_LSB		(0)
#define BMI160_MGA_FIFO_M_X_MSB		(1)
#define BMI160_MGA_FIFO_M_Y_LSB		(2)
#define BMI160_MGA_FIFO_M_Y_MSB		(3)
#define BMI160_MGA_FIFO_M_Z_LSB		(4)
#define BMI160_MGA_FIFO_M_Z_MSB		(5)
#define BMI160_MGA_FIFO_M_R_LSB		(6)
#define BMI160_MGA_FIFO_M_R_MSB		(7)
#define BMI160_MGA_FIFO_G_X_LSB		(8)
#define BMI160_MGA_FIFO_G_X_MSB		(9)
#define BMI160_MGA_FIFO_G_Y_LSB		(10)
#define BMI160_MGA_FIFO_G_Y_MSB		(11)
#define BMI160_MGA_FIFO_G_Z_LSB		(12)
#define BMI160_MGA_FIFO_G_Z_MSB		(13)
#define BMI160_MGA_FIFO_A_X_LSB		(14)
#define BMI160_MGA_FIFO_A_X_MSB		(15)
#define BMI160_MGA_FIFO_A_Y_LSB		(16)
#define BMI160_MGA_FIFO_A_Y_MSB		(17)
#define BMI160_MGA_FIFO_A_Z_LSB		(18)
#define BMI160_MGA_FIFO_A_Z_MSB		(19)
/* FIFO mag definition*/
#define BMI160_MA_FIFO_M_X_LSB		(0)
#define BMI160_MA_FIFO_M_X_MSB		(1)
#define BMI160_MA_FIFO_M_Y_LSB		(2)
#define BMI160_MA_FIFO_M_Y_MSB		(3)
#define BMI160_MA_FIFO_M_Z_LSB		(4)
#define BMI160_MA_FIFO_M_Z_MSB		(5)
#define BMI160_MA_FIFO_M_R_LSB		(6)
#define BMI160_MA_FIFO_M_R_MSB		(7)
#define BMI160_MA_FIFO_A_X_LSB		(8)
#define BMI160_MA_FIFO_A_X_MSB		(9)
#define BMI160_MA_FIFO_A_Y_LSB		(10)
#define BMI160_MA_FIFO_A_Y_MSB		(11)
#define BMI160_MA_FIFO_A_Z_LSB		(12)
#define BMI160_MA_FIFO_A_Z_MSB		(13)
/* FIFO mag/gyro definition*/
#define BMI160_MG_FIFO_M_X_LSB		(0)
#define BMI160_MG_FIFO_M_X_MSB		(1)
#define BMI160_MG_FIFO_M_Y_LSB		(2)
#define BMI160_MG_FIFO_M_Y_MSB		(3)
#define BMI160_MG_FIFO_M_Z_LSB		(4)
#define BMI160_MG_FIFO_M_Z_MSB		(5)
#define BMI160_MG_FIFO_M_R_LSB		(6)
#define BMI160_MG_FIFO_M_R_MSB		(7)
#define BMI160_MG_FIFO_G_X_LSB		(8)
#define BMI160_MG_FIFO_G_X_MSB		(9)
#define BMI160_MG_FIFO_G_Y_LSB		(10)
#define BMI160_MG_FIFO_G_Y_MSB		(11)
#define BMI160_MG_FIFO_G_Z_LSB		(12)
#define BMI160_MG_FIFO_G_Z_MSB		(13)
/* FIFO length definitions*/
#define BMI160_FIFO_SENSOR_TIME_LSB     (0)
#define BMI160_FIFO_SENSOR_TIME_XLSB    (1)
#define BMI160_FIFO_SENSOR_TIME_MSB     (2)
#define BMI160_FIFO_SENSOR_TIME_LENGTH  (3)
#define BMI160_FIFO_A_LENGTH            (6)
#define BMI160_FIFO_G_LENGTH            (6)
#define BMI160_FIFO_M_LENGTH            (8)
#define BMI160_FIFO_AG_LENGTH           (12)
#define BMI160_FIFO_AMG_LENGTH          (20)
#define BMI160_FIFO_MA_OR_MG_LENGTH     (14)

/* bus read and write length for mag, accel and gyro*/
#define BMI160_GYRO_DATA_LENGTH		 (2)
#define BMI160_GYRO_XYZ_DATA_LENGTH	 (6)
#define BMI160_ACCEL_DATA_LENGTH	 (2)
#define BMI160_ACCEL_XYZ_DATA_LENGTH (6)
#define BMI160_TEMP_DATA_LENGTH		 (2)
#define BMI160_FIFO_DATA_LENGTH		 (2)
#define BMI160_STEP_COUNTER_LENGTH	 (2)
#define BMI160_SENSOR_TIME_LENGTH	 (3)

/* Delay definitions*/
#define BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY    (5)
#define BMI160_BMM150_WAKEUP_DELAY1                  (2)
#define BMI160_BMM150_WAKEUP_DELAY2                  (3)
#define BMI160_BMM150_WAKEUP_DELAY3                  (1)
#define BMI160_YAS532_OFFSET_DELAY                   (2)
#define BMI160_GEN_READ_WRITE_DELAY                  (1)
#define BMI160_YAS532_MEASUREMENT_DELAY              (25)
#define BMI160_YAS_ACQ_COMMAND_DELAY                 (50)
#define BMI160_YAS532_SET_INITIAL_VALUE_DELAY        (200)
#define BMI160_AKM_INIT_DELAY                        (60)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define	BMI160_ACCEL_X_DATA_SIZE   (2)
#define	BMI160_ACCEL_Y_DATA_SIZE   (2)
#define	BMI160_ACCEL_Z_DATA_SIZE   (2)
#define	BMI160_ACCEL_XYZ_DATA_SIZE (6)

#define	BMI160_GYRO_X_DATA_SIZE    (2)
#define	BMI160_GYRO_Y_DATA_SIZE    (2)
#define	BMI160_GYRO_Z_DATA_SIZE    (2)
#define	BMI160_GYRO_XYZ_DATA_SIZE  (6)

#define	BMI160_TEMP_DATA_SIZE       (2)
#define	BMI160_FIFO_DATA_SIZE       (2)
#define	BMI160_STEP_COUNT_DATA_SIZE (2)

#define	BMI160_SENSOR_TIME_DATA_SIZE      (3)
#define	BMI160_AKM_SENSITIVITY_DATA_SIZE  (3)
#define	BMI160_HARD_OFFSET_DATA_SIZE      (3)
#define	BMI160_YAS_XY1Y2_DATA_SIZE        (3)
#define	BMI160_YAS_FLAG_DATA_SIZE         (3)
#define	BMI160_YAS_TEMP_DATA_SIZE         (3)
#define	BMI160_YAS_H_DATA_SIZE            (3)
#define	BMI160_YAS_S_DATA_SIZE            (3)
#define BMI160_YAS_CORRECT_DATA_SIZE      (5)
#define BMI160_YAS_XY1Y2T_DATA_SIZE       (8)
#define BMI160_YAS537_CALIB_DATA_SIZE     (17)
#define BMI160_YAS532_CALIB_DATA_SIZE     (14)

/****************************************************/
/**\name	ARRAY PARAMETER DEFINITIONS      */
/***************************************************/
#define BMI160_SENSOR_TIME_MSB_BYTE   (2)
#define BMI160_SENSOR_TIME_XLSB_BYTE  (1)
#define BMI160_SENSOR_TIME_LSB_BYTE   (0)


#define BMI160_GYRO_X_LSB_BYTE              (0)
#define BMI160_GYRO_X_MSB_BYTE              (1)
#define BMI160_GYRO_Y_LSB_BYTE              (0)
#define BMI160_GYRO_Y_MSB_BYTE              (1)
#define BMI160_GYRO_Z_LSB_BYTE              (0)
#define BMI160_GYRO_Z_MSB_BYTE              (1)
#define BMI160_DATA_FRAME_GYRO_X_LSB_BYTE   (0)
#define BMI160_DATA_FRAME_GYRO_X_MSB_BYTE   (1)
#define BMI160_DATA_FRAME_GYRO_Y_LSB_BYTE   (2)
#define BMI160_DATA_FRAME_GYRO_Y_MSB_BYTE   (3)
#define BMI160_DATA_FRAME_GYRO_Z_LSB_BYTE   (4)
#define BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE   (5)

#define BMI160_ACCEL_X_LSB_BYTE              (0)
#define BMI160_ACCEL_X_MSB_BYTE              (1)
#define BMI160_ACCEL_Y_LSB_BYTE              (0)
#define BMI160_ACCEL_Y_MSB_BYTE              (1)
#define BMI160_ACCEL_Z_LSB_BYTE              (0)
#define BMI160_ACCEL_Z_MSB_BYTE              (1)
#define BMI160_DATA_FRAME_ACCEL_X_LSB_BYTE   (0)
#define BMI160_DATA_FRAME_ACCEL_X_MSB_BYTE   (1)
#define BMI160_DATA_FRAME_ACCEL_Y_LSB_BYTE   (2)
#define BMI160_DATA_FRAME_ACCEL_Y_MSB_BYTE   (3)
#define BMI160_DATA_FRAME_ACCEL_Z_LSB_BYTE   (4)
#define BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE   (5)

#define	BMI160_TEMP_LSB_BYTE    (0)
#define	BMI160_TEMP_MSB_BYTE    (1)

#define	BMI160_FIFO_LENGTH_LSB_BYTE    (0)
#define	BMI160_FIFO_LENGTH_MSB_BYTE    (1)

#define	BMI160_STEP_COUNT_LSB_BYTE    (0)
#define	BMI160_STEP_COUNT_MSB_BYTE    (1)
/****************************************************/
/**\name	BMI160_ERROR CODES       */
/***************************************************/

#define E_BMI160_NULL_PTR			((int8_ts)-127)
#define E_BMI160_COMM_RES			((int8_ts)-1)
#define E_BMI160_OUT_OF_RANGE		((int8_ts)-2)
#define E_BMI160_BUSY				((int8_ts)-3)
#define	BMI160_SUCCESS						((us8)0)
#define	BMI160_ERROR						((int8_ts)-1)

/* Constants */
#define BMI160_NULL						(0)
#define BMI160_DELAY_SETTLING_TIME		(5)
/*This refers BMI160 return type as int8_ts */
#define BMI160_RETURN_FUNCTION_TYPE		int8_ts
/****************************************************/
/**\name	REGISTER DEFINITIONS       */
/***************************************************/
/*******************/
/**\name CHIP ID */
/*******************/
#define BMI160_USER_CHIP_ID_ADDR				(0x00)
/*******************/
/**\name BMI160_ERROR STATUS */
/*******************/
#define BMI160_USER_BMI160_ERROR_ADDR					(0X02)
/*******************/
/**\name POWER MODE STATUS */
/*******************/
#define BMI160_USER_PMU_STAT_ADDR				(0X03)
/*******************/
/**\name MAG DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_0_ADDR					(0X04)
#define BMI160_USER_DATA_1_ADDR					(0X05)
#define BMI160_USER_DATA_2_ADDR					(0X06)
#define BMI160_USER_DATA_3_ADDR					(0X07)
#define BMI160_USER_DATA_4_ADDR					(0X08)
#define BMI160_USER_DATA_5_ADDR					(0X09)
#define BMI160_USER_DATA_6_ADDR					(0X0A)
#define BMI160_USER_DATA_7_ADDR					(0X0B)
/*******************/
/**\name GYRO DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_8_ADDR					(0X0C)
#define BMI160_USER_DATA_9_ADDR					(0X0D)
#define BMI160_USER_DATA_10_ADDR				(0X0E)
#define BMI160_USER_DATA_11_ADDR				(0X0F)
#define BMI160_USER_DATA_12_ADDR				(0X10)
#define BMI160_USER_DATA_13_ADDR				(0X11)
#define BMI160_USER_DATA_14_ADDR				(0X12)
#define BMI160_USER_DATA_15_ADDR				(0X13)
/*******************/
/**\name ACCEL DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_16_ADDR				(0X14)
#define BMI160_USER_DATA_17_ADDR				(0X15)
#define BMI160_USER_DATA_18_ADDR				(0X16)
#define BMI160_USER_DATA_19_ADDR				(0X17)
/*******************/
/**\name SENSOR TIME REGISTERS */
/*******************/
#define BMI160_USER_SENSORTIME_0_ADDR			(0X18)
#define BMI160_USER_SENSORTIME_1_ADDR			(0X19)
#define BMI160_USER_SENSORTIME_2_ADDR			(0X1A)
/*******************/
/**\name STATUS REGISTER FOR SENSOR STATUS FLAG */
/*******************/
#define BMI160_USER_STAT_ADDR					(0X1B)
/*******************/
/**\name INTERRUPY STATUS REGISTERS */
/*******************/
#define BMI160_USER_INTR_STAT_0_ADDR			(0X1C)
#define BMI160_USER_INTR_STAT_1_ADDR			(0X1D)
#define BMI160_USER_INTR_STAT_2_ADDR			(0X1E)
#define BMI160_USER_INTR_STAT_3_ADDR			(0X1F)
/*******************/
/**\name TEMPERATURE REGISTERS */
/*******************/
#define BMI160_USER_TEMPERATURE_0_ADDR			(0X20)
#define BMI160_USER_TEMPERATURE_1_ADDR			(0X21)
/*******************/
/**\name FIFO REGISTERS */
/*******************/
#define BMI160_USER_FIFO_LENGTH_0_ADDR			(0X22)
#define BMI160_USER_FIFO_LENGTH_1_ADDR			(0X23)
#define BMI160_USER_FIFO_DATA_ADDR				(0X24)
/***************************************************/
/**\name ACCEL CONFIG REGISTERS  FOR ODR, BANDWIDTH AND UNDERSAMPLING*/
/******************************************************/
#define BMI160_USER_ACCEL_CONFIG_ADDR			(0X40)
/*******************/
/**\name ACCEL RANGE */
/*******************/
#define BMI160_USER_ACCEL_RANGE_ADDR            (0X41)
/***************************************************/
/**\name GYRO CONFIG REGISTERS  FOR ODR AND BANDWIDTH */
/******************************************************/
#define BMI160_USER_GYRO_CONFIG_ADDR            (0X42)
/*******************/
/**\name GYRO RANGE */
/*******************/
#define BMI160_USER_GYRO_RANGE_ADDR             (0X43)
/***************************************************/
/**\name MAG CONFIG REGISTERS  FOR ODR*/
/******************************************************/
#define BMI160_USER_MAG_CONFIG_ADDR				(0X44)
/***************************************************/
/**\name REGISTER FOR GYRO AND ACCEL DOWNSAMPLING RATES FOR FIFO*/
/******************************************************/
#define BMI160_USER_FIFO_DOWN_ADDR              (0X45)
/***************************************************/
/**\name FIFO CONFIG REGISTERS*/
/******************************************************/
#define BMI160_USER_FIFO_CONFIG_0_ADDR          (0X46)
#define BMI160_USER_FIFO_CONFIG_1_ADDR          (0X47)
/***************************************************/
/**\name MAG INTERFACE REGISTERS*/
/******************************************************/
#define BMI160_USER_MAG_IF_0_ADDR				(0X4B)
#define BMI160_USER_MAG_IF_1_ADDR				(0X4C)
#define BMI160_USER_MAG_IF_2_ADDR				(0X4D)
#define BMI160_USER_MAG_IF_3_ADDR				(0X4E)
#define BMI160_USER_MAG_IF_4_ADDR				(0X4F)
/***************************************************/
/**\name INTERRUPT ENABLE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_ENABLE_0_ADDR			(0X50)
#define BMI160_USER_INTR_ENABLE_1_ADDR			(0X51)
#define BMI160_USER_INTR_ENABLE_2_ADDR			(0X52)
#define BMI160_USER_INTR_OUT_CTRL_ADDR			(0X53)
/***************************************************/
/**\name LATCH DURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LATCH_ADDR				(0X54)
/***************************************************/
/**\name MAP INTERRUPT 1 and 2 REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_MAP_0_ADDR				(0X55)
#define BMI160_USER_INTR_MAP_1_ADDR				(0X56)
#define BMI160_USER_INTR_MAP_2_ADDR				(0X57)
/***************************************************/
/**\name DATA SOURCE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_DATA_0_ADDR			(0X58)
#define BMI160_USER_INTR_DATA_1_ADDR			(0X59)
/***************************************************/
/**\name
INTERRUPT THRESHOLD, HYSTERESIS, DURATION, MODE CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LOWHIGH_0_ADDR			(0X5A)
#define BMI160_USER_INTR_LOWHIGH_1_ADDR			(0X5B)
#define BMI160_USER_INTR_LOWHIGH_2_ADDR			(0X5C)
#define BMI160_USER_INTR_LOWHIGH_3_ADDR			(0X5D)
#define BMI160_USER_INTR_LOWHIGH_4_ADDR			(0X5E)
#define BMI160_USER_INTR_MOTION_0_ADDR			(0X5F)
#define BMI160_USER_INTR_MOTION_1_ADDR			(0X60)
#define BMI160_USER_INTR_MOTION_2_ADDR			(0X61)
#define BMI160_USER_INTR_MOTION_3_ADDR			(0X62)
#define BMI160_USER_INTR_TAP_0_ADDR				(0X63)
#define BMI160_USER_INTR_TAP_1_ADDR				(0X64)
#define BMI160_USER_INTR_ORIENT_0_ADDR			(0X65)
#define BMI160_USER_INTR_ORIENT_1_ADDR			(0X66)
#define BMI160_USER_INTR_FLAT_0_ADDR			(0X67)
#define BMI160_USER_INTR_FLAT_1_ADDR			(0X68)
/***************************************************/
/**\name FAST OFFSET CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_FOC_CONFIG_ADDR				(0X69)
/***************************************************/
/**\name MISCELLANEOUS CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_CONFIG_ADDR					(0X6A)
/***************************************************/
/**\name SERIAL INTERFACE SETTINGS REGISTER*/
/******************************************************/
#define BMI160_USER_IF_CONFIG_ADDR				(0X6B)
/***************************************************/
/**\name GYRO POWER MODE TRIGGER REGISTER */
/******************************************************/
#define BMI160_USER_PMU_TRIGGER_ADDR			(0X6C)
/***************************************************/
/**\name SELF_TEST REGISTER*/
/******************************************************/
#define BMI160_USER_SELF_TEST_ADDR				(0X6D)
/***************************************************/
/**\name SPI,I2C SELECTION REGISTER*/
/******************************************************/
#define BMI160_USER_NV_CONFIG_ADDR				(0x70)
/***************************************************/
/**\name ACCEL AND GYRO OFFSET REGISTERS*/
/******************************************************/
#define BMI160_USER_OFFSET_0_ADDR				(0X71)
#define BMI160_USER_OFFSET_1_ADDR				(0X72)
#define BMI160_USER_OFFSET_2_ADDR				(0X73)
#define BMI160_USER_OFFSET_3_ADDR				(0X74)
#define BMI160_USER_OFFSET_4_ADDR				(0X75)
#define BMI160_USER_OFFSET_5_ADDR				(0X76)
#define BMI160_USER_OFFSET_6_ADDR				(0X77)
/***************************************************/
/**\name STEP COUNTER INTERRUPT REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_COUNT_0_ADDR			(0X78)
#define BMI160_USER_STEP_COUNT_1_ADDR			(0X79)
/***************************************************/
/**\name STEP COUNTER CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_CONFIG_0_ADDR			(0X7A)
#define BMI160_USER_STEP_CONFIG_1_ADDR			(0X7B)
/***************************************************/
/**\name COMMAND REGISTER*/
/******************************************************/
#define BMI160_CMD_COMMANDS_ADDR				(0X7E)
/***************************************************/
/**\name PAGE REGISTERS*/
/******************************************************/
#define BMI160_CMD_EXT_MODE_ADDR				(0X7F)
#define BMI160_COM_C_TRIM_FIVE_ADDR				(0X05)

/****************************************************/
/**\name	SHIFT VALUE DEFINITION       */
/***************************************************/
#define BMI160_SHIFT_BIT_POSITION_BY_01_BIT      (1)
#define BMI160_SHIFT_BIT_POSITION_BY_02_BITS     (2)
#define BMI160_SHIFT_BIT_POSITION_BY_03_BITS     (3)
#define BMI160_SHIFT_BIT_POSITION_BY_04_BITS     (4)
#define BMI160_SHIFT_BIT_POSITION_BY_05_BITS     (5)
#define BMI160_SHIFT_BIT_POSITION_BY_06_BITS     (6)
#define BMI160_SHIFT_BIT_POSITION_BY_07_BITS     (7)
#define BMI160_SHIFT_BIT_POSITION_BY_08_BITS     (8)
#define BMI160_SHIFT_BIT_POSITION_BY_09_BITS     (9)
#define BMI160_SHIFT_BIT_POSITION_BY_12_BITS     (12)
#define BMI160_SHIFT_BIT_POSITION_BY_13_BITS     (13)
#define BMI160_SHIFT_BIT_POSITION_BY_14_BITS     (14)
#define BMI160_SHIFT_BIT_POSITION_BY_15_BITS     (15)
#define BMI160_SHIFT_BIT_POSITION_BY_16_BITS     (16)


#define BMI160_FIFO_FRAME_CNT			(146)
#define	BMI160_FRAME_COUNT				(1)

/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
*	@brief bmi160 structure
*	This structure holds all relevant information about bmi160
*/

struct bmi160_t {
us8 chip_id;/**< chip id of BMI160 */
us32 dev_addr;/**< device address of BMI160 */
int8_ts mag_manual_enable;/**< used for check the mag manual/auto mode status */
int8_ts (*bus_write)(us32, us8,us8 *, us8);/**< bus write function pointer */
int8_ts (*bus_read)(us32,us8, us8 *, us8);/**< bus read function pointer */
int8_ts (*burst_read)(us32, us8, us8 *, us32);/**< burst write function pointer */
void (*delay_msec)(us32);/**< delay function pointer */
}; 

/*!
 * @brief Structure containing bmm150 and akm09911
 *	magnetometer values for x,y and
 *	z-axis in int16_ts
 */
struct bmi160_mag_t {
int32_ts x;/**< BMM150 and AKM09911 and AKM09912 X raw data*/
int32_ts y;/**< BMM150 and AKM09911 and AKM09912 Y raw data*/
int32_ts z;/**< BMM150 and AKM09911 and AKM09912 Z raw data*/
};
/*!
 * @brief Structure containing gyro xyz data
 */
struct bmi160_gyro_t {
int16_ts x;/**<gyro X  data*/
int16_ts y;/**<gyro Y  data*/
int16_ts z;/**<gyro Z  data*/
};
/*!
 * @brief Structure containing accel xyz data
 */
struct bmi160_accel_t {
int16_ts x;/**<accel X  data*/
int16_ts y;/**<accel Y  data*/
int16_ts z;/**<accel Z  data*/
};
/*!
* @brief FIFO used to store the FIFO header less data
*/
struct bmi160_fifo_data_header_less_t {

struct bmi160_accel_t accel_fifo[BMI160_FIFO_FRAME_CNT];/**<
Accel data of XYZ */
struct bmi160_mag_t mag_fifo[BMI160_FIFO_FRAME_CNT];/**<
Mag data of XYZ */
struct bmi160_gyro_t gyro_fifo[BMI160_FIFO_FRAME_CNT];/**<
Gyro data of XYZ */
us8 accel_frame_count;/**< The total number of accel frame stored
in the FIFO*/
us8 gyro_frame_count;/**< The total number of gyro frame stored
in the FIFO*/
us8 mag_frame_count;/**< The total number of mag frame stored
in the FIFO*/
};
/*!
* @brief Struct used to store the FIFO header data
*/
struct bmi160_fifo_data_header_t {
struct bmi160_accel_t accel_fifo[BMI160_FIFO_FRAME_CNT];/**<
Accel data of XYZ */
struct bmi160_mag_t mag_fifo[BMI160_FIFO_FRAME_CNT];/**<
Mag data of XYZ */
struct bmi160_gyro_t gyro_fifo[BMI160_FIFO_FRAME_CNT];/**<
Gyro data of XYZ */
us32 fifo_time;/**< Value of fifo time*/
us8 skip_frame;/**< The value of skip frame information */
us8 fifo_input_config_info; /**< FIFO input config info*/
us8 accel_frame_count; /**< The total number of accel frame stored
in the FIFO*/
us8 gyro_frame_count; /**< The total number of gyro frame stored
in the FIFO*/
us8 mag_frame_count; /**< The total number of mag frame stored
in the FIFO*/
us8 fifo_header[BMI160_FIFO_FRAME_CNT]; /**< FIFO header info*/
};
/**************************************************************/
/**\name	USER DATA REGISTERS DEFINITION START    */
/**************************************************************/

/**************************************************************/
/**\name	CHIP ID LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Chip ID Description - Reg Addr --> (0x00), Bit --> 0...7 */
#define BMI160_USER_CHIP_ID__POS             (0)
#define BMI160_USER_CHIP_ID__MSK            (0xFF)
#define BMI160_USER_CHIP_ID__LEN             (8)
#define BMI160_USER_CHIP_ID__REG             (BMI160_USER_CHIP_ID_ADDR)
/**************************************************************/
/**\name	BMI160_ERROR STATUS LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Error Description - Reg Addr --> (0x02), Bit --> 0 */
#define BMI160_USER_ERR_STAT__POS               (0)
#define BMI160_USER_ERR_STAT__LEN               (8)
#define BMI160_USER_ERR_STAT__MSK               (0xFF)
#define BMI160_USER_ERR_STAT__REG               (BMI160_USER_BMI160_ERROR_ADDR)

#define BMI160_USER_FATAL_ERR__POS               (0)
#define BMI160_USER_FATAL_ERR__LEN               (1)
#define BMI160_USER_FATAL_ERR__MSK               (0x01)
#define BMI160_USER_FATAL_ERR__REG               (BMI160_USER_BMI160_ERROR_ADDR)

/* Error Description - Reg Addr --> (0x02), Bit --> 1...4 */
#define BMI160_USER_ERR_CODE__POS               (1)
#define BMI160_USER_ERR_CODE__LEN               (4)
#define BMI160_USER_ERR_CODE__MSK               (0x1E)
#define BMI160_USER_ERR_CODE__REG               (BMI160_USER_BMI160_ERROR_ADDR)

/* Error Description - Reg Addr --> (0x02), Bit --> 5 */
#define BMI160_USER_I2C_FAIL_ERR__POS               (5)
#define BMI160_USER_I2C_FAIL_ERR__LEN               (1)
#define BMI160_USER_I2C_FAIL_ERR__MSK               (0x20)
#define BMI160_USER_I2C_FAIL_ERR__REG               (BMI160_USER_BMI160_ERROR_ADDR)

/* Error Description - Reg Addr --> (0x02), Bit --> 6 */
#define BMI160_USER_DROP_CMD_ERR__POS              (6)
#define BMI160_USER_DROP_CMD_ERR__LEN              (1)
#define BMI160_USER_DROP_CMD_ERR__MSK              (0x40)
#define BMI160_USER_DROP_CMD_ERR__REG              (BMI160_USER_BMI160_ERROR_ADDR)
/**************************************************************/
/**\name	MAG DATA READY LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Error Description - Reg Addr --> (0x02), Bit --> 7 */
#define BMI160_USER_MAG_DADA_RDY_ERR__POS               (7)
#define BMI160_USER_MAG_DADA_RDY_ERR__LEN               (1)
#define BMI160_USER_MAG_DADA_RDY_ERR__MSK               (0x80)
#define BMI160_USER_MAG_DADA_RDY_ERR__REG               (BMI160_USER_BMI160_ERROR_ADDR)
/**************************************************************/
/**\name	MAG POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of MAG - Reg Addr --> (0x03), Bit --> 1..0 */
#define BMI160_USER_MAG_POWER_MODE_STAT__POS		(0)
#define BMI160_USER_MAG_POWER_MODE_STAT__LEN		(2)
#define BMI160_USER_MAG_POWER_MODE_STAT__MSK		(0x03)
#define BMI160_USER_MAG_POWER_MODE_STAT__REG		\
(BMI160_USER_PMU_STAT_ADDR)
/**************************************************************/
/**\name	GYRO POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of GYRO - Reg Addr --> (0x03), Bit --> 3...2 */
#define BMI160_USER_GYRO_POWER_MODE_STAT__POS               (2)
#define BMI160_USER_GYRO_POWER_MODE_STAT__LEN               (2)
#define BMI160_USER_GYRO_POWER_MODE_STAT__MSK               (0x0C)
#define BMI160_USER_GYRO_POWER_MODE_STAT__REG		      \
(BMI160_USER_PMU_STAT_ADDR)
/**************************************************************/
/**\name	ACCEL POWER MODE LENGTH, POSITION AND MASK    */
/**************************************************************/
/* PMU_Status Description of ACCEL - Reg Addr --> (0x03), Bit --> 5...4 */
#define BMI160_USER_ACCEL_POWER_MODE_STAT__POS               (4)
#define BMI160_USER_ACCEL_POWER_MODE_STAT__LEN               (2)
#define BMI160_USER_ACCEL_POWER_MODE_STAT__MSK               (0x30)
#define BMI160_USER_ACCEL_POWER_MODE_STAT__REG		    \
(BMI160_USER_PMU_STAT_ADDR)
/**************************************************************/
/**\name	MAG DATA XYZ LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Mag_X(LSB) Description - Reg Addr --> (0x04), Bit --> 0...7 */
#define BMI160_USER_DATA_0_MAG_X_LSB__POS           (0)
#define BMI160_USER_DATA_0_MAG_X_LSB__LEN           (8)
#define BMI160_USER_DATA_0_MAG_X_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_0_MAG_X_LSB__REG          (BMI160_USER_DATA_0_ADDR)

/* Mag_X(LSB) Description - Reg Addr --> (0x04), Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_X_LSB__POS           (3)
#define BMI160_USER_DATA_MAG_X_LSB__LEN           (5)
#define BMI160_USER_DATA_MAG_X_LSB__MSK          (0xF8)
#define BMI160_USER_DATA_MAG_X_LSB__REG          (BMI160_USER_DATA_0_ADDR)

/* Mag_X(MSB) Description - Reg Addr --> (0x05), Bit --> 0...7 */
#define BMI160_USER_DATA_1_MAG_X_MSB__POS           (0)
#define BMI160_USER_DATA_1_MAG_X_MSB__LEN           (8)
#define BMI160_USER_DATA_1_MAG_X_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_1_MAG_X_MSB__REG          (BMI160_USER_DATA_1_ADDR)

/* Mag_Y(LSB) Description - Reg Addr --> (0x06), Bit --> 0...7 */
#define BMI160_USER_DATA_2_MAG_Y_LSB__POS           (0)
#define BMI160_USER_DATA_2_MAG_Y_LSB__LEN           (8)
#define BMI160_USER_DATA_2_MAG_Y_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_2_MAG_Y_LSB__REG          (BMI160_USER_DATA_2_ADDR)

/* Mag_Y(LSB) Description - Reg Addr --> (0x06), Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Y_LSB__POS           (3)
#define BMI160_USER_DATA_MAG_Y_LSB__LEN           (5)
#define BMI160_USER_DATA_MAG_Y_LSB__MSK          (0xF8)
#define BMI160_USER_DATA_MAG_Y_LSB__REG          (BMI160_USER_DATA_2_ADDR)

/* Mag_Y(MSB) Description - Reg Addr --> (0x07), Bit --> 0...7 */
#define BMI160_USER_DATA_3_MAG_Y_MSB__POS           (0)
#define BMI160_USER_DATA_3_MAG_Y_MSB__LEN           (8)
#define BMI160_USER_DATA_3_MAG_Y_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_3_MAG_Y_MSB__REG          (BMI160_USER_DATA_3_ADDR)

/* Mag_Z(LSB) Description - Reg Addr --> (0x08), Bit --> 0...7 */
#define BMI160_USER_DATA_4_MAG_Z_LSB__POS           (0)
#define BMI160_USER_DATA_4_MAG_Z_LSB__LEN           (8)
#define BMI160_USER_DATA_4_MAG_Z_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_4_MAG_Z_LSB__REG          (BMI160_USER_DATA_4_ADDR)

/* Mag_X(LSB) Description - Reg Addr --> (0x08), Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Z_LSB__POS           (1)
#define BMI160_USER_DATA_MAG_Z_LSB__LEN           (7)
#define BMI160_USER_DATA_MAG_Z_LSB__MSK          (0xFE)
#define BMI160_USER_DATA_MAG_Z_LSB__REG          (BMI160_USER_DATA_4_ADDR)

/* Mag_Z(MSB) Description - Reg Addr --> (0x09), Bit --> 0...7 */
#define BMI160_USER_DATA_5_MAG_Z_MSB__POS           (0)
#define BMI160_USER_DATA_5_MAG_Z_MSB__LEN           (8)
#define BMI160_USER_DATA_5_MAG_Z_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_5_MAG_Z_MSB__REG          (BMI160_USER_DATA_5_ADDR)

/* RHALL(LSB) Description - Reg Addr --> (0x0A), Bit --> 0...7 */
#define BMI160_USER_DATA_6_RHALL_LSB__POS           (0)
#define BMI160_USER_DATA_6_RHALL_LSB__LEN           (8)
#define BMI160_USER_DATA_6_RHALL_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_6_RHALL_LSB__REG          (BMI160_USER_DATA_6_ADDR)

/* Mag_R(LSB) Description - Reg Addr --> (0x0A), Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_R_LSB__POS           (2)
#define BMI160_USER_DATA_MAG_R_LSB__LEN           (6)
#define BMI160_USER_DATA_MAG_R_LSB__MSK          (0xFC)
#define BMI160_USER_DATA_MAG_R_LSB__REG          (BMI160_USER_DATA_6_ADDR)

/* RHALL(MSB) Description - Reg Addr --> (0x0B), Bit --> 0...7 */
#define BMI160_USER_DATA_7_RHALL_MSB__POS           (0)
#define BMI160_USER_DATA_7_RHALL_MSB__LEN           (8)
#define BMI160_USER_DATA_7_RHALL_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_7_RHALL_MSB__REG          (BMI160_USER_DATA_7_ADDR)
/**************************************************************/
/**\name	GYRO DATA XYZ LENGTH, POSITION AND MASK    */
/**************************************************************/
/* GYR_X (LSB) Description - Reg Addr --> (0x0C), Bit --> 0...7 */
#define BMI160_USER_DATA_8_GYRO_X_LSB__POS           (0)
#define BMI160_USER_DATA_8_GYRO_X_LSB__LEN           (8)
#define BMI160_USER_DATA_8_GYRO_X_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_8_GYRO_X_LSB__REG          (BMI160_USER_DATA_8_ADDR)

/* GYR_X (MSB) Description - Reg Addr --> (0x0D), Bit --> 0...7 */
#define BMI160_USER_DATA_9_GYRO_X_MSB__POS           (0)
#define BMI160_USER_DATA_9_GYRO_X_MSB__LEN           (8)
#define BMI160_USER_DATA_9_GYRO_X_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_9_GYRO_X_MSB__REG          (BMI160_USER_DATA_9_ADDR)

/* GYR_Y (LSB) Description - Reg Addr --> 0x0E, Bit --> 0...7 */
#define BMI160_USER_DATA_10_GYRO_Y_LSB__POS           (0)
#define BMI160_USER_DATA_10_GYRO_Y_LSB__LEN           (8)
#define BMI160_USER_DATA_10_GYRO_Y_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_10_GYRO_Y_LSB__REG          (BMI160_USER_DATA_10_ADDR)

/* GYR_Y (MSB) Description - Reg Addr --> (0x0F), Bit --> 0...7 */
#define BMI160_USER_DATA_11_GYRO_Y_MSB__POS           (0)
#define BMI160_USER_DATA_11_GYRO_Y_MSB__LEN           (8)
#define BMI160_USER_DATA_11_GYRO_Y_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_11_GYRO_Y_MSB__REG          (BMI160_USER_DATA_11_ADDR)

/* GYR_Z (LSB) Description - Reg Addr --> (0x10), Bit --> 0...7 */
#define BMI160_USER_DATA_12_GYRO_Z_LSB__POS           (0)
#define BMI160_USER_DATA_12_GYRO_Z_LSB__LEN           (8)
#define BMI160_USER_DATA_12_GYRO_Z_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_12_GYRO_Z_LSB__REG          (BMI160_USER_DATA_12_ADDR)

/* GYR_Z (MSB) Description - Reg Addr --> (0x11), Bit --> 0...7 */
#define BMI160_USER_DATA_13_GYRO_Z_MSB__POS           (0)
#define BMI160_USER_DATA_13_GYRO_Z_MSB__LEN           (8)
#define BMI160_USER_DATA_13_GYRO_Z_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_13_GYRO_Z_MSB__REG          (BMI160_USER_DATA_13_ADDR)
/**************************************************************/
/**\name	ACCEL DATA XYZ LENGTH, POSITION AND MASK    */
/**************************************************************/
/* ACC_X (LSB) Description - Reg Addr --> (0x12), Bit --> 0...7 */
#define BMI160_USER_DATA_14_ACCEL_X_LSB__POS           (0)
#define BMI160_USER_DATA_14_ACCEL_X_LSB__LEN           (8)
#define BMI160_USER_DATA_14_ACCEL_X_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_14_ACCEL_X_LSB__REG          (BMI160_USER_DATA_14_ADDR)

/* ACC_X (MSB) Description - Reg Addr --> 0x13, Bit --> 0...7 */
#define BMI160_USER_DATA_15_ACCEL_X_MSB__POS           (0)
#define BMI160_USER_DATA_15_ACCEL_X_MSB__LEN           (8)
#define BMI160_USER_DATA_15_ACCEL_X_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_15_ACCEL_X_MSB__REG          (BMI160_USER_DATA_15_ADDR)

/* ACC_Y (LSB) Description - Reg Addr --> (0x14), Bit --> 0...7 */
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__POS           (0)
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__LEN           (8)
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__REG          (BMI160_USER_DATA_16_ADDR)

/* ACC_Y (MSB) Description - Reg Addr --> (0x15), Bit --> 0...7 */
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__POS           (0)
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__LEN           (8)
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__REG          (BMI160_USER_DATA_17_ADDR)

/* ACC_Z (LSB) Description - Reg Addr --> 0x16, Bit --> 0...7 */
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__POS           (0)
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__LEN           (8)
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__MSK          (0xFF)
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__REG          (BMI160_USER_DATA_18_ADDR)

/* ACC_Z (MSB) Description - Reg Addr --> (0x17), Bit --> 0...7 */
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__POS           (0)
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__LEN           (8)
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__MSK          (0xFF)
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__REG          (BMI160_USER_DATA_19_ADDR)
/**************************************************************/
/**\name	SENSOR TIME LENGTH, POSITION AND MASK    */
/**************************************************************/
/* SENSORTIME_0 (LSB) Description - Reg Addr --> (0x18), Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__POS           (0)
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__LEN           (8)
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__MSK          (0xFF)
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__REG          \
		(BMI160_USER_SENSORTIME_0_ADDR)

/* SENSORTIME_1 (MSB) Description - Reg Addr --> (0x19), Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__POS           (0)
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__LEN           (8)
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__MSK          (0xFF)
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__REG          \
		(BMI160_USER_SENSORTIME_1_ADDR)

/* SENSORTIME_2 (MSB) Description - Reg Addr --> (0x1A), Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__POS           (0)
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__LEN           (8)
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__MSK          (0xFF)
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__REG          \
		(BMI160_USER_SENSORTIME_2_ADDR)
/**************************************************************/
/**\name	GYRO SELF TEST LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Status Description - Reg Addr --> 0x1B, Bit --> 1 */
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__POS          (1)
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__LEN          (1)
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__MSK          (0x02)
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__REG         \
		(BMI160_USER_STAT_ADDR)
/**************************************************************/
/**\name	MAG MANUAL OPERATION LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Status Description - Reg Addr --> 0x1B, Bit --> 2 */
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__POS          (2)
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__LEN          (1)
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__MSK          (0x04)
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__REG          \
		(BMI160_USER_STAT_ADDR)
/**************************************************************/
/**\name	FOC STATUS LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Status Description - Reg Addr --> 0x1B, Bit --> 3 */
#define BMI160_USER_STAT_FOC_RDY__POS          (3)
#define BMI160_USER_STAT_FOC_RDY__LEN          (1)
#define BMI160_USER_STAT_FOC_RDY__MSK          (0x08)
#define BMI160_USER_STAT_FOC_RDY__REG          (BMI160_USER_STAT_ADDR)
/**************************************************************/
/**\name	NVM READY LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Status Description - Reg Addr --> 0x1B, Bit --> 4 */
#define BMI160_USER_STAT_NVM_RDY__POS           (4)
#define BMI160_USER_STAT_NVM_RDY__LEN           (1)
#define BMI160_USER_STAT_NVM_RDY__MSK           (0x10)
#define BMI160_USER_STAT_NVM_RDY__REG           (BMI160_USER_STAT_ADDR)
/**************************************************************/
/**\name	DATA READY LENGTH, POSITION AND MASK FOR ACCEL, MAG AND GYRO*/
/**************************************************************/
/* Status Description - Reg Addr --> 0x1B, Bit --> 5 */
#define BMI160_USER_STAT_DATA_RDY_MAG__POS           (5)
#define BMI160_USER_STAT_DATA_RDY_MAG__LEN           (1)
#define BMI160_USER_STAT_DATA_RDY_MAG__MSK           (0x20)
#define BMI160_USER_STAT_DATA_RDY_MAG__REG           (BMI160_USER_STAT_ADDR)

/* Status Description - Reg Addr --> 0x1B, Bit --> 6 */
#define BMI160_USER_STAT_DATA_RDY_GYRO__POS           (6)
#define BMI160_USER_STAT_DATA_RDY_GYRO__LEN           (1)
#define BMI160_USER_STAT_DATA_RDY_GYRO__MSK           (0x40)
#define BMI160_USER_STAT_DATA_RDY_GYRO__REG           (BMI160_USER_STAT_ADDR)

/* Status Description - Reg Addr --> 0x1B, Bit --> 7 */
#define BMI160_USER_STAT_DATA_RDY_ACCEL__POS           (7)
#define BMI160_USER_STAT_DATA_RDY_ACCEL__LEN           (1)
#define BMI160_USER_STAT_DATA_RDY_ACCEL__MSK           (0x80)
#define BMI160_USER_STAT_DATA_RDY_ACCEL__REG           (BMI160_USER_STAT_ADDR)
/**************************************************************/
/**\name	INTERRUPT STATUS LENGTH, POSITION AND MASK    */
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 0 */
#define BMI160_USER_INTR_STAT_0_STEP_INTR__POS           (0)
#define BMI160_USER_INTR_STAT_0_STEP_INTR__LEN           (1)
#define BMI160_USER_INTR_STAT_0_STEP_INTR__MSK          (0x01)
#define BMI160_USER_INTR_STAT_0_STEP_INTR__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	SIGNIFICANT INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 1 */
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__POS		(1)
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__LEN		(1)
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__MSK		(0x02)
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__REG       \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	ANY_MOTION INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 2 */
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__POS           (2)
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__LEN           (1)
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__MSK          (0x04)
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	PMU TRIGGER INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 3 */
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__POS           3
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__LEN           (1)
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__MSK          (0x08)
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	DOUBLE TAP INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 4 */
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__POS           4
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__LEN           (1)
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__MSK          (0x10)
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	SINGLE TAP INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 5 */
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__POS           5
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__LEN           (1)
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__MSK          (0x20)
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	ORIENT INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 6 */
#define BMI160_USER_INTR_STAT_0_ORIENT__POS           (6)
#define BMI160_USER_INTR_STAT_0_ORIENT__LEN           (1)
#define BMI160_USER_INTR_STAT_0_ORIENT__MSK          (0x40)
#define BMI160_USER_INTR_STAT_0_ORIENT__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	FLAT INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 7 */
#define BMI160_USER_INTR_STAT_0_FLAT__POS           (7)
#define BMI160_USER_INTR_STAT_0_FLAT__LEN           (1)
#define BMI160_USER_INTR_STAT_0_FLAT__MSK          (0x80)
#define BMI160_USER_INTR_STAT_0_FLAT__REG          \
		(BMI160_USER_INTR_STAT_0_ADDR)
/**************************************************************/
/**\name	HIGH_G INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 2 */
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__POS               (2)
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__MSK              (0x04)
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__REG              \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name	LOW_G INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 3 */
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__POS               (3)
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__MSK              (0x08)
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__REG              \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name	DATA READY INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 4 */
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__POS               (4)
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__MSK               (0x10)
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__REG               \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name	FIFO FULL INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 5 */
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__POS               (5)
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__MSK               (0x20)
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__REG               \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name FIFO WATERMARK INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 6 */
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__POS               (6)
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__MSK               (0x40)
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__REG               \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name	NO MOTION INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 7 */
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__POS               (7)
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__LEN               (1)
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__MSK               (0x80)
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__REG               \
		(BMI160_USER_INTR_STAT_1_ADDR)
/**************************************************************/
/**\name	ANY MOTION-XYZ AXIS INTERRUPT STATUS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__POS               (0)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__LEN               (1)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__MSK               (0x01)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__POS               (1)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__LEN               (1)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__MSK               (0x02)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 2 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__POS               (2)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__LEN               (1)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__MSK               (0x04)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)
/**************************************************************/
/**\name	ANY MOTION SIGN LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 3 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__POS               (3)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__LEN               (1)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__MSK               (0x08)
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)
/**************************************************************/
/**\name	TAP_XYZ AND SIGN LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 4 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__POS               (4)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__LEN               (1)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__MSK               (0x10)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 5 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__POS               (5)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__LEN               (1)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__MSK               (0x20)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 6 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__POS               (6)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__LEN               (1)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__MSK               (0x40)
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 7 */
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__POS               (7)
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__LEN               (1)
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__MSK               (0x80)
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT SATAUS FOR WHOLE 0x1E LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_2__POS               (0)
#define BMI160_USER_INTR_STAT_2__LEN               (8)
#define BMI160_USER_INTR_STAT_2__MSK               (0xFF)
#define BMI160_USER_INTR_STAT_2__REG               \
		(BMI160_USER_INTR_STAT_2_ADDR)
/**************************************************************/
/**\name	HIGH_G-XYZ AND SIGN LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 0 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__POS               (0)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__LEN               (1)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__MSK               (0x01)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)

/* Int_Status_3 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__POS               (1)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__LEN               (1)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__MSK               (0x02)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)

/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 2 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__POS               (2)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__LEN               (1)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__MSK               (0x04)
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)

/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 3 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__POS               (3)
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__LEN               (1)
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__MSK               (0x08)
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)
/**************************************************************/
/**\name	ORIENT XY and Z AXIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 4...5 */
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__POS               (4)
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__LEN               (2)
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__MSK               (0x30)
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)

/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 6 */
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__POS               (6)
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__LEN               (1)
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__MSK               (0x40)
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)
/**************************************************************/
/**\name	FLAT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 7 */
#define BMI160_USER_INTR_STAT_3_FLAT__POS               (7)
#define BMI160_USER_INTR_STAT_3_FLAT__LEN               (1)
#define BMI160_USER_INTR_STAT_3_FLAT__MSK               (0x80)
#define BMI160_USER_INTR_STAT_3_FLAT__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)
/**************************************************************/
/**\name	(0x1F) LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Status_3 Description - Reg Addr --> (0x1F), Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_3__POS               (0)
#define BMI160_USER_INTR_STAT_3__LEN               (8)
#define BMI160_USER_INTR_STAT_3__MSK               (0xFF)
#define BMI160_USER_INTR_STAT_3__REG               \
		(BMI160_USER_INTR_STAT_3_ADDR)
/**************************************************************/
/**\name	TEMPERATURE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Temperature Description - LSB Reg Addr --> (0x20), Bit --> 0...7 */
#define BMI160_USER_TEMP_LSB_VALUE__POS               (0)
#define BMI160_USER_TEMP_LSB_VALUE__LEN               (8)
#define BMI160_USER_TEMP_LSB_VALUE__MSK               (0xFF)
#define BMI160_USER_TEMP_LSB_VALUE__REG               \
		(BMI160_USER_TEMPERATURE_0_ADDR)

/* Temperature Description - LSB Reg Addr --> 0x21, Bit --> 0...7 */
#define BMI160_USER_TEMP_MSB_VALUE__POS               (0)
#define BMI160_USER_TEMP_MSB_VALUE__LEN               (8)
#define BMI160_USER_TEMP_MSB_VALUE__MSK               (0xFF)
#define BMI160_USER_TEMP_MSB_VALUE__REG               \
		(BMI160_USER_TEMPERATURE_1_ADDR)
/**************************************************************/
/**\name	FIFO BYTE COUNTER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Length0 Description - Reg Addr --> 0x22, Bit --> 0...7 */
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__POS           (0)
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__LEN           (8)
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__MSK          (0xFF)
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG          \
		(BMI160_USER_FIFO_LENGTH_0_ADDR)

/*Fifo_Length1 Description - Reg Addr --> 0x23, Bit --> 0...2 */
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__POS           (0)
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__LEN           3
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__MSK          (0x07)
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__REG          \
		(BMI160_USER_FIFO_LENGTH_1_ADDR)

/**************************************************************/
/**\name	FIFO DATA LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Data Description - Reg Addr --> 0x24, Bit --> 0...7 */
#define BMI160_USER_FIFO_DATA__POS           (0)
#define BMI160_USER_FIFO_DATA__LEN           (8)
#define BMI160_USER_FIFO_DATA__MSK          (0xFF)
#define BMI160_USER_FIFO_DATA__REG          (BMI160_USER_FIFO_DATA_ADDR)

/**************************************************************/
/**\name	ACCEL CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 0...3 */
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__POS               (0)
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__LEN               (4)
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__MSK               (0x0F)
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG		       \
(BMI160_USER_ACCEL_CONFIG_ADDR)

/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 4...6 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__POS               (4)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__LEN               (3)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__MSK               (0x70)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG	(BMI160_USER_ACCEL_CONFIG_ADDR)

/* Acc_Conf Description - Reg Addr --> (0x40), Bit --> 7 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__POS           (7)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__LEN           (1)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__MSK           (0x80)
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG	\
(BMI160_USER_ACCEL_CONFIG_ADDR)

/* Acc_Range Description - Reg Addr --> 0x41, Bit --> 0...3 */
#define BMI160_USER_ACCEL_RANGE__POS               (0)
#define BMI160_USER_ACCEL_RANGE__LEN               (4)
#define BMI160_USER_ACCEL_RANGE__MSK               (0x0F)
#define BMI160_USER_ACCEL_RANGE__REG              \
(BMI160_USER_ACCEL_RANGE_ADDR)
/**************************************************************/
/**\name	GYRO CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Gyro_Conf Description - Reg Addr --> (0x42), Bit --> 0...3 */
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__POS               (0)
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__LEN               (4)
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__MSK               (0x0F)
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG               \
(BMI160_USER_GYRO_CONFIG_ADDR)

/* Gyro_Conf Description - Reg Addr --> (0x42), Bit --> 4...5 */
#define BMI160_USER_GYRO_CONFIG_BW__POS               (4)
#define BMI160_USER_GYRO_CONFIG_BW__LEN               (2)
#define BMI160_USER_GYRO_CONFIG_BW__MSK               (0x30)
#define BMI160_USER_GYRO_CONFIG_BW__REG               \
(BMI160_USER_GYRO_CONFIG_ADDR)

/* Gyr_Range Description - Reg Addr --> 0x43, Bit --> 0...2 */
#define BMI160_USER_GYRO_RANGE__POS               (0)
#define BMI160_USER_GYRO_RANGE__LEN               (3)
#define BMI160_USER_GYRO_RANGE__MSK               (0x07)
#define BMI160_USER_GYRO_RANGE__REG               (BMI160_USER_GYRO_RANGE_ADDR)
/**************************************************************/
/**\name	MAG CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Mag_Conf Description - Reg Addr --> (0x44), Bit --> 0...3 */
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__POS               (0)
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__LEN               (4)
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__MSK               (0x0F)
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG               \
(BMI160_USER_MAG_CONFIG_ADDR)
/**************************************************************/
/**\name	FIFO DOWNS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 0...2 */
#define BMI160_USER_FIFO_DOWN_GYRO__POS               (0)
#define BMI160_USER_FIFO_DOWN_GYRO__LEN               (3)
#define BMI160_USER_FIFO_DOWN_GYRO__MSK               (0x07)
#define BMI160_USER_FIFO_DOWN_GYRO__REG	(BMI160_USER_FIFO_DOWN_ADDR)
/**************************************************************/
/**\name	FIFO FILTER FOR ACCEL AND GYRO LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_filt Description - Reg Addr --> 0x45, Bit --> 3 */
#define BMI160_USER_FIFO_FILTER_GYRO__POS               (3)
#define BMI160_USER_FIFO_FILTER_GYRO__LEN               (1)
#define BMI160_USER_FIFO_FILTER_GYRO__MSK               (0x08)
#define BMI160_USER_FIFO_FILTER_GYRO__REG	  (BMI160_USER_FIFO_DOWN_ADDR)

/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 4...6 */
#define BMI160_USER_FIFO_DOWN_ACCEL__POS               (4)
#define BMI160_USER_FIFO_DOWN_ACCEL__LEN               (3)
#define BMI160_USER_FIFO_DOWN_ACCEL__MSK               (0x70)
#define BMI160_USER_FIFO_DOWN_ACCEL__REG	(BMI160_USER_FIFO_DOWN_ADDR)

/* Fifo_FILT Description - Reg Addr --> 0x45, Bit --> 7 */
#define BMI160_USER_FIFO_FILTER_ACCEL__POS               (7)
#define BMI160_USER_FIFO_FILTER_ACCEL__LEN               (1)
#define BMI160_USER_FIFO_FILTER_ACCEL__MSK               (0x80)
#define BMI160_USER_FIFO_FILTER_ACCEL__REG	(BMI160_USER_FIFO_DOWN_ADDR)
/**************************************************************/
/**\name	FIFO WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_0 Description - Reg Addr --> 0x46, Bit --> 0...7 */
#define BMI160_USER_FIFO_WM__POS               (0)
#define BMI160_USER_FIFO_WM__LEN               (8)
#define BMI160_USER_FIFO_WM__MSK               (0xFF)
#define BMI160_USER_FIFO_WM__REG	(BMI160_USER_FIFO_CONFIG_0_ADDR)
/**************************************************************/
/**\name	FIFO TIME LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 1 */
#define BMI160_USER_FIFO_TIME_ENABLE__POS               (1)
#define BMI160_USER_FIFO_TIME_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_TIME_ENABLE__MSK               (0x02)
#define BMI160_USER_FIFO_TIME_ENABLE__REG	(BMI160_USER_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO TAG INTERRUPT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 2 */
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__POS               (2)
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__MSK               (0x04)
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG	(BMI160_USER_FIFO_CONFIG_1_ADDR)

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 3 */
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__POS               (3)
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__MSK               (0x08)
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG	(BMI160_USER_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO HEADER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 4 */
#define BMI160_USER_FIFO_HEADER_ENABLE__POS               (4)
#define BMI160_USER_FIFO_HEADER_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_HEADER_ENABLE__MSK               (0x10)
#define BMI160_USER_FIFO_HEADER_ENABLE__REG		         \
(BMI160_USER_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO MAG ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 5 */
#define BMI160_USER_FIFO_MAG_ENABLE__POS               (5)
#define BMI160_USER_FIFO_MAG_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_MAG_ENABLE__MSK               (0x20)
#define BMI160_USER_FIFO_MAG_ENABLE__REG		     \
(BMI160_USER_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO ACCEL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 6 */
#define BMI160_USER_FIFO_ACCEL_ENABLE__POS               (6)
#define BMI160_USER_FIFO_ACCEL_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_ACCEL_ENABLE__MSK               (0x40)
#define BMI160_USER_FIFO_ACCEL_ENABLE__REG		        \
(BMI160_USER_FIFO_CONFIG_1_ADDR)
/**************************************************************/
/**\name	FIFO GYRO ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 7 */
#define BMI160_USER_FIFO_GYRO_ENABLE__POS               (7)
#define BMI160_USER_FIFO_GYRO_ENABLE__LEN               (1)
#define BMI160_USER_FIFO_GYRO_ENABLE__MSK               (0x80)
#define BMI160_USER_FIFO_GYRO_ENABLE__REG		       \
(BMI160_USER_FIFO_CONFIG_1_ADDR)

/**************************************************************/
/**\name	MAG I2C ADDRESS SELECTION LENGTH, POSITION AND MASK*/
/**************************************************************/

/* Mag_IF_0 Description - Reg Addr --> 0x4b, Bit --> 1...7 */
#define BMI160_USER_I2C_DEVICE_ADDR__POS               (1)
#define BMI160_USER_I2C_DEVICE_ADDR__LEN               (7)
#define BMI160_USER_I2C_DEVICE_ADDR__MSK               (0xFE)
#define BMI160_USER_I2C_DEVICE_ADDR__REG	(BMI160_USER_MAG_IF_0_ADDR)
/**************************************************************/
/**\name MAG CONFIGURATION FOR SECONDARY
	INTERFACE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 0...1 */
#define BMI160_USER_MAG_BURST__POS               (0)
#define BMI160_USER_MAG_BURST__LEN               (2)
#define BMI160_USER_MAG_BURST__MSK               (0x03)
#define BMI160_USER_MAG_BURST__REG               (BMI160_USER_MAG_IF_1_ADDR)

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 2...5 */
#define BMI160_USER_MAG_OFFSET__POS               (2)
#define BMI160_USER_MAG_OFFSET__LEN               (4)
#define BMI160_USER_MAG_OFFSET__MSK               (0x3C)
#define BMI160_USER_MAG_OFFSET__REG               (BMI160_USER_MAG_IF_1_ADDR)

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 7 */
#define BMI160_USER_MAG_MANUAL_ENABLE__POS               (7)
#define BMI160_USER_MAG_MANUAL_ENABLE__LEN               (1)
#define BMI160_USER_MAG_MANUAL_ENABLE__MSK               (0x80)
#define BMI160_USER_MAG_MANUAL_ENABLE__REG               \
(BMI160_USER_MAG_IF_1_ADDR)

/* Mag_IF_2 Description - Reg Addr --> 0x4d, Bit -->0... 7 */
#define BMI160_USER_READ_ADDR__POS               (0)
#define BMI160_USER_READ_ADDR__LEN               (8)
#define BMI160_USER_READ_ADDR__MSK               (0xFF)
#define BMI160_USER_READ_ADDR__REG               (BMI160_USER_MAG_IF_2_ADDR)

/* Mag_IF_3 Description - Reg Addr --> 0x4e, Bit -->0... 7 */
#define BMI160_USER_WRITE_ADDR__POS               (0)
#define BMI160_USER_WRITE_ADDR__LEN               (8)
#define BMI160_USER_WRITE_ADDR__MSK               (0xFF)
#define BMI160_USER_WRITE_ADDR__REG               (BMI160_USER_MAG_IF_3_ADDR)

/* Mag_IF_4 Description - Reg Addr --> 0x4f, Bit -->0... 7 */
#define BMI160_USER_WRITE_DATA__POS               (0)
#define BMI160_USER_WRITE_DATA__LEN               (8)
#define BMI160_USER_WRITE_DATA__MSK               (0xFF)
#define BMI160_USER_WRITE_DATA__REG               (BMI160_USER_MAG_IF_4_ADDR)
/**************************************************************/
/**\name	ANY MOTION XYZ AXIS ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__POS               (0)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__MSK               (0x01)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG	              \
(BMI160_USER_INTR_ENABLE_0_ADDR)

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__POS               (1)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__MSK               (0x02)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG	          \
(BMI160_USER_INTR_ENABLE_0_ADDR)

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__POS               (2)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__MSK               (0x04)
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG	            \
(BMI160_USER_INTR_ENABLE_0_ADDR)
/**************************************************************/
/**\name	DOUBLE TAP ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__POS               (4)
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__MSK               (0x10)
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG	        \
(BMI160_USER_INTR_ENABLE_0_ADDR)
/**************************************************************/
/**\name	SINGLE TAP ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__POS               (5)
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__MSK               (0x20)
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG	       \
(BMI160_USER_INTR_ENABLE_0_ADDR)
/**************************************************************/
/**\name	ORIENT ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__POS               (6)
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__MSK               (0x40)
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_0_ADDR)
/**************************************************************/
/**\name	FLAT ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->7 */
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__POS               (7)
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__MSK               (0x80)
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_0_ADDR)
/**************************************************************/
/**\name	HIGH_G XYZ ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->0 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__POS               (0)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__MSK               (0x01)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_1_ADDR)

/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->1 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__POS               (1)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__MSK               (0x02)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_1_ADDR)

/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->2 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__POS               (2)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__MSK               (0x04)
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_1_ADDR)
/**************************************************************/
/**\name	LOW_G ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->3 */
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__POS               (3)
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__MSK               (0x08)
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG	          \
(BMI160_USER_INTR_ENABLE_1_ADDR)
/**************************************************************/
/**\name	DATA READY ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->4 */
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__POS               (4)
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__MSK               (0x10)
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG	            \
(BMI160_USER_INTR_ENABLE_1_ADDR)
/**************************************************************/
/**\name	FIFO FULL AND WATER MARK ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->5 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__POS               (5)
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__MSK               (0x20)
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG	              \
(BMI160_USER_INTR_ENABLE_1_ADDR)

/* Int_En_1 Description - Reg Addr --> (0x51), Bit -->6 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__POS               (6)
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__MSK               (0x40)
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG	           \
(BMI160_USER_INTR_ENABLE_1_ADDR)
/**************************************************************/
/**\name	NO MOTION XYZ ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_2 Description - Reg Addr --> (0x52), Bit -->0 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__POS               (0)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__MSK               (0x01)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG	  \
(BMI160_USER_INTR_ENABLE_2_ADDR)

/* Int_En_2 Description - Reg Addr --> (0x52), Bit -->1 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__POS               (1)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__MSK               (0x02)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG	  \
(BMI160_USER_INTR_ENABLE_2_ADDR)

/* Int_En_2 Description - Reg Addr --> (0x52), Bit -->2 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__POS               (2)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__MSK               (0x04)
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG	  \
(BMI160_USER_INTR_ENABLE_2_ADDR)
/**************************************************************/
/**\name	STEP DETECTOR ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_En_2 Description - Reg Addr --> (0x52), Bit -->3 */
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__POS               (3)
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__LEN               (1)
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__MSK               (0x08)
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG	  \
(BMI160_USER_INTR_ENABLE_2_ADDR)
/**************************************************************/
/**\name	EDGE CONTROL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->0 */
#define BMI160_USER_INTR1_EDGE_CTRL__POS               (0)
#define BMI160_USER_INTR1_EDGE_CTRL__LEN               (1)
#define BMI160_USER_INTR1_EDGE_CTRL__MSK               (0x01)
#define BMI160_USER_INTR1_EDGE_CTRL__REG		\
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	LEVEL CONTROL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->1 */
#define BMI160_USER_INTR1_LEVEL__POS               (1)
#define BMI160_USER_INTR1_LEVEL__LEN               (1)
#define BMI160_USER_INTR1_LEVEL__MSK               (0x02)
#define BMI160_USER_INTR1_LEVEL__REG               \
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	OUTPUT TYPE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->2 */
#define BMI160_USER_INTR1_OUTPUT_TYPE__POS               (2)
#define BMI160_USER_INTR1_OUTPUT_TYPE__LEN               (1)
#define BMI160_USER_INTR1_OUTPUT_TYPE__MSK               (0x04)
#define BMI160_USER_INTR1_OUTPUT_TYPE__REG               \
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	OUTPUT TYPE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->3 */
#define BMI160_USER_INTR1_OUTPUT_ENABLE__POS               (3)
#define BMI160_USER_INTR1_OUTPUT_ENABLE__LEN               (1)
#define BMI160_USER_INTR1_OUTPUT_ENABLE__MSK               (0x08)
#define BMI160_USER_INTR1_OUTPUT_ENABLE__REG		\
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	EDGE CONTROL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->4 */
#define BMI160_USER_INTR2_EDGE_CTRL__POS               (4)
#define BMI160_USER_INTR2_EDGE_CTRL__LEN               (1)
#define BMI160_USER_INTR2_EDGE_CTRL__MSK               (0x10)
#define BMI160_USER_INTR2_EDGE_CTRL__REG		\
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	LEVEL CONTROL ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->5 */
#define BMI160_USER_INTR2_LEVEL__POS               (5)
#define BMI160_USER_INTR2_LEVEL__LEN               (1)
#define BMI160_USER_INTR2_LEVEL__MSK               (0x20)
#define BMI160_USER_INTR2_LEVEL__REG               \
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	OUTPUT TYPE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->6 */
#define BMI160_USER_INTR2_OUTPUT_TYPE__POS               (6)
#define BMI160_USER_INTR2_OUTPUT_TYPE__LEN               (1)
#define BMI160_USER_INTR2_OUTPUT_TYPE__MSK               (0x40)
#define BMI160_USER_INTR2_OUTPUT_TYPE__REG               \
(BMI160_USER_INTR_OUT_CTRL_ADDR)

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->7 */
#define BMI160_USER_INTR2_OUTPUT_EN__POS               (7)
#define BMI160_USER_INTR2_OUTPUT_EN__LEN               (1)
#define BMI160_USER_INTR2_OUTPUT_EN__MSK               (0x80)
#define BMI160_USER_INTR2_OUTPUT_EN__REG		\
(BMI160_USER_INTR_OUT_CTRL_ADDR)
/**************************************************************/
/**\name	LATCH INTERRUPT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Latch Description - Reg Addr --> 0x54, Bit -->0...3 */
#define BMI160_USER_INTR_LATCH__POS               (0)
#define BMI160_USER_INTR_LATCH__LEN               (4)
#define BMI160_USER_INTR_LATCH__MSK               (0x0F)
#define BMI160_USER_INTR_LATCH__REG               (BMI160_USER_INTR_LATCH_ADDR)
/**************************************************************/
/**\name	INPUT ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Latch Description - Reg Addr --> 0x54, Bit -->4 */
#define BMI160_USER_INTR1_INPUT_ENABLE__POS               (4)
#define BMI160_USER_INTR1_INPUT_ENABLE__LEN               (1)
#define BMI160_USER_INTR1_INPUT_ENABLE__MSK               (0x10)
#define BMI160_USER_INTR1_INPUT_ENABLE__REG               \
(BMI160_USER_INTR_LATCH_ADDR)

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->5*/
#define BMI160_USER_INTR2_INPUT_ENABLE__POS               (5)
#define BMI160_USER_INTR2_INPUT_ENABLE__LEN               (1)
#define BMI160_USER_INTR2_INPUT_ENABLE__MSK               (0x20)
#define BMI160_USER_INTR2_INPUT_ENABLE__REG              \
(BMI160_USER_INTR_LATCH_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF LOW_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->0 */
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__POS               (0)
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__MSK               (0x01)
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG	(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->1 */
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__POS               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__MSK               (0x02)
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG	\
(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT MAPPIONG OF ANY MOTION_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->2 */
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__POS               (2)
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__MSK               (0x04)
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG            \
(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->3 */
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__POS               (3)
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__MSK               (0x08)
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG (BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF DOUBLE TAP LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->4 */
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__POS               (4)
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__MSK               (0x10)
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG	\
(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF SINGLE TAP LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->5 */
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__POS               (5)
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__MSK               (0x20)
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG	      \
(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF ORIENT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->6 */
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__POS               (6)
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__MSK               (0x40)
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG	          \
(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT MAPPIONG OF FLAT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_0 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__POS               (7)
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__LEN               (1)
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__MSK               (0x80)
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG	(BMI160_USER_INTR_MAP_0_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF PMU TRIGGER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->0 */
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__POS               (0)
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__MSK               (0x01)
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG (BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF FIFO FULL AND
	WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__POS               (1)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__MSK               (0x02)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG	         \
(BMI160_USER_INTR_MAP_1_ADDR)

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__POS               (2)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__MSK               (0x04)
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG	         \
(BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__POS               (3)
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__MSK               (0x08)
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG	      \
(BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF PMU TRIGGER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->4 */
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__POS               (4)
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__MSK               (0x10)
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG (BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF FIFO FULL AND
	WATER MARK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->5 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__POS               (5)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__MSK               (0x20)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG	       \
(BMI160_USER_INTR_MAP_1_ADDR)

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__POS               (6)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__MSK               (0x40)
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG	\
(BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT1 MAPPIONG OF DATA READY LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__POS               (7)
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__LEN               (1)
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__MSK               (0x80)
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG	\
(BMI160_USER_INTR_MAP_1_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF LOW_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->0 */
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__POS               (0)
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__MSK               (0x01)
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG	(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF HIGH_G LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->1 */
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__POS               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__MSK               (0x02)
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG	\
(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ANY MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->2 */
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__POS      (2)
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__LEN      (1)
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__MSK     (0x04)
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG     \
(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF NO MOTION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->3 */
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__POS               (3)
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__MSK               (0x08)
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG (BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF DOUBLE TAP LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->4 */
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__POS               (4)
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__MSK               (0x10)
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG	\
(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF SINGLE TAP LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->5 */
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__POS               (5)
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__MSK               (0x20)
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG	\
(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF ORIENT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->6 */
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__POS               (6)
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__MSK               (0x40)
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG	\
(BMI160_USER_INTR_MAP_2_ADDR)
/**************************************************************/
/**\name	INTERRUPT2 MAPPIONG OF FLAT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->7 */
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__POS               (7)
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__LEN               (1)
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__MSK               (0x80)
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG	(BMI160_USER_INTR_MAP_2_ADDR)

/**************************************************************/
/**\name	TAP SOURCE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 3 */
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__POS               (3)
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__LEN               (1)
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__MSK               (0x08)
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG	           \
(BMI160_USER_INTR_DATA_0_ADDR)

/**************************************************************/
/**\name	HIGH SOURCE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 7 */
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__POS           (7)
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__LEN           (1)
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__MSK           (0x80)
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG            \
(BMI160_USER_INTR_DATA_0_ADDR)

/**************************************************************/
/**\name	MOTION SOURCE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Data_1 Description - Reg Addr --> 0x59, Bit --> 7 */
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__POS               (7)
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__LEN               (1)
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__MSK               (0x80)
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG               \
		(BMI160_USER_INTR_DATA_1_ADDR)
/**************************************************************/
/**\name	LOW HIGH DURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_0 Description - Reg Addr --> 0x5a, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__POS               (0)
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__LEN               (8)
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__MSK               (0xFF)
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG               \
		(BMI160_USER_INTR_LOWHIGH_0_ADDR)
/**************************************************************/
/**\name	LOW THRESHOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_1 Description - Reg Addr --> 0x5b, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__POS               (0)
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__LEN               (8)
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__MSK               (0xFF)
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG               \
		(BMI160_USER_INTR_LOWHIGH_1_ADDR)
/**************************************************************/
/**\name	LOW HYSTERESIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 0...1 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__POS               (0)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__LEN               (2)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__MSK               (0x03)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG               \
		(BMI160_USER_INTR_LOWHIGH_2_ADDR)
/**************************************************************/
/**\name	LOW MODE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 2 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__POS               (2)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__LEN               (1)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__MSK               (0x04)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG               \
		(BMI160_USER_INTR_LOWHIGH_2_ADDR)
/**************************************************************/
/**\name	HIGH_G HYSTERESIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 6...7 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__POS               (6)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__LEN               (2)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__MSK               (0xC0)
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG               \
		(BMI160_USER_INTR_LOWHIGH_2_ADDR)
/**************************************************************/
/**\name	HIGH_G DURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_3 Description - Reg Addr --> 0x5d, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__POS               (0)
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__LEN               (8)
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__MSK               (0xFF)
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG               \
		(BMI160_USER_INTR_LOWHIGH_3_ADDR)
/**************************************************************/
/**\name	HIGH_G THRESHOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_LowHigh_4 Description - Reg Addr --> 0x5e, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__POS               (0)
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__LEN               (8)
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__MSK               (0xFF)
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG               \
		(BMI160_USER_INTR_LOWHIGH_4_ADDR)
/**************************************************************/
/**\name	ANY MOTION DURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 0...1 */
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__POS               (0)
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__LEN               (2)
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__MSK               (0x03)
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG               \
		(BMI160_USER_INTR_MOTION_0_ADDR)
/**************************************************************/
/**\name	SLOW/NO MOTION DURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
	/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 2...7 */
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__POS      (2)
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__LEN      (6)
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__MSK      (0xFC)
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG       \
		(BMI160_USER_INTR_MOTION_0_ADDR)
/**************************************************************/
/**\name	ANY MOTION THRESHOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Motion_1 Description - Reg Addr --> (0x60), Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__POS      (0)
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__LEN      (8)
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__MSK      (0xFF)
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG               \
		(BMI160_USER_INTR_MOTION_1_ADDR)
/**************************************************************/
/**\name	SLOW/NO MOTION THRESHOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Motion_2 Description - Reg Addr --> 0x61, Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__POS       (0)
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__LEN       (8)
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__MSK       (0xFF)
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG       \
		(BMI160_USER_INTR_MOTION_2_ADDR)
/**************************************************************/
/**\name	SLOW/NO MOTION SELECT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Motion_3 Description - Reg Addr --> (0x62), Bit --> 0 */
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__POS	(0)
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__LEN	(1)
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__MSK	(0x01)
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG   \
(BMI160_USER_INTR_MOTION_3_ADDR)
/**************************************************************/
/**\name	SIGNIFICANT MOTION SELECT LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Motion_3 Description - Reg Addr --> (0x62), Bit --> 1 */
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__POS		(1)
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__LEN		(1)
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__MSK		(0x02)
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG		\
		(BMI160_USER_INTR_MOTION_3_ADDR)

/* Int_Motion_3 Description - Reg Addr --> (0x62), Bit --> 3..2 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__POS		(2)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__LEN		(2)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__MSK		(0x0C)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG		\
		(BMI160_USER_INTR_MOTION_3_ADDR)

/* Int_Motion_3 Description - Reg Addr --> (0x62), Bit --> 5..4 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__POS		(4)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__LEN		(2)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__MSK		(0x30)
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG		\
		(BMI160_USER_INTR_MOTION_3_ADDR)
/**************************************************************/
/**\name	TAP DURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* INT_TAP_0 Description - Reg Addr --> (0x63), Bit --> 0..2*/
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__POS               (0)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__LEN               (3)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__MSK               (0x07)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG	\
(BMI160_USER_INTR_TAP_0_ADDR)
/**************************************************************/
/**\name	TAP SHOCK LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Tap_0 Description - Reg Addr --> (0x63), Bit --> 6 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__POS               (6)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__LEN               (1)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__MSK               (0x40)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG (BMI160_USER_INTR_TAP_0_ADDR)
/**************************************************************/
/**\name	TAP QUIET LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Tap_0 Description - Reg Addr --> (0x63), Bit --> 7 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__POS               (7)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__LEN               (1)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__MSK               (0x80)
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG (BMI160_USER_INTR_TAP_0_ADDR)
/**************************************************************/
/**\name	TAP THRESHOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Tap_1 Description - Reg Addr --> (0x64), Bit --> 0...4 */
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__POS               (0)
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__LEN               (5)
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__MSK               (0x1F)
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG (BMI160_USER_INTR_TAP_1_ADDR)
/**************************************************************/
/**\name	ORIENT MODE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_0 Description - Reg Addr --> (0x65), Bit --> 0...1 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__POS               (0)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__LEN               (2)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__MSK               (0x03)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG               \
		(BMI160_USER_INTR_ORIENT_0_ADDR)
/**************************************************************/
/**\name	ORIENT BLOCKING LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_0 Description - Reg Addr --> (0x65), Bit --> 2...3 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__POS               (2)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__LEN               (2)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__MSK               (0x0C)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG               \
		(BMI160_USER_INTR_ORIENT_0_ADDR)
/**************************************************************/
/**\name	ORIENT HYSTERESIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_0 Description - Reg Addr --> (0x65), Bit --> 4...7 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__POS               (4)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__LEN               (4)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__MSK               (0xF0)
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG               \
		(BMI160_USER_INTR_ORIENT_0_ADDR)
/**************************************************************/
/**\name	ORIENT THETA LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 0...5 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__POS               (0)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__LEN               (6)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__MSK               (0x3F)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG               \
		(BMI160_USER_INTR_ORIENT_1_ADDR)
/**************************************************************/
/**\name	ORIENT UD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 6 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__POS         (6)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__LEN         (1)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__MSK         (0x40)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG          \
		(BMI160_USER_INTR_ORIENT_1_ADDR)
/**************************************************************/
/**\name	ORIENT AXIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 7 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__POS               (7)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__LEN               (1)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__MSK               (0x80)
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG               \
		(BMI160_USER_INTR_ORIENT_1_ADDR)
/**************************************************************/
/**\name	FLAT THETA LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Flat_0 Description - Reg Addr --> 0x67, Bit --> 0...5 */
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__POS               (0)
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__LEN               (6)
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__MSK               (0x3F)
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG  \
		(BMI160_USER_INTR_FLAT_0_ADDR)
/**************************************************************/
/**\name	FLAT HYSTERESIS LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Flat_1 Description - Reg Addr --> (0x68), Bit --> 0...3 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__POS		(0)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__LEN		(4)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__MSK		(0x0F)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG	 \
(BMI160_USER_INTR_FLAT_1_ADDR)
/**************************************************************/
/**\name	FLAT HOLD LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Int_Flat_1 Description - Reg Addr --> (0x68), Bit --> 4...5 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__POS                (4)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__LEN                (2)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__MSK                (0x30)
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG  \
(BMI160_USER_INTR_FLAT_1_ADDR)
/**************************************************************/
/**\name	FOC ACCEL XYZ LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 0...1 */
#define BMI160_USER_FOC_ACCEL_Z__POS               (0)
#define BMI160_USER_FOC_ACCEL_Z__LEN               (2)
#define BMI160_USER_FOC_ACCEL_Z__MSK               (0x03)
#define BMI160_USER_FOC_ACCEL_Z__REG               (BMI160_USER_FOC_CONFIG_ADDR)

/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 2...3 */
#define BMI160_USER_FOC_ACCEL_Y__POS               (2)
#define BMI160_USER_FOC_ACCEL_Y__LEN               (2)
#define BMI160_USER_FOC_ACCEL_Y__MSK               (0x0C)
#define BMI160_USER_FOC_ACCEL_Y__REG               (BMI160_USER_FOC_CONFIG_ADDR)

/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 4...5 */
#define BMI160_USER_FOC_ACCEL_X__POS               (4)
#define BMI160_USER_FOC_ACCEL_X__LEN               (2)
#define BMI160_USER_FOC_ACCEL_X__MSK               (0x30)
#define BMI160_USER_FOC_ACCEL_X__REG               (BMI160_USER_FOC_CONFIG_ADDR)
/**************************************************************/
/**\name	FOC GYRO LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Foc_Conf Description - Reg Addr --> (0x69), Bit --> 6 */
#define BMI160_USER_FOC_GYRO_ENABLE__POS               (6)
#define BMI160_USER_FOC_GYRO_ENABLE__LEN               (1)
#define BMI160_USER_FOC_GYRO_ENABLE__MSK               (0x40)
#define BMI160_USER_FOC_GYRO_ENABLE__REG               \
(BMI160_USER_FOC_CONFIG_ADDR)
/**************************************************************/
/**\name	NVM PROGRAM LENGTH, POSITION AND MASK*/
/**************************************************************/
/* CONF Description - Reg Addr --> (0x6A), Bit --> 1 */
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__POS               (1)
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__LEN               (1)
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__MSK               (0x02)
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG               \
(BMI160_USER_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x6B), Bit --> 0 */

#define BMI160_USER_IF_CONFIG_SPI3__POS               (0)
#define BMI160_USER_IF_CONFIG_SPI3__LEN               (1)
#define BMI160_USER_IF_CONFIG_SPI3__MSK               (0x01)
#define BMI160_USER_IF_CONFIG_SPI3__REG               \
(BMI160_USER_IF_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x6B), Bit --> 5..4 */
#define BMI160_USER_IF_CONFIG_IF_MODE__POS               (4)
#define BMI160_USER_IF_CONFIG_IF_MODE__LEN               (2)
#define BMI160_USER_IF_CONFIG_IF_MODE__MSK               (0x30)
#define BMI160_USER_IF_CONFIG_IF_MODE__REG		\
(BMI160_USER_IF_CONFIG_ADDR)
/**************************************************************/
/**\name	GYRO SLEEP CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 0...2 */
#define BMI160_USER_GYRO_SLEEP_TRIGGER__POS               (0)
#define BMI160_USER_GYRO_SLEEP_TRIGGER__LEN               (3)
#define BMI160_USER_GYRO_SLEEP_TRIGGER__MSK               (0x07)
#define BMI160_USER_GYRO_SLEEP_TRIGGER__REG	(BMI160_USER_PMU_TRIGGER_ADDR)

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 3...4 */
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__POS               (3)
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__LEN               (2)
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__MSK               (0x18)
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__REG	(BMI160_USER_PMU_TRIGGER_ADDR)

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 5 */
#define BMI160_USER_GYRO_SLEEP_STATE__POS               (5)
#define BMI160_USER_GYRO_SLEEP_STATE__LEN               (1)
#define BMI160_USER_GYRO_SLEEP_STATE__MSK               (0x20)
#define BMI160_USER_GYRO_SLEEP_STATE__REG	(BMI160_USER_PMU_TRIGGER_ADDR)

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 6 */
#define BMI160_USER_GYRO_WAKEUP_INTR__POS               (6)
#define BMI160_USER_GYRO_WAKEUP_INTR__LEN               (1)
#define BMI160_USER_GYRO_WAKEUP_INTR__MSK               (0x40)
#define BMI160_USER_GYRO_WAKEUP_INTR__REG	(BMI160_USER_PMU_TRIGGER_ADDR)
/**************************************************************/
/**\name	ACCEL SELF TEST LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 0...1 */
#define BMI160_USER_ACCEL_SELFTEST_AXIS__POS               (0)
#define BMI160_USER_ACCEL_SELFTEST_AXIS__LEN               (2)
#define BMI160_USER_ACCEL_SELFTEST_AXIS__MSK               (0x03)
#define BMI160_USER_ACCEL_SELFTEST_AXIS__REG	(BMI160_USER_SELF_TEST_ADDR)

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 2 */
#define BMI160_USER_ACCEL_SELFTEST_SIGN__POS               (2)
#define BMI160_USER_ACCEL_SELFTEST_SIGN__LEN               (1)
#define BMI160_USER_ACCEL_SELFTEST_SIGN__MSK               (0x04)
#define BMI160_USER_ACCEL_SELFTEST_SIGN__REG	(BMI160_USER_SELF_TEST_ADDR)

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 3 */
#define BMI160_USER_SELFTEST_AMP__POS               (3)
#define BMI160_USER_SELFTEST_AMP__LEN               (1)
#define BMI160_USER_SELFTEST_AMP__MSK               (0x08)
#define BMI160_USER_SELFTEST_AMP__REG		(BMI160_USER_SELF_TEST_ADDR)
/**************************************************************/
/**\name	GYRO SELF TEST LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 4 */
#define BMI160_USER_GYRO_SELFTEST_START__POS               (4)
#define BMI160_USER_GYRO_SELFTEST_START__LEN               (1)
#define BMI160_USER_GYRO_SELFTEST_START__MSK               (0x10)
#define BMI160_USER_GYRO_SELFTEST_START__REG		    \
(BMI160_USER_SELF_TEST_ADDR)
/**************************************************************/
/**\name	NV_CONFIG LENGTH, POSITION AND MASK*/
/**************************************************************/
/* NV_CONF Description - Reg Addr --> (0x70), Bit --> 0 */
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__POS               (0)
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__LEN               (1)
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__MSK               (0x01)
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__REG	 (BMI160_USER_NV_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x70), Bit --> 1 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__POS               (1)
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__LEN               (1)
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__MSK               (0x02)
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG		\
(BMI160_USER_NV_CONFIG_ADDR)

/*IF_CONF Description - Reg Addr --> (0x70), Bit --> 2 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__POS               (2)
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__LEN               (1)
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__MSK               (0x04)
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG		\
(BMI160_USER_NV_CONFIG_ADDR)

/* NV_CONF Description - Reg Addr --> (0x70), Bit --> 3 */
#define BMI160_USER_NV_CONFIG_SPARE0__POS               (3)
#define BMI160_USER_NV_CONFIG_SPARE0__LEN               (1)
#define BMI160_USER_NV_CONFIG_SPARE0__MSK               (0x08)
#define BMI160_USER_NV_CONFIG_SPARE0__REG	(BMI160_USER_NV_CONFIG_ADDR)

/* NV_CONF Description - Reg Addr --> (0x70), Bit --> 4...7 */
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__POS               (4)
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__LEN               (4)
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__MSK               (0xF0)
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__REG	(BMI160_USER_NV_CONFIG_ADDR)
/**************************************************************/
/**\name	ACCEL MANUAL OFFSET LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_0 Description - Reg Addr --> (0x71), Bit --> 0...7 */
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__POS               (0)
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__LEN               (8)
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__MSK               (0xFF)
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG	(BMI160_USER_OFFSET_0_ADDR)

/* Offset_1 Description - Reg Addr --> 0x72, Bit --> 0...7 */
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__POS               (0)
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__LEN               (8)
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__MSK               (0xFF)
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG	(BMI160_USER_OFFSET_1_ADDR)

/* Offset_2 Description - Reg Addr --> 0x73, Bit --> 0...7 */
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__POS               (0)
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__LEN               (8)
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__MSK               (0xFF)
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG	(BMI160_USER_OFFSET_2_ADDR)
/**************************************************************/
/**\name	GYRO MANUAL OFFSET LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_3 Description - Reg Addr --> 0x74, Bit --> 0...7 */
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__POS               (0)
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__LEN               (8)
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__MSK               (0xFF)
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__REG	(BMI160_USER_OFFSET_3_ADDR)

/* Offset_4 Description - Reg Addr --> 0x75, Bit --> 0...7 */
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__POS               (0)
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__LEN               (8)
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__MSK               (0xFF)
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG	(BMI160_USER_OFFSET_4_ADDR)

/* Offset_5 Description - Reg Addr --> 0x76, Bit --> 0...7 */
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__POS               (0)
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__LEN               (8)
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__MSK               (0xFF)
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG	(BMI160_USER_OFFSET_5_ADDR)


/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 0..1 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__POS               (0)
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__LEN               (2)
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__MSK               (0x03)
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__REG	(BMI160_USER_OFFSET_6_ADDR)

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 2...3 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__POS               (2)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__LEN               (2)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__MSK               (0x0C)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG	(BMI160_USER_OFFSET_6_ADDR)

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 4...5 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__POS               (4)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__LEN               (2)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__MSK               (0x30)
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG	 (BMI160_USER_OFFSET_6_ADDR)
/**************************************************************/
/**\name	ACCEL OFFSET  ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 6 */
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__POS               (6)
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__LEN               (1)
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__MSK               (0x40)
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG	 \
(BMI160_USER_OFFSET_6_ADDR)
/**************************************************************/
/**\name	GYRO OFFSET  ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Offset_6 Description - Reg Addr --> 0x77, Bit -->  7 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__POS               (7)
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__LEN               (1)
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__MSK               (0x80)
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG	 (BMI160_USER_OFFSET_6_ADDR)
/**************************************************************/
/**\name	STEP COUNTER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* STEP_CNT_0  Description - Reg Addr --> 0x78, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_LSB__POS               (0)
#define BMI160_USER_STEP_COUNT_LSB__LEN               (7)
#define BMI160_USER_STEP_COUNT_LSB__MSK               (0xFF)
#define BMI160_USER_STEP_COUNT_LSB__REG	 (BMI160_USER_STEP_COUNT_0_ADDR)

/* STEP_CNT_1  Description - Reg Addr --> 0x79, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_MSB__POS               (0)
#define BMI160_USER_STEP_COUNT_MSB__LEN               (7)
#define BMI160_USER_STEP_COUNT_MSB__MSK               (0xFF)
#define BMI160_USER_STEP_COUNT_MSB__REG	 (BMI160_USER_STEP_COUNT_1_ADDR)
/**************************************************************/
/**\name	STEP COUNTER CONFIGURATION LENGTH, POSITION AND MASK*/
/**************************************************************/
/* STEP_CONFIG_0  Description - Reg Addr --> 0x7A, Bit -->  0 to 7 */
#define BMI160_USER_STEP_CONFIG_ZERO__POS               (0)
#define BMI160_USER_STEP_CONFIG_ZERO__LEN               (7)
#define BMI160_USER_STEP_CONFIG_ZERO__MSK               (0xFF)
#define BMI160_USER_STEP_CONFIG_ZERO__REG	 \
(BMI160_USER_STEP_CONFIG_0_ADDR)


/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 and
4 to 7 */
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__POS               (0)
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__LEN               (3)
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__MSK               (0x07)
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__REG	 \
(BMI160_USER_STEP_CONFIG_1_ADDR)

#define BMI160_USER_STEP_CONFIG_ONE_CNF2__POS               (4)
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__LEN               (4)
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__MSK               (0xF0)
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__REG	 \
(BMI160_USER_STEP_CONFIG_1_ADDR)
/**************************************************************/
/**\name	STEP COUNTER ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 */
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__POS		(3)
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__LEN		(1)
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__MSK		(0x08)
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG	\
(BMI160_USER_STEP_CONFIG_1_ADDR)

/* USER REGISTERS DEFINITION END */
/**************************************************************************/
/* CMD REGISTERS DEFINITION START */
/**************************************************************/
/**\name	COMMAND REGISTER LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Command description address - Reg Addr --> 0x7E, Bit -->  0....7 */
#define BMI160_CMD_COMMANDS__POS              (0)
#define BMI160_CMD_COMMANDS__LEN              (8)
#define BMI160_CMD_COMMANDS__MSK              (0xFF)
#define BMI160_CMD_COMMANDS__REG	 (BMI160_CMD_COMMANDS_ADDR)
/**************************************************************/
/**\name	PAGE ENABLE LENGTH, POSITION AND MASK*/
/**************************************************************/
/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_CMD_TARGET_PAGE__POS           (4)
#define BMI160_CMD_TARGET_PAGE__LEN           (2)
#define BMI160_CMD_TARGET_PAGE__MSK           (0x30)
#define BMI160_CMD_TARGET_PAGE__REG	 (BMI160_CMD_EXT_MODE_ADDR)

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_CMD_PAGING_EN__POS           (7)
#define BMI160_CMD_PAGING_EN__LEN           (1)
#define BMI160_CMD_PAGING_EN__MSK           (0x80)
#define BMI160_CMD_PAGING_EN__REG		(BMI160_CMD_EXT_MODE_ADDR)

/* Target page address - Reg Addr --> 0x7F, Bit -->  4....5 */
#define BMI160_COM_C_TRIM_FIVE__POS           (0)
#define BMI160_COM_C_TRIM_FIVE__LEN           (8)
#define BMI160_COM_C_TRIM_FIVE__MSK           (0xFF)
#define BMI160_COM_C_TRIM_FIVE__REG		(BMI160_COM_C_TRIM_FIVE_ADDR)

/**************************************************************************/
/* CMD REGISTERS DEFINITION END */

/**************************************************/
/**\name	FIFO FRAME COUNT DEFINITION           */
/*************************************************/
#define FIFO_FRAME				(1024)
#define FIFO_CONFIG_CHECK1		(0x00)
#define FIFO_CONFIG_CHECK2		(0x80)
/**************************************************/
/**\name	ACCEL RANGE          */
/*************************************************/
#define BMI160_ACCEL_RANGE_2G           (0X03)
#define BMI160_ACCEL_RANGE_4G           (0X05)
#define BMI160_ACCEL_RANGE_8G           (0X08)
#define BMI160_ACCEL_RANGE_16G          (0X0C)
/**************************************************/
/**\name	ACCEL ODR          */
/*************************************************/
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED       (0x00)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ         (0x01)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ         (0x02)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ         (0x03)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ         (0x04)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ         (0x05)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ           (0x06)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ           (0x07)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ          (0x08)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ          (0x09)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ          (0x0A)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ          (0x0B)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ         (0x0C)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0      (0x0D)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1      (0x0E)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2      (0x0F)
/**************************************************/
/**\name	ACCEL BANDWIDTH PARAMETER         */
/*************************************************/
#define BMI160_ACCEL_OSR4_AVG1			(0)
#define BMI160_ACCEL_OSR2_AVG2			(1)
#define BMI160_ACCEL_NORMAL_AVG4		(2)
#define BMI160_ACCEL_CIC_AVG8			(3)
#define BMI160_ACCEL_RES_AVG2			(4)
#define BMI160_ACCEL_RES_AVG4			(5)
#define BMI160_ACCEL_RES_AVG8			(6)
#define BMI160_ACCEL_RES_AVG16			(7)
#define BMI160_ACCEL_RES_AVG32			(8)
#define BMI160_ACCEL_RES_AVG64			(9)
#define BMI160_ACCEL_RES_AVG128			(10)

#define BMI160_US_DISABLE				(0)
#define BMI160_US_ENABLE				(1)
/**************************************************/
/**\name	GYRO ODR         */
/*************************************************/
#define BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED		(0x00)
#define BMI160_GYRO_OUTPUT_DATA_RATE_25HZ			(0x06)
#define BMI160_GYRO_OUTPUT_DATA_RATE_50HZ			(0x07)
#define BMI160_GYRO_OUTPUT_DATA_RATE_100HZ			(0x08)
#define BMI160_GYRO_OUTPUT_DATA_RATE_200HZ			(0x09)
#define BMI160_GYRO_OUTPUT_DATA_RATE_400HZ			(0x0A)
#define BMI160_GYRO_OUTPUT_DATA_RATE_800HZ			(0x0B)
#define BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ			(0x0C)
#define BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ			(0x0D)
/**************************************************/
/**\name	GYRO BANDWIDTH PARAMETER         */
/*************************************************/
#define BMI160_GYRO_OSR4_MODE		(0x00)
#define BMI160_GYRO_OSR2_MODE		(0x01)
#define BMI160_GYRO_NORMAL_MODE		(0x02)
#define BMI160_GYRO_CIC_MODE		(0x03)
/**************************************************/
/**\name	GYROSCOPE RANGE PARAMETER         */
/*************************************************/
#define BMI160_GYRO_RANGE_2000_DEG_SEC	(0x00)
#define BMI160_GYRO_RANGE_1000_DEG_SEC	(0x01)
#define BMI160_GYRO_RANGE_500_DEG_SEC	(0x02)
#define BMI160_GYRO_RANGE_250_DEG_SEC	(0x03)
#define BMI160_GYRO_RANGE_125_DEG_SEC	(0x04)

/**************************************************/
/**\name	ENABLE/DISABLE SELECTIONS        */
/*************************************************/

/* Enable accel and gyro offset */
#define ACCEL_OFFSET_ENABLE		(0x01)
#define GYRO_OFFSET_ENABLE		(0x01)

/* command register definition */
#define START_FOC_ACCEL_GYRO	(0X03)

 /* INT ENABLE 1 */
#define BMI160_ANY_MOTION_X_ENABLE       (0)
#define BMI160_ANY_MOTION_Y_ENABLE       (1)
#define BMI160_ANY_MOTION_Z_ENABLE       (2)
#define BMI160_DOUBLE_TAP_ENABLE         (4)
#define BMI160_SINGLE_TAP_ENABLE         (5)
#define BMI160_ORIENT_ENABLE             (6)
#define BMI160_FLAT_ENABLE               (7)

/* INT ENABLE 1 */
#define BMI160_HIGH_G_X_ENABLE       (0)
#define BMI160_HIGH_G_Y_ENABLE       (1)
#define BMI160_HIGH_G_Z_ENABLE       (2)
#define BMI160_LOW_G_ENABLE          (3)
#define BMI160_DATA_RDY_ENABLE       (4)
#define BMI160_FIFO_FULL_ENABLE      (5)
#define BMI160_FIFO_WM_ENABLE        (6)

/* INT ENABLE 2 */
#define  BMI160_NOMOTION_X_ENABLE	(0)
#define  BMI160_NOMOTION_Y_ENABLE	(1)
#define  BMI160_NOMOTION_Z_ENABLE	(2)

/* FOC axis selection for accel*/
#define	FOC_X_AXIS		(0)
#define	FOC_Y_AXIS		(1)
#define	FOC_Z_AXIS		(2)

/* IN OUT CONTROL */
#define BMI160_INTR1_EDGE_CTRL			(0)
#define BMI160_INTR2_EDGE_CTRL			(1)
#define BMI160_INTR1_LEVEL				(0)
#define BMI160_INTR2_LEVEL				(1)
#define BMI160_INTR1_OUTPUT_TYPE		(0)
#define BMI160_INTR2_OUTPUT_TYPE		(1)
#define BMI160_INTR1_OUTPUT_ENABLE		(0)
#define BMI160_INTR2_OUTPUT_ENABLE		(1)

#define BMI160_INTR1_INPUT_ENABLE	(0)
#define BMI160_INTR2_INPUT_ENABLE	(1)

/*  INTERRUPT MAPS    */
#define BMI160_INTR1_MAP_LOW_G			(0)
#define BMI160_INTR2_MAP_LOW_G			(1)
#define BMI160_INTR1_MAP_HIGH_G			(0)
#define BMI160_INTR2_MAP_HIGH_G			(1)
#define BMI160_INTR1_MAP_ANY_MOTION		(0)
#define BMI160_INTR2_MAP_ANY_MOTION		(1)
#define BMI160_INTR1_MAP_NOMO			(0)
#define BMI160_INTR2_MAP_NOMO			(1)
#define BMI160_INTR1_MAP_DOUBLE_TAP		(0)
#define BMI160_INTR2_MAP_DOUBLE_TAP		(1)
#define BMI160_INTR1_MAP_SINGLE_TAP		(0)
#define BMI160_INTR2_MAP_SINGLE_TAP		(1)
#define BMI160_INTR1_MAP_ORIENT			(0)
#define BMI160_INTR2_MAP_ORIENT			(1)
#define BMI160_INTR1_MAP_FLAT			(0)
#define BMI160_INTR2_MAP_FLAT			(1)
#define BMI160_INTR1_MAP_DATA_RDY		(0)
#define BMI160_INTR2_MAP_DATA_RDY		(1)
#define BMI160_INTR1_MAP_FIFO_WM		(0)
#define BMI160_INTR2_MAP_FIFO_WM		(1)
#define BMI160_INTR1_MAP_FIFO_FULL      (0)
#define BMI160_INTR2_MAP_FIFO_FULL      (1)
#define BMI160_INTR1_MAP_PMUTRIG        (0)
#define BMI160_INTR2_MAP_PMUTRIG		(1)

/* Interrupt mapping*/
#define	BMI160_MAP_INTR1		(0)
#define	BMI160_MAP_INTR2		(1)
/**************************************************/
/**\name	 TAP DURATION         */
/*************************************************/
#define BMI160_TAP_DURN_50MS     (0x00)
#define BMI160_TAP_DURN_100MS    (0x01)
#define BMI160_TAP_DURN_150MS    (0x02)
#define BMI160_TAP_DURN_200MS    (0x03)
#define BMI160_TAP_DURN_250MS    (0x04)
#define BMI160_TAP_DURN_375MS    (0x05)
#define BMI160_TAP_DURN_500MS    (0x06)
#define BMI160_TAP_DURN_700MS    (0x07)
/**************************************************/
/**\name	TAP SHOCK         */
/*************************************************/
#define BMI160_TAP_SHOCK_50MS	(0x00)
#define BMI160_TAP_SHOCK_75MS	(0x01)
/**************************************************/
/**\name	TAP QUIET        */
/*************************************************/
#define BMI160_TAP_QUIET_30MS	(0x00)
#define BMI160_TAP_QUIET_20MS	(0x01)
/**************************************************/
/**\name	STEP DETECTION SELECTION MODES      */
/*************************************************/
#define	BMI160_STEP_NORMAL_MODE			(0)
#define	BMI160_STEP_SENSITIVE_MODE		(1)
#define	BMI160_STEP_ROBUST_MODE			(2)
/**************************************************/
/**\name	STEP CONFIGURATION SELECT MODE    */
/*************************************************/
#define	STEP_CONFIG_NORMAL		(0X315)
#define	STEP_CONFIG_SENSITIVE	(0X2D)
#define	STEP_CONFIG_ROBUST		(0X71D)


/**************************************************/
/**\name	MAG POWER MODE SELECTION    */
/*************************************************/
#define	FORCE_MODE		(0)
#define	SUSPEND_MODE	(1)
#define	NORMAL_MODE		(2)
#define MAG_SUSPEND_MODE (1)
/**************************************************/
/**\name	FIFO CONFIGURATIONS    */
/*************************************************/
#define FIFO_HEADER_ENABLE			(0x01)
#define FIFO_MAG_ENABLE				(0x01)
#define FIFO_ACCEL_ENABLE			(0x01)
#define FIFO_GYRO_ENABLE			(0x01)
#define FIFO_TIME_ENABLE			(0x01)
#define FIFO_STOPONFULL_ENABLE		(0x01)
#define FIFO_WM_INTERRUPT_ENABLE	(0x01)
#define	BMI160_FIFO_INDEX_LENGTH	(1)
#define	BMI160_FIFO_TAG_INTR_MASK	(0xFC)

/* FIFO definitions*/
#define FIFO_HEAD_A        (0x84)
#define FIFO_HEAD_G        (0x88)
#define FIFO_HEAD_M        (0x90)

#define FIFO_HEAD_G_A	(0x8C)
#define FIFO_HEAD_M_A   (0x94)
#define FIFO_HEAD_M_G   (0x98)

#define FIFO_HEAD_M_G_A		(0x9C)

#define FIFO_HEAD_SENSOR_TIME			(0x44)
#define FIFO_HEAD_INPUT_CONFIG			(0x48)
#define FIFO_HEAD_SKIP_FRAME			(0x40)
#define FIFO_HEAD_OVER_READ_LSB			(0x80)
#define FIFO_HEAD_OVER_READ_MSB			(0x00)


/* FIFO 1024 byte, max fifo frame count not over 150 */

#define	FIFO_INPUT_CONFIG_OVER_LEN  ((int8_ts)-11)
#define	FIFO_OVER_READ_RETURN		((int8_ts)-10)
#define	FIFO_SENSORTIME_RETURN		((int8_ts)-9)
#define	FIFO_SKIP_OVER_LEN			((int8_ts)-8)
#define	FIFO_M_G_A_OVER_LEN			((int8_ts)-7)
#define	FIFO_M_G_OVER_LEN			((int8_ts)-6)
#define	FIFO_M_A_OVER_LEN			((int8_ts)-5)
#define	FIFO_G_A_OVER_LEN			((int8_ts)-4)
#define	FIFO_M_OVER_LEN				((int8_ts)-3)
#define	FIFO_G_OVER_LEN				((int8_ts)-2)
#define	FIFO_A_OVER_LEN				((int8_ts)-1)
/**************************************************/
/**\name	ACCEL POWER MODE    */
/*************************************************/
#define ACCEL_MODE_NORMAL	(0x11)
#define	ACCEL_LOWPOWER		(0X12)
#define	ACCEL_SUSPEND		(0X10)
/**************************************************/
/**\name	GYRO POWER MODE    */
/*************************************************/
#define GYRO_MODE_SUSPEND		(0x14)
#define GYRO_MODE_NORMAL		(0x15)
#define GYRO_MODE_FASTSTARTUP	(0x17)
/**************************************************/
/**\name	MAG POWER MODE    */
/*************************************************/
#define MAG_MODE_SUSPEND	(0x18)
#define MAG_MODE_NORMAL		(0x19)
#define MAG_MODE_LOWPOWER	(0x1A)
/**************************************************/
/**\name	ENABLE/DISABLE BIT VALUES    */
/*************************************************/
#define BMI160_ENABLE	(0x01)
#define BMI160_DISABLE	(0x00)
/**************************************************/
/**\name	INTERRUPT EDGE TRIGGER ENABLE    */
/*************************************************/
#define BMI160_EDGE		(0x01)
#define BMI160_LEVEL	(0x00)
/**************************************************/
/**\name	INTERRUPT LEVEL ENABLE    */
/*************************************************/
#define BMI160_LEVEL_LOW		(0x00)
#define BMI160_LEVEL_HIGH		(0x01)
/**************************************************/
/**\name	INTERRUPT OUTPUT ENABLE    */
/*************************************************/
#define BMI160_OPEN_DRAIN	(0x01)
#define BMI160_PUSH_PULL	(0x00)

/* interrupt output enable*/
#define BMI160_INPUT	(0x01)
#define BMI160_OUTPUT	(0x00)

/**************************************************/
/**\name	INTERRUPT TAP SOURCE ENABLE    */
/*************************************************/
#define FILTER_DATA		(0x00)
#define UNFILTER_DATA	(0x01)
/**************************************************/
/**\name	SLOW MOTION/ NO MOTION SELECT   */
/*************************************************/
#define SLOW_MOTION		(0x00)
#define NO_MOTION		(0x01)
/**************************************************/
/**\name	SIGNIFICANT MOTION SELECTION   */
/*************************************************/
#define ANY_MOTION			(0x00)
#define SIGNIFICANT_MOTION	(0x01)
/**************************************************/
/**\name	LATCH DURATION   */
/*************************************************/
#define BMI160_LATCH_DUR_NONE				(0x00)
#define BMI160_LATCH_DUR_312_5_MICRO_SEC	(0x01)
#define BMI160_LATCH_DUR_625_MICRO_SEC		(0x02)
#define BMI160_LATCH_DUR_1_25_MILLI_SEC		(0x03)
#define BMI160_LATCH_DUR_2_5_MILLI_SEC		(0x04)
#define BMI160_LATCH_DUR_5_MILLI_SEC		(0x05)
#define BMI160_LATCH_DUR_10_MILLI_SEC		(0x06)
#define BMI160_LATCH_DUR_20_MILLI_SEC		(0x07)
#define BMI160_LATCH_DUR_40_MILLI_SEC		(0x08)
#define BMI160_LATCH_DUR_80_MILLI_SEC		(0x09)
#define BMI160_LATCH_DUR_160_MILLI_SEC		(0x0A)
#define BMI160_LATCH_DUR_320_MILLI_SEC		(0x0B)
#define BMI160_LATCH_DUR_640_MILLI_SEC		(0x0C)
#define BMI160_LATCH_DUR_1_28_SEC			(0x0D)
#define BMI160_LATCH_DUR_2_56_SEC			(0x0E)
#define BMI160_LATCHED						(0x0F)
/**************************************************/
/**\name	GYRO OFFSET MASK DEFINITION   */
/*************************************************/
#define BMI160_GYRO_MANUAL_OFFSET_0_7	(0x00FF)
#define BMI160_GYRO_MANUAL_OFFSET_8_9	(0x0300)
/**************************************************/
/**\name	STEP CONFIGURATION MASK DEFINITION   */
/*************************************************/
#define BMI160_STEP_CONFIG_0_7		(0x00FF)
#define BMI160_STEP_CONFIG_8_10		(0x0700)
#define BMI160_STEP_CONFIG_11_14	(0xF000)
/**************************************************/
/**\name	DEFINITION USED FOR DIFFERENT WRITE   */
/*************************************************/
#define	BMI160_WRITE_TARGET_PAGE0	(0x00)
#define	BMI160_WRITE_TARGET_PAGE1	(0x01)
#define	BMI160_WRITE_ENABLE_PAGE1	(0x01)
#define	BMI160_MANUAL_DISABLE	    (0x00)
#define	BMI160_MANUAL_ENABLE	    (0x01)
#define	BMI160_YAS_DISABLE_RCOIL	(0x00)
#define	BMI160_ENABLE_MAG_IF_MODE	(0x02)
#define	BMI160_ENABLE_ANY_MOTION_INTR1	(0x04)
#define	BMI160_ENABLE_ANY_MOTION_INTR2	(0x04)
#define	BMI160_MAG_DATA_READ_REG        (0x04)
#define BMI160_BMM_POWER_MODE_REG		(0x06)
#define	BMI160_ENABLE_ANY_MOTION_AXIS	(0x07)
#define	BMI160_ENABLE_LOW_G             (0x08)
#define	BMI160_YAS532_ACQ_START         (0x11)
#define	BMI160_YAS_DEVICE_ID_REG        (0x80)
#define	BMI160_FIFO_GYRO_ENABLE         (0x80)
#define	BMI160_SIG_MOTION_INTR_ENABLE   (0x01)
#define	BMI160_STEP_DETECT_INTR_ENABLE  (0x01)
#define	BMI160_LOW_G_INTR_STAT          (0x01)
#define BMI160_PULL_UP_DATA             (0x30)
#define BMI160_FIFO_M_G_A_ENABLE        (0xE0)
#define BMI160_FIFO_M_G_ENABLE          (0xA0)
#define BMI160_FIFO_M_A_ENABLE          (0x60)
#define BMI160_FIFO_G_A_ENABLE          (0xC0)
#define BMI160_FIFO_A_ENABLE            (0x40)
#define BMI160_FIFO_M_ENABLE            (0x20)

#define BMI160_SEC_IF_BMM150	(0)
#define BMI160_SEC_IF_AKM09911	(1)
#define BMI160_SEC_IF_AKM09912	(2)
#define BMI160_SEC_IF_YAS532	(3)
#define BMI160_SEC_IF_YAS537	(4)
/**************************************************/
/**\name	MAG INIT DEFINITION  */
/*************************************************/
#define BMI160_COMMAND_REG_ONE		(0x37)
#define BMI160_COMMAND_REG_TWO		(0x9A)
#define BMI160_COMMAND_REG_THREE	(0xC0)
#define	RESET_STEP_COUNTER			(0xB2)
/**************************************************/
/**\name	BIT SLICE GET AND SET FUNCTIONS  */
/*************************************************/
#define BMI160_GET_BITSLICE(regvar, bitname)\
		((regvar & bitname##__MSK) >> bitname##__POS)


#define BMI160_SET_BITSLICE(regvar, bitname, val)\
		((regvar & ~bitname##__MSK) | \
		((val<<bitname##__POS)&bitname##__MSK))

/**************************************************/
/**\name	 FUNCTION DECLARATIONS  */
/*************************************************/
/**************************************************/
/**\name	 FUNCTION FOR BMI160 INITIALIZE  */
/*************************************************/
/*!
 *	@brief
 *	This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	@param bmi160 : structure pointer
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	While changing the parameter of the bmi160_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_init(struct bmi160_t *bmi160);
/**************************************************/
/**\name	 FUNCTION FOR READ AND WRITE REGISTERS  */
/*************************************************/
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_us8 -> Address of the register
 *	@param v_data_us8 -> The data from the register
 *	@param v_len_us8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_write_reg(us8 v_addr_us8,
us8 *v_data_us8, us8 v_len_us8);
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_us8 -> Address of the register
 *	@param v_data_us8 -> The data from the register
 *	@param v_len_us8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_reg(us8 v_addr_us8,
us8 *v_data_us8, us8 v_len_us8);
/**************************************************/
/**\name	 FUNCTION FOR BMI160_ERROR CODES  */
/*************************************************/
/*!
 *	@brief This API used to reads the fatal error
 *	from the Register 0x02 bit 0
 *	This flag will be reset only by power-on-reset and soft reset
 *
 *
 *  @param v_fatal_err_us8 : The status of fatal error
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fatal_err(us8
*v_fatal_err_us8);
/*!
 *	@brief This API used to read the error code
 *	from register 0x02 bit 1 to 4
 *
 *
 *  @param v_err_code_us8 : The status of error codes
 *	error_code  |    description
 *  ------------|---------------
 *	0x00        |no error
 *	0x01        |ACC_CONF error (accel ODR and bandwidth not compatible)
 *	0x02        |GYR_CONF error (Gyroscope ODR and bandwidth not compatible)
 *	0x03        |Under sampling mode and interrupt uses pre filtered data
 *	0x04        |reserved
 *	0x05        |Selected trigger-readout offset in
 *    -         |MAG_IF greater than selected ODR
 *	0x06        |FIFO configuration error for header less mode
 *	0x07        |Under sampling mode and pre filtered data as FIFO source
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_err_code(us8
*v_error_code_us8);
/*!
 *	@brief This API Reads the i2c error code from the
 *	Register 0x02 bit 5.
 *	This error occurred in I2C master detected
 *
 *  @param v_i2c_err_code_us8 : The status of i2c fail error
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_fail_err(us8
*v_i2c_error_code_us8);
 /*!
 *	@brief This API Reads the dropped command error
 *	from the register 0x02 bit 6
 *
 *
 *  @param v_drop_cmd_err_us8 : The status of drop command error
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_drop_cmd_err(us8
*v_drop_cmd_err_us8);
/*!
 *	@brief This API reads the error status
 *	from the error register 0x02 bit 0 to 7
 *
 *  @param v_mag_data_rdy_err_us8 : The status of mag data ready interrupt
 *  @param v_fatal_er_us8r : The status of fatal error
 *  @param v_err_code_us8 : The status of error code
 *  @param v_i2c_fail_err_us8 : The status of I2C fail error
 *  @param v_drop_cmd_err_us8 : The status of drop command error
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_error_status(us8 *v_fatal_er_us8r,
us8 *v_err_code_us8, us8 *v_i2c_fail_err_us8,
us8 *v_drop_cmd_err_us8, us8 *v_mag_data_rdy_err_us8);
/******************************************************************/
/**\name	 FUNCTIONS FOR MAG,ACCEL AND GYRO POWER MODE STATUS  */
/*****************************************************************/
/*!
 *	@brief This API reads the gyroscope power mode from
 *	PMU status register 0x03 bit 2 and 3
 *
 *  @param v_gyro_power_mode_stat_us8 :	The value of gyro power mode
 *	gyro_powermode   |   value
 * ------------------|----------
 *    SUSPEND        |   0x00
 *    NORMAL         |   0x01
 *   FAST POWER UP   |   0x03
 *
 * @note The power mode of gyro set by the 0x7E command register
 * @note using the function "bmi160_set_command_register()"
 *  value    |   mode
 *  ---------|----------------
 *   0x14    | GYRO_MODE_SUSPEND
 *   0x15    | GYRO_MODE_NORMAL
 *   0x17    | GYRO_MODE_FASTSTARTUP
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_power_mode_stat(us8
*v_gyro_power_mode_stat_us8);
/*!
 *	@brief This API reads the accelerometer power mode from
 *	PMU status register 0x03 bit 4 and 5
 *
 *
 *  @param v_accel_power_mode_stat_us8 :	The value of accel power mode
 *	accel_powermode  |   value
 * ------------------|----------
 *    SUSPEND        |   0x00
 *    NORMAL         |   0x01
 *  LOW POWER        |   0x03
 *
 * @note The power mode of accel set by the 0x7E command register
 * @note using the function "bmi160_set_command_register()"
 *  value    |   mode
 *  ---------|----------------
 *   0x11    | ACCEL_MODE_NORMAL
 *   0x12    | ACCEL_LOWPOWER
 *   0x10    | ACCEL_SUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_power_mode_stat(us8
*v_accel_power_mode_stat_us8);
/**************************************************/
/**\name	 FUNCTION FOR GYRO XYZ DATA READ  */
/*************************************************/
/*!
 *	@brief This API reads gyro data X values
 *	form the register 0x0C and 0x0D
 *
 *
 *
 *
 *  @param v_gyro_x_int16_ts : The value of gyro x data
 *
 *	@note Gyro Configuration use the following function
 *	@note bmi160_set_gyro_output_data_rate()
 *	@note bmi160_set_gyro_bw()
 *	@note bmi160_set_gyro_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_x(
int16_ts *v_gyro_x_int16_ts);
/*!
 *	@brief This API reads gyro data Y values
 *	form the register 0x0E and 0x0F
 *
 *
 *
 *
 *  @param v_gyro_y_int16_ts : The value of gyro y data
 *
 *	@note Gyro Configuration use the following function
 *	@note bmi160_set_gyro_output_data_rate()
 *	@note bmi160_set_gyro_bw()
 *	@note bmi160_set_gyro_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error result of communication routines
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_y(
int16_ts *v_gyro_y_int16_ts);
/*!
 *	@brief This API reads gyro data Z values
 *	form the register 0x10 and 0x11
 *
 *
 *
 *
 *  @param v_gyro_z_int16_ts : The value of gyro z data
 *
 *	@note Gyro Configuration use the following function
 *	@note bmi160_set_gyro_output_data_rate()
 *	@note bmi160_set_gyro_bw()
 *	@note bmi160_set_gyro_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_z(
int16_ts *v_gyro_z_int16_ts);
/*!
 *	@brief This API reads gyro data X,Y,Z values
 *	from the register 0x0C to 0x11
 *
 *
 *
 *
 *  @param gyro : The value of gyro xyz
 *
 *	@note Gyro Configuration use the following function
 *	@note bmi160_set_gyro_output_data_rate()
 *	@note bmi160_set_gyro_bw()
 *	@note bmi160_set_gyro_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_xyz(
struct bmi160_gyro_t *gyro);
/**************************************************/
/**\name	 FUNCTION FOR ACCEL XYZ DATA READ  */
/*************************************************/
/*!
 *	@brief This API reads accelerometer data X values
 *	form the register 0x12 and 0x13
 *
 *
 *
 *
 *  @param v_accel_x_int16_ts : The value of accel x
 *
 *	@note For accel configuration use the following functions
 *	@note bmi160_set_accel_output_data_rate()
 *	@note bmi160_set_accel_bw()
 *	@note bmi160_set_accel_under_sampling_parameter()
 *	@note bmi160_set_accel_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_x(
int16_ts *v_accel_x_int16_ts);
/*!
 *	@brief This API reads accelerometer data Y values
 *	form the register 0x14 and 0x15
 *
 *
 *
 *
 *  @param v_accel_y_int16_ts : The value of accel y
 *
 *	@note For accel configuration use the following functions
 *	@note bmi160_set_accel_output_data_rate()
 *	@note bmi160_set_accel_bw()
 *	@note bmi160_set_accel_under_sampling_parameter()
 *	@note bmi160_set_accel_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_y(
int16_ts *v_accel_y_int16_ts);
/*!
 *	@brief This API reads accelerometer data Z values
 *	form the register 0x16 and 0x17
 *
 *
 *
 *
 *  @param v_accel_z_int16_ts : The value of accel z
 *
 *	@note For accel configuration use the following functions
 *	@note bmi160_set_accel_output_data_rate()
 *	@note bmi160_set_accel_bw()
 *	@note bmi160_set_accel_under_sampling_parameter()
 *	@note bmi160_set_accel_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_z(
int16_ts *v_accel_z_int16_ts);
/*!
 *	@brief This API reads accelerometer data X,Y,Z values
 *	from the register 0x12 to 0x17
 *
 *
 *
 *
 *  @param accel :The value of accel xyz
 *
 *	@note For accel configuration use the following functions
 *	@note bmi160_set_accel_output_data_rate()
 *	@note bmi160_set_accel_bw()
 *	@note bmi160_set_accel_under_sampling_parameter()
 *	@note bmi160_set_accel_range()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_xyz(
struct bmi160_accel_t *accel);
/**************************************************/
/**\name	 FUNCTION FOR SENSOR TIME */
/*************************************************/
/*!
 *	@brief This API reads sensor_time from the register
 *	0x18 to 0x1A
 *
 *
 *  @param v_sensor_time_us32 : The value of sensor time
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_sensor_time(
us32 *v_sensor_time_us32);
/**************************************************/
/**\name	 FUNCTION FOR GYRO SLEF TEST  */
/*************************************************/
/*!
 *	@brief This API reads the Gyroscope self test
 *	status from the register 0x1B bit 1
 *
 *
 *  @param v_gyro_selftest_us8 : The value of gyro self test status
 *  value    |   status
 *  ---------|----------------
 *   0       | Gyroscope self test is running or failed
 *   1       | Gyroscope self test completed successfully
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest(us8
*v_gyro_selftest_us8);
/**************************************************/
/**\name	 FUNCTION FOR FAST OFFSET READY  */
/*************************************************/
/*!
 *	@brief This API reads the fast offset compensation
 *	status form the register 0x1B bit 3
 *
 *
 *  @param v_foc_rdy_us8 : The status of fast compensation
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_rdy(us8
*v_foc_rdy_us8);
/**************************************************/
/**\name	 FUNCTION FOR NVM READY  */
/*************************************************/
/*!
 * @brief This API Reads the nvm_rdy status from the
 *	resister 0x1B bit 4
 *
 *
 *  @param v_nvm_rdy_us8 : The value of NVM ready status
 *  value    |   status
 *  ---------|----------------
 *   0       | NVM write operation in progress
 *   1       | NVM is ready to accept a new write trigger
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_rdy(us8
*v_nvm_rdy_us8);
/**************************************************/
/**\name	 FUNCTION FOR DATA READY FOR MAG, GYRO, AND ACCEL */
/*************************************************/
/*!
 *	@brief This API reads the status of mag data ready
 *	from the register 0x1B bit 5
 *	The status get reset when one mag data register is read out
 *
 *  @param v_data_rdy_us8 : The value of mag data ready status
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_data_rdy_mag(us8
*v_data_rdy_us8);
/*!
 *	@brief This API reads the status of gyro data ready form the
 *	register 0x1B bit 6
 *	The status get reset when gyro data register read out
 *
 *
 *	@param v_data_rdy_us8 :	The value of gyro data ready
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_data_rdy(us8
*v_data_rdy_us8);
/*!
 *	@brief This API reads the status of accel data ready form the
 *	register 0x1B bit 7
 *	The status get reset when accel data register read out
 *
 *
 *	@param v_data_rdy_us8 :	The value of accel data ready status
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_data_rdy(us8
*drdy_acc);
/**************************************************/
/**\name	 FUNCTION FOR STEP INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the step detector interrupt status
 *	from the register 0x1C bit 0
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_step_intr_us8 : The status of step detector interrupt
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_step_intr(us8
*v_step_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR SIGNIFICANT INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the
 *	significant motion interrupt status
 *	from the register 0x1C bit 1
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  @param v_significant_intr_us8 : The status of step
 *	motion interrupt
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_significant_intr(us8
*sigmot_intr);
/**************************************************/
/**\name	 FUNCTION FOR ANY MOTION INTERRUPT STATUS  */
/*************************************************/
 /*!
 *	@brief This API reads the any motion interrupt status
 *	from the register 0x1C bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *  @param v_any_motion_intr_us8 : The status of any-motion interrupt
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_any_motion_intr(us8
*v_any_motion_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR PMU TRIGGER INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the power mode trigger interrupt status
 *	from the register 0x1C bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *
 *  @param v_pmu_trigger_intr_us8 : The status of power mode trigger interrupt
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_pmu_trigger_intr(us8
*v_pmu_trigger_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR DOUBLE TAB STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the double tab status
 *	from the register 0x1C bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_double_tap_intr_us8 :The status of double tab interrupt
 *
 *	@note Double tap interrupt can be configured by the following functions
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_double_tap()
 *	@note AXIS MAPPING
 *	@note bmi160_get_stat2_tap_first_x()
 *	@note bmi160_get_stat2_tap_first_y()
 *	@note bmi160_get_stat2_tap_first_z()
 *	@note DURATION
 *	@note bmi160_set_intr_tap_durn()
 *	@note THRESHOLD
 *	@note bmi160_set_intr_tap_thres()
 *	@note TAP QUIET
 *	@note bmi160_set_intr_tap_quiet()
 *	@note TAP SHOCK
 *	@note bmi160_set_intr_tap_shock()
 *	@note TAP SOURCE
 *	@note bmi160_set_intr_tap_source()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_double_tap_intr(us8
*v_double_tap_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR SINGLE TAB STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the single tab status
 *	from the register 0x1C bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the single tab interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt
 *	signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_single_tap_intr_us8 :The status of single tap interrupt
 *
 *	@note Single tap interrupt can be configured by the following functions
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_single_tap()
 *	@note AXIS MAPPING
 *	@note bmi160_get_stat2_tap_first_x()
 *	@note bmi160_get_stat2_tap_first_y()
 *	@note bmi160_get_stat2_tap_first_z()
 *	@note DURATION
 *	@note bmi160_set_intr_tap_durn()
 *	@note THRESHOLD
 *	@note bmi160_set_intr_tap_thres()
 *	@note TAP QUIET
 *	@note bmi160_set_intr_tap_quiet()
 *	@note TAP SHOCK
 *	@note bmi160_set_intr_tap_shock()
 *	@note TAP SOURCE
 *	@note bmi160_set_intr_tap_source()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_single_tap_intr(us8
*v_single_tap_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR ORIENT INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the orient status
 *	from the register 0x1C bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the orient interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_orient_intr_us8 : The status of orient interrupt
 *
 *	@note For orient interrupt configuration use the following functions
 *	@note STATUS
 *	@note bmi160_get_stat0_orient_intr()
 *	@note AXIS MAPPING
 *	@note bmi160_get_stat3_orient_xy()
 *	@note bmi160_get_stat3_orient_z()
 *	@note bmi160_set_intr_orient_axes_enable()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_orient()
 *	@note INTERRUPT OUTPUT
 *	@note bmi160_set_intr_orient_ud_enable()
 *	@note THETA
 *	@note bmi160_set_intr_orient_theta()
 *	@note HYSTERESIS
 *	@note bmi160_set_intr_orient_hyst()
 *	@note BLOCKING
 *	@note bmi160_set_intr_orient_blocking()
 *	@note MODE
 *	@note bmi160_set_intr_orient_mode()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_orient_intr(us8
*v_orient_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR FLAT INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the flat interrupt status
 *	from the register 0x1C bit 7
 *	flag is associated with a specific interrupt function.
 *	It is set when the flat interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_flat_intr_us8 : The status of  flat interrupt
 *
 *	@note For flat configuration use the following functions
 *	@note STATS
 *	@note bmi160_get_stat0_flat_intr()
 *	@note bmi160_get_stat3_flat()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_flat()
 *	@note THETA
 *	@note bmi160_set_intr_flat_theta()
 *	@note HOLD TIME
 *	@note bmi160_set_intr_flat_hold()
 *	@note HYSTERESIS
 *	@note bmi160_set_intr_flat_hyst()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_flat_intr(us8
*v_flat_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR HIGH_G INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the high_g interrupt status
 *	from the register 0x1D bit 2
 *	flag is associated with a specific interrupt function.
 *	It is set when the high g  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be permanently
 *	latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_high_g_intr_us8 : The status of high_g interrupt
 *
 *	@note High_g interrupt configured by following functions
 *	@note STATUS
 *	@note bmi160_get_stat1_high_g_intr()
 *	@note AXIS MAPPING
 *	@note bmi160_get_stat3_high_g_first_x()
 *	@note bmi160_get_stat3_high_g_first_y()
 *	@note bmi160_get_stat3_high_g_first_z()
 *	@note SIGN MAPPING
 *	@note bmi160_get_stat3_high_g_first_sign()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_high_g()
  *	@note HYSTERESIS
 *	@note bmi160_set_intr_high_g_hyst()
 *	@note DURATION
 *	@note bmi160_set_intr_high_g_durn()
 *	@note THRESHOLD
 *	@note bmi160_set_intr_high_g_thres()
 *	@note SOURCE
 *	@note bmi160_set_intr_low_high_source()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_high_g_intr(us8
*v_high_g_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR LOW_G INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads the low g interrupt status
 *	from the register 0x1D bit 3
 *	flag is associated with a specific interrupt function.
 *	It is set when the low g  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_low_g_intr_us8 : The status of low_g interrupt
 *
 *	@note Low_g interrupt configured by following functions
 *	@note STATUS
 *	@note bmi160_get_stat1_low_g_intr()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_low_g()
 *	@note SOURCE
 *	@note bmi160_set_intr_low_high_source()
 *	@note DURATION
 *	@note bmi160_set_intr_low_g_durn()
 *	@note THRESHOLD
 *	@note bmi160_set_intr_low_g_thres()
 *	@note HYSTERESIS
 *	@note bmi160_set_intr_low_g_hyst()
 *	@note MODE
 *	@note bmi160_set_intr_low_g_mode()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_low_g_intr(us8
*v_low_g_intr_us8);
/**************************************************/
/**\name	 FUNCTION FOR DATA READY INTERRUPT STATUS  */
/*************************************************/
/*!
 *	@brief This API reads data ready interrupt status
 *	from the register 0x1D bit 4
 *	flag is associated with a specific interrupt function.
 *	It is set when the  data ready  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_data_rdy_intr_us8 : The status of data ready interrupt
 *
 *	@note Data ready interrupt configured by following functions
 *	@note STATUS
 *	@note bmi160_get_stat1_data_rdy_intr()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_data_rdy()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_data_rdy_intr(us8
*v_data_rdy_intr_us8);
/**************************************************/
/**\name	 FUNCTIONS FOR FIFO FULL AND WATER MARK INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads data ready FIFO full interrupt status
 *	from the register 0x1D bit 5
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO full interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will
 *	be permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_fifo_full_intr_us8 : The status of fifo full interrupt
 *
 *	@note FIFO full interrupt can be configured by following functions
 *	@note bmi160_set_intr_fifo_full()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_full_intr(us8
*v_fifo_full_intr_us8);
/*!
 *	@brief This API reads data
 *	 ready FIFO watermark interrupt status
 *	from the register 0x1D bit 6
 *	flag is associated with a specific interrupt function.
 *	It is set when the FIFO watermark interrupt triggers. The
 *	setting of INT_LATCH controls if the
 *	interrupt signal and hence the
 *	respective interrupt flag will be
 *	permanently latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_fifo_wm_intr_us8 : The status of fifo water mark interrupt
 *
 *	@note FIFO full interrupt can be configured by following functions
 *	@note bmi160_set_intr_fifo_wm()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_wm_intr(us8
*v_fifo_wm_intr_us8);
/**************************************************/
/**\name	 FUNCTIONS FOR NO MOTION INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads data ready no motion interrupt status
 *	from the register 0x1D bit 7
 *	flag is associated with a specific interrupt function.
 *	It is set when the no motion  interrupt triggers. The
 *	setting of INT_LATCH controls if the interrupt signal and hence the
 *	respective interrupt flag will be permanently
 *	latched, temporarily latched
 *	or not latched.
 *
 *
 *
 *
 *  @param v_nomotion_intr_us8 : The status of no motion interrupt
 *
 *	@note No motion interrupt can be configured by following function
 *	@note STATUS
 *	@note bmi160_get_stat1_nomotion_intr()
 *	@note INTERRUPT MAPPING
 *	@note bmi160_set_intr_nomotion()
 *	@note DURATION
 *	@note bmi160_set_intr_slow_no_motion_durn()
 *	@note THRESHOLD
 *	@note bmi160_set_intr_slow_no_motion_thres()
 *	@note SLOW/NO MOTION SELECT
 *	@note bmi160_set_intr_slow_no_motion_select()
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_nomotion_intr(us8
*nomo_intr);
/**************************************************/
/**\name	 FUNCTIONS FOR ANY MOTION FIRST XYZ AND SIGN INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads the status of any motion first x
 *	from the register 0x1E bit 0
 *
 *
 *  @param v_anymotion_first_x_us8 : The status of any motion first x interrupt
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by x axis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_x(us8
*v_anymotion_first_x_us8);
/*!
 *	@brief This API reads the status of any motion first y interrupt
 *	from the register 0x1E bit 1
 *
 *
 *
 *@param v_any_motion_first_y_us8 : The status of any motion first y interrupt
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_y(us8
*v_any_motion_first_y_us8);
/*!
 *	@brief This API reads the status of any motion first z interrupt
 *	from the register 0x1E bit 2
 *
 *
 *
 *
 *@param v_any_motion_first_z_us8 : The status of any motion first z interrupt
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_z(us8
*v_any_motion_first_z_us8);
/*!
 *	@brief This API reads the any motion sign status from the
 *	register 0x1E bit 3
 *
 *
 *
 *
 *  @param v_anymotion_sign_us8 : The status of any motion sign
 *  value     |  sign
 * -----------|-------------
 *   0        | positive
 *   1        | negative
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_sign(us8
*v_anymotion_sign_us8);
/**************************************************/
/**\name	 FUNCTIONS FOR TAP FIRST XYZ AND SIGN INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads the any motion tap first x status from the
 *	register 0x1E bit 4
 *
 *
 *
 *
 *  @param v_tap_first_x_us8 :The status of any motion tap first x
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by x axis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_x(us8
*v_tap_first_x_us8);
/*!
 *	@brief This API reads the tap first y interrupt status from the
 *	register 0x1E bit 5
 *
 *
 *
 *
 *  @param v_tap_first_y_us8 :The status of tap first y interrupt
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_y(us8
*v_tap_first_y_us8);
/*!
 *	@brief This API reads the tap first z interrupt status  from the
 *	register 0x1E bit 6
 *
 *
 *
 *
 *  @param v_tap_first_z_us8 :The status of tap first z interrupt
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_z(us8
*v_tap_first_z_us8);
/*!
 *	@brief This API reads the tap sign status from the
 *	register 0x1E bit 7
 *
 *
 *
 *
 *  @param v_tap_sign_us8 : The status of tap sign
 *  value     |  sign
 * -----------|-------------
 *   0        | positive
 *   1        | negative
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_sign(us8
*tap_sign);
/**************************************************/
/**\name	 FUNCTIONS FOR HIGH_G FIRST XYZ AND SIGN INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads the high_g first x status from the
 *	register 0x1F bit 0
 *
 *
 *
 *
 *  @param v_high_g_first_x_us8 :The status of high_g first x
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by x axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_x(us8
*v_high_g_first_x_us8);
/*!
 *	@brief This API reads the high_g first y status from the
 *	register 0x1F bit 1
 *
 *
 *
 *
 *  @param v_high_g_first_y_us8 : The status of high_g first y
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_y(us8
*v_high_g_first_y_us8);
/*!
 *	@brief This API reads the high_g first z status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  @param v_high_g_first_z_us8 : The status of high_g first z
 *  value     |  status
 * -----------|-------------
 *   0        | not triggered
 *   1        | triggered by z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_z(us8
*v_high_g_first_z_us8);
/*!
 *	@brief This API reads the high sign status from the
 *	register 0x1F bit 3
 *
 *
 *
 *
 *  @param v_high_g_sign_us8 :The status of high sign
 *  value     |  sign
 * -----------|-------------
 *   0        | positive
 *   1        | negative
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_sign(us8
*v_high_g_sign_us8);
/**************************************************/
/**\name	 FUNCTIONS FOR ORIENT XY AND Z INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads the status of orient_xy plane
 *	from the register 0x1F bit 4 and 5
 *
 *
 *  @param v_orient_xy_us8 :The status of orient_xy plane
 *  value     |  status
 * -----------|-------------
 *   0x00     | portrait upright
 *   0x01     | portrait upside down
 *   0x02     | landscape left
 *   0x03     | landscape right
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_xy(us8
*v_orient_xy_us8);
/*!
 *	@brief This API reads the status of orient z plane
 *	from the register 0x1F bit 6
 *
 *
 *  @param v_orient_z_us8 :The status of orient z
 *  value     |  status
 * -----------|-------------
 *   0x00     | upward looking
 *   0x01     | downward looking
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_z(us8
*v_orient_z_us8);
/**************************************************/
/**\name	 FUNCTIONS FOR FLAT INTERRUPT STATUS*/
/*************************************************/
/*!
 *	@brief This API reads the flat status from the register
 *	0x1F bit 7
 *
 *
 *  @param v_flat_us8 : The status of flat interrupt
 *  value     |  status
 * -----------|-------------
 *   0x00     | non flat
 *   0x01     | flat position
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_flat(us8
*flat);
/**************************************************/
/**\name	 FUNCTION FOR TEMPERATUE READ */
/*************************************************/
/*!
 *	@brief This API reads the temperature of the sensor
 *	from the register 0x21 bit 0 to 7
 *
 *
 *
 *  @param v_temp_int16_ts : The value of temperature
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_temp(int16_ts
*v_temp_int16_ts);
/**************************************************/
/**\name	 FUNCTION FOR FIFO LENGTH AND FIFO DATA READ */
/*************************************************/
/*!
 *	@brief This API reads the  of the sensor
 *	form the register 0x23 and 0x24 bit 0 to 7 and 0 to 2
 *	@brief this byte counter is updated each time a complete frame
 *	was read or writtern
 *
 *
 *  @param v_fifo_length_us32 : The value of fifo byte counter
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_length(
us32 *v_fifo_length_us32);
/*!
 *	@brief This API reads the fifo data of the sensor
 *	from the register 0x24
 *	@brief Data format depends on the setting of register FIFO_CONFIG
 *
 *
 *
 *  @param v_fifodata_us8 : Pointer holding the fifo data
 *  @param v_fifo_length_us16 : The value of fifo length maximum
 *	1024
 *
 *	@note For reading FIFO data use the following functions
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_data(
us8 *v_fifodata_us8, us16 v_fifo_length_us16);
/**************************************************/
/**\name	 FUNCTION FOR ACCEL CONFIGURATIONS */
/*************************************************/
/*!
 *	@brief This API is used to get the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *  @param  v_output_data_rate_us8 :The value of accel output date rate
 *  value |  output data rate
 * -------|--------------------------
 *	 0    |	BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	 1	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	 2	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	 3    |	BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	 4    | BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	 5	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	 6	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	 7	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	 8	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	 9	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	 10	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	 11	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	 12	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_output_data_rate(
us8 *v_output_data_rate_us8);
/*!
 *	@brief This API is used to set the
 *	accel output date rate form the register 0x40 bit 0 to 3
 *
 *
 *  @param  v_output_data_rate_us8 :The value of accel output date rate
 *  value |  output data rate
 * -------|--------------------------
 *	 0    |	BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED
 *	 1	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ
 *	 2	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ
 *	 3    |	BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ
 *	 4    | BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ
 *	 5	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ
 *	 6	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ
 *	 7	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ
 *	 8	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ
 *	 9	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ
 *	 10	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ
 *	 11	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ
 *	 12	  |	BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ
 *
 *  @param  v_accel_bw_us8 :The value of accel selected accel bandwidth
 *  value |  output data rate
 * -------|--------------------------
 *    0   |  BMI160_ACCEL_OSR4_AVG1
 *    1   |  BMI160_ACCEL_OSR2_AVG2
 *    2   |  BMI160_ACCEL_NORMAL_AVG4
 *    3   |  BMI160_ACCEL_CIC_AVG8
 *    4   |  BMI160_ACCEL_RES_AVG2
 *    5   |  BMI160_ACCEL_RES_AVG4
 *    6   |  BMI160_ACCEL_RES_AVG8
 *    7   |  BMI160_ACCEL_RES_AVG16
 *    8   |  BMI160_ACCEL_RES_AVG32
 *    9   |  BMI160_ACCEL_RES_AVG64
 *    10  |  BMI160_ACCEL_RES_AVG128
 *
 *
 *
 *	@note Verify the accel bandwidth before seting the
 *  output data rate
 *  bandwidth  | output data rate |  under sampling
 *-------------|------------------|----------------
 *   OSR4      |  12.5 TO 1600    |   0
 *   OSR2      |  12.5 TO 1600    |   0
 *  NORMAL     |  12.5 TO 1600    |   0
 *   CIC       |  12.5 TO 1600    |   0
 *   AVG2      |  0.78 TO 400     |   1
 *   AVG4      |  0.78 TO 200     |   1
 *   AVG8      |  0.78 TO 100     |   1
 *   AVG16     |  0.78 TO 50      |   1
 *   AVG32     |  0.78 TO 25      |   1
 *   AVG64     |  0.78 TO 12.5    |   1
 *   AVG128    |  0.78 TO 6.25    |   1
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_output_data_rate(
us8 v_output_data_rate_us8, us8 v_accel_bw_us8);
/*!
 *	@brief This API is used to get the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *	@brief bandwidth parameter determines filter configuration(acc_us=0)
 *	and averaging for under sampling mode(acc_us=1)
 *
 *
 *  @param  v_bw_us8 : The value of accel bandwidth
 *
 *	@note accel bandwidth depends on under sampling parameter
 *	@note under sampling parameter cab be set by the function
 *	"BMI160_SET_ACCEL_UNDER_SAMPLING_PARAMETER"
 *
 *	@note Filter configuration
 *  accel_us  | Filter configuration
 * -----------|---------------------
 *    0x00    |  OSR4 mode
 *    0x01    |  OSR2 mode
 *    0x02    |  normal mode
 *    0x03    |  CIC mode
 *    0x04    |  Reserved
 *    0x05    |  Reserved
 *    0x06    |  Reserved
 *    0x07    |  Reserved
 *
 *	@note accel under sampling mode
 *  accel_us  | Under sampling mode
 * -----------|---------------------
 *    0x00    |  no averaging
 *    0x01    |  average 2 samples
 *    0x02    |  average 4 samples
 *    0x03    |  average 8 samples
 *    0x04    |  average 16 samples
 *    0x05    |  average 32 samples
 *    0x06    |  average 64 samples
 *    0x07    |  average 128 samples
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_bw(us8 *v_bw_us8);
/*!
 *	@brief This API is used to set the
 *	accel bandwidth from the register 0x40 bit 4 to 6
 *	@brief bandwidth parameter determines filter configuration(acc_us=0)
 *	and averaging for under sampling mode(acc_us=1)
 *
 *
 *  @param  v_bw_us8 : The value of accel bandwidth
 *
 *	@note accel bandwidth depends on under sampling parameter
 *	@note under sampling parameter cab be set by the function
 *	"BMI160_SET_ACCEL_UNDER_SAMPLING_PARAMETER"
 *
 *	@note Filter configuration
 *  accel_us  | Filter configuration
 * -----------|---------------------
 *    0x00    |  OSR4 mode
 *    0x01    |  OSR2 mode
 *    0x02    |  normal mode
 *    0x03    |  CIC mode
 *    0x04    |  Reserved
 *    0x05    |  Reserved
 *    0x06    |  Reserved
 *    0x07    |  Reserved
 *
 *	@note accel under sampling mode
 *  accel_us  | Under sampling mode
 * -----------|---------------------
 *    0x00    |  no averaging
 *    0x01    |  average 2 samples
 *    0x02    |  average 4 samples
 *    0x03    |  average 8 samples
 *    0x04    |  average 16 samples
 *    0x05    |  average 32 samples
 *    0x06    |  average 64 samples
 *    0x07    |  average 128 samples
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_bw(us8 v_bw_us8);
/*!
 *	@brief This API is used to get the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *
 *
 *
 *	@param  v_accel_under_sampling_us8 : The value of accel under sampling
 *	value    | under_sampling
 * ----------|---------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_under_sampling_parameter(
us8 *v_accel_under_sampling_us8);
/*!
 *	@brief This API is used to set the accel
 *	under sampling parameter form the register 0x40 bit 7
 *
 *
 *
 *
 *	@param  v_accel_under_sampling_us8 : The value of accel under sampling
 *	value    | under_sampling
 * ----------|---------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_under_sampling_parameter(
us8 v_accel_under_sampling_us8);
/*!
 *	@brief This API is used to get the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  @param v_range_us8 : The value of accel g range
 *	value    | g_range
 * ----------|-----------
 *   0x03    | BMI160_ACCEL_RANGE_2G
 *   0x05    | BMI160_ACCEL_RANGE_4G
 *   0x08    | BMI160_ACCEL_RANGE_8G
 *   0x0C    | BMI160_ACCEL_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_range(
us8 *v_range_us8);
/*!
 *	@brief This API is used to set the ranges
 *	(g values) of the accel from the register 0x41 bit 0 to 3
 *
 *
 *
 *
 *  @param v_range_us8 : The value of accel g range
 *	value    | g_range
 * ----------|-----------
 *   0x03    | BMI160_ACCEL_RANGE_2G
 *   0x05    | BMI160_ACCEL_RANGE_4G
 *   0x08    | BMI160_ACCEL_RANGE_8G
 *   0x0C    | BMI160_ACCEL_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_range(
us8 v_range_us8);
/**************************************************/
/**\name	 FUNCTION FOR GYRO CONFIGURATIONS */
/*************************************************/
/*!
 *	@brief This API is used to get the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *
 *  @param  v_output_data_rate_us8 :The value of gyro output data rate
 *  value     |      gyro output data rate
 * -----------|-----------------------------
 *   0x00     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x01     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x02     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x03     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x04     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x05     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x06     | BMI160_GYRO_OUTPUT_DATA_RATE_25HZ
 *   0x07     | BMI160_GYRO_OUTPUT_DATA_RATE_50HZ
 *   0x08     | BMI160_GYRO_OUTPUT_DATA_RATE_100HZ
 *   0x09     | BMI160_GYRO_OUTPUT_DATA_RATE_200HZ
 *   0x0A     | BMI160_GYRO_OUTPUT_DATA_RATE_400HZ
 *   0x0B     | BMI160_GYRO_OUTPUT_DATA_RATE_800HZ
 *   0x0C     | BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ
 *   0x0D     | BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ
 *   0x0E     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x0F     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_output_data_rate(
us8 *gyro_output_typer);
/*!
 *	@brief This API is used to set the
 *	gyroscope output data rate from the register 0x42 bit 0 to 3
 *
 *
 *
 *
 *  @param  v_output_data_rate_us8 :The value of gyro output data rate
 *  value     |      gyro output data rate
 * -----------|-----------------------------
 *   0x00     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x01     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x02     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x03     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x04     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x05     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x06     | BMI160_GYRO_OUTPUT_DATA_RATE_25HZ
 *   0x07     | BMI160_GYRO_OUTPUT_DATA_RATE_50HZ
 *   0x08     | BMI160_GYRO_OUTPUT_DATA_RATE_100HZ
 *   0x09     | BMI160_GYRO_OUTPUT_DATA_RATE_200HZ
 *   0x0A     | BMI160_GYRO_OUTPUT_DATA_RATE_400HZ
 *   0x0B     | BMI160_GYRO_OUTPUT_DATA_RATE_800HZ
 *   0x0C     | BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ
 *   0x0D     | BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ
 *   0x0E     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *   0x0F     | BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_output_data_rate(
us8 gyro_output_typer);
/*!
 *	@brief This API is used to get the
 *	data of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  @param  v_bw_us8 : The value of gyro bandwidth
 *  value     | gyro bandwidth
 *  ----------|----------------
 *   0x00     | BMI160_GYRO_OSR4_MODE
 *   0x01     | BMI160_GYRO_OSR2_MODE
 *   0x02     | BMI160_GYRO_NORMAL_MODE
 *   0x03     | BMI160_GYRO_CIC_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_bw(us8 *v_bw_us8);
/*!
 *	@brief This API is used to set the
 *	data of gyro from the register 0x42 bit 4 to 5
 *
 *
 *
 *
 *  @param  v_bw_us8 : The value of gyro bandwidth
 *  value     | gyro bandwidth
 *  ----------|----------------
 *   0x00     | BMI160_GYRO_OSR4_MODE
 *   0x01     | BMI160_GYRO_OSR2_MODE
 *   0x02     | BMI160_GYRO_NORMAL_MODE
 *   0x03     | BMI160_GYRO_CIC_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_bw(us8 v_bw_us8);
/*!
 *	@brief This API reads the range
 *	of gyro from the register 0x43 bit 0 to 2
 *
 *  @param  v_range_us8 : The value of gyro range
 *   value    |    range
 *  ----------|-------------------------------
 *    0x00    | BMI160_GYRO_RANGE_2000_DEG_SEC
 *    0x01    | BMI160_GYRO_RANGE_1000_DEG_SEC
 *    0x02    | BMI160_GYRO_RANGE_500_DEG_SEC
 *    0x03    | BMI160_GYRO_RANGE_250_DEG_SEC
 *    0x04    | BMI160_GYRO_RANGE_125_DEG_SEC
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_range(
us8 *v_range_us8);
/*!
 *	@brief This API set the range
 *	of gyro from the register 0x43 bit 0 to 2
 *
 *  @param  v_range_us8 : The value of gyro range
 *   value    |    range
 *  ----------|-------------------------------
 *    0x00    | BMI160_GYRO_RANGE_2000_DEG_SEC
 *    0x01    | BMI160_GYRO_RANGE_1000_DEG_SEC
 *    0x02    | BMI160_GYRO_RANGE_500_DEG_SEC
 *    0x03    | BMI160_GYRO_RANGE_250_DEG_SEC
 *    0x04    | BMI160_GYRO_RANGE_125_DEG_SEC
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_range(
us8 v_range_us8);
/**************************************************/
/**\name	 FUNCTION FOR FIFO CONFIGURATIONS */
/*************************************************/
 /*!
 *	@brief This API is used to read Down sampling
 *	for gyro (2**downs_gyro) in the register 0x45 bit 0 to 2
 *
 *
 *
 *
 *  @param v_fifo_down_gyro_us8 :The value of gyro fifo down
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_gyro(
us8 *v_fifo_down_gyro_us8);
 /*!
 *	@brief This API is used to set Down sampling
 *	for gyro (2**downs_gyro) in the register 0x45 bit 0 to 2
 *
 *
 *
 *
 *  @param v_fifo_down_gyro_us8 :The value of gyro fifo down
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_gyro(
us8 v_fifo_down_gyro_us8);
/*!
 *	@brief This API is used to read gyro fifo filter data
 *	from the register 0x45 bit 3
 *
 *
 *
 *  @param v_gyro_fifo_filter_data_us8 :The value of gyro filter data
 *  value      |  gyro_fifo_filter_data
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_fifo_filter_data(
us8 *v_gyro_fifo_filter_data_us8);
/*!
 *	@brief This API is used to set gyro fifo filter data
 *	from the register 0x45 bit 3
 *
 *
 *
 *  @param v_gyro_fifo_filter_data_us8 :The value of gyro filter data
 *  value      |  gyro_fifo_filter_data
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_fifo_filter_data(
us8 v_gyro_fifo_filter_data_us8);
/*!
 *	@brief This API is used to read Down sampling
 *	for accel (2*downs_accel) from the register 0x45 bit 4 to 6
 *
 *
 *
 *
 *  @param v_fifo_down_us8 :The value of accel fifo down
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_accel(
us8 *v_fifo_down_us8);
 /*!
 *	@brief This API is used to set Down sampling
 *	for accel (2*downs_accel) from the register 0x45 bit 4 to 6
 *
 *
 *
 *
 *  @param v_fifo_down_us8 :The value of accel fifo down
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_accel(
us8 v_fifo_down_us8);
/*!
 *	@brief This API is used to read accel fifo filter data
 *	from the register 0x45 bit 7
 *
 *
 *
 *  @param v_accel_fifo_filter_us8 :The value of accel filter data
 *  value      |  accel_fifo_filter_data
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_fifo_filter_data(
us8 *v_accel_fifo_filter_us8);
/*!
 *	@brief This API is used to set accel fifo filter data
 *	from the register 0x45 bit 7
 *
 *
 *
 *  @param v_accel_fifo_filter_us8 :The value of accel filter data
 *  value      |  accel_fifo_filter_data
 * ------------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_fifo_filter_data(
us8 v_accel_fifo_filter_us8);
/**************************************************/
/**\name	 FUNCTION FOR FIFO WATER MARK ENABLE */
/*************************************************/
/*!
 *	@brief This API is used to Trigger an interrupt
 *	when FIFO contains water mark level from the register 0x46 bit 0 to 7
 *
 *
 *
 *  @param  v_fifo_wm_us8 : The value of fifo water mark level
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_wm(
us8 *v_fifo_wm_us8);
/*!
 *	@brief This API is used to Trigger an interrupt
 *	when FIFO contains water mark level from the register 0x46 bit 0 to 7
 *
 *
 *
 *  @param  v_fifo_wm_us8 : The value of fifo water mark level
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_wm(
us8 v_fifo_wm_us8);
/**************************************************/
/**\name	 FUNCTION FOR FIFO CONFIGURATIONS */
/*************************************************/
/*!
 *	@brief This API reads fifo sensor time
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  @param v_fifo_time_enable_us8 : The value of sensor time
 *  value      |  fifo sensor time
 * ------------|-------------------------
 *    0x00     |  do not return sensortime frame
 *    0x01     |  return sensortime frame
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_time_enable(
us8 *v_fifo_time_enable_us8);
/*!
 *	@brief This API set fifo sensor time
 *	frame after the last valid data frame form the register  0x47 bit 1
 *
 *
 *
 *
 *  @param v_fifo_time_enable_us8 : The value of sensor time
 *  value      |  fifo sensor time
 * ------------|-------------------------
 *    0x00     |  do not return sensortime frame
 *    0x01     |  return sensortime frame
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_time_enable(
us8 v_fifo_time_enable_us8);
/*!
 *	@brief This API reads FIFO tag interrupt2 enable status
 *	from the resister 0x47 bit 2
 *
 *  @param v_fifo_tag_intr2_us8 : The value of fifo tag interrupt
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr2_enable(
us8 *v_fifo_tag_intr2_us8);
/*!
 *	@brief This API set FIFO tag interrupt2 enable status
 *	from the resister 0x47 bit 2
 *
 *  @param v_fifo_tag_intr2_us8 : The value of fifo tag interrupt
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr2_enable(
us8 v_fifo_tag_intr2_us8);
/*!
 *	@brief This API get FIFO tag interrupt1 enable status
 *	from the resister 0x47 bit 3
 *
 *  @param v_fifo_tag_intr1_us8 :The value of fifo tag interrupt1
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr1_enable(
us8 *v_fifo_tag_intr1_us8);
/*!
 *	@brief This API set FIFO tag interrupt1 enable status
 *	from the resister 0x47 bit 3
 *
 *  @param v_fifo_tag_intr1_us8 :The value of fifo tag interrupt1
 *	value    | fifo tag interrupt
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr1_enable(
us8 v_fifo_tag_intr1_us8);
/*!
 *	@brief This API reads FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *  @param v_fifo_header_us8 :The value of fifo header
 *	value    | fifo header
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_header_enable(
us8 *v_fifo_header_us8);
/*!
 *	@brief This API set FIFO frame
 *	header enable from the register 0x47 bit 4
 *
 *  @param v_fifo_header_us8 :The value of fifo header
 *	value    | fifo header
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_header_enable(
us8 v_fifo_header_us8);
/*!
 *	@brief This API is used to read stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *  @param v_fifo_accel_us8 : The value of fifo accel enble
 *	value    | fifo accel
 * ----------|-------------------
 *  0x00     |  no accel data is stored
 *  0x01     |  accel data is stored
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_accel_enable(
us8 *v_fifo_accel_us8);
/*!
 *	@brief This API is used to set stored
 *	accel data in FIFO (all 3 axes) from the register 0x47 bit 6
 *
 *  @param v_fifo_accel_us8 : The value of fifo accel enble
 *	value    | fifo accel
 * ----------|-------------------
 *  0x00     |  no accel data is stored
 *  0x01     |  accel data is stored
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_accel_enable(
us8 v_fifo_accel_us8);
/*!
 *	@brief This API is used to read stored
 *	 gyro data in FIFO (all 3 axes) from the resister 0x47 bit 7
 *
 *
 *  @param v_fifo_gyro_us8 : The value of fifo gyro enble
 *	value    | fifo gyro
 * ----------|-------------------
 *  0x00     |  no gyro data is stored
 *  0x01     |  gyro data is stored
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_gyro_enable(
us8 *v_fifo_gyro_us8);
/*!
 *	@brief This API is used to set stored
 *	gyro data in FIFO (all 3 axes) from the resister 0x47 bit 7
 *
 *
 *  @param v_fifo_gyro_us8 : The value of fifo gyro enble
 *	value    | fifo gyro
 * ----------|-------------------
 *  0x00     |  no gyro data is stored
 *  0x01     |  gyro data is stored
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_gyro_enable(
us8 v_fifo_gyro_us8);
/***************************************************************/
/**\name	FUNCTION FOR MAG I2C ADDRESS SELECTION          */
/***************************************************************/
/*!
 *	@brief This API is used to read
 *	I2C device address of auxiliary mag from the register 0x4B bit 1 to 7
 *
 *
 *
 *
 *  @param v_i2c_device_addr_us8 : The value of mag I2C device address
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_device_addr(
us8 *v_i2c_device_addr_us8);
/*!
 *	@brief This API is used to set
 *	I2C device address of auxiliary mag from the register 0x4B bit 1 to 7
 *
 *
 *
 *
 *  @param v_i2c_device_addr_us8 : The value of mag I2C device address
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_device_addr(
us8 v_i2c_device_addr_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT ENABLE OF
ANY-MOTION XYZ, DOUBLE AND SINGLE TAP, ORIENT AND FLAT         */
/***************************************************************/
/*!
 *	@brief  This API is used to read
 *	interrupt enable from the register 0x50 bit 0 to 7
 *
 *
 *
 *
 *	@param v_enable_us8 : Value to decided to select interrupt
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_ANY_MOTION_X_ENABLE
 *       1         | BMI160_ANY_MOTION_Y_ENABLE
 *       2         | BMI160_ANY_MOTION_Z_ENABLE
 *       3         | BMI160_DOUBLE_TAP_ENABLE
 *       4         | BMI160_SINGLE_TAP_ENABLE
 *       5         | BMI160_ORIENT_ENABLE
 *       6         | BMI160_FLAT_ENABLE
 *
 *	@param v_intr_enable_zero_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_0(
us8 enable, us8 *v_intr_enable_zero_us8);
/*!
 *	@brief  This API is used to set
 *	interrupt enable from the register 0x50 bit 0 to 7
 *
 *
 *
 *
 *	@param v_enable_us8 : Value to decided to select interrupt
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_ANY_MOTION_X_ENABLE
 *       1         | BMI160_ANY_MOTION_Y_ENABLE
 *       2         | BMI160_ANY_MOTION_Z_ENABLE
 *       3         | BMI160_DOUBLE_TAP_ENABLE
 *       4         | BMI160_SINGLE_TAP_ENABLE
 *       5         | BMI160_ORIENT_ENABLE
 *       6         | BMI160_FLAT_ENABLE
 *
 *	@param v_intr_enable_zero_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_0(
us8 enable, us8 v_intr_enable_zero_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT ENABLE OF
HIGH_G XYZ, LOW_G, DATA READY, FIFO FULL AND FIFO WATER MARK  */
/***************************************************************/
/*!
 *	@brief  This API is used to read
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *	@brief It read the high_g_x,high_g_y,high_g_z,low_g_enable
 *	data ready, fifo full and fifo water mark.
 *
 *
 *
 *  @param  v_enable_us8 :  The value of interrupt enable
 *	@param v_enable_us8 : Value to decided to select interrupt
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_HIGH_G_X_ENABLE
 *       1         | BMI160_HIGH_G_Y_ENABLE
 *       2         | BMI160_HIGH_G_Z_ENABLE
 *       3         | BMI160_LOW_G_ENABLE
 *       4         | BMI160_DATA_RDY_ENABLE
 *       5         | BMI160_FIFO_FULL_ENABLE
 *       6         | BMI160_FIFO_WM_ENABLE
 *
 *	@param v_intr_enable_1_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_1(
us8 enable, us8 *v_intr_enable_1_us8);
/*!
 *	@brief  This API is used to set
 *	interrupt enable byte1 from the register 0x51 bit 0 to 6
 *	@brief It read the high_g_x,high_g_y,high_g_z,low_g_enable
 *	data ready, fifo full and fifo water mark.
 *
 *
 *
 *  @param  v_enable_us8 :  The value of interrupt enable
 *	@param v_enable_us8 : Value to decided to select interrupt
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_HIGH_G_X_ENABLE
 *       1         | BMI160_HIGH_G_Y_ENABLE
 *       2         | BMI160_HIGH_G_Z_ENABLE
 *       3         | BMI160_LOW_G_ENABLE
 *       4         | BMI160_DATA_RDY_ENABLE
 *       5         | BMI160_FIFO_FULL_ENABLE
 *       6         | BMI160_FIFO_WM_ENABLE
 *
 *	@param v_intr_enable_1_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_1(
us8 enable, us8 v_intr_enable_1_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT ENABLE OF
NO MOTION XYZ  */
/***************************************************************/
/*!
 *	@brief  This API is used to read
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *	@brief It reads no motion x,y and z
 *
 *
 *
 *	@param v_enable_us8: The value of interrupt enable
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_NOMOTION_X_ENABLE
 *       1         | BMI160_NOMOTION_Y_ENABLE
 *       2         | BMI160_NOMOTION_Z_ENABLE
 *
 *	@param v_intr_enable_2_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_2(
us8 enable, us8 *v_intr_enable_2_us8);
/*!
 *	@brief  This API is used to set
 *	interrupt enable byte2 from the register bit 0x52 bit 0 to 3
 *	@brief It reads no motion x,y and z
 *
 *
 *
 *	@param v_enable_us8: The value of interrupt enable
 *   v_enable_us8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_NOMOTION_X_ENABLE
 *       1         | BMI160_NOMOTION_Y_ENABLE
 *       2         | BMI160_NOMOTION_Z_ENABLE
 *
 *	@param v_intr_enable_2_us8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_2(
us8 enable, us8 v_intr_enable_2_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT ENABLE OF
  STEP DETECTOR */
/***************************************************************/
 /*!
 *	@brief This API is used to read
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *
 *
 *
 *
 *	@param v_step_intr_us8 : The value of step detector interrupt enable
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_detector_enable(
us8 *v_step_intr_us8);
 /*!
 *	@brief This API is used to set
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *
 *
 *
 *
 *	@param v_step_intr_us8 : The value of step detector interrupt enable
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_detector_enable(
us8 v_step_intr_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT CONTROL */
/***************************************************************/
/*!
 *	@brief  Configure trigger condition of interrupt1
 *	and interrupt2 pin from the register 0x53
 *	@brief interrupt1 - bit 0
 *	@brief interrupt2 - bit 4
 *
 *  @param v_channel_us8: The value of edge trigger selection
 *   v_channel_us8  |   Edge trigger
 *  ---------------|---------------
 *       0         | BMI160_INTR1_EDGE_CTRL
 *       1         | BMI160_INTR2_EDGE_CTRL
 *
 *	@param v_intr_edge_ctrl_us8 : The value of edge trigger enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_EDGE
 *  0x00     |  BMI160_LEVEL
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_edge_ctrl(
us8 v_channel_us8, us8 *v_intr_edge_ctrl_us8);
/*!
 *	@brief  Configure trigger condition of interrupt1
 *	and interrupt2 pin from the register 0x53
 *	@brief interrupt1 - bit 0
 *	@brief interrupt2 - bit 4
 *
 *  @param v_channel_us8: The value of edge trigger selection
 *   v_channel_us8  |   Edge trigger
 *  ---------------|---------------
 *       0         | BMI160_INTR1_EDGE_CTRL
 *       1         | BMI160_INTR2_EDGE_CTRL
 *
 *	@param v_intr_edge_ctrl_us8 : The value of edge trigger enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_EDGE
 *  0x00     |  BMI160_LEVEL
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_edge_ctrl(
us8 v_channel_us8, us8 v_intr_edge_ctrl_us8);
/*!
 *	@brief  API used for get the Configure level condition of interrupt1
 *	and interrupt2 pin form the register 0x53
 *	@brief interrupt1 - bit 1
 *	@brief interrupt2 - bit 5
 *
 *  @param v_channel_us8: The value of level condition selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_LEVEL
 *       1         | BMI160_INTR2_LEVEL
 *
 *	@param v_intr_level_us8 : The value of level of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_LEVEL_HIGH
 *  0x00     |  BMI160_LEVEL_LOW
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_level(
us8 v_channel_us8, us8 *v_intr_level_us8);
/*!
 *	@brief  API used for set the Configure level condition of interrupt1
 *	and interrupt2 pin form the register 0x53
 *	@brief interrupt1 - bit 1
 *	@brief interrupt2 - bit 5
 *
 *  @param v_channel_us8: The value of level condition selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_LEVEL
 *       1         | BMI160_INTR2_LEVEL
 *
 *	@param v_intr_level_us8 : The value of level of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_LEVEL_HIGH
 *  0x00     |  BMI160_LEVEL_LOW
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_level(
us8 v_channel_us8, us8 v_intr_level_us8);
/*!
 *	@brief  API used to get configured output enable of interrupt1
 *	and interrupt2 from the register 0x53
 *	@brief interrupt1 - bit 2
 *	@brief interrupt2 - bit 6
 *
 *
 *  @param v_channel_us8: The value of output type enable selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_OUTPUT_TYPE
 *       1         | BMI160_INTR2_OUTPUT_TYPE
 *
 *	@param v_intr_output_type_us8 :
 *	The value of output type of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_OPEN_DRAIN
 *  0x00     |  BMI160_PUSH_PULL
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_output_type(
us8 v_channel_us8, us8 *v_intr_output_type_us8);
/*!
 *	@brief  API used to set output enable of interrupt1
 *	and interrupt2 from the register 0x53
 *	@brief interrupt1 - bit 2
 *	@brief interrupt2 - bit 6
 *
 *
 *  @param v_channel_us8: The value of output type enable selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_OUTPUT_TYPE
 *       1         | BMI160_INTR2_OUTPUT_TYPE
 *
 *	@param v_intr_output_type_us8 :
 *	The value of output type of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_OPEN_DRAIN
 *  0x00     |  BMI160_PUSH_PULL
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_output_type(
us8 v_channel_us8, us8 v_intr_output_type_us8);
 /*!
 *	@brief API used to get the Output enable for interrupt1
 *	and interrupt1 pin from the register 0x53
 *	@brief interrupt1 - bit 3
 *	@brief interrupt2 - bit 7
 *
 *  @param v_channel_us8: The value of output enable selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_OUTPUT_TYPE
 *       1         | BMI160_INTR2_OUTPUT_TYPE
 *
 *	@param v_output_enable_us8 :
 *	The value of output enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_INPUT
 *  0x00     |  BMI160_OUTPUT
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_output_enable(
us8 v_channel_us8, us8 *v_output_enable_us8);
 /*!
 *	@brief API used to set the Output enable for interrupt1
 *	and interrupt1 pin from the register 0x53
 *	@brief interrupt1 - bit 3
 *	@brief interrupt2 - bit 7
 *
 *  @param v_channel_us8: The value of output enable selection
 *   v_channel_us8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_OUTPUT_TYPE
 *       1         | BMI160_INTR2_OUTPUT_TYPE
 *
 *	@param v_output_enable_us8 :
 *	The value of output enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_INPUT
 *  0x00     |  BMI160_OUTPUT
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_output_enable(
us8 v_channel_us8, us8 v_output_enable_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT LATCH INTERRUPT  */
/***************************************************************/
/*!
*	@brief This API is used to get the latch duration
*	from the register 0x54 bit 0 to 3
*	@brief This latch selection is not applicable for data ready,
*	orientation and flat interrupts.
*
*
*
*  @param v_latch_intr_us8 : The value of latch duration
*	Latch Duration                      |     value
* --------------------------------------|------------------
*    BMI160_LATCH_DUR_NONE              |      0x00
*    BMI160_LATCH_DUR_312_5_MICRO_SEC   |      0x01
*    BMI160_LATCH_DUR_625_MICRO_SEC     |      0x02
*    BMI160_LATCH_DUR_1_25_MILLI_SEC    |      0x03
*    BMI160_LATCH_DUR_2_5_MILLI_SEC     |      0x04
*    BMI160_LATCH_DUR_5_MILLI_SEC       |      0x05
*    BMI160_LATCH_DUR_10_MILLI_SEC      |      0x06
*    BMI160_LATCH_DUR_20_MILLI_SEC      |      0x07
*    BMI160_LATCH_DUR_40_MILLI_SEC      |      0x08
*    BMI160_LATCH_DUR_80_MILLI_SEC      |      0x09
*    BMI160_LATCH_DUR_160_MILLI_SEC     |      0x0A
*    BMI160_LATCH_DUR_320_MILLI_SEC     |      0x0B
*    BMI160_LATCH_DUR_640_MILLI_SEC     |      0x0C
*    BMI160_LATCH_DUR_1_28_SEC          |      0x0D
*    BMI160_LATCH_DUR_2_56_SEC          |      0x0E
*    BMI160_LATCHED                     |      0x0F
*
*
*
*	@return results of bus communication function
*	@retval 0 -> Success
*	@retval -1 -> Error
*
*
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_latch_intr(
us8 *v_latch_intr_us8);
/*!
*	@brief This API is used to set the latch duration
*	from the register 0x54 bit 0 to 3
*	@brief This latch selection is not applicable for data ready,
*	orientation and flat interrupts.
*
*
*
*  @param v_latch_intr_us8 : The value of latch duration
*	Latch Duration                      |     value
* --------------------------------------|------------------
*    BMI160_LATCH_DUR_NONE              |      0x00
*    BMI160_LATCH_DUR_312_5_MICRO_SEC   |      0x01
*    BMI160_LATCH_DUR_625_MICRO_SEC     |      0x02
*    BMI160_LATCH_DUR_1_25_MILLI_SEC    |      0x03
*    BMI160_LATCH_DUR_2_5_MILLI_SEC     |      0x04
*    BMI160_LATCH_DUR_5_MILLI_SEC       |      0x05
*    BMI160_LATCH_DUR_10_MILLI_SEC      |      0x06
*    BMI160_LATCH_DUR_20_MILLI_SEC      |      0x07
*    BMI160_LATCH_DUR_40_MILLI_SEC      |      0x08
*    BMI160_LATCH_DUR_80_MILLI_SEC      |      0x09
*    BMI160_LATCH_DUR_160_MILLI_SEC     |      0x0A
*    BMI160_LATCH_DUR_320_MILLI_SEC     |      0x0B
*    BMI160_LATCH_DUR_640_MILLI_SEC     |      0x0C
*    BMI160_LATCH_DUR_1_28_SEC          |      0x0D
*    BMI160_LATCH_DUR_2_56_SEC          |      0x0E
*    BMI160_LATCHED                     |      0x0F
*
*
*
*	@return results of bus communication function
*	@retval 0 -> Success
*	@retval -1 -> Error
*
*
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_latch_intr(
us8 v_latch_intr_us8);
/*!
 *	@brief API used to get input enable for interrupt1
 *	and interrupt2 pin from the register 0x54
 *	@brief interrupt1 - bit 4
 *	@brief interrupt2 - bit 5
 *
 *  @param v_channel_us8: The value of input enable selection
 *   v_channel_us8  |   input selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_INPUT_ENABLE
 *       1         | BMI160_INTR2_INPUT_ENABLE
 *
 *	@param v_input_en_us8 :
 *	The value of input enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_INPUT
 *  0x00     |  BMI160_OUTPUT
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_input_enable(
us8 v_channel_us8, us8 *v_input_en_us8);
/*!
 *	@brief API used to set input enable for interrupt1
 *	and interrupt2 pin from the register 0x54
 *	@brief interrupt1 - bit 4
 *	@brief interrupt2 - bit 5
 *
 *  @param v_channel_us8: The value of input enable selection
 *   v_channel_us8  |   input selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_INPUT_ENABLE
 *       1         | BMI160_INTR2_INPUT_ENABLE
 *
 *	@param v_input_en_us8 :
 *	The value of input enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_INPUT
 *  0x00     |  BMI160_OUTPUT
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_input_enable(
us8 v_channel_us8, us8 v_input_en_us8);
/***************************************************************/
/**\name	FUNCTION FOR INTERRUPT1 AND INTERRUPT2 MAPPING */
/***************************************************************/
 /*!
 *	@brief reads the Low g interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 0 in the register 0x55
 *	@brief interrupt2 bit 0 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of low_g selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_LOW_G
 *       1         | BMI160_INTR2_MAP_LOW_G
 *
 *	@param v_intr_low_g_us8 : The value of low_g enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g(
us8 v_channel_us8, us8 *v_intr_low_g_us8);
 /*!
 *	@brief set the Low g interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 0 in the register 0x55
 *	@brief interrupt2 bit 0 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of low_g selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_LOW_G
 *       1         | BMI160_INTR2_MAP_LOW_G
 *
 *	@param v_intr_low_g_us8 : The value of low_g enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g(
us8 v_channel_us8, us8 v_intr_low_g_us8);
/*!
 *	@brief Reads the HIGH g interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 1 in the register 0x55
 *	@brief interrupt2 bit 1 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of high_g selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_HIGH_G
 *       1         | BMI160_INTR2_MAP_HIGH_G
 *
 *	@param v_intr_high_g_us8 : The value of high_g enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g(
us8 v_channel_us8, us8 *v_intr_high_g_us8);
/*!
 *	@brief Write the HIGH g interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 1 in the register 0x55
 *	@brief interrupt2 bit 1 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of high_g selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_HIGH_G
 *       1         | BMI160_INTR2_MAP_HIGH_G
 *
 *	@param v_intr_high_g_us8 : The value of high_g enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g(
us8 v_channel_us8, us8 v_intr_high_g_us8);
/*!
 *	@brief Reads the Any motion interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 2 in the register 0x55
 *	@brief interrupt2 bit 2 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of any motion selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_ANY_MOTION
 *       1         | BMI160_INTR2_MAP_ANY_MOTION
 *
 *	@param v_intr_any_motion_us8 : The value of any motion enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion(
us8 v_channel_us8, us8 *v_intr_any_motion_us8);
/*!
 *	@brief Write the Any motion interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 2 in the register 0x55
 *	@brief interrupt2 bit 2 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of any motion selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_ANY_MOTION
 *       1         | BMI160_INTR2_MAP_ANY_MOTION
 *
 *	@param v_intr_any_motion_us8 : The value of any motion enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion(
us8 v_channel_us8, us8 v_intr_any_motion_us8);
/*!
 *	@brief Reads the No motion interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 3 in the register 0x55
 *	@brief interrupt2 bit 3 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of no motion selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_NOMO
 *       1         | BMI160_INTR2_MAP_NOMO
 *
 *	@param v_intr_nomotion_us8 : The value of no motion enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_nomotion(
us8 v_channel_us8, us8 *v_intr_nomotion_us8);
/*!
 *	@brief Write the No motion interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 3 in the register 0x55
 *	@brief interrupt2 bit 3 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of no motion selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_NOMO
 *       1         | BMI160_INTR2_MAP_NOMO
 *
 *	@param v_intr_nomotion_us8 : The value of no motion enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_nomotion(
us8 v_channel_us8, us8 v_intr_nomotion_us8);
/*!
 *	@brief Reads the Double Tap interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 4 in the register 0x55
 *	@brief interrupt2 bit 4 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of double tap interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_DOUBLE_TAP
 *       1         | BMI160_INTR2_MAP_DOUBLE_TAP
 *
 *	@param v_intr_double_tap_us8 : The value of double tap enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_double_tap(
us8 v_channel_us8, us8 *v_intr_double_tap_us8);
/*!
 *	@brief Write the Double Tap interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 4 in the register 0x55
 *	@brief interrupt2 bit 4 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of double tap interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_DOUBLE_TAP
 *       1         | BMI160_INTR2_MAP_DOUBLE_TAP
 *
 *	@param v_intr_double_tap_us8 : The value of double tap enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_double_tap(
us8 v_channel_us8, us8 v_intr_double_tap_us8);
/*!
 *	@brief Reads the Single Tap interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 5 in the register 0x55
 *	@brief interrupt2 bit 5 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of single tap interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_SINGLE_TAP
 *       1         | BMI160_INTR2_MAP_SINGLE_TAP
 *
 *	@param v_intr_single_tap_us8 : The value of single tap  enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_single_tap(
us8 v_channel_us8, us8 *v_intr_single_tap_us8);
/*!
 *	@brief Write the Single Tap interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 5 in the register 0x55
 *	@brief interrupt2 bit 5 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of single tap interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_SINGLE_TAP
 *       1         | BMI160_INTR2_MAP_SINGLE_TAP
 *
 *	@param v_intr_single_tap_us8 : The value of single tap  enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_single_tap(
us8 v_channel_us8, us8 v_intr_single_tap_us8);
/*!
 *	@brief Reads the Orient interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 6 in the register 0x55
 *	@brief interrupt2 bit 6 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of orient interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_ORIENT
 *       1         | BMI160_INTR2_MAP_ORIENT
 *
 *	@param v_intr_orient_us8 : The value of orient enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient(
us8 v_channel_us8, us8 *v_intr_orient_us8);
/*!
 *	@brief Write the Orient interrupt
 *	interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 6 in the register 0x55
 *	@brief interrupt2 bit 6 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of orient interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_ORIENT
 *       1         | BMI160_INTR2_MAP_ORIENT
 *
 *	@param v_intr_orient_us8 : The value of orient enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient(
us8 v_channel_us8, us8 v_intr_orient_us8);
 /*!
 *	@brief Reads the Flat interrupt
 *	mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 7 in the register 0x55
 *	@brief interrupt2 bit 7 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of flat interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FLAT
 *       1         | BMI160_INTR2_MAP_FLAT
 *
 *	@param v_intr_flat_us8 : The value of flat enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat(
us8 v_channel_us8, us8 *v_intr_flat_us8);
 /*!
 *	@brief Write the Flat interrupt
 *	mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 7 in the register 0x55
 *	@brief interrupt2 bit 7 in the register 0x57
 *
 *
 *	@param v_channel_us8: The value of flat interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FLAT
 *       1         | BMI160_INTR2_MAP_FLAT
 *
 *	@param v_intr_flat_us8 : The value of flat enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat(
us8 v_channel_us8, us8 v_intr_flat_us8);
/*!
 *	@brief Reads PMU trigger interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 0 and 4
 *	@brief interrupt1 bit 0 in the register 0x56
 *	@brief interrupt2 bit 4 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of pmu trigger selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_PMUTRIG
 *       1         | BMI160_INTR2_MAP_PMUTRIG
 *
 *	@param v_intr_pmu_trig_us8 : The value of pmu trigger enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_pmu_trig(
us8 v_channel_us8, us8 *v_intr_pmu_trig_us8);
/*!
 *	@brief Write PMU trigger interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 0 and 4
 *	@brief interrupt1 bit 0 in the register 0x56
 *	@brief interrupt2 bit 4 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of pmu trigger selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_PMUTRIG
 *       1         | BMI160_INTR2_MAP_PMUTRIG
 *
 *	@param v_intr_pmu_trig_us8 : The value of pmu trigger enable
 *	value    | trigger enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_pmu_trig(
us8 v_channel_us8, us8 v_intr_pmu_trig_us8);
/*!
 *	@brief Reads FIFO Full interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 5 and 1
 *	@brief interrupt1 bit 5 in the register 0x56
 *	@brief interrupt2 bit 1 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of fifo full interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FIFO_FULL
 *       1         | BMI160_INTR2_MAP_FIFO_FULL
 *
 *	@param v_intr_fifo_full_us8 : The value of fifo full interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_full(
us8 v_channel_us8, us8 *v_intr_fifo_full_us8);
/*!
 *	@brief Write FIFO Full interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 5 and 1
 *	@brief interrupt1 bit 5 in the register 0x56
 *	@brief interrupt2 bit 1 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of fifo full interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FIFO_FULL
 *       1         | BMI160_INTR2_MAP_FIFO_FULL
 *
 *	@param v_intr_fifo_full_us8 : The value of fifo full interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_full(
us8 v_channel_us8, us8 v_intr_fifo_full_us8);
/*!
 *	@brief Reads FIFO Watermark interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 6 and 2
 *	@brief interrupt1 bit 6 in the register 0x56
 *	@brief interrupt2 bit 2 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of fifo Watermark interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FIFO_WM
 *       1         | BMI160_INTR2_MAP_FIFO_WM
 *
 *	@param v_intr_fifo_wm_us8 : The value of fifo Watermark interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_wm(
us8 v_channel_us8, us8 *v_intr_fifo_wm_us8);
/*!
 *	@brief Write FIFO Watermark interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56 bit 6 and 2
 *	@brief interrupt1 bit 6 in the register 0x56
 *	@brief interrupt2 bit 2 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of fifo Watermark interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_FIFO_WM
 *       1         | BMI160_INTR2_MAP_FIFO_WM
 *
 *	@param v_intr_fifo_wm_us8 : The value of fifo Watermark interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_wm(
us8 v_channel_us8, us8 v_intr_fifo_wm_us8);
/*!
 *	@brief Reads Data Ready interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56
 *	@brief interrupt1 bit 7 in the register 0x56
 *	@brief interrupt2 bit 3 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of data ready interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_DATA_RDY
 *       1         | BMI160_INTR2_MAP_DATA_RDY
 *
 *	@param v_intr_data_rdy_us8 : The value of data ready interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_data_rdy(
us8 v_channel_us8, us8 *v_intr_data_rdy_us8);
/*!
 *	@brief Write Data Ready interrupt mapped to interrupt1
 *	and interrupt2 form the register 0x56
 *	@brief interrupt1 bit 7 in the register 0x56
 *	@brief interrupt2 bit 3 in the register 0x56
 *
 *
 *	@param v_channel_us8: The value of data ready interrupt selection
 *   v_channel_us8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_DATA_RDY
 *       1         | BMI160_INTR2_MAP_DATA_RDY
 *
 *	@param v_intr_data_rdy_us8 : The value of data ready interrupt enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_data_rdy(
us8 v_channel_us8, us8 v_intr_data_rdy_us8);
/***************************************************************/
/**\name	FUNCTION FOR TAP SOURCE CONFIGURATION          */
/***************************************************************/
 /*!
 *	@brief This API reads data source for the interrupt
 *	engine for the single and double tap interrupts from the register
 *	0x58 bit 3
 *
 *
 *  @param v_tap_source_us8 : The value of the tap source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_source(
us8 *v_tap_source_us8);
 /*!
 *	@brief This API write data source for the interrupt
 *	engine for the single and double tap interrupts from the register
 *	0x58 bit 3
 *
 *
 *  @param v_tap_source_us8 : The value of the tap source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_source(
us8 v_tap_source_us8);
/***************************************************************/
/**\name	FUNCTION FOR LOW_G AND HIGH_G SOURCE CONFIGURATION */
/***************************************************************/
 /*!
 *	@brief This API Reads Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  @param v_low_high_source_us8 : The value of the tap source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_high_source(
us8 *v_low_high_source_us8);
 /*!
 *	@brief This API write Data source for the
 *	interrupt engine for the low and high g interrupts
 *	from the register 0x58 bit 7
 *
 *  @param v_low_high_source_us8 : The value of the tap source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_high_source(
us8 v_low_high_source_us8);
/***************************************************************/
/**\name	FUNCTION FOR MOTION SOURCE CONFIGURATION          */
/***************************************************************/
 /*!
 *	@brief This API reads Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  @param v_motion_source_us8 :
 *	The value of the any/no motion interrupt source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_motion_source(
us8 *v_motion_source_us8);
 /*!
 *	@brief This API write Data source for the
 *	interrupt engine for the nomotion and anymotion interrupts
 *	from the register 0x59 bit 7
 *
 *  @param v_motion_source_us8 :
 *	The value of the any/no motion interrupt source
 *	value    | Description
 * ----------|-------------------
 *  0x01     |  UNFILTER_DATA
 *  0x00     |  FILTER_DATA
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_motion_source(
us8 v_motion_source_us8);
/***************************************************************/
/**\name	FUNCTION FOR LOW_G DURATION CONFIGURATION          */
/***************************************************************/
/*!
 *	@brief This API is used to read the low_g duration from register
 *	0x5A bit 0 to 7
 *
 *
 *
 *
 *  @param v_low_g_durn_us8 : The value of low_g duration
 *
 *	@note Low_g duration trigger trigger delay according to
 *	"(v_low_g_durn_us8 * 2.5)ms" in a range from 2.5ms to 640ms.
 *	the default corresponds delay is 20ms
 *	@note When low_g data source of interrupt is unfiltered
 *	the sensor must not be in low power mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_durn(
us8 *v_low_durn_us8);
 /*!
 *	@brief This API is used to write the low_g duration from register
 *	0x5A bit 0 to 7
 *
 *
 *
 *
 *  @param v_low_g_durn_us8 : The value of low_g duration
 *
 *	@note Low_g duration trigger trigger delay according to
 *	"(v_low_g_durn_us8 * 2.5)ms" in a range from 2.5ms to 640ms.
 *	the default corresponds delay is 20ms
 *	@note When low_g data source of interrupt is unfiltered
 *	the sensor must not be in low power mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_durn(
us8 v_low_durn_us8);
/***************************************************************/
/**\name	FUNCTION FOR LOW_G THRESH CONFIGURATION          */
/***************************************************************/
/*!
 *	@brief This API is used to read Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  @param v_low_g_thres_us8 : The value of low_g threshold
 *
 *	@note Low_g interrupt trigger threshold according to
 *	(v_low_g_thres_us8 * 7.81)mg for v_low_g_thres_us8 > 0
 *	3.91 mg for v_low_g_thres_us8 = 0
 *	The threshold range is form 3.91mg to 2.000mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_thres(
us8 *v_low_g_thres_us8);
/*!
 *	@brief This API is used to write Threshold
 *	definition for the low-g interrupt from the register 0x5B bit 0 to 7
 *
 *
 *
 *
 *  @param v_low_g_thres_us8 : The value of low_g threshold
 *
 *	@note Low_g interrupt trigger threshold according to
 *	(v_low_g_thres_us8 * 7.81)mg for v_low_g_thres_us8 > 0
 *	3.91 mg for v_low_g_thres_us8 = 0
 *	The threshold range is form 3.91mg to 2.000mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_thres(
us8 v_low_g_thres_us8);
/***************************************************************/
/**\name	FUNCTION FOR LOW_G HYSTERESIS CONFIGURATION     */
/***************************************************************/
 /*!
 *	@brief This API Reads Low-g interrupt hysteresis
 *	from the register 0x5C bit 0 to 1
 *
 *  @param v_low_hyst_us8 :The value of low_g hysteresis
 *
 *	@note Low_g hysteresis calculated by v_low_hyst_us8*125 mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_hyst(
us8 *v_low_hyst_us8);
 /*!
 *	@brief This API write Low-g interrupt hysteresis
 *	from the register 0x5C bit 0 to 1
 *
 *  @param v_low_hyst_us8 :The value of low_g hysteresis
 *
 *	@note Low_g hysteresis calculated by v_low_hyst_us8*125 mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_hyst(
us8 v_low_hyst_us8);
/***************************************************************/
/**\name	FUNCTION FOR LOW_G MODE CONFIGURATION     */
/***************************************************************/
/*!
 *	@brief This API reads Low-g interrupt mode
 *	from the register 0x5C bit 2
 *
 *  @param v_low_g_mode_us8 : The value of low_g mode
 *	Value    |  Description
 * ----------|-----------------
 *	   0     | single-axis
 *     1     | axis-summing
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_mode(
us8 *v_low_g_mode_us8);
/*!
 *	@brief This API write Low-g interrupt mode
 *	from the register 0x5C bit 2
 *
 *  @param v_low_g_mode_us8 : The value of low_g mode
 *	Value    |  Description
 * ----------|-----------------
 *	   0     | single-axis
 *     1     | axis-summing
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_mode(
us8 v_low_g_mode_us8);
/***************************************************************/
/**\name	FUNCTION FOR HIGH_G HYST CONFIGURATION     */
/***************************************************************/
/*!
 *	@brief This API reads High-g interrupt hysteresis
 *	from the register 0x5C bit 6 and 7
 *
 *  @param v_high_g_hyst_us8 : The value of high hysteresis
 *
 *	@note High_g hysteresis changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | high_g hysteresis
 *  ----------------|---------------------
 *      2g          |  high_hy*125 mg
 *      4g          |  high_hy*250 mg
 *      8g          |  high_hy*500 mg
 *      16g         |  high_hy*1000 mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_hyst(
us8 *v_high_g_hyst_us8);
/*!
 *	@brief This API write High-g interrupt hysteresis
 *	from the register 0x5C bit 6 and 7
 *
 *  @param v_high_g_hyst_us8 : The value of high hysteresis
 *
 *	@note High_g hysteresis changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | high_g hysteresis
 *  ----------------|---------------------
 *      2g          |  high_hy*125 mg
 *      4g          |  high_hy*250 mg
 *      8g          |  high_hy*500 mg
 *      16g         |  high_hy*1000 mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_hyst(
us8 v_high_g_hyst_us8);
/***************************************************************/
/**\name	FUNCTION FOR HIGH_G DURATION CONFIGURATION     */
/***************************************************************/
/*!
 *	@brief This API is used to read Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  @param  v_high_g_durn_us8 :  The value of high duration
 *
 *	@note High_g interrupt delay triggered according to
 *	v_high_g_durn_us8 * 2.5ms in a range from 2.5ms to 640ms
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_durn(
us8 *v_high_g_durn_us8);
/*!
 *	@brief This API is used to write Delay
 *	time definition for the high-g interrupt from the register
 *	0x5D bit 0 to 7
 *
 *
 *
 *  @param  v_high_g_durn_us8 :  The value of high duration
 *
 *	@note High_g interrupt delay triggered according to
 *	v_high_g_durn_us8 * 2.5ms in a range from 2.5ms to 640ms
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_durn(
us8 v_high_g_durn_us8);
/***************************************************************/
/**\name	FUNCTION FOR HIGH_G THRESHOLD CONFIGURATION     */
/***************************************************************/
/*!
 *	@brief This API is used to read Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  @param  v_high_g_thres_us8 : Pointer holding the value of Threshold
 *	@note High_g threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | high_g threshold
 *  ----------------|---------------------
 *      2g          |  v_high_g_thres_us8*7.81 mg
 *      4g          |  v_high_g_thres_us8*15.63 mg
 *      8g          |  v_high_g_thres_us8*31.25 mg
 *      16g         |  v_high_g_thres_us8*62.5 mg
 *	@note when v_high_g_thres_us8 = 0
 *   accel_range    | high_g threshold
 *  ----------------|---------------------
 *      2g          |  3.91 mg
 *      4g          |  7.81 mg
 *      8g          |  15.63 mg
 *      16g         |  31.25 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_thres(
us8 *v_high_g_thres_us8);
/*!
 *	@brief This API is used to write Threshold
 *	definition for the high-g interrupt from the register 0x5E 0 to 7
 *
 *
 *
 *
 *  @param  v_high_g_thres_us8 : Pointer holding the value of Threshold
 *	@note High_g threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | high_g threshold
 *  ----------------|---------------------
 *      2g          |  v_high_g_thres_us8*7.81 mg
 *      4g          |  v_high_g_thres_us8*15.63 mg
 *      8g          |  v_high_g_thres_us8*31.25 mg
 *      16g         |  v_high_g_thres_us8*62.5 mg
 *	@note when v_high_g_thres_us8 = 0
 *   accel_range    | high_g threshold
 *  ----------------|---------------------
 *      2g          |  3.91 mg
 *      4g          |  7.81 mg
 *      8g          |  15.63 mg
 *      16g         |  31.25 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_thres(
us8 v_high_g_thres_us8);
/***************************************************************/
/**\name	FUNCTION FOR ANY MOTION DURATION CONFIGURATION     */
/***************************************************************/
/*!
 *	@brief This API reads any motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  @param v_any_motion_durn_us8 : The value of any motion duration
 *
 *	@note Any motion duration can be calculated by "v_any_motion_durn_us8 + 1"
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_durn(
us8 *v_any_motion_durn_us8);
/*!
 *	@brief This API write any motion duration
 *	from the register 0x5F bit 0 and 1
 *
 *  @param v_any_motion_durn_us8 : The value of any motion duration
 *
 *	@note Any motion duration can be calculated by "v_any_motion_durn_us8 + 1"
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_durn(
us8 nomotion);
/***************************************************************/
/**\name	FUNCTION FOR SLOW NO MOTION DURATION CONFIGURATION  */
/***************************************************************/
 /*!
 *	@brief This API read Slow/no-motion
 *	interrupt trigger delay duration from the register 0x5F bit 2 to 7
 *
 *  @param v_slow_no_motion_us8 :The value of slow no motion duration
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	@note v_slow_no_motion_us8(5:4)=0b00 ->
 *	[v_slow_no_motion_us8(3:0) + 1] * 1.28s (1.28s-20.48s)
 *	@note v_slow_no_motion_us8(5:4)=1 ->
 *	[v_slow_no_motion_us8(3:0)+5] * 5.12s (25.6s-102.4s)
 *	@note v_slow_no_motion_us8(5)='1' ->
 *	[(v_slow_no_motion_us8:0)+11] * 10.24s (112.64s-430.08s);
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_durn(
us8 *v_slow_no_motion_us8);
 /*!
 *	@brief This API write Slow/no-motion
 *	interrupt trigger delay duration from the register 0x5F bit 2 to 7
 *
 *  @param v_slow_no_motion_us8 :The value of slow no motion duration
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	@note v_slow_no_motion_us8(5:4)=0b00 ->
 *	[v_slow_no_motion_us8(3:0) + 1] * 1.28s (1.28s-20.48s)
 *	@note v_slow_no_motion_us8(5:4)=1 ->
 *	[v_slow_no_motion_us8(3:0)+5] * 5.12s (25.6s-102.4s)
 *	@note v_slow_no_motion_us8(5)='1' ->
 *	[(v_slow_no_motion_us8:0)+11] * 10.24s (112.64s-430.08s);
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_durn(
us8 v_slow_no_motion_us8);
/***************************************************************/
/**\name	FUNCTION FOR ANY MOTION THRESHOLD CONFIGURATION  */
/***************************************************************/
/*!
 *	@brief This API is used to read threshold
 *	definition for the any-motion interrupt
 *	from the register 0x60 bit 0 to 7
 *
 *
 *  @param  v_any_motion_thres_us8 : The value of any motion threshold
 *
 *	@note any motion threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | any motion threshold
 *  ----------------|---------------------
 *      2g          |  v_any_motion_thres_us8*3.91 mg
 *      4g          |  v_any_motion_thres_us8*7.81 mg
 *      8g          |  v_any_motion_thres_us8*15.63 mg
 *      16g         |  v_any_motion_thres_us8*31.25 mg
 *	@note when v_any_motion_thres_us8 = 0
 *   accel_range    | any motion threshold
 *  ----------------|---------------------
 *      2g          |  1.95 mg
 *      4g          |  3.91 mg
 *      8g          |  7.81 mg
 *      16g         |  15.63 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_thres(
us8 *v_any_motion_thres_us8);
/*!
 *	@brief This API is used to write threshold
 *	definition for the any-motion interrupt
 *	from the register 0x60 bit 0 to 7
 *
 *
 *  @param  v_any_motion_thres_us8 : The value of any motion threshold
 *
 *	@note any motion threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | any motion threshold
 *  ----------------|---------------------
 *      2g          |  v_any_motion_thres_us8*3.91 mg
 *      4g          |  v_any_motion_thres_us8*7.81 mg
 *      8g          |  v_any_motion_thres_us8*15.63 mg
 *      16g         |  v_any_motion_thres_us8*31.25 mg
 *	@note when v_any_motion_thres_us8 = 0
 *   accel_range    | any motion threshold
 *  ----------------|---------------------
 *      2g          |  1.95 mg
 *      4g          |  3.91 mg
 *      8g          |  7.81 mg
 *      16g         |  15.63 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_thres(
us8 v_any_motion_thres_us8);
/***************************************************************/
/**\name	FUNCTION FOR SLO/NO MOTION THRESHOLD CONFIGURATION  */
/***************************************************************/
 /*!
 *	@brief This API is used to read threshold
 *	for the slow/no-motion interrupt
 *	from the register 0x61 bit 0 to 7
 *
 *
 *
 *
 *  @param v_slow_no_motion_thres_us8 : The value of slow no motion threshold
 *	@note slow no motion threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | slow no motion threshold
 *  ----------------|---------------------
 *      2g          |  v_slow_no_motion_thres_us8*3.91 mg
 *      4g          |  v_slow_no_motion_thres_us8*7.81 mg
 *      8g          |  v_slow_no_motion_thres_us8*15.63 mg
 *      16g         |  v_slow_no_motion_thres_us8*31.25 mg
 *	@note when v_slow_no_motion_thres_us8 = 0
 *   accel_range    | slow no motion threshold
 *  ----------------|---------------------
 *      2g          |  1.95 mg
 *      4g          |  3.91 mg
 *      8g          |  7.81 mg
 *      16g         |  15.63 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_thres(
us8 *v_slow_no_motion_thres_us8);
 /*!
 *	@brief This API is used to write threshold
 *	for the slow/no-motion interrupt
 *	from the register 0x61 bit 0 to 7
 *
 *
 *
 *
 *  @param v_slow_no_motion_thres_us8 : The value of slow no motion threshold
 *	@note slow no motion threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | slow no motion threshold
 *  ----------------|---------------------
 *      2g          |  v_slow_no_motion_thres_us8*3.91 mg
 *      4g          |  v_slow_no_motion_thres_us8*7.81 mg
 *      8g          |  v_slow_no_motion_thres_us8*15.63 mg
 *      16g         |  v_slow_no_motion_thres_us8*31.25 mg
 *	@note when v_slow_no_motion_thres_us8 = 0
 *   accel_range    | slow no motion threshold
 *  ----------------|---------------------
 *      2g          |  1.95 mg
 *      4g          |  3.91 mg
 *      8g          |  7.81 mg
 *      16g         |  15.63 mg
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_thres(
us8 v_slow_no_motion_thres_us8);
/***************************************************************/
/**\name	FUNCTION FOR SLO/NO MOTION SELECT CONFIGURATION  */
/***************************************************************/
 /*!
 *	@brief This API is used to read
 *	the slow/no-motion selection from the register 0x62 bit 0
 *
 *
 *
 *
 *  @param  v_intr_slow_no_motion_select_us8 :
 *	The value of slow/no-motion select
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  SLOW_MOTION
 *  0x01     |  NO_MOTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_select(
us8 *v_intr_slow_no_motion_select_us8);
 /*!
 *	@brief This API is used to write
 *	the slow/no-motion selection from the register 0x62 bit 0
 *
 *
 *
 *
 *  @param  v_intr_slow_no_motion_select_us8 :
 *	The value of slow/no-motion select
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  SLOW_MOTION
 *  0x01     |  NO_MOTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_select(
us8 v_intr_slow_no_motion_select_us8);
/***************************************************************/
/**\name	FUNCTION FOR SIGNIFICANT MOTION SELECT CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API is used to select
 *	the significant or any motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  @param  v_intr_significant_motion_select_us8 :
 *	the value of significant or any motion interrupt selection
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  ANY_MOTION
 *  0x01     |  SIGNIFICANT_MOTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_select(
us8 *int_sig_mot_sel);
 /*!
 *	@brief This API is used to write, select
 *	the significant or any motion interrupt from the register 0x62 bit 1
 *
 *
 *
 *
 *  @param  v_intr_significant_motion_select_us8 :
 *	the value of significant or any motion interrupt selection
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  ANY_MOTION
 *  0x01     |  SIGNIFICANT_MOTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_select(
us8 int_sig_mot_sel);
 /*!
 *	@brief This API is used to read
 *	the significant skip time from the register 0x62 bit  2 and 3
 *
 *
 *
 *
 *  @param  v_int_sig_mot_skip_us8 : the value of significant skip time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  skip time 1.5 seconds
 *  0x01     |  skip time 3 seconds
 *  0x02     |  skip time 6 seconds
 *  0x03     |  skip time 12 seconds
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_skip(
us8 *v_int_sig_mot_skip_us8);
 /*!
 *	@brief This API is used to write
 *	the significant skip time from the register 0x62 bit  2 and 3
 *
 *
 *
 *
 *  @param  v_int_sig_mot_skip_us8 : the value of significant skip time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  skip time 1.5 seconds
 *  0x01     |  skip time 3 seconds
 *  0x02     |  skip time 6 seconds
 *  0x03     |  skip time 12 seconds
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_skip(
us8 v_int_sig_mot_skip_us8);
 /*!
 *	@brief This API is used to read
 *	the significant proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  @param  v_significant_motion_proof_us8 :
 *	the value of significant proof time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  proof time 0.25 seconds
 *  0x01     |  proof time 0.5 seconds
 *  0x02     |  proof time 1 seconds
 *  0x03     |  proof time 2 seconds
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_proof(
us8 *int_sig_mot_proof);
 /*!
 *	@brief This API is used to write
 *	the significant proof time from the register 0x62 bit  4 and 5
 *
 *
 *
 *
 *  @param  v_significant_motion_proof_us8 :
 *	the value of significant proof time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     |  proof time 0.25 seconds
 *  0x01     |  proof time 0.5 seconds
 *  0x02     |  proof time 1 seconds
 *  0x03     |  proof time 2 seconds
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_proof(
us8 int_sig_mot_proof);
/***************************************************************/
/**\name	FUNCTION FOR TAP DURATION CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API is used to get the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  @param v_tap_durn_us8 : The value of tap duration
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_DURN_50MS
 *  0x01     | BMI160_TAP_DURN_100MS
 *  0x03     | BMI160_TAP_DURN_150MS
 *  0x04     | BMI160_TAP_DURN_200MS
 *  0x05     | BMI160_TAP_DURN_250MS
 *  0x06     | BMI160_TAP_DURN_375MS
 *  0x07     | BMI160_TAP_DURN_700MS
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_durn(
us8 *v_tap_durn_us8);
/*!
 *	@brief This API is used to write the tap duration
 *	from the register 0x63 bit 0 to 2
 *
 *
 *
 *  @param v_tap_durn_us8 : The value of tap duration
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_DURN_50MS
 *  0x01     | BMI160_TAP_DURN_100MS
 *  0x03     | BMI160_TAP_DURN_150MS
 *  0x04     | BMI160_TAP_DURN_200MS
 *  0x05     | BMI160_TAP_DURN_250MS
 *  0x06     | BMI160_TAP_DURN_375MS
 *  0x07     | BMI160_TAP_DURN_700MS
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_durn(
us8 v_tap_durn_us8);
/***************************************************************/
/**\name	FUNCTION FOR TAP SHOCK CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  @param v_tap_shock_us8 :The value of tap shock
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_SHOCK_50MS
 *  0x01     | BMI160_TAP_SHOCK_75MS
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_shock(
us8 *v_tap_shock_us8);
 /*!
 *	@brief This API write the
 *	tap shock duration from the register 0x63 bit 2
 *
 *  @param v_tap_shock_us8 :The value of tap shock
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_SHOCK_50MS
 *  0x01     | BMI160_TAP_SHOCK_75MS
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_shock(
us8 v_tap_shock_us8);
/***************************************************************/
/**\name	FUNCTION FOR TAP QUIET CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  @param v_tap_quiet_us8 : The value of tap quiet
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_QUIET_30MS
 *  0x01     | BMI160_TAP_QUIET_20MS
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_quiet(
us8 *v_tap_quiet_us8);
/*!
 *	@brief This API write
 *	tap quiet duration from the register 0x63 bit 7
 *
 *
 *  @param v_tap_quiet_us8 : The value of tap quiet
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | BMI160_TAP_QUIET_30MS
 *  0x01     | BMI160_TAP_QUIET_20MS
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_quiet(
us8 v_tap_quiet_us8);
/***************************************************************/
/**\name	FUNCTION FOR TAP THRESHOLD CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read Threshold of the
 *	single/double tap interrupt from the register 0x64 bit 0 to 4
 *
 *
 *	@param v_tap_thres_us8 : The value of single/double tap threshold
 *
 *	@note single/double tap threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | single/double tap threshold
 *  ----------------|---------------------
 *      2g          |  ((v_tap_thres_us8 + 1) * 62.5)mg
 *      4g          |  ((v_tap_thres_us8 + 1) * 125)mg
 *      8g          |  ((v_tap_thres_us8 + 1) * 250)mg
 *      16g         |  ((v_tap_thres_us8 + 1) * 500)mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_thres(
us8 *v_tap_thres_us8);
 /*!
 *	@brief This API write Threshold of the
 *	single/double tap interrupt from the register 0x64 bit 0 to 4
 *
 *
 *	@param v_tap_thres_us8 : The value of single/double tap threshold
 *
 *	@note single/double tap threshold changes according to accel g range
 *	accel g range can be set by the function ""
 *   accel_range    | single/double tap threshold
 *  ----------------|---------------------
 *      2g          |  ((v_tap_thres_us8 + 1) * 62.5)mg
 *      4g          |  ((v_tap_thres_us8 + 1) * 125)mg
 *      8g          |  ((v_tap_thres_us8 + 1) * 250)mg
 *      16g         |  ((v_tap_thres_us8 + 1) * 500)mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_thres(
us8 v_tap_thres_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT MODE CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read the threshold for orientation interrupt
 *	from the register 0x65 bit 0 and 1
 *
 *  @param v_orient_mode_us8 : The value of threshold for orientation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | symmetrical
 *  0x01     | high-asymmetrical
 *  0x02     | low-asymmetrical
 *  0x03     | symmetrical
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_mode(
us8 *v_orient_mode_us8);
 /*!
 *	@brief This API write the threshold for orientation interrupt
 *	from the register 0x65 bit 0 and 1
 *
 *  @param v_orient_mode_us8 : The value of threshold for orientation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | symmetrical
 *  0x01     | high-asymmetrical
 *  0x02     | low-asymmetrical
 *  0x03     | symmetrical
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_mode(
us8 v_orient_mode_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT BLOCKING CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read the orient blocking mode
 *	that is used for the generation of the orientation interrupt.
 *	from the register 0x65 bit 2 and 3
 *
 *  @param v_orient_blocking_us8 : The value of orient blocking mode
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | No blocking
 *  0x01     | Theta blocking or acceleration in any axis > 1.5g
 *  0x02     | Theta blocking or acceleration slope in any axis >
 *   -       | 0.2g or acceleration in any axis > 1.5g
 *  0x03     | Theta blocking or acceleration slope in any axis >
 *   -       | 0.4g or acceleration in any axis >
 *   -       | 1.5g and value of orient is not stable
 *   -       | for at least 100 ms
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_blocking(
us8 *v_orient_blocking_us8);
/*!
 *	@brief This API write the orient blocking mode
 *	that is used for the generation of the orientation interrupt.
 *	from the register 0x65 bit 2 and 3
 *
 *  @param v_orient_blocking_us8 : The value of orient blocking mode
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | No blocking
 *  0x01     | Theta blocking or acceleration in any axis > 1.5g
 *  0x02     | Theta blocking or acceleration slope in any axis >
 *   -       | 0.2g or acceleration in any axis > 1.5g
 *  0x03     | Theta blocking or acceleration slope in any axis >
 *   -       | 0.4g or acceleration in any axis >
 *   -       | 1.5g and value of orient is not stable
 *   -       | for at least 100 ms
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_blocking(
us8 v_orient_blocking_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT HYSTERESIS CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read Orient interrupt
 *	hysteresis, from the register 0x64 bit 4 to 7
 *
 *
 *
 *  @param v_orient_hyst_us8 : The value of orient hysteresis
 *
 *	@note 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected accel range
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_hyst(
us8 *v_orient_hyst_us8);
/*!
 *	@brief This API write Orient interrupt
 *	hysteresis, from the register 0x64 bit 4 to 7
 *
 *
 *
 *  @param v_orient_hyst_us8 : The value of orient hysteresis
 *
 *	@note 1 LSB corresponds to 62.5 mg,
 *	irrespective of the selected accel range
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_hyst(
us8 v_orient_hyst_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT THETA CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read Orient
 *	blocking angle (0 to 44.8) from the register 0x66 bit 0 to 5
 *
 *  @param v_orient_theta_us8 : The value of Orient blocking angle
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_theta(
us8 *v_orient_theta_us8);
 /*!
 *	@brief This API write Orient
 *	blocking angle (0 to 44.8) from the register 0x66 bit 0 to 5
 *
 *  @param v_orient_theta_us8 : The value of Orient blocking angle
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_theta(
us8 v_orient_theta_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT OUTPUT ENABLE CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read orient change
 *	of up/down bit from the register 0x66 bit 6
 *
 *  @param v_orient_ud_us8 : The value of orient change of up/down
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | Is ignored
 *  0x01     | Generates orientation interrupt
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_ud_enable(
us8 *v_orient_ud_us8);
/*!
 *	@brief This API write orient change
 *	of up/down bit from the register 0x66 bit 6
 *
 *  @param v_orient_ud_us8 : The value of orient change of up/down
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | Is ignored
 *  0x01     | Generates orientation interrupt
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_ud_enable(
us8 v_orient_ud_us8);
/***************************************************************/
/**\name	FUNCTION FOR ORIENT AXIS ENABLE CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read orientation axes changes
 *	from the register 0x66 bit 7
 *
 *  @param v_orient_axes_us8 : The value of orient axes assignment
 *	value    |       Behaviour    | Name
 * ----------|--------------------|------
 *  0x00     | x = x, y = y, z = z|orient_ax_noex
 *  0x01     | x = y, y = z, z = x|orient_ax_ex
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_axes_enable(
us8 *v_orient_axes_us8);
 /*!
 *	@brief This API write orientation axes changes
 *	from the register 0x66 bit 7
 *
 *  @param v_orient_axes_us8 : The value of orient axes assignment
 *	value    |       Behaviour    | Name
 * ----------|--------------------|------
 *  0x00     | x = x, y = y, z = z|orient_ax_noex
 *  0x01     | x = y, y = z, z = x|orient_ax_ex
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_axes_enable(
us8 v_orient_axes_us8);
/***************************************************************/
/**\name	FUNCTION FOR FLAT THETA CONFIGURATION*/
/***************************************************************/
 /*!
 *	@brief This API read Flat angle (0 to 44.8) for flat interrupt
 *	from the register 0x67 bit 0 to 5
 *
 *  @param v_flat_theta_us8 : The value of flat angle
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_theta(
us8 *v_flat_theta_us8);
 /*!
 *	@brief This API write Flat angle (0 to 44.8) for flat interrupt
 *	from the register 0x67 bit 0 to 5
 *
 *  @param v_flat_theta_us8 : The value of flat angle
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_theta(
us8 v_flat_theta_us8);
/***************************************************************/
/**\name	FUNCTION FOR FLAT HOLD CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read Flat interrupt hold time;
 *	from the register 0x68 bit 4 and 5
 *
 *  @param v_flat_hold_us8 : The value of flat hold time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | 0ms
 *  0x01     | 512ms
 *  0x01     | 1024ms
 *  0x01     | 2048ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hold(
us8 *v_flat_hold_us8);
/*!
 *	@brief This API write Flat interrupt hold time;
 *	from the register 0x68 bit 4 and 5
 *
 *  @param v_flat_hold_us8 : The value of flat hold time
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | 0ms
 *  0x01     | 512ms
 *  0x01     | 1024ms
 *  0x01     | 2048ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hold(
us8 v_flat_hold_us8);
/***************************************************************/
/**\name	FUNCTION FOR FLAT HYSTERESIS CONFIGURATION*/
/***************************************************************/
/*!
 *	@brief This API read flat interrupt hysteresis
 *	from the register 0x68 bit 0 to 3
 *
 *  @param v_flat_hyst_us8 : The value of flat hysteresis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hyst(
us8 *v_flat_hyst_us8);
/*!
 *	@brief This API write flat interrupt hysteresis
 *	from the register 0x68 bit 0 to 3
 *
 *  @param v_flat_hyst_us8 : The value of flat hysteresis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hyst(
us8 v_flat_hyst_us8);
/***************************************************************/
/**\name	FUNCTION FAST OFFSET COMPENSATION FOR ACCEL */
/***************************************************************/
 /*!
 *	@brief This API read accel offset compensation
 *	target value for z-axis from the register 0x69 bit 0 and 1
 *
 *  @param v_foc_accel_z_us8 : the value of accel offset compensation z axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_z(
us8 *v_foc_accel_z_us8);
 /*!
 *	@brief This API write accel offset compensation
 *	target value for z-axis from the register 0x69 bit 0 and 1
 *
 *  @param v_foc_accel_z_us8 : the value of accel offset compensation z axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_z(
us8 v_foc_accel_z_us8);
/*!
 *	@brief This API read accel offset compensation
 *	target value for y-axis
 *	from the register 0x69 bit 2 and 3
 *
 *  @param v_foc_accel_y_us8 : the value of accel offset compensation y axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_y(
us8 *v_foc_accel_y_us8);
/*!
 *	@brief This API write accel offset compensation
 *	target value for y-axis
 *	from the register 0x69 bit 2 and 3
 *
 *  @param v_foc_accel_y_us8 : the value of accel offset compensation y axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_y(
us8 v_foc_accel_y_us8);
/*!
 *	@brief This API read accel offset compensation
 *	target value for x-axis is
 *	from the register 0x69 bit 4 and 5
 *
 *  @param v_foc_accel_x_us8 : the value of accel offset compensation x axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_x(
us8 *v_foc_accel_x_us8);
/*!
 *	@brief This API write accel offset compensation
 *	target value for x-axis is
 *	from the register 0x69 bit 4 and 5
 *
 *  @param v_foc_accel_x_us8 : the value of accel offset compensation x axis
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_x(
us8 v_foc_accel_x_us8);
/***************************************************************/
/**\name	FUNCTION FAST OFFSET COMPENSATION FOR GYRO */
/***************************************************************/
/*!
 *	@brief This API write gyro fast offset enable
 *	from the register 0x69 bit 6
 *
 *  @param v_foc_gyro_us8 : The value of gyro fast offset enable
 *  value    |  Description
 * ----------|-------------
 *    0      | fast offset compensation disabled
 *    1      |  fast offset compensation enabled
 *
 *	@param v_gyro_off_x_int16_ts : The value of gyro fast offset x axis data
 *	@param v_gyro_off_y_int16_ts : The value of gyro fast offset y axis data
 *	@param v_gyro_off_z_int16_ts : The value of gyro fast offset z axis data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_gyro_enable(
us8 v_foc_gyro_us8, int16_ts *v_gyro_off_x_int16_ts,
int16_ts *v_gyro_off_y_int16_ts, int16_ts *v_gyro_off_z_int16_ts);
/***************************************************/
/**\name	FUNCTION FOR NVM*/
/***************************************************/
 /*!
 *	@brief This API read NVM program enable
 *	from the register 0x6A bit 1
 *
 *  @param v_nvm_prog_us8 : The value of NVM program enable
 *  Value  |  Description
 * --------|-------------
 *   0     |  DISABLE
 *   1     |  ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_prog_enable(
us8 *v_nvm_prog_us8);
 /*!
 *	@brief This API write NVM program enable
 *	from the register 0x6A bit 1
 *
 *  @param v_nvm_prog_us8 : The value of NVM program enable
 *  Value  |  Description
 * --------|-------------
 *   0     |  DISABLE
 *   1     |  ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_nvm_prog_enable(
us8 v_nvm_prog_us8);
/***************************************************/
/**\name	FUNCTION FOR SPI MODE*/
/***************************************************/
/*!
 * @brief This API read to configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *
 *  @param v_spi3_us8 : The value of SPI mode selection
 *  Value  |  Description
 * --------|-------------
 *   0     |  SPI 4-wire mode
 *   1     |  SPI 3-wire mode
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi3(
us8 *v_spi3_us8);
/*!
 * @brief This API write to configure SPI
 * Interface Mode for primary and OIS interface
 * from the register 0x6B bit 0
 *
 *  @param v_spi3_us8 : The value of SPI mode selection
 *  Value  |  Description
 * --------|-------------
 *   0     |  SPI 4-wire mode
 *   1     |  SPI 3-wire mode
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi3(
us8 v_spi3_us8);
/***************************************************/
/**\name	FUNCTION FOR FOC GYRO */
/***************************************************/
/*!
 *	@brief This API read gyro fast offset enable
 *	from the register 0x69 bit 6
 *
 *  @param v_foc_gyro_us8 : The value of gyro fast offset enable
 *  value    |  Description
 * ----------|-------------
 *    0      | fast offset compensation disabled
 *    1      |  fast offset compensation enabled
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_gyro_enable(
us8 *v_foc_gyro_us8);
/***************************************************/
/**\name	FUNCTION FOR I2C WATCHDOG TIMBER */
/***************************************************/
/*!
 *	@brief This API read I2C Watchdog timer
 *	from the register 0x70 bit 1
 *
 *  @param v_i2c_wdt_us8 : The value of I2C watch dog timer
 *  Value  |  Description
 * --------|-------------
 *   0     |  I2C watchdog v_timeout_us8 after 1 ms
 *   1     |  I2C watchdog v_timeout_us8 after 50 ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_select(
us8 *v_i2c_wdt_us8);
/*!
 *	@brief This API write I2C Watchdog timer
 *	from the register 0x70 bit 1
 *
 *  @param v_i2c_wdt_us8 : The value of I2C watch dog timer
 *  Value  |  Description
 * --------|-------------
 *   0     |  I2C watchdog v_timeout_us8 after 1 ms
 *   1     |  I2C watchdog v_timeout_us8 after 50 ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE
bmi160_set_i2c_wdt_select(us8 v_i2c_wdt_us8);
/*!
 *	@brief This API read I2C watchdog enable
 *	from the register 0x70 bit 2
 *
 *  @param v_i2c_wdt_us8 : The value of I2C watchdog enable
 *  Value  |  Description
 * --------|-------------
 *   0     |  DISABLE
 *   1     |  ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_enable(
us8 *v_i2c_wdt_us8);
/*!
 *	@brief This API write I2C watchdog enable
 *	from the register 0x70 bit 2
 *
 *  @param v_i2c_wdt_us8 : The value of I2C watchdog enable
 *  Value  |  Description
 * --------|-------------
 *   0     |  DISABLE
 *   1     |  ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_wdt_enable(
us8 v_i2c_wdt_us8);
/***************************************************/
/**\name	FUNCTION FOR IF MODE*/
/***************************************************/
/*!
 * @brief This API read I2C interface configuration(if) moe
 * from the register 0x6B bit 4 and 5
 *
 *  @param  v_if_mode_us8 : The value of interface configuration mode
 *  Value  |  Description
 * --------|-------------
 *   0x00  |  Primary interface:autoconfig / secondary interface:off
 *   0x01  |  Primary interface:I2C / secondary interface:OIS
 *   0x02  |  Primary interface:autoconfig/secondary interface:Magnetometer
 *   0x03  |   Reserved
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_if_mode(
us8 *v_if_mode_us8);
/*!
 * @brief This API write I2C interface configuration(if) moe
 * from the register 0x6B bit 4 and 5
 *
 *  @param  v_if_mode_us8 : The value of interface configuration mode
 *  Value  |  Description
 * --------|-------------
 *   0x00  |  Primary interface:autoconfig / secondary interface:off
 *   0x01  |  Primary interface:I2C / secondary interface:OIS
 *   0x02  |  Primary interface:autoconfig/secondary interface:Magnetometer
 *   0x03  |   Reserved
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_if_mode(
us8 v_if_mode_us8);
/***************************************************/
/**\name	FUNCTION FOR GYRO SLEEP TRIGGER INTERRUPT CONFIGURATION*/
/***************************************************/
/*!
 *	@brief This API read gyro sleep trigger
 *	from the register 0x6C bit 0 to 2
 *
 *  @param v_gyro_sleep_trigger_us8 : The value of gyro sleep trigger
 *  Value  |  Description
 * --------|-------------
 *   0x00  | nomotion: no / Not INT1 pin: no / INT2 pin: no
 *   0x01  | nomotion: no / Not INT1 pin: no / INT2 pin: yes
 *   0x02  | nomotion: no / Not INT1 pin: yes / INT2 pin: no
 *   0x03  | nomotion: no / Not INT1 pin: yes / INT2 pin: yes
 *   0x04  | nomotion: yes / Not INT1 pin: no / INT2 pin: no
 *   0x05  | anymotion: yes / Not INT1 pin: no / INT2 pin: yes
 *   0x06  | anymotion: yes / Not INT1 pin: yes / INT2 pin: no
 *   0x07  | anymotion: yes / Not INT1 pin: yes / INT2 pin: yes
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_trigger(
us8 *v_gyro_sleep_trigger_us8);
/*!
 *	@brief This API write gyro sleep trigger
 *	from the register 0x6C bit 0 to 2
 *
 *  @param v_gyro_sleep_trigger_us8 : The value of gyro sleep trigger
 *  Value  |  Description
 * --------|-------------
 *   0x00  | nomotion: no / Not INT1 pin: no / INT2 pin: no
 *   0x01  | nomotion: no / Not INT1 pin: no / INT2 pin: yes
 *   0x02  | nomotion: no / Not INT1 pin: yes / INT2 pin: no
 *   0x03  | nomotion: no / Not INT1 pin: yes / INT2 pin: yes
 *   0x04  | nomotion: yes / Not INT1 pin: no / INT2 pin: no
 *   0x05  | anymotion: yes / Not INT1 pin: no / INT2 pin: yes
 *   0x06  | anymotion: yes / Not INT1 pin: yes / INT2 pin: no
 *   0x07  | anymotion: yes / Not INT1 pin: yes / INT2 pin: yes
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_trigger(
us8 v_gyro_sleep_trigger_us8);
/*!
 *	@brief This API read gyro wakeup trigger
 *	from the register 0x6C bit 3 and 4
 *
 *  @param v_gyro_wakeup_trigger_us8 : The value of gyro wakeup trigger
 *  Value  |  Description
 * --------|-------------
 *   0x00  | anymotion: no / INT1 pin: no
 *   0x01  | anymotion: no / INT1 pin: yes
 *   0x02  | anymotion: yes / INT1 pin: no
 *   0x03  | anymotion: yes / INT1 pin: yes
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_trigger(
us8 *v_gyro_wakeup_trigger_us8);
/*!
 *	@brief This API write gyro wakeup trigger
 *	from the register 0x6C bit 3 and 4
 *
 *  @param v_gyro_wakeup_trigger_us8 : The value of gyro wakeup trigger
 *  Value  |  Description
 * --------|-------------
 *   0x00  | anymotion: no / INT1 pin: no
 *   0x01  | anymotion: no / INT1 pin: yes
 *   0x02  | anymotion: yes / INT1 pin: no
 *   0x03  | anymotion: yes / INT1 pin: yes
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_trigger(
us8 v_gyro_wakeup_trigger_us8);
/*!
 *	@brief This API read Target state for gyro sleep mode
 *	from the register 0x6C bit 5
 *
 *  @param v_gyro_sleep_state_us8 : The value of gyro sleep mode
 *  Value  |  Description
 * --------|-------------
 *   0x00  | Sleep transition to fast wake up state
 *   0x01  | Sleep transition to suspend state
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_state(
us8 *v_gyro_sleep_state_us8);
/*!
 *	@brief This API write Target state for gyro sleep mode
 *	from the register 0x6C bit 5
 *
 *  @param v_gyro_sleep_state_us8 : The value of gyro sleep mode
 *  Value  |  Description
 * --------|-------------
 *   0x00  | Sleep transition to fast wake up state
 *   0x01  | Sleep transition to suspend state
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_state(
us8 v_gyro_sleep_state_us8);
/*!
 *	@brief This API read gyro wakeup interrupt
 *	from the register 0x6C bit 6
 *
 *  @param v_gyro_wakeup_intr_us8 : The valeu of gyro wakeup interrupt
 *  Value  |  Description
 * --------|-------------
 *   0x00  | DISABLE
 *   0x01  | ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_intr(
us8 *v_gyro_wakeup_intr_us8);
/*!
 *	@brief This API write gyro wakeup interrupt
 *	from the register 0x6C bit 6
 *
 *  @param v_gyro_wakeup_intr_us8 : The valeu of gyro wakeup interrupt
 *  Value  |  Description
 * --------|-------------
 *   0x00  | DISABLE
 *   0x01  | ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_intr(
us8 v_gyro_wakeup_intr_us8);
/***************************************************/
/**\name	FUNCTION FOR ACCEL SELF TEST */
/***************************************************/
/*!
 * @brief This API read accel select axis to be self-test
 *
 *  @param v_accel_selftest_axis_us8 :
 *	The value of accel self test axis selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | disabled
 *   0x01  | x-axis
 *   0x02  | y-axis
 *   0x03  | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_axis(
us8 *acc_selftest_axis);
/*!
 * @brief This API write accel select axis to be self-test
 *
 *  @param v_accel_selftest_axis_us8 :
 *	The value of accel self test axis selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | disabled
 *   0x01  | x-axis
 *   0x02  | y-axis
 *   0x03  | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_axis(
us8 acc_selftest_axis);
/*!
 *	@brief This API read accel self test axis sign
 *	from the register 0x6D bit 2
 *
 *  @param v_accel_selftest_sign_us8: The value of accel self test axis sign
 *  Value  |  Description
 * --------|-------------
 *   0x00  | negative
 *   0x01  | positive
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_sign(
us8 *acc_selftest_sign);
/*!
 *	@brief This API write accel self test axis sign
 *	from the register 0x6D bit 2
 *
 *  @param v_accel_selftest_sign_us8: The value of accel self test axis sign
 *  Value  |  Description
 * --------|-------------
 *   0x00  | negative
 *   0x01  | positive
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_sign(
us8 acc_selftest_sign);
/*!
 *	@brief This API read accel self test amplitude
 *	from the register 0x6D bit 3
 *        select amplitude of the selftest deflection:
 *
 *  @param v_accel_selftest_amp_us8 : The value of accel self test amplitude
 *  Value  |  Description
 * --------|-------------
 *   0x00  | LOW
 *   0x01  | HIGH
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_amp(
us8 *acc_selftest_amp);
/*!
 *	@brief This API write accel self test amplitude
 *	from the register 0x6D bit 3
 *        select amplitude of the selftest deflection:
 *
 *  @param v_accel_selftest_amp_us8 : The value of accel self test amplitude
 *  Value  |  Description
 * --------|-------------
 *   0x00  | LOW
 *   0x01  | HIGH
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_amp(
us8 acc_selftest_amp);
/***************************************************/
/**\name	FUNCTION FOR GYRO SELF TEST */
/***************************************************/
/*!
 *	@brief This API read gyro self test trigger
 *
 *	@param v_gyro_selftest_start_us8: The value of gyro self test start
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest_start(
us8 *v_gyro_selftest_start_us8);
/*!
 *	@brief This API write gyro self test trigger
 *
 *	@param v_gyro_selftest_start_us8: The value of gyro self test start
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_selftest_start(
us8 v_gyro_selftest_start_us8);
/***************************************************/
/**\name	FUNCTION FOR SPI/I2C ENABLE */
/***************************************************/
 /*!
 * @brief This API read primary interface selection I2C or SPI
 *	from the register 0x70 bit 0
 *
 *  @param v_spi_enable_us8: The value of Interface selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | I2C Enable
 *   0x01  | I2C DISBALE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi_enable(
us8 *v_spi_enable_us8);
 /*!
 * @brief This API write primary interface selection I2C or SPI
 *	from the register 0x70 bit 0
 *
 *  @param v_spi_enable_us8: The value of Interface selection
 *  Value  |  Description
 * --------|-------------
 *   0x00  | I2C Enable
 *   0x01  | I2C DISBALE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi_enable(
us8 v_spi_enable_us8);
 /*!
 *	@brief This API read the spare zero
 *	form register 0x70 bit 3
 *
 *
 *  @param v_spare0_trim_us8: The value of spare zero
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spare0_trim
(us8 *v_spare0_trim_us8);
 /*!
 *	@brief This API write the spare zero
 *	form register 0x70 bit 3
 *
 *
 *  @param v_spare0_trim_us8: The value of spare zero
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_spare0_trim
(us8 v_spare0_trim_us8);
/***************************************************/
/**\name	FUNCTION FOR NVM COUNTER */
/***************************************************/
 /*!
 *	@brief This API read the NVM counter
 *	form register 0x70 bit 4 to 7
 *
 *
 *  @param v_nvm_counter_us8: The value of NVM counter
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_nvm_counter(
us8 *v_nvm_counter_us8);
 /*!
 *	@brief This API write the NVM counter
 *	form register 0x70 bit 4 to 7
 *
 *
 *  @param v_nvm_counter_us8: The value of NVM counter
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_nvm_counter(
us8 v_nvm_counter_us8);
/***************************************************/
/**\name	FUNCTION FOR ACCEL MANUAL OFFSET COMPENSATION */
/***************************************************/
/*!
 *	@brief This API read accel manual offset compensation of x axis
 *	from the register 0x71 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_x_int8_ts:
 *	The value of accel manual offset compensation of x axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_xaxis(
int8_ts *v_accel_off_x_int8_ts);
/*!
 *	@brief This API write accel manual offset compensation of x axis
 *	from the register 0x71 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_x_int8_ts:
 *	The value of accel manual offset compensation of x axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_xaxis(
int8_ts v_accel_off_x_int8_ts);
/*!
 *	@brief This API read accel manual offset compensation of y axis
 *	from the register 0x72 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_y_int8_ts:
 *	The value of accel manual offset compensation of y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_yaxis(
int8_ts *v_accel_off_y_int8_ts);
/*!
 *	@brief This API write accel manual offset compensation of y axis
 *	from the register 0x72 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_y_int8_ts:
 *	The value of accel manual offset compensation of y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_yaxis(
int8_ts v_accel_off_y_int8_ts);
/*!
 *	@brief This API read accel manual offset compensation of z axis
 *	from the register 0x73 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_z_int8_ts:
 *	The value of accel manual offset compensation of z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_zaxis(
int8_ts *v_accel_off_z_int8_ts);
/*!
 *	@brief This API write accel manual offset compensation of z axis
 *	from the register 0x73 bit 0 to 7
 *
 *
 *
 *  @param v_accel_off_z_int8_ts:
 *	The value of accel manual offset compensation of z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_zaxis(
int8_ts v_accel_off_z_int8_ts);
/***************************************************/
/**\name	FUNCTION FOR GYRO MANUAL OFFSET COMPENSATION */
/***************************************************/
/*!
 *	@brief This API read gyro manual offset compensation of x axis
 *	from the register 0x74 bit 0 to 7 and 0x77 bit 0 and 1
 *
 *
 *
 *  @param v_gyro_off_x_int16_ts:
 *	The value of gyro manual offset compensation of x axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_xaxis(
int16_ts *v_gyro_off_x_int16_ts);
/*!
 *	@brief This API write gyro manual offset compensation of x axis
 *	from the register 0x74 bit 0 to 7 and 0x77 bit 0 and 1
 *
 *
 *
 *  @param v_gyro_off_x_int16_ts:
 *	The value of gyro manual offset compensation of x axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_xaxis(
int16_ts v_gyro_off_x_int16_ts);
/*!
 *	@brief This API read gyro manual offset compensation of y axis
 *	from the register 0x75 bit 0 to 7 and 0x77 bit 2 and 3
 *
 *
 *
 *  @param v_gyro_off_y_int16_ts:
 *	The value of gyro manual offset compensation of y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_yaxis(
int16_ts *v_gyro_off_y_int16_ts);
/*!
 *	@brief This API write gyro manual offset compensation of y axis
 *	from the register 0x75 bit 0 to 7 and 0x77 bit 2 and 3
 *
 *
 *
 *  @param v_gyro_off_y_int16_ts:
 *	The value of gyro manual offset compensation of y axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_yaxis(
int16_ts v_gyro_off_y_int16_ts);
/*!
 *	@brief This API read gyro manual offset compensation of z axis
 *	from the register 0x76 bit 0 to 7 and 0x77 bit 4 and 5
 *
 *
 *
 *  @param v_gyro_off_z_int16_ts:
 *	The value of gyro manual offset compensation of z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_zaxis(
int16_ts *v_gyro_off_z_int16_ts);
/*!
 *	@brief This API write gyro manual offset compensation of z axis
 *	from the register 0x76 bit 0 to 7 and 0x77 bit 4 and 5
 *
 *
 *
 *  @param v_gyro_off_z_int16_ts:
 *	The value of gyro manual offset compensation of z axis
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_zaxis(
int16_ts v_gyro_off_z_int16_ts);
/*!
 *	@brief This API writes accel fast offset compensation
 *	from the register 0x69 bit 0 to 5
 *	@brief This API writes each axis individually
 *	FOC_X_AXIS - bit 4 and 5
 *	FOC_Y_AXIS - bit 2 and 3
 *	FOC_Z_AXIS - bit 0 and 1
 *
 *  @param  v_foc_accel_us8: The value of accel offset compensation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *  @param  v_axis_us8: The value of accel offset axis selection
  *	value    | axis
 * ----------|-------------------
 *  0        | FOC_X_AXIS
 *  1        | FOC_Y_AXIS
 *  2        | FOC_Z_AXIS
 *
 *	@param v_accel_offset_int8_ts: The accel offset value
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_foc_trigger(us8 axis,
us8 foc_acc, int8_ts *accel_offset);
/*!
 *	@brief This API write fast accel offset compensation
 *	it writes all axis together.To the register 0x69 bit 0 to 5
 *	FOC_X_AXIS - bit 4 and 5
 *	FOC_Y_AXIS - bit 2 and 3
 *	FOC_Z_AXIS - bit 0 and 1
 *
 *  @param  v_foc_accel_x_us8: The value of accel offset x compensation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *  @param  v_foc_accel_y_us8: The value of accel offset y compensation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *  @param  v_foc_accel_z_us8: The value of accel offset z compensation
 *	value    | Behaviour
 * ----------|-------------------
 *  0x00     | disable
 *  0x01     | +1g
 *  0x01     | -1g
 *  0x01     | 0g
 *
 *  @param  v_accel_off_x_int8_ts: The value of accel offset x axis
 *  @param  v_accel_off_y_int8_ts: The value of accel offset y axis
 *  @param  v_accel_off_z_int8_ts: The value of accel offset z axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_accel_foc_trigger_xyz(us8 v_foc_accel_x_us8,
us8 v_foc_accel_y_us8, us8 v_foc_accel_z_us8,
int8_ts *acc_off_x, int8_ts *acc_off_y, int8_ts *acc_off_z);
/***************************************************/
/**\name	FUNCTION FOR ACEL AND GYRO OFFSET ENABLE */
/***************************************************/
/*!
 *	@brief This API read the accel offset enable bit
 *	from the register 0x77 bit 6
 *
 *
 *
 *  @param v_accel_off_enable_us8: The value of accel offset enable
 *  value    |  Description
 * ----------|--------------
 *   0x01    | ENABLE
 *   0x00    | DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_enable(
us8 *acc_off_en);
/*!
 *	@brief This API write the accel offset enable bit
 *	from the register 0x77 bit 6
 *
 *
 *
 *  @param v_accel_off_enable_us8: The value of accel offset enable
 *  value    |  Description
 * ----------|--------------
 *   0x01    | ENABLE
 *   0x00    | DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_enable(
us8 acc_off_en);
/*!
 *	@brief This API read the accel offset enable bit
 *	from the register 0x77 bit 7
 *
 *
 *
 *  @param v_gyro_off_enable_us8: The value of gyro offset enable
 *  value    |  Description
 * ----------|--------------
 *   0x01    | ENABLE
 *   0x00    | DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_enable(
us8 *v_gyro_off_enable_us8);
/*!
 *	@brief This API write the accel offset enable bit
 *	from the register 0x77 bit 7
 *
 *
 *
 *  @param v_gyro_off_enable_us8: The value of gyro offset enable
 *  value    |  Description
 * ----------|--------------
 *   0x01    | ENABLE
 *   0x00    | DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_enable(
us8 v_gyro_off_enable_us8);
/***************************************************/
/**\name	FUNCTION FOR STEP COUNTER INTERRUPT */
/***************************************************/
/*!
 *	@brief This API reads step counter value
 *	form the register 0x78 and 0x79
 *
 *
 *
 *
 *  @param v_step_cnt_int16_ts : The value of step counter
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_step_count(int16_ts *v_step_cnt_int16_ts);
 /*!
 *	@brief This API Reads
 *	step counter configuration
 *	from the register 0x7A bit 0 to 7
 *	and from the register 0x7B bit 0 to 2 and 4 to 7
 *
 *
 *  @param v_step_config_us16 : The value of step configuration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_config(
us16 *v_step_config_us16);
 /*!
 *	@brief This API write
 *	step counter configuration
 *	from the register 0x7A bit 0 to 7
 *	and from the register 0x7B bit 0 to 2 and 4 to 7
 *
 *
 *  @param v_step_config_us16   :
 *	the value of  Enable step configuration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_config(
us16 v_step_config_us16);
 /*!
 *	@brief This API read enable step counter
 *	from the register 0x7B bit 3
 *
 *
 *  @param v_step_counter_us8 : The value of step counter enable
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_counter_enable(
us8 *v_step_counter_us8);
 /*!
 *	@brief This API write enable step counter
 *	from the register 0x7B bit 3
 *
 *
 *  @param v_step_counter_us8 : The value of step counter enable
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_counter_enable(
us8 v_step_counter_us8);
 /*!
 *	@brief This API set Step counter modes
 *
 *
 *  @param  v_step_mode_us8 : The value of step counter mode
 *  value    |   mode
 * ----------|-----------
 *   0       | BMI160_STEP_NORMAL_MODE
 *   1       | BMI160_STEP_SENSITIVE_MODE
 *   2       | BMI160_STEP_ROBUST_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_mode(us8 v_step_mode_us8);
/*!
 *	@brief This API used to trigger the  signification motion
 *	interrupt
 *
 *
 *  @param  v_significant_us8 : The value of interrupt selection
 *  value    |  interrupt
 * ----------|-----------
 *   0       |  BMI160_MAP_INTR1
 *   1       |  BMI160_MAP_INTR2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_significant_motion_intr(
us8 v_significant_us8);
/*!
 *	@brief This API used to trigger the step detector
 *	interrupt
 *
 *
 *  @param  v_step_detector_us8 : The value of interrupt selection
 *  value    |  interrupt
 * ----------|-----------
 *   0       |  BMI160_MAP_INTR1
 *   1       |  BMI160_MAP_INTR2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_step_detector_intr(
us8 v_step_detector_us8);
 /*!
 *	@brief This API used to clear the step counter interrupt
 *	interrupt
 *
 *
 *  @param  : None
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_clear_step_counter(void);
/***************************************************/
/**\name	FUNCTION FOR STEP COMMAND REGISTER WRITE */
/***************************************************/
 /*!
 *	@brief This API writes value to the register 0x7E bit 0 to 7
 *
 *
 *  @param  v_command_reg_us8 : The value to write command register
 *  value   |  Description
 * ---------|--------------------------------------------------------
 *	0x00	|	Reserved
 *  0x03	|	Starts fast offset calibration for the accel and gyro
 *	0x10	|	Sets the PMU mode for the Accelerometer to suspend
 *	0x11	|	Sets the PMU mode for the Accelerometer to normal
 *	0x12	|	Sets the PMU mode for the Accelerometer Lowpower
 *  0x14	|	Sets the PMU mode for the Gyroscope to suspend
 *	0x15	|	Sets the PMU mode for the Gyroscope to normal
 *	0x16	|	Reserved
 *	0x17	|	Sets the PMU mode for the Gyroscope to fast start-up
 *  0x18	|	Sets the PMU mode for the Magnetometer to suspend
 *	0x19	|	Sets the PMU mode for the Magnetometer to normal
 *	0x1A	|	Sets the PMU mode for the Magnetometer to Lowpower
 *	0xB0	|	Clears all data in the FIFO
 *  0xB1	|	Resets the interrupt engine
 *	0xB2	|	step_cnt_clr Clears the step counter
 *	0xB6	|	Triggers a reset
 *	0x37	|	See extmode_en_last
 *	0x9A	|	See extmode_en_last
 *	0xC0	|	Enable the extended mode
 *  0xC4	|	Erase NVM cell
 *	0xC8	|	Load NVM cell
 *	0xF0	|	Reset acceleration data path
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_command_register(
us8 v_command_reg_us8);
/***************************************************/
/**\name	FUNCTION FOR PAGE ENABLE */
/***************************************************/
 /*!
 *	@brief This API read target page from the register 0x7F bit 4 and 5
 *
 *  @param v_target_page_us8: The value of target page
 *  value   |  page
 * ---------|-----------
 *   0      |  User data/configure page
 *   1      |  Chip level trim/test page
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_target_page(
us8 *v_target_page_us8);
 /*!
 *	@brief This API write target page from the register 0x7F bit 4 and 5
 *
 *  @param v_target_page_us8: The value of target page
 *  value   |  page
 * ---------|-----------
 *   0      |  User data/configure page
 *   1      |  Chip level trim/test page
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_target_page(
us8 v_target_page_us8);
 /*!
 *	@brief This API read page enable from the register 0x7F bit 7
 *
 *
 *
 *  @param v_page_enable_us8: The value of page enable
 *  value   |  page
 * ---------|-----------
 *   0      |  DISABLE
 *   1      |  ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_paging_enable(
us8 *v_page_enable_us8);
 /*!
 *	@brief This API write page enable from the register 0x7F bit 7
 *
 *
 *
 *  @param v_page_enable_us8: The value of page enable
 *  value   |  page
 * ---------|-----------
 *   0      |  DISABLE
 *   1      |  ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_paging_enable(
us8 v_page_enable_us8);
 /*!
 *	@brief This API read
 *	pull up configuration from the register 0X85 bit 4 an 5
 *
 *
 *
 *  @param v_control_pullup_us8: The value of pull up register
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_get_pullup_configuration(
us8 *v_control_pullup_us8);
 /*!
 *	@brief This API write
 *	pull up configuration from the register 0X85 bit 4 an 5
 *
 *
 *
 *  @param v_control_pullup_us8: The value of pull up register
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_pullup_configuration(
us8 v_control_pullup_us8);
/*!
 *	@brief This function used for read the
 *	YAMAHA YAS537 xy1y2 data
 *
 *	@param v_ouflow_us8: The value of overflow
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */

/***************************************************/
/**\name	FUNCTIONS FOR FIFO DATA READ */
/***************************************************/
/*!
 *	@brief This function used for reading the
 *	fifo data of  header less mode
 *
 *
 *
 *	@note Configure the below functions for FIFO header less mode
 *	@note 1. bmi160_set_fifo_down_gyro
 *	@note 2. bmi160_set_gyro_fifo_filter_data
 *	@note 3. bmi160_set_fifo_down_accel
 *	@note 4. bmi160_set_accel_fifo_filter_dat
 *	@note 5. bmi160_set_fifo_mag_enable
 *	@note 6. bmi160_set_fifo_accel_enable
 *	@note 7. bmi160_set_fifo_gyro_enable
 *	@note For interrupt configuration
 *	@note 1. bmi160_set_intr_fifo_full
 *	@note 2. bmi160_set_intr_fifo_wm
 *	@note 3. bmi160_set_fifo_tag_intr2_enable
 *	@note 4. bmi160_set_fifo_tag_intr1_enable
 *
 *	@note The fifo reads the whole 1024 bytes
 *	and processing the data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_headerless_mode(
us8 v_mag_if_us8);
/*!
 *	@brief This function used for reading the
 *	fifo data of  header less mode for using user defined length
 *
 *
 *	@param v_fifo_user_length_us16: The value of length of fifo read data
 *
 *	@note Configure the below functions for FIFO header less mode
 *	@note 1. bmi160_set_fifo_down_gyro
 *	@note 2. bmi160_set_gyro_fifo_filter_data
 *	@note 3. bmi160_set_fifo_down_accel
 *	@note 4. bmi160_set_accel_fifo_filter_dat
 *	@note 5. bmi160_set_fifo_mag_enable
 *	@note 6. bmi160_set_fifo_accel_enable
 *	@note 7. bmi160_set_fifo_gyro_enable
 *	@note For interrupt configuration
 *	@note 1. bmi160_set_intr_fifo_full
 *	@note 2. bmi160_set_intr_fifo_wm
 *	@note 3. bmi160_set_fifo_tag_intr2_enable
 *	@note 4. bmi160_set_fifo_tag_intr1_enable
 *
 *	@note The fifo reads the whole 1024 bytes
 *	and processing the data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE
bmi160_read_fifo_headerless_mode_user_defined_length(
us16 v_fifo_user_length_us16,
struct bmi160_fifo_data_header_less_t *fifo_data, us8 v_mag_if_mag_us8);
/*!
 *	@brief This function used for reading the
 *	fifo data of  header mode
 *
 *
 *	@note Configure the below functions for FIFO header mode
 *	@note 1. bmi160_set_fifo_down_gyro()
 *	@note 2. bmi160_set_gyro_fifo_filter_data()
 *	@note 3. bmi160_set_fifo_down_accel()
 *	@note 4. bmi160_set_accel_fifo_filter_dat()
 *	@note 5. bmi160_set_fifo_mag_enable()
 *	@note 6. bmi160_set_fifo_accel_enable()
 *	@note 7. bmi160_set_fifo_gyro_enable()
 *	@note 8. bmi160_set_fifo_header_enable()
 *	@note For interrupt configuration
 *	@note 1. bmi160_set_intr_fifo_full()
 *	@note 2. bmi160_set_intr_fifo_wm()
 *	@note 3. bmi160_set_fifo_tag_intr2_enable()
 *	@note 4. bmi160_set_fifo_tag_intr1_enable()
 *
 *	@note The fifo reads the whole 1024 bytes
 *	and processing the data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data(
us8 v_mag_if_us8);
/*!
 *	@brief This function used for reading the
 *	fifo data of  header mode for using user defined length
 *
 *
 *	@note Configure the below functions for FIFO header mode
 *	@note 1. bmi160_set_fifo_down_gyro()
 *	@note 2. bmi160_set_gyro_fifo_filter_data()
 *	@note 3. bmi160_set_fifo_down_accel()
 *	@note 4. bmi160_set_accel_fifo_filter_dat()
 *	@note 5. bmi160_set_fifo_mag_enable()
 *	@note 6. bmi160_set_fifo_accel_enable()
 *	@note 7. bmi160_set_fifo_gyro_enable()
 *	@note 8. bmi160_set_fifo_header_enable()
 *	@note For interrupt configuration
 *	@note 1. bmi160_set_intr_fifo_full()
 *	@note 2. bmi160_set_intr_fifo_wm()
 *	@note 3. bmi160_set_fifo_tag_intr2_enable()
 *	@note 4. bmi160_set_fifo_tag_intr1_enable()
 *
 *	@note The fifo reads the whole 1024 bytes
 *	and processing the data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data_user_defined_length(
us16 v_fifo_user_length_us16, us8 v_mag_if_mag_us8,
struct bmi160_fifo_data_header_t *fifo_header_data);
/*!
 *	@brief This function used for reading
 *	bmi160_t structure
 *
 *  @return the reference and values of bmi160_t
 *
 *
*/
struct bmi160_t *bmi160_get_ptr(void);
/*!
 *	@brief Used for SPI initialization
*/
char spi_routine(void);

/*!
 *	@brief This function used for initialize the sensor
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_init_sensor(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_power_mode_stat(us8
*v_mag_power_mode_stat_u8);

#endif

