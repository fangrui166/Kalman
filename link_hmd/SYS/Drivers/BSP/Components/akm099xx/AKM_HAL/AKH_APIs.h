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
#ifndef INCLUDE_AKH_APIS_H
#define INCLUDE_AKH_APIS_H
#include "stm32f4xx_hal.h"
#include "AKM_Common.h"
#include "magnetometer.h"


/******************************************************************************
 * API decleration.
 ******************************************************************************/
/*!
 * Initialize system. This function must be called before any other function
 * that is provided by the driver is called.
 * \return No return value is reserved.
 */
void AKH_Init(
    void
);


/*!
 * Write data to a device via serial interface. When more than one byte of
 * data is specified, the data is written in contiguous locations starting
 * at an address specified in address.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval negative-value When operation failed.
 * \param stype Specify sensor type.
 * \param address Register address to write.
 * \param data A pointer to a data buffer to be written.
 * \param numberOfBytesToWrite The number of byte to be written.
 */
int16_t AKH_TxData(
    const AKM_SENSOR_TYPE stype,
    const uint8_t         address,
    const uint8_t         *data,
    const uint16_t        numberOfBytesToWrite
);


/*!
 * Acquire data from the device via serial interface.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval negative-value When operation failed.
 * \param stype Specify sensor type.
 * \param address Register address to read.
 * \param data a Pointer to the data to be read.
 * \param numberOfBytesToRead The number of byte to be read.
 */
int16_t AKH_RxData(
    const AKM_SENSOR_TYPE stype,
    const uint8_t         address,
    uint8_t               *data,
    const uint16_t        numberOfBytesToRead
);

/*!
 * Reset device. This function resets the devicy physically. Sometimes it is
 * called as 'hard reset'. If the device does not support hard rest operation
 * or RESET signal is not connected to GPIO pins, this function returns
 * #AKM_ERR_NOT_SUPPORT.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval negative-value When operation failed.
 * \param stype Specify sensor type.
 */
int16_t AKH_Reset(
    const AKM_SENSOR_TYPE stype
);

/*!
 * Suspend execution.
 * \return No return value is reserved.
 * \param us Duration time in micro-seconds unit.
 */
void AKH_DelayMicro(
    const uint16_t us
);

/*!
 * Suspend execution.
 * \return No return value is reserved.
 * \param ms Duration time in milli-seconds unit.
 */
void AKH_DelayMilli(
    const uint16_t ms
);

/*!
 * Save data to non-volatile storage.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval #AKM_ERR_NOT_SUPPORT When this platform does not support non-volatile
 * storage.
 * \param param A pointer to a data buffer to be saved.
 * \param nBytes The lenght of \c param.
 */
int16_t AKH_SaveParameter(
    const uint8_t  *param,
    const uint16_t nBytes
);

/*!
 * Load data from non-volatile storage.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval #AKM_ERR_NOT_SUPPORT When this platform does not support non-volatile
 * storage.
 * \param param A pointer to a data buffer which loaded data is stored.
 * \param nBytes The lenght of \c param.
 */
int16_t AKH_LoadParameter(
    uint8_t        *param,
    const uint16_t nBytes
);

#if 0
/*!
 * Print function.
 * \return No return value is reserved yet.
 * \param format A pointer to a format string.
 */
void AKH_Print(
    const char *format,
    ...
);
#else
#define AKH_Print(fmt, ...) \
	mag_info("[AKM] " fmt, ##__VA_ARGS__)
#endif

/*!
 * Check whether user input (or any staus changed signal) is occured.
 * This function does not block current process (or thread).
 * It means this function return the result immediately.
 * \retval AKM_SUCCESS When user input is detected.
 * \retval AKM_ERR_TIMEOUT No user input is detected.
 * \retval negative-value Other error happend.
 * \param code This variable is used to determine what type of input
 * was happend. For example, this function detects keyboard input,
 * this function may return the pressed key code.
 * \c NULL is acceptable for this parameter, but you cannot get the event
 * code by any other method.
 */
int16_t AKH_CheckUserInput(
    uint32_t *code
);

/*!
 * Disable magnetic sensor function in BMI160. This function
 * must be called in magnetic sensor suspend flow.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval negative-value When operation failed.
 */
int16_t AKH_Disable(
    void
);

/*!
 * enable magnetic sensor function in BMI160. This function
 * must be called in magnetic sensor resume flow.
 * \retval #AKM_SUCCESS When operation is succeeded.
 * \retval negative-value When operation failed.
 */
int16_t AKH_Enable(
    void
);

/*
 * TODO: to be implemented function list
 *
 * int16_t AKH_SetTimer()
 * int16_t AKH_SetCallback()
 */
#endif /* INCLUDE_AKH_APIS_H */
