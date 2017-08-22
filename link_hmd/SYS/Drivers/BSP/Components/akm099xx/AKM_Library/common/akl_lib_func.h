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
#ifndef AKL_LIB_FUNC_H
#define AKL_LIB_FUNC_H
#include "AKL_APIs.h"

/**
 * @brief Print software version information.
 */
void print_version(
    void
);

/*! This function restore previous parameter from non-volatile
 * area (for example, EEPROM on MCU, a file on filesystem).
 * If previous parameter is not restored correctly (may be it's
 * collapsed), a default parameter is set.
 * When this function succeeds, it is completely ready to start
 * measurement.
 * \return When function succeeds, #AKM_SUCCESS is returned.
 * \param prm A pointer to #AKL_SCL_PRMS struct. */
int16_t load_and_start(
	struct AKL_SCL_PRMS *prm
);

/*! This function save parameter to non-volatile area for next
 * measurement. If this function fails, next measurement will
 * start with a default parameters.
 * \return When function succeeds, #AKM_SUCCESS is returned.
 * \param prm A pointer to #AKL_SCL_PRMS struct. */
int16_t stop_and_save(
	struct AKL_SCL_PRMS *prm
);

/*! Library initialize function.
 * This function allocate new memory area for #AKL_SCL_PRMS.
 * When this function succeeds, #AKL_SCL_PRMS parameter is ready to
 * start measurement. But please note that this function does
 * not restore previously saved data, for example, offset and its
 * accuracy etc.\n
 * When this function fails, \c prm is cleaned up internally,
 * so you don't need to \c 'free' outside of this function.
 * \return When function succeeds, #AKM_SUCCESS is returned.
 * \param prm A pointer of pointer to #AKL_SCL_PRMS struct. */
int16_t library_init(
	struct AKL_SCL_PRMS **prm
);

#endif /* AKL_LIB_FUNC_H */
