/**
 *******************************************************************************
 * @file    EPL88051_PROXIMITY_driver_HL.c
 * @author  hTC BSP Team
 * @version V1.0.0
 * @date    11-November-2016
 * @brief   This file provides a set of high-level functions needed to manage
 the ePL88051 proximity sensor
 *******************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include <string.h>
#include "EPL88051_PROXIMITY_driver_HL.h"

static DrvStatusTypeDef EPL88051_Init( DrvContextTypeDef *handle);
static DrvStatusTypeDef EPL88051_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef EPL88051_Enable( DrvContextTypeDef *handle);
static DrvStatusTypeDef EPL88051_Disable( DrvContextTypeDef *handle);
static DrvStatusTypeDef EPL88051_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef EPL88051_Check_WhoAmI( DrvContextTypeDef *handle);
static DrvStatusTypeDef EPL88051_Get_Proximity( DrvContextTypeDef *handle, uint8_t *proximity );
/**
 * @brief EPL88051 driver function  structure
 */
PROXIMITY_Drv_t EPL88051Drv =
{
	EPL88051_Init,
	EPL88051_DeInit,
	EPL88051_Enable,
	EPL88051_Disable,
	EPL88051_Get_WhoAmI,
	EPL88051_Check_WhoAmI,
	EPL88051_Get_Proximity
};

/**
 * @brief Initialize the EPL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Init( DrvContextTypeDef *handle )
{
	if (EPL88051_Check_WhoAmI(handle) == COMPONENT_ERROR){
		prox_err("%s EPL88051_Check_WhoAmI error!\n",__func__);
		return COMPONENT_ERROR;
	}
	if (epl88051_ps_init() < 0){
		prox_err("%s proximity sensor initial fail!\n",__func__);
		return COMPONENT_ERROR;
	}
	handle->isInitialized = 1;
	prox_info("proximity sensor initial success!\n");
	return COMPONENT_OK;
}


/**
 * @brief Deinitialize the ePL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_DeInit( DrvContextTypeDef *handle )
{
	if (EPL88051_Check_WhoAmI(handle) == COMPONENT_ERROR){
		prox_err("%s EPL88051_Check_WhoAmI error!\n",__func__);
		return COMPONENT_ERROR;
	}
	if (epl88051_ps_deinit() < 0){
		prox_err("%s proximity sensor deinitial fail!\n",__func__);
		return COMPONENT_ERROR;
	}
	handle->isInitialized = 0;
	return COMPONENT_OK;
}



/**
 * @brief Enable the ePL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Enable( DrvContextTypeDef *handle )
{
	if ( handle->isEnabled == 1 ){
		return COMPONENT_OK;
	}
	if (epl88051_ps_enable(1) < 0){
		prox_err("%s proximity sensor enable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	handle->isEnabled = 1;
	prox_info("proximity sensor enable success!\n");
	return COMPONENT_OK;
}



/**
 * @brief Disable the ePL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Disable( DrvContextTypeDef *handle )
{
	if ( handle->isEnabled == 0 ){
		return COMPONENT_OK;
	}
	if (epl88051_ps_enable(0) < 0){
		prox_err("%s proximity sensor disable fail\n",__func__);
		return COMPONENT_ERROR;
	}
	handle->isEnabled = 0;
	return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the ePL88051 proximity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{
	if (epl88051_ps_get_revno(who_am_i) < 0){
		prox_err("Read proximity sensor revision number error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the ePL88051 proximity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Check_WhoAmI( DrvContextTypeDef *handle )
{
	uint8_t who_am_i = 0x00;
	if (EPL88051_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR){
		return COMPONENT_ERROR;
	}
	if (who_am_i != handle->who_am_i){
		prox_err("proximity sensor ePL88051 revision number check error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

/**
 * @brief Get the proximity value of ePL88051 proximity sensor
 * @param handle the device handle
 * @param proximity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef EPL88051_Get_Proximity( DrvContextTypeDef *handle, uint8_t *proximity )
{
	if (handle->isEnabled  == 0){
		prox_err("proximity sensor should enabled before get proximity value\n");
		return COMPONENT_ERROR;
	}
	if (epl88051_ps_proximity(proximity) < 0){
		prox_err("Get proximity sensor proximity value error!\n");
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}
