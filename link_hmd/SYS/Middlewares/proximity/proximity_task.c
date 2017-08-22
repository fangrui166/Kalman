/**
 ******************************************************************************
 * File Name          : proximity_task.c
 * Description        : proximity sensor task to get proximity data
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
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

#include "cmsis_os.h"
#include "proximity_task.h"
#include "x_nucleo_iks01a1_proximity.h"
#include "gpio.h"
#include "htc_3dof_transfer_service.h"
#include "PowerManager_system.h"

static void *PROXIMITY_handle = NULL;

osThreadId proximityTaskHandle;
osMessageQId proximityMsgQueueHandle;
BaseType_t xProximityTaskWoken = pdFALSE;

void startProximityTask(void const * argument);

int proximitySensorEnable(int enable)
{
	if (enable){
		if (BSP_PROXIMITY_Sensor_Enable(PROXIMITY_handle) != COMPONENT_OK){
			prox_err("proximity sensor epl88051 enable fail\n");
			return -1;
		}
	}
	else{
		if (BSP_PROXIMITY_Sensor_Disable(PROXIMITY_handle) != COMPONENT_OK){
			prox_err("proximity sensor epl88051 disable fail\n");
			return -1;
		}
	}
	return 0;
}

void proximityTaskInit(void)
{
	BSP_PROXIMITY_Init(EPL88051, &PROXIMITY_handle);
	proximitySensorEnable(1);
	osThreadDef(proximityTask, startProximityTask, osPriorityNormal, 0, 256);
	proximityTaskHandle = osThreadCreate(osThread(proximityTask), NULL);
	osMessageQDef(proximityMsgQueue, 4, uint16_t);
	proximityMsgQueueHandle = osMessageCreate(osMessageQ(proximityMsgQueue), NULL);
}

void proximitySensorIsr(uint16_t GPIO_Pin)
{
	if(proximityTaskHandle) {
		//xTaskNotifyFromISR(proximityTaskHandle, GPIO_Pin, eSetBits, &xProximityTaskWoken);
		if(proximityMsgQueueHandle != NULL){
			xQueueSendFromISR(proximityMsgQueueHandle,&GPIO_Pin,&xProximityTaskWoken);
			portYIELD_FROM_ISR(xProximityTaskWoken);
		}
	}
}

void startProximityTask(void const * argument)
{
	uint8_t proximityValue;
	//uint32_t ulNotificationValue;
	uint16_t msgQueueBuffer;

	for(;;)
	{
		//xTaskNotifyWait(0,0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY );
		xQueueReceive( proximityMsgQueueHandle, &msgQueueBuffer, portMAX_DELAY );
		xProximityTaskWoken = pdFALSE;
		if((msgQueueBuffer & GPIO_PS_INT) == GPIO_PS_INT){
			BSP_PROXIMITY_Get_Proximity(PROXIMITY_handle, &proximityValue);
			prox_info("proximity value is %d\n",proximityValue);
			htc_3dof_update_psensor_key(proximityValue);
			pwrmgr_system_send_message(PM_SYS_CMD_PSENSOR_DET, proximityValue);
		}
	}
}

