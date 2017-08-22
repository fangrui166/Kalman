/**
  ******************************************************************************
  * @file           : USB_DEVICE
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_comp.h"
#include "usbd_comp_if.h"
#include "PowerManager_power.h"
#include "PowerManager_notify_func.h"

struct usbd_data_str {
	uint8_t is_initialized;
	USBD_HandleTypeDef *pdev;
	USBD_DescriptorsTypeDef *pdesc;
	uint8_t id;
	USBD_ClassTypeDef *pclass;
	USBD_COMP_ItfTypeDef *fops;
	struct pwrmgr_notify_func_data pm_notify_data;
	enum usbd_drv_state state;
};

static struct usbd_data_str g_usbd_data;

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_START(void)
{
  USBD_Start(&hUsbDeviceFS);
}

void MX_USB_DEVICE_STOP(void)
{
  USBD_Stop(&hUsbDeviceFS);
}

int set_usb_device_control(enum usbd_drv_state state)
{
	int ret = 0;
	struct usbd_data_str *usbd_data = &g_usbd_data;

	if (usbd_data->state == state) {
		usbd_dev_info("%s: service state is same\r\n", __func__);
		return ret;
	}
	if (state == USBD_DRV_STOP) {
		usbd_data->state = USBD_DRV_STOPING;
		HAL_Delay(20);
		MX_USB_DEVICE_STOP();
		usbd_data->state = state;
	}
	else if (state == USBD_DRV_START) {
		usbd_data->state = USBD_DRV_STARTING;
		MX_USB_DEVICE_START();
		HAL_Delay(20);
		usbd_data->state = state;
	}
	usbd_dev_debug("%s: usb device driver is %s\r\n",
		__func__, (state == USBD_DRV_STOP) ? " STOP" :
		(state == USBD_DRV_START) ? "START" : "UNKNOWN");
	return ret;
}

enum usbd_drv_state get_usb_device_control(void)
{
	struct usbd_data_str *usbd_data = &g_usbd_data;

	return usbd_data->state;
}

int usb_device_pm_notify_func(uint32_t flag, uint32_t state, void *data)
{
	int ret = 0;
	//struct usbd_data_str *usbd_data = (struct usbd_data_str *)data;

	if(flag == PWRMGR_NOTIFY_STOP_STATE ||
					flag == PWRMGR_NOTIFY_USB_DP_HOLE) {
		if (state == STOP_ENTER || state == PLUG_OUT)
			ret = set_usb_device_control(USBD_DRV_STOP);
		else
		if (state == STOP_LEAVE || state == PLUG_IN)
			ret = set_usb_device_control(USBD_DRV_START);
	} else {
		usbd_dev_warning("%s: unsupported flag\r\n", __func__);
	}
	return ret;
}

static int usb_device_pm_notify_create(
				struct usbd_data_str *usbd_data)
{
	struct pwrmgr_notify_func_data *pm_not_p = &usbd_data->pm_notify_data;

	pm_not_p->func_name = "usbd_drv";
	pm_not_p->data = usbd_data;
	pm_not_p->callback = usb_device_pm_notify_func;
	pm_not_p->notify_flag = PWRMGR_NOTIFY_STOP_STATE |
						PWRMGR_NOTIFY_USB_DP_HOLE;
	pm_not_p->func_level = PWRMGR_FUNC_MAX_LEVEL;
	return PWRMGR_register_notify_func(pm_not_p);
}


/* init function */
void MX_USB_DEVICE_Init(void)
{
  struct usbd_data_str *usbd_data = &g_usbd_data;
  if (usbd_data->is_initialized) {
	usbd_dev_info("%s: initialized, exit\r\n", __func__);
	return;
  }
  memset(&g_usbd_data, 0x0, sizeof(g_usbd_data));

  usbd_data->pdev = &hUsbDeviceFS;
  usbd_data->pdesc = &FS_Desc;
  usbd_data->id = DEVICE_FS;
  usbd_data->pclass = &USBD_COMP;
  usbd_data->fops = &USBD_Interface_fops_FS;

  if (usb_device_pm_notify_create(usbd_data) != 0)
	  usbd_dev_err("%s: create pm notifier failed\n", __func__);


  /* Init Device Library,Add Supported Class and Start the library */
  USBD_Init(usbd_data->pdev, usbd_data->pdesc, usbd_data->id);

  USBD_RegisterClass(usbd_data->pdev, usbd_data->pclass);

  USBD_COMP_RegisterInterface(usbd_data->pdev, usbd_data->fops);

  set_usb_device_control(USBD_DRV_START);

  usbd_data->is_initialized = 1;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
