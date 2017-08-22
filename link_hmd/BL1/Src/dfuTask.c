#include "dfuTask.h"
#include <stdio.h>
#include "usbd_def.h"
extern void MX_USB_DEVICE_BL_Init(void);


extern void DFU_move_img(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern USBD_HandleTypeDef hUsbDeviceFS;
uint32_t g_FOTAMoveFlag=0;
TASKCFUNC(DFU)
{

    if (!TaskInitFlag){
		printf("%s start\r\n",__func__);
		MX_USB_DEVICE_BL_Init();
		TaskInitFlag = 1;
    }
	if(g_FOTAMoveFlag == 1){
		printf("start move fota image\r\n");
		HAL_Delay(1000);
		DFU_move_img(&hUsbDeviceFS, NULL);
		g_FOTAMoveFlag = 0;
	}
}
