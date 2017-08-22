/*
 * fotamode.h
 *
 *  Created on: Jul 13, 2016
 *      Author: 
 */

#ifndef BOOTLOADER_INC_FOTAMODE_H_
#define BOOTLOADER_INC_FOTAMODE_H_
#include "usbd_def.h"

#define BL_HEADER		"bootload"
#define SYS_HEADER		"systrtos"
#define TRA_HEADER		"trackpad"
#define BLE_HEADER		"blutooth"
#define CCGX_FW1_HEADER  "ccgx_fw1"
#define CCGX_FW2_HEADER  "ccgx_fw2"

typedef void dfu_move_img_callback(USBD_HandleTypeDef *pdev,int state);

void  fota_move_image(uint16_t value,dfu_move_img_callback fota_callback,USBD_HandleTypeDef *cb_data); 


#endif /* BOOTLOADER_INC_BOOTMODE_H_ */
