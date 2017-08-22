/**
  ******************************************************************************
  * @file    usbd_comp.h
  * @author  TOMAS
  * @version V2.4.0t
  * @date    6-June-2015
  * @brief   Header for usbd_comp file.
  ******************************************************************************
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
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
	**/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_COMP_H
#define __USB_COMP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include  "usbd_hid.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */ 


/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */ 
     
/*Define devices used in application*/
#define COMP_USE_CDC 1
#define COMP_USE_HID 1
#define COMP_USE_MSC 0

#if (COMP_USE_CDC+COMP_USE_HID+COMP_USE_MSC)!=2
#error There is need to define exactly two USB classes.
#endif

#define    COMP_HID_INTERFACE                            0x00
#define    COMP_CDC_INTERFACE1                           0x01
#define    COMP_CDC_INTERFACE2                           0x02
#define    COMP_MSC_INTERFACE                            0x03

#if COMP_USE_CDC
#define COMP_CDC_EPIN_ADDR                               0x82  /* EP1 for data IN */
#define COMP_CDC_EPOUT_ADDR                              0x02  /* EP1 for data OUT */
#define COMP_CDC_EPCMD_ADDR                              0x83  /* EP2 for CDC commands */

#define COMP_CDC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define COMP_CDC_DATA_FS_MAX_PACKET_SIZE                 64   /* Endpoint IN & OUT Packet size */
#define COMP_CDC_CMD_PACKET_SIZE                         8    /* Control Endpoint Packet size */ 

#define COMP_CDC_DATA_HS_IN_PACKET_SIZE                  COMP_CDC_DATA_HS_MAX_PACKET_SIZE
#define COMP_CDC_DATA_HS_OUT_PACKET_SIZE                 COMP_CDC_DATA_HS_MAX_PACKET_SIZE
#define COMP_CDC_DATA_FS_IN_PACKET_SIZE                  COMP_CDC_DATA_FS_MAX_PACKET_SIZE
#define COMP_CDC_DATA_FS_OUT_PACKET_SIZE                 COMP_CDC_DATA_FS_MAX_PACKET_SIZE

#define COMP_CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define COMP_CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define COMP_CDC_SET_COMM_FEATURE                        0x02
#define COMP_CDC_GET_COMM_FEATURE                        0x03
#define COMP_CDC_CLEAR_COMM_FEATURE                      0x04
#define COMP_CDC_SET_LINE_CODING                         0x20
#define COMP_CDC_GET_LINE_CODING                         0x21
#define COMP_CDC_SET_CONTROL_LINE_STATE                  0x22
#define COMP_CDC_SEND_BREAK                              0x23

#else
#define COMP_CDC_EPIN_ADDR                               0xff  /* EP1 for data IN */
#define COMP_CDC_EPOUT_ADDR                              0xff  /* EP1 for data OUT */
#define COMP_CDC_EPCMD_ADDR                              0xff  /* EP2 for CDC commands */
#endif

#if COMP_USE_HID
#define COMP_HID_EPIN_ADDR                               0x81
#else
#define COMP_HID_EPIN_ADDR                               0xff
#endif

#if COMP_USE_MSC
#define COMP_MSC_EPIN_ADDR                               0x84 
#define COMP_MSC_EPOUT_ADDR                              0x04
#else
#define COMP_MSC_EPIN_ADDR                               0xff 
#define COMP_MSC_EPOUT_ADDR                              0xff
#endif

#define COMP_HID_EPIN_SIZE                               0x04

/******************************** USER String *************************************/
//Define User Prop  Index
enum {
  /********************* General Property  Begin*******************/
   COMP_USR_PROP_TRACKING_SYSTEM_NAME_STRING =0x0306,
   COMP_USR_PROP_MODEL_NUMBER_STRING,
   COMP_USR_PROP_SERIAL_NUMBER_STRING,
   COMP_USR_PROP_MANUFACTURER_NAME_STRING,
   COMP_USR_PROP_HARDWARE_REVISION_STRING,
   COMP_USR_PROP_DEVICE_IS_WIRELESS_BOOL,
   COMP_USR_PROP_DEVICE_IS_CHARGING_BOOL,
   COMP_USR_PROP_DEVICE_BATTERY_PERCENTAGE_FLOAT,
   COMP_USR_PROP_PROVIDES_BATTERY_STATUS_BOOL,
   COMP_USR_PROP_DEVICE_CAN_POWEROFF_BOOL,
   COMP_USR_PROP_HARDWARE_REVISION_UINT64,
   COMP_USR_PROP_HAS_CAMERA_BOOL,
   COMP_USR_PROP_TRACKING_FIRMWARE_VERSION_STRING,
   COMP_USR_PROP_FIRMWARE_UPDATE_AVAILABLE_BOOL,
   COMP_USR_PROP_FIRMWARE_VERSION_UINT64,
 /*************** Tracking Device Class - HMD  **************/
   COMP_USR_PROP_LENS_CENTER_LEFT_U_FLOAT,
   COMP_USR_PROP_LENS_CENTER_LEFT_V_FLOAT,
   COMP_USR_PROP_LENS_CENTER_RIGHT_U_FLOAT,
   COMP_USR_PROP_LENS_CENTER_RIGHT_V_FLOAT,
   COMP_USR_PROP_USER_HEAD_TO_EYE_DEPTH_METERS_FLOAT,
 /*************** Tracking Device Class - Controller  **************/   
   COMP_USR_PROP_ATTACHED_DEVICE_ID_STRING,
   COMP_USR_PROP_SUPPORTED_BUTTONS_UINT64,
   COMP_USR_PROP_AXIS0_TYPE_INT32,
   COMP_USR_PROP_AXIS1_TYPE_INT32,
   COMP_USR_PROP_AXIS2_TYPE_INT32,
   COMP_USR_PROP_AXIS3_TYPE_INT32,
   COMP_USR_PROP_AXIS4_TYPE_INT32,
};
/*****************************************************************/

typedef struct _USBD_COMP_Itf
{
#if COMP_USE_CDC
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  
#endif    
#if COMP_USE_MSC
    int8_t (* Init_msc) (uint8_t lun);
  int8_t (* GetCapacity) (uint8_t lun, uint32_t *block_num, uint16_t *block_size);
  int8_t (* IsReady) (uint8_t lun);
  int8_t (* IsWriteProtected) (uint8_t lun);
  int8_t (* Read) (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;
#endif

}USBD_COMP_ItfTypeDef;



typedef struct
{
#if COMP_USE_CDC
  uint32_t data[COMP_CDC_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState; 
#endif
#if COMP_USE_HID    
  uint32_t             Protocol;   
  uint32_t             IdleState;  
  uint32_t             AltSetting;
  HID_StateTypeDef     state;     
#endif
#if COMP_USE_MSC        
    uint32_t                 max_lun;   
  uint32_t                 interface; 
  uint8_t                  bot_state;
  uint8_t                  bot_status;  
  uint16_t                 bot_data_length;
  uint8_t                  bot_data[MSC_MEDIA_PACKET];  
  USBD_MSC_BOT_CBWTypeDef  cbw;
  USBD_MSC_BOT_CSWTypeDef  csw;
  
  USBD_SCSI_SenseTypeDef   scsi_sense [SENSE_LIST_DEEPTH];
  uint8_t                  scsi_sense_head;
  uint8_t                  scsi_sense_tail;
  
  uint16_t                 scsi_blk_size;
  uint32_t                 scsi_blk_nbr;
  
  uint32_t                 scsi_blk_addr;
  uint32_t                 scsi_blk_len;
#endif
}
USBD_COMP_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

#define USB_COMP_DEF_CONFIG_DESC_INC                66
#define USB_HID_DEF_CONFIG_DESC_INC                  25
#define USB_MSC_DEF_CONFIG_DESC_INC                  23
#define USB_COMP_CONFIG_DESC_SIZ                     9+USB_HID_DEF_CONFIG_DESC_INC*COMP_USE_HID+COMP_USE_CDC*USB_COMP_DEF_CONFIG_DESC_INC+COMP_USE_MSC*USB_MSC_DEF_CONFIG_DESC_INC
#define BNUMINTERFACES                               COMP_USE_CDC*2+COMP_USE_HID+COMP_USE_MSC

extern USBD_ClassTypeDef  USBD_COMP;
#define USBD_COMP_CLASS    &USBD_COMP
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
    uint8_t  USBD_COMP_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_COMP_ItfTypeDef *fops);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_COMP_CDC_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
