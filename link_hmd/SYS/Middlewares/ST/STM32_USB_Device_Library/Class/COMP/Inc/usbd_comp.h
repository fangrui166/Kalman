/**
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_cdc.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_COMP_H
#define __USB_COMP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */ 

#define COMP_USE_VENDOR_SPEC_1

/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */
#define  USBD_IMU_DATA_IF_INDEX			0x06
#define  USBD_CDC_CTRL_IF_INDEX			0x07
#define  USBD_CDC_DATA_IF_INDEX			0x08
#define USBD_IMU_DATA_IF_STR			"imu channel"
#define USBD_CDC_CTRL_IF_STR			"cdc channel"
#define USBD_CDC_DATA_IF_STR			"cdc channel"

#define    COMP_HID_INTERFACE                            0x00
#define    COMP_VENDOR_SPEC_1_INTERFACE                  COMP_HID_INTERFACE
#define    COMP_CDC_INTERFACE1                           0x01
#define    COMP_CDC_INTERFACE2                           0x02
#define    COMP_MSC_INTERFACE                            0x03

#ifdef COMP_USE_VENDOR_SPEC_1
#define COMP_VENDOR_SPEC_1_EPIN_ADDR			0x81
#define COMP_VENDOR_SPEC_1_EPOUT_ADDR			0x01
#define COMP_VENDOR_SPEC_1_EP_MAX_SIZE			0x40
#else
#define COMP_VENDOR_SPEC_1_EPIN_ADDR			0xFF
#define COMP_VENDOR_SPEC_1_EPOUT_ADDR			0xFF
#define COMP_VENDOR_SPEC_1_EP_MAX_SIZE			0x00
#endif


#define COMP_CDC_IN_EP                                   0x82  /* EP1 for data IN */
#define COMP_CDC_OUT_EP                                  0x02  /* EP1 for data OUT */
#define COMP_CDC_CMD_EP                                  0x83  /* EP2 for CDC commands */


/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define COMP_CDC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define COMP_CDC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define COMP_CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 

#define USB_COMP_CONFIG_DESC_SIZ                         (75 + 16)
#define COMP_CDC_DATA_HS_IN_PACKET_SIZE                  COMP_CDC_DATA_HS_MAX_PACKET_SIZE
#define COMP_CDC_DATA_HS_OUT_PACKET_SIZE                 COMP_CDC_DATA_HS_MAX_PACKET_SIZE

#define COMP_CDC_DATA_FS_IN_PACKET_SIZE                  COMP_CDC_DATA_FS_MAX_PACKET_SIZE
#define COMP_CDC_DATA_FS_OUT_PACKET_SIZE                 COMP_CDC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define COMP_CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define COMP_CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define COMP_CDC_SET_COMM_FEATURE                        0x02
#define COMP_CDC_GET_COMM_FEATURE                        0x03
#define COMP_CDC_CLEAR_COMM_FEATURE                      0x04
#define COMP_CDC_SET_LINE_CODING                         0x20
#define COMP_CDC_GET_LINE_CODING                         0x21
#define COMP_CDC_SET_CONTROL_LINE_STATE                  0x22
#define COMP_CDC_SEND_BREAK                              0x23

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_COMP_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_COMP_ItfTypeDef;


typedef struct
{
  uint32_t data[COMP_CDC_DATA_HS_MAX_PACKET_SIZE / 4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;

#ifdef COMP_USE_VENDOR_SPEC_1
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint8_t              *VS_RxBuffer;
  uint8_t              *VS_TxBuffer;
  uint32_t             VS_RxLength;
  uint32_t             VS_TxLength;
  uint8_t	       VS_state;
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

uint8_t  USBD_COMP_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_COMP_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_COMP_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_COMP_TransmitPacket     (USBD_HandleTypeDef *pdev);

#ifdef COMP_USE_VENDOR_SPEC_1
uint8_t  USBD_COMP_VS_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
					uint8_t  *pbuff, uint16_t length);
#endif /* COMP_USE_VENDOR_SPEC_1 */


/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_COMP_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
