/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTOS_I2C_DRV_H
#define __RTOS_I2C_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "integer.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "hlog_api.h"


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* This example use TIMING to 0x00D00E28 to reach 1 MHz speed (Rise time = 120ns, Fall time = 25ns) */
#define I2C_TIMING_400K    0x00A0298C
#define I2C_TIMING_1M      0x00D00E28

#ifdef I2C_FLAG_TIMEOUT
#undef I2C_FLAG_TIMEOUT
#define I2C_FLAG_TIMEOUT             ((u32)0x1000)
#endif
#define I2C_LONG_TIMEOUT             ((u32)(10 * I2C_FLAG_TIMEOUT))
#define I2C_RETRY_WAIT               (100)
#define I2C_RESET_WAIT               (100)
#define I2C_NO_RETRY                 (1)
#define I2C_SHORT_RETRY              (20)
#define I2C_LONG_RETRY               (I2C_SHORT_RETRY*6)

/* distinguish I2C device with the same devices address */
#define I2C1_FLAG 						(0x10000)
#define I2C2_FLAG 						(0x20000)
/* I2C Device Address */
#define I2C_DEVICE_IOEXP1_ADDR          (I2C1_FLAG | 0x68)
#define I2C_DEVICE_PMIC_ADDR            (I2C1_FLAG | 0x20)
#define I2C_DEVICE_LED_CTRL_ADDR        (I2C1_FLAG | 0x60)
#define I2C_DEVICE_AUDIOCODEC_ADDR      (I2C1_FLAG | 0x36)
#define I2C_DEVICE_PSENSOR_ADDR         (I2C1_FLAG | 0x92)
#define I2C_DEVICE_CHARGER_ADDR         (I2C1_FLAG | 0xD6)
#define I2C_DEVICE_GAUGE_ADDR           (I2C1_FLAG | 0x6C)
#define I2C_DEVICE_CCG4_ADDR            (I2C1_FLAG | 0x10)

#define I2C_DEVICE_IOEXP2_ADDR          (I2C2_FLAG | 0x68)
#define I2C_DEVICE_TC358870_ADDR        (I2C2_FLAG | 0x0F<<1) /*0x1F for pull up or 0x0F for pull down*/
#define I2C_DEVICE_ECOMPASS_ADDR        (I2C2_FLAG | 0x5D)
#define I2C_DEVICE_ANX_RX_ADDR          (I2C2_FLAG | 0x50)
#define I2C_DEVICE_ANX_RX_ADDR1         (I2C2_FLAG | 0x8C)
#define I2C_DEVICE_ANX_TX_1_ADDR        (I2C2_FLAG | 0x7A)
#define I2C_DEVICE_ANX_TX_2_ADDR        (I2C2_FLAG | 0x72)
#define I2C_DEVICE_CCG4_I2C2_ADDR       (I2C2_FLAG | 0x10)

#define I2C_DEVICE_IOEXP1_NAME          ("IOEXP_1")
#define I2C_DEVICE_IOEXP2_NAME          ("IOEXP_2")
#define I2C_DEVICE_PMIC_NAME            ("PMIC")
#define I2C_DEVICE_LED_CTRL_NAME        ("LEDCTRL")
#define I2C_DEVICE_AUDIOCODEC_NAME      ("AUDIOCODEC")
#define I2C_DEVICE_PSENSOR_NAME         ("PSENSOR")
#define I2C_DEVICE_ECOMPASS_NAME        ("ECOMPASS")
#define I2C_DEVICE_CHARGER_NAME         ("CHARGER")
#define I2C_DEVICE_GAUGE_NAME           ("GAUGE")
#define I2C_DEVICE_TC358870_NAME        ("TC358870")
#define I2C_DEVICE_CCG4_NAME            ("CCG4")
#define I2C_DEVICE_CCG4_I2C2_NAME       ("CCG4_I2C2")
#define I2C_DEVICE_ANX_NAME             ("ANX7737")

#define I2C_8BIT                     I2C_MEMADD_SIZE_8BIT
#define I2C_16BIT                    I2C_MEMADD_SIZE_16BIT
#define I2C_POLLING_RW               1
#if I2C_POLLING_RW
	#define I2C_USE_SEMAPHORE        0
#else
	#define I2C_USE_SEMAPHORE        1
#endif
#define I2C_POLLING_RW               1
#define I2C_SHORT_DELAY              (100)
#define I2C_MUTEX_WAIT_MS            portMAX_DELAY
#define HTC_USE_I2C1                 1
#define HTC_USE_I2C2                 1
#define HTC_LOG_I2C                  1

#if HTC_LOG_I2C
	#define log_d(args...)				hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_I2C, ##args)
	#define log_e(args...)				hlog_printf(HLOG_LVL_ERR, HLOG_TAG_I2C, ##args)
#else
	#define log_d(args...)				printf("[I2C]" args)
	#define log_e(args...)				printf("[I2C][ERROR]" args)
#endif

#if 0
#define I2C1_SDA_GPIO_POART          GPIOB
#define I2C1_SCL_GPIO_POART          GPIOB
#define I2C1_SDA_GPIO_PIN            GPIO_PIN_7
#define I2C1_SCL_GPIO_PIN            GPIO_PIN_6
#if HTC_USE_I2C2
#define I2C2_SDA_GPIO_POART          GPIOB
#define I2C2_SCL_GPIO_POART          GPIOB
#define I2C2_SDA_GPIO_PIN            GPIO_PIN_3
#define I2C2_SCL_GPIO_PIN            GPIO_PIN_10
#endif
#endif

/* Exported macro ------------------------------------------------------------*/
typedef enum
{
    I2C_OK = 0 ,
    I2C_ERROR_NONE = I2C_OK,
    I2C_ERROR_BERR,
    I2C_ERROR_ARLO,
    I2C_ERROR_NAK,
    I2C_ERROR_OVR,
    I2C_ERROR_DMA,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_ADDR,
    I2C_ERROR_HAL_BUSY,
    I2C_ERROR_HAL
}I2C_STATUS;

typedef enum
{
    I2C_WRITE_START,
    I2C_WRITE_STOP,
    I2C_READ_START,
    I2C_READ_STOP,
    I2C_ERROR
} I2C_MASTER_STATE;

typedef struct
{
    SemaphoreHandle_t         xSemaphoreMutex;
    xSemaphoreHandle          t_semaphoreI2C;
    I2C_HandleTypeDef         *t_pI2Ch;
    BOOL                      u_isOccupy;
    volatile I2C_MASTER_STATE t_masterState;
}T_HTC_I2C_INFO;

/* Exported functions ------------------------------------------------------- */
//void MX_I2C2_Init(void);
void RTOS_I2C_Init(void);
void RTOS_I2C_PostInit(void);
I2C_STATUS RTOS_I2C_Master_Receive(u32 DevAddr, u8 *pB, u16 Num, u32 Timeout);
I2C_STATUS RTOS_I2C_Master_Transmit(u32 DevAddr, u8 *pB, u16 Num, u32 Timeout);
I2C_STATUS RTOS_I2C_ReadBuffer(u32 DevAddr, u16 Addr, u16 MemAddSize, u8 *pB, u16 Num, u32 Timeout);
I2C_STATUS RTOS_I2C_WriteBuffer(u32 DevAddr, u16 Addr, u16 MemAddSize, u8* pB, u16 Num, u32 Timeout);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
I2C_HandleTypeDef* I2C_GetHandleByAddr(u32 _addr);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ifndef __RTOS_I2C_DRV_H*/
