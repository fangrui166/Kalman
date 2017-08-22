/*
 * HTC Corporation Proprietary Rights Acknowledgment
 *
 * Copyright (C) 2015 HTC Corporation
 *
 * All Rights Reserved.
 *
 * The information contained in this work is the exclusive property of HTC Corporation
 * ("HTC").  Only the user who is legally authorized by HTC ("Authorized User") has
 * right to employ this work within the scope of this statement.  Nevertheless, the
 * Authorized User shall not use this work for any purpose other than the purpose
 * agreed by HTC.  Any and all addition or modification to this work shall be
 * unconditionally granted back to HTC and such addition or modification shall be
 * solely owned by HTC.  No right is granted under this statement, including but not
 * limited to, distribution, reproduction, and transmission, except as otherwise
 * provided in this statement.  Any other usage of this work shall be subject to the
 * further written consent of HTC.
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c_drv.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

//-----------------------------------------------------------------------------

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static T_HTC_I2C_INFO t_infoI2C[2] ;

static u32  I2C_FailCounter[2] = {0} ;

I2C_HandleTypeDef hi2c1;
#if HTC_USE_I2C2
I2C_HandleTypeDef hi2c2;
#endif

/* Private function prototypes -----------------------------------------------*/
void searchDeviceName(u32 DevAddr, char *buffer);


/* Private functions ---------------------------------------------------------*/
I2C_HandleTypeDef* I2C_GetHandleByAddr(u32 _addr)
{
    I2C_HandleTypeDef* handle = 0;

    switch (_addr)
    {
        /* device on I2C1 */
        case I2C_DEVICE_IOEXP1_ADDR:
        case I2C_DEVICE_PMIC_ADDR:
        case I2C_DEVICE_AUDIOCODEC_ADDR:
        case I2C_DEVICE_LED_CTRL_ADDR:
        case I2C_DEVICE_PSENSOR_ADDR:
        case I2C_DEVICE_CHARGER_ADDR:
        case I2C_DEVICE_CCG4_ADDR:
        case I2C_DEVICE_GAUGE_ADDR: handle = &hi2c1; break;
        /* device on I2C2 */
        case I2C_DEVICE_IOEXP2_ADDR:
        case I2C_DEVICE_ANX_RX_ADDR:
        case I2C_DEVICE_ANX_RX_ADDR1:
        case I2C_DEVICE_ANX_TX_1_ADDR:
        case I2C_DEVICE_ANX_TX_2_ADDR:
        case I2C_DEVICE_TC358870_ADDR:
        case I2C_DEVICE_CCG4_I2C2_ADDR:
        case I2C_DEVICE_ECOMPASS_ADDR: handle = &hi2c2; break;
        default: handle = NULL; break;
    }

    return handle;
}

void searchDeviceName(u32 DevAddr, char *buffer)
{
    switch(DevAddr)
    {
        case I2C_DEVICE_ANX_RX_ADDR:
        case I2C_DEVICE_ANX_RX_ADDR1:
            strncpy(buffer, I2C_DEVICE_ANX_NAME, 10); break;
        case I2C_DEVICE_ANX_TX_1_ADDR:
            strncpy(buffer, I2C_DEVICE_ANX_NAME, 10); break;
        case I2C_DEVICE_ANX_TX_2_ADDR:
            strncpy(buffer, I2C_DEVICE_ANX_NAME, 10); break;
        case I2C_DEVICE_IOEXP1_ADDR:
            strncpy(buffer, I2C_DEVICE_IOEXP1_NAME, 10); break;
        case I2C_DEVICE_IOEXP2_ADDR:
            strncpy(buffer, I2C_DEVICE_IOEXP2_NAME, 10); break;
        case I2C_DEVICE_PMIC_ADDR:
            strncpy(buffer, I2C_DEVICE_PMIC_NAME, 10); break;
        case I2C_DEVICE_AUDIOCODEC_ADDR:
            strncpy(buffer, I2C_DEVICE_AUDIOCODEC_NAME, 10); break;
        case I2C_DEVICE_LED_CTRL_ADDR:
            strncpy(buffer, I2C_DEVICE_LED_CTRL_NAME, 10); break;
        case I2C_DEVICE_PSENSOR_ADDR:
            strncpy(buffer, I2C_DEVICE_PSENSOR_NAME, 10); break;
        case I2C_DEVICE_ECOMPASS_ADDR:
            strncpy(buffer, I2C_DEVICE_ECOMPASS_NAME, 10); break;
        case I2C_DEVICE_CHARGER_ADDR:
            strncpy(buffer, I2C_DEVICE_CHARGER_NAME, 10); break;
        case I2C_DEVICE_GAUGE_ADDR:
            strncpy(buffer, I2C_DEVICE_GAUGE_NAME, 10); break;
        case I2C_DEVICE_TC358870_ADDR:
            strncpy(buffer, I2C_DEVICE_TC358870_NAME, 10);
            break;
        case I2C_DEVICE_CCG4_ADDR:
            strncpy(buffer, I2C_DEVICE_CCG4_NAME, strlen(I2C_DEVICE_CCG4_NAME));
            break;
        case I2C_DEVICE_CCG4_I2C2_ADDR:
            strncpy(buffer, I2C_DEVICE_CCG4_I2C2_NAME, strlen(I2C_DEVICE_CCG4_I2C2_NAME));
            break;
        default:
            snprintf(buffer, 10, "0x%X", DevAddr);
            break;
    }
}

/* transmit interrupt r/w hal error to I2C_STATUS type */
I2C_STATUS trans_interrupt_hal_to_i2c(u32 e)
{
    switch(e)
    {
        case HAL_I2C_ERROR_NONE :
            return I2C_ERROR_NONE ;
        case HAL_I2C_ERROR_BERR :
            return I2C_ERROR_BERR ;
        case HAL_I2C_ERROR_ARLO :
            return I2C_ERROR_ARLO ;
        case HAL_I2C_ERROR_AF :
            return I2C_ERROR_NAK ;
        case HAL_I2C_ERROR_OVR :
            return I2C_ERROR_OVR ;
        case HAL_I2C_ERROR_DMA :
            return I2C_ERROR_DMA ;
        case HAL_I2C_ERROR_TIMEOUT :
            return I2C_ERROR_TIMEOUT ;
        default:
            return I2C_ERROR_HAL ;
    }
}

/* transmit polling r/w hal error to I2C_STATUS type  */
I2C_STATUS trans_polling_hal_to_i2c(u32 e)
{
	switch(e)
	{
		case HAL_OK:
			return I2C_ERROR_NONE;
		case HAL_BUSY:
			return I2C_ERROR_HAL;
		case HAL_TIMEOUT:
			return I2C_ERROR_TIMEOUT;
		case HAL_ERROR:
			return I2C_ERROR_HAL;
        default:
            return I2C_ERROR_HAL;
	}
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    while(1);
  }

}

#if HTC_USE_I2C2
/* I2C2 init function */
void MX_I2C2_Init(void)
{
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c2);
}
#endif

I2C_STATUS HTC_i2c_polling_Read(T_HTC_I2C_INFO *t_infoI2C, u16 DevAddr, u16 MemAddr, u16 MemAddSize, u8 *pB, u16 Num, u32 Timeout)
{
    HAL_StatusTypeDef status = HAL_OK;

    //Hold on bus flag.
    t_infoI2C->u_isOccupy = true ;
    t_infoI2C->t_masterState = I2C_READ_START ;
    //Trigger I2C HW
    status = HAL_I2C_Mem_Read(t_infoI2C->t_pI2Ch, DevAddr, MemAddr, MemAddSize, pB, Num, Timeout);
    if(status != HAL_OK)
    {
        return trans_polling_hal_to_i2c(status);
#if 0
        u32 u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode ;
        //Clean I2C
        HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
        HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;

		return trans_err_hal_to_i2c(u_ErrorCode);
#endif
    }
    //Release bus flag.
    t_infoI2C->u_isOccupy = false ;
    return I2C_OK ;
}


I2C_STATUS HTC_i2c_interrupt_read(T_HTC_I2C_INFO *t_infoI2C, u16 DevAddr, u16 Addr, u16 MemAddSize, u8* pB, u16 Num)
{
    HAL_StatusTypeDef status = HAL_OK;
    u32 u_ErrorCode;

    //Hold on bus flag.
    t_infoI2C->u_isOccupy = true ;
    t_infoI2C->t_masterState = I2C_READ_START ;
    //Trigger I2C HW
    status = HAL_I2C_Mem_Read_IT(t_infoI2C->t_pI2Ch, DevAddr, Addr, MemAddSize, pB, Num);
    if(status != HAL_OK)
    {
        u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode ;
        log_e("%s HAL_I2C_Mem_Read_IT fail, ErrorCode:%d.\r\n",__func__, u_ErrorCode);
        //Clean I2C
        HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
        HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;

		return trans_interrupt_hal_to_i2c(u_ErrorCode);
    }
#if I2C_USE_SEMAPHORE
    if ((xSemaphoreTake( &t_infoI2C->t_semaphoreI2C, Num*100 ) != pdTRUE)
     || (t_infoI2C->t_masterState != I2C_READ_STOP ))
    {
        t_infoI2C->u_isOccupy = false ;
        if (t_infoI2C->t_masterState != I2C_READ_STOP)//IT Timeout
        {
            //Clean I2C
            HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
            HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;
            return I2C_ERROR_TIMEOUT ;
        }
        else//Error
        {
            u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode ;
            //Clean I2C
            HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
            HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;
            log_e("Read I2C fail, status=%u \r\n", u_ErrorCode);

			return trans_err_hal_to_i2c(u_ErrorCode);
        }
    }
#endif
    //Release bus flag.
    t_infoI2C->u_isOccupy = false ;
    return I2C_OK ;
}


I2C_STATUS HTC_i2c_polling_Write(T_HTC_I2C_INFO *t_infoI2C, u16 DevAddr, u16 Addr, u16 MemAddSize, u8* pB, u16 Num, u32 Timeout)
{
    HAL_StatusTypeDef status = HAL_OK;

    //Hold on bus flag.
    t_infoI2C->u_isOccupy = true ;
    t_infoI2C->t_masterState = I2C_READ_START ;
    //Trigger I2C HW
    status = HAL_I2C_Mem_Write(t_infoI2C->t_pI2Ch, DevAddr, Addr, MemAddSize, (u8*)pB, Num, Timeout);
    if(status != HAL_OK)
    {
        return trans_polling_hal_to_i2c(status);
#if 0
        u32 u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode ;
        //Clean I2C
        HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
        HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;

		return trans_err_hal_to_i2c(u_ErrorCode);
#endif
    }
    //Release bus flag.
    t_infoI2C->u_isOccupy = false ;
    return I2C_OK ;
}


I2C_STATUS HTC_i2c_interrupt_write(T_HTC_I2C_INFO *t_infoI2C, u16 DevAddr, u16 Addr, u16 MemAddSize, u8* pB, u16 Num)
{
    HAL_StatusTypeDef status = HAL_OK;
    u32 u_ErrorCode;

    //Hold on bus flag.
    t_infoI2C->u_isOccupy = true ;
    t_infoI2C->t_masterState = I2C_READ_START ;
    //Trigger I2C HW
    status = HAL_I2C_Mem_Write_IT(t_infoI2C->t_pI2Ch, DevAddr, Addr, MemAddSize, (u8*)pB, Num);
    if(status != HAL_OK)
    {
        u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode;
        log_e("%s HAL_I2C_Mem_Write_IT fail, ErrorCode:%d.\r\n",__func__, u_ErrorCode);
        //Clean I2C
        HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
        HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;

		return trans_interrupt_hal_to_i2c(u_ErrorCode);
    }
#if I2C_USE_SEMAPHORE
    if ((xSemaphoreTake( &t_infoI2C->t_semaphoreI2C, Num*100 ) != pdTRUE)
     || (t_infoI2C->t_masterState != I2C_WRITE_STOP ))
    {
        t_infoI2C->u_isOccupy = false ;
        if (t_infoI2C->t_masterState != I2C_READ_STOP)//IT Timeout
        {
            //Clean I2C
            HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
            HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;
            return I2C_ERROR_TIMEOUT ;
        }
        else//Error
        {
            u_ErrorCode = t_infoI2C->t_pI2Ch->ErrorCode ;
            //Clean I2C
            HAL_I2C_MspDeInit(t_infoI2C->t_pI2Ch) ;
            HAL_I2C_MspInit(t_infoI2C->t_pI2Ch) ;
            log_e("Write I2C fail, status=%u \r\n", u_ErrorCode);

			return trans_err_hal_to_i2c(u_ErrorCode);
        }
    }
#endif
    //Release bus flag.
    t_infoI2C->u_isOccupy = false ;
    return I2C_OK ;
}

I2C_STATUS RTOS_I2C_ReadBuffer(u32 DevAddr, u16 Addr, u16 MemAddSize, u8 *pB, u16 Num, u32 Timeout)
{
    I2C_STATUS loResult = I2C_ERROR_TIMEOUT;
    u8 retry = I2C_NO_RETRY ;
    u8 index = 0;
    I2C_HandleTypeDef *i2ch ;
    char buffer[10] = {0};

    if ((i2ch = I2C_GetHandleByAddr(DevAddr)) == NULL)
		return I2C_ERROR_ADDR ;

    if (i2ch->Instance == I2C1)
        index = 0;
#if HTC_USE_I2C2
    else if (i2ch->Instance == I2C2)
        index = 1;
#endif

#if I2C_USE_MUTEX
    if( osMutexWait(t_infoI2C[index].xSemaphoreMutex, I2C_MUTEX_WAIT_MS) == osOK)
#else
	if(1)
#endif
    {
        //PWRMGR_VoteDontStop(&i2c_vote_Ballot[1]);
        do
        {
#if I2C_POLLING_RW
            loResult = HTC_i2c_polling_Read(&t_infoI2C[index], DevAddr, Addr, MemAddSize, pB, Num, Timeout);
#else
            loResult = HTC_i2c_interrupt_read(&t_infoI2C[index], DevAddr, Addr, MemAddSize, pB, Num);
#endif
            if (loResult != I2C_OK)
            {
                searchDeviceName(DevAddr, buffer);
                if (loResult == I2C_ERROR_NAK)
					log_e("I2C%d READ \"NACK\", dev=%s, Addr=0x%X, len=%u\n", index+1, buffer, Addr, Num);
                else if (loResult == I2C_ERROR_TIMEOUT)
                    log_e( "I2C%d READ \"TIMEOUT\", dev=%s, Addr=0x%X, len=%u\n", index+1, buffer, Addr, Num);
                else
                    log_e("I2C%d READ \"FAIL(%u)\", dev=%s, Addr=0x%X, len=%u\n", index+1, loResult, buffer, Addr, Num);
                if (++I2C_FailCounter[index] > I2C_LONG_RETRY)
                {
                    log_e("I2C%d HW Fail\n", index+1);
                    //PWRMGR_ResetDevice(PWRMGR_RESET_I2C_FAIL);
                }
                //tx_thread_sleep(I2C_RETRY_WAIT);
            }
            else
            {
                I2C_FailCounter[index] = 0 ;
            }
        }while(loResult != I2C_OK && retry--);
        //PWRMGR_VoteStop(&i2c_vote_Ballot[1]);
#if I2C_USE_MUTEX
        osMutexRelease(t_infoI2C[index].xSemaphoreMutex);
#endif
    }
    else
        log_e("xSemaphoreTake I2C%d xMutex FAIL\r\n", index+1);
    return loResult ;
}

I2C_STATUS RTOS_I2C_WriteBuffer(u32 DevAddr, u16 Addr, u16 MemAddSize, u8* pB, u16 Num, u32 Timeout)
{
    I2C_STATUS loResult = I2C_ERROR_TIMEOUT;
    u8 retry = I2C_NO_RETRY ;
    u8 index = 0;
    I2C_HandleTypeDef *i2ch ;
    char buffer[10] = {0};

    if ((i2ch = I2C_GetHandleByAddr(DevAddr)) == NULL)
		return I2C_ERROR_ADDR ;

    if (i2ch->Instance == I2C1)
        index = 0;
#if HTC_USE_I2C2
    else if (i2ch->Instance == I2C2)
        index = 1;
#endif

#if I2C_USE_MUTEX
    if( osMutexWait(t_infoI2C[index].xSemaphoreMutex, I2C_MUTEX_WAIT_MS) == osOK)
#else
	if(1)
#endif
    {
        //PWRMGR_VoteDontStop(&i2c_vote_Ballot[0]);
        do
        {
#if I2C_POLLING_RW
            loResult = HTC_i2c_polling_Write(&t_infoI2C[index], DevAddr, Addr, MemAddSize, pB, Num, Timeout);
#else
            loResult = HTC_i2c_interrupt_write(&t_infoI2C[index], DevAddr, Addr, MemAddSize, pB, Num);
#endif
            if (loResult != I2C_OK)
            {
                searchDeviceName(DevAddr, buffer);
                if (loResult == I2C_ERROR_NAK)
                    log_e("I2C%d WRITE \"NACK\", dev=%s, Addr=0x%X, len=%u\n", index+1, buffer, Addr, Num);
                else if (loResult == I2C_ERROR_TIMEOUT)
                    log_e("I2C%d WRITE \"TIMEOUT\", dev=%s, Addr=0x%X, len=%u\n", index+1, buffer, Addr, Num);
                else
                    log_e("I2C%d WRITE \"FAIL(%u)\", dev=%s, Addr=0x%X, len=%u\n", index+1, loResult, buffer, Addr, Num);
                if (++I2C_FailCounter[index] > I2C_LONG_RETRY)
                {
                    log_e("I2C%d HW Fail\n", index+1);
                    //PWRMGR_ResetDevice(PWRMGR_RESET_I2C_FAIL);
                }
                //tx_thread_sleep(I2C_RETRY_WAIT);
            }
            else
            {
                I2C_FailCounter[index] = 0 ;
            }
        }while(loResult != I2C_OK && retry--);
        //PWRMGR_VoteStop(&i2c_vote_Ballot[0]);

#if I2C_USE_MUTEX
        osMutexRelease(t_infoI2C[index].xSemaphoreMutex);
#endif
    }
    else
        log_e("xSemaphoreTake I2C%d xSemaphoreMutex FAIL\r\n", index+1);

    return loResult ;
}

I2C_STATUS RTOS_I2C_Master_Receive(u32 DevAddr, u8 *pB, u16 Num, u32 Timeout)
{
    I2C_STATUS loResult = I2C_ERROR_HAL;
    HAL_StatusTypeDef ret = HAL_ERROR;
    u8 retry = I2C_NO_RETRY ;
    u8 index = 0;
    I2C_HandleTypeDef *i2ch ;
    char buffer[10] = {0};

    if ((i2ch = I2C_GetHandleByAddr(DevAddr)) == NULL)
		return I2C_ERROR_ADDR ;

    if (i2ch->Instance == I2C1)
        index = 0;
#if HTC_USE_I2C2
    else if (i2ch->Instance == I2C2)
        index = 1;
#endif

#if I2C_USE_MUTEX
    if( osMutexWait(t_infoI2C[index].xSemaphoreMutex, I2C_MUTEX_WAIT_MS) == osOK)
#else
	if(1)
#endif
    {
        //PWRMGR_VoteDontStop(&i2c_vote_Ballot[0]);
        do
        {
            ret = HAL_I2C_Master_Receive(t_infoI2C[index].t_pI2Ch, DevAddr, pB, Num, Timeout);
            if (loResult != I2C_OK)
            {
                searchDeviceName(DevAddr, buffer);
                if (ret == HAL_BUSY){
                    loResult = I2C_ERROR_NAK;
                    log_e("I2C%d READ \"BUSY\", dev=%s, len=%u\n", index+1, buffer, Num);
                }
                else if (ret == HAL_TIMEOUT){
                    loResult = I2C_ERROR_TIMEOUT;
                    log_e("I2C%d READ \"TIMEOUT\", dev=%s, len=%u\n", index+1, buffer,Num);
                }
                else{
                    loResult = I2C_ERROR_NAK;
                    log_e("I2C%d READ \"FAIL(%u)\", dev=%s, len=%u\n", index+1, loResult, buffer, Num);
                }
                if (++I2C_FailCounter[index] > I2C_LONG_RETRY)
                {
                    log_e("I2C%d HW Fail\n", index+1);
                    //PWRMGR_ResetDevice(PWRMGR_RESET_I2C_FAIL);
                }
                //tx_thread_sleep(I2C_RETRY_WAIT);
            }
            else
            {
                I2C_FailCounter[index] = 0 ;
                loResult = I2C_OK;
            }
        }while(loResult != I2C_OK && retry--);
        //PWRMGR_VoteStop(&i2c_vote_Ballot[0]);

#if I2C_USE_MUTEX
        osMutexRelease(t_infoI2C[index].xSemaphoreMutex);
#endif
    }
    else
        log_e("xSemaphoreTake I2C%d xSemaphoreMutex FAIL\r\n", index+1);

    return loResult ;
}

I2C_STATUS RTOS_I2C_Master_Transmit(u32 DevAddr, u8 *pB, u16 Num, u32 Timeout)
{
    I2C_STATUS loResult = I2C_ERROR_HAL;
    HAL_StatusTypeDef ret = HAL_ERROR;
    u8 retry = I2C_NO_RETRY ;
    u8 index = 0;
    I2C_HandleTypeDef *i2ch ;
    char buffer[10] = {0};

    if ((i2ch = I2C_GetHandleByAddr(DevAddr)) == NULL)
		return I2C_ERROR_ADDR ;

    if (i2ch->Instance == I2C1)
        index = 0;
#if HTC_USE_I2C2
    else if (i2ch->Instance == I2C2)
        index = 1;
#endif

#if I2C_USE_MUTEX
    if( osMutexWait(t_infoI2C[index].xSemaphoreMutex, I2C_MUTEX_WAIT_MS) == osOK)
#else
	if(1)
#endif
    {
        //PWRMGR_VoteDontStop(&i2c_vote_Ballot[0]);
        do
        {
            ret = HAL_I2C_Master_Transmit(t_infoI2C[index].t_pI2Ch, DevAddr, pB, Num, Timeout);
            if (loResult != I2C_OK)
            {
                searchDeviceName(DevAddr, buffer);
                if (ret == HAL_BUSY){
                    loResult = I2C_ERROR_NAK;
                    log_e("I2C%d WRITE \"BUSY\", dev=%s, len=%u\n", index+1, buffer, Num);
                }
                else if (ret == HAL_TIMEOUT){
                    loResult = I2C_ERROR_TIMEOUT;
                    log_e("I2C%d WRITE \"TIMEOUT\", dev=%s, len=%u\n", index+1, buffer,Num);
                }
                else{
                    loResult = I2C_ERROR_NAK;
                    log_e("I2C%d WRITE \"FAIL(%u)\", dev=%s, len=%u\n", index+1, loResult, buffer, Num);
                }
                 if (++I2C_FailCounter[index] > I2C_LONG_RETRY)
                 {
                    log_e("I2C%d HW Fail\n", index+1);
                     //PWRMGR_ResetDevice(PWRMGR_RESET_I2C_FAIL);
                 }
                 //tx_thread_sleep(I2C_RETRY_WAIT);
            }
            else
            {
                I2C_FailCounter[index] = 0 ;
                loResult = I2C_OK;
            }
        }while(loResult != I2C_OK && retry--);
        //PWRMGR_VoteStop(&i2c_vote_Ballot[0]);

#if I2C_USE_MUTEX
        osMutexRelease(t_infoI2C[index].xSemaphoreMutex);
#endif
    }
    else
        log_e(  "xSemaphoreTake I2C%d xSemaphoreMutex FAIL\r\n", index+1);

    return loResult ;
}

/* Public functions ---------------------------------------------------------*/
void RTOS_I2C_Init(void)
{
    MX_I2C1_Init();
#if HTC_USE_I2C2
    MX_I2C2_Init();
#endif
    t_infoI2C[0].t_pI2Ch = &hi2c1 ;
#if HTC_USE_I2C2
    t_infoI2C[1].t_pI2Ch = &hi2c2 ;
#endif
#if I2C_USE_SEMAPHORE
    t_infoI2C[0].t_semaphoreI2C = NULL;
#if HTC_USE_I2C2
    t_infoI2C[1].t_semaphoreI2C = NULL;
#endif
#if I2C_USE_MUTEX
    t_infoI2C[0].xSemaphoreMutex = NULL;
#if HTC_USE_I2C2
    t_infoI2C[1].xSemaphoreMutex = NULL;
#endif
#endif
#endif
    //Create System resource
    do
    {
#if I2C_USE_MUTEX
        t_infoI2C[0].xSemaphoreMutex = osMutexCreate(NULL);
        if ( t_infoI2C[0].xSemaphoreMutex == NULL )
        {
            log_e("Create I2C1 mutex fail\r\n");
            break ;
        }
#if HTC_USE_I2C2
        t_infoI2C[1].xSemaphoreMutex = osMutexCreate(NULL);
        if ( t_infoI2C[1].xSemaphoreMutex == NULL )
        {
            log_e("Create I2C2 mutex fail\r\n");
            break ;
        }
#endif
#endif
#if I2C_USE_SEMAPHORE
        vSemaphoreCreateBinary(t_infoI2C[0].t_semaphoreI2C);
        if( t_infoI2C[0].t_semaphoreI2C == NULL )
            log_e ("Create I2C1 semaphore fail\r\n");

#if HTC_USE_I2C2
        vSemaphoreCreateBinary(t_infoI2C[1].t_semaphoreI2C);
        if( t_infoI2C[1].t_semaphoreI2C == NULL )
            log_e ("Create I2C2 semaphore fail\r\n");
#endif
#endif
    }while(0);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    log_d("%s \r\n", __func__);
#if I2C_USE_SEMAPHORE
    u8 i = 0;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1)
        i = 0;
    else if (hi2c->Instance == I2C2)
        i = 1;

    t_infoI2C[i].t_masterState = I2C_WRITE_STOP ;
    if (t_infoI2C[i].u_isOccupy)
        //xSemaphoreGive(&t_infoI2C[0].t_semaphoreI2C);
        xSemaphoreGiveFromISR(&t_infoI2C[0].t_semaphoreI2C, &xHigherPriorityTaskWoken);
#endif
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    log_d("%s \r\n", __func__);
#if I2C_USE_SEMAPHORE
	log_e("[I2C] %s: receive data is 0x%x.\r\n",__func__, *hi2c->pBuffPtr);
    u8 i = 0;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1)
        i = 0;
    else if (hi2c->Instance == I2C2)
        i = 1;

    t_infoI2C[i].t_masterState = I2C_READ_STOP ;
    if (t_infoI2C[i].u_isOccupy)
        //xSemaphoreGive(&t_infoI2C[0].t_semaphoreI2C);
        xSemaphoreGiveFromISR(&t_infoI2C[0].t_semaphoreI2C, &xHigherPriorityTaskWoken);

#endif
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    log_d("%s \r\n", __func__);
#if I2C_USE_SEMAPHORE
    u8 i = 0;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1)
        i = 0;
    else if (hi2c->Instance == I2C2)
        i = 1;

    t_infoI2C[i].t_masterState = I2C_ERROR ;
    if (t_infoI2C[i].u_isOccupy)
        //xSemaphoreGive(&t_infoI2C[0].t_semaphoreI2C);
        xSemaphoreGiveFromISR(&t_infoI2C[0].t_semaphoreI2C, &xHigherPriorityTaskWoken);

#endif
}
