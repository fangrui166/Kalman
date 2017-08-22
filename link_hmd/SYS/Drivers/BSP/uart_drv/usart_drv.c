#include <string.h>
#include "gpio.h"
#include "usart_drv.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "command.h"
#include "PowerManager_notify_func.h"

#define DBG_UART_RX_BUFFER_SIZE  (256)
extern pcbid_t pcb_id;

static IRQn_Type dbg_uart_irq_num;

UART_HandleTypeDef dbg_uart;
osMutexId dbg_uart_mutex = NULL;
static osThreadId dbg_uartTaskHandle;
static osMessageQId UsartMsgQueueHandle;

static unsigned char dbg_uart_ready = 0;
static uint8_t RxChar[DBG_UART_RX_BUFFER_SIZE] = {0};
static uint32_t RxCharIndexIn = 0;
static unsigned int dbg_uart_rxne_flag = 0;
static unsigned int dbg_uart_rx_cplt = 0;

UsartMsgQueue U_MsgQueue;
cmd_struct parse_buf;

static struct pwrmgr_notify_func_data PmNotifyData = {0};
static uint32_t filter_useless_string(char *dest,const char *src);

/* USART2 init function */
void MX_DBG_UART(pcbid_t pcb_id)
{
    if(XA0n == pcb_id){
        dbg_uart.Instance = USART2;
        dbg_uart_irq_num = USART2_IRQn;
    }
    else{
        dbg_uart.Instance = USART1;
        dbg_uart_irq_num = USART1_IRQn;
    }
    dbg_uart.Init.BaudRate = 115200;
    dbg_uart.Init.WordLength = UART_WORDLENGTH_8B;
    dbg_uart.Init.StopBits = UART_STOPBITS_1;
    dbg_uart.Init.Parity = UART_PARITY_NONE;
    dbg_uart.Init.Mode = UART_MODE_TX_RX;
    dbg_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    dbg_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&dbg_uart);

}

void dbg_uart_init(pcbid_t pcb_id)
{
    MX_DBG_UART(pcb_id);
    HAL_NVIC_SetPriority(dbg_uart_irq_num, 5, 0);
    HAL_NVIC_EnableIRQ(dbg_uart_irq_num);
    if (NULL == dbg_uart_mutex)
        dbg_uart_mutex = osMutexCreate(NULL);
    RxCharIndexIn = 0;

    dbg_uart_ready = 1;
    //dbg_uart_postinit();
}

void dbg_uart_deinit(void)
{
    if (dbg_uart_ready == 1)
    {
        dbg_uart_ready = 0;
        HAL_UART_DeInit(&dbg_uart);
        HAL_NVIC_DisableIRQ(dbg_uart_irq_num);
        if (dbg_uart_mutex != NULL) {
                osMutexDelete(dbg_uart_mutex);
                dbg_uart_mutex = NULL;
        }
    }
}

void dbg_uart_disable(void)
{
    dbg_uart_ready = 0;
    HAL_UART_DeInit(&dbg_uart);
    HAL_NVIC_DisableIRQ(dbg_uart_irq_num);
}
void dbg_uart_enable(void)
{


    MX_DBG_UART(pcb_id);
    RxCharIndexIn = 0;
    HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar, DBG_UART_RX_BUFFER_SIZE);
    HAL_NVIC_EnableIRQ(dbg_uart_irq_num);

    dbg_uart_ready = 1;
}

static void dbg_uart_wakeup_GPIO_Config(void)
{
    #if 0
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_DEBUG_UART_TX;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIO_PORT_DEBUG_UART, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = GPIO_DEBUG_UART_RX;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIO_PORT_DEBUG_UART, &GPIO_InitStruct);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_DEBUG_UART_RX);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    /* when set uart drver enter suspend at the last one
    *  we need to avoid the first interrupt wakeup system
    *  immediately.
    */
    HAL_Delay(100);
    #endif
}

static void dbg_uart_wakeup_GPIO_DeConfig(void)
{
    #if 0
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    HAL_GPIO_DeInit(GPIO_PORT_DEBUG_UART, GPIO_DEBUG_UART_TX|GPIO_DEBUG_UART_RX);
    #endif
}

#if 0
static void dbg_uart_reset_rx_buffer(void)
{
    __HAL_USART_DISABLE_IT(&dbg_uart, USART_IT_RXNE);
    dbg_uart.RxXferCount = DBG_UART_RX_BUFFER_SIZE;
    dbg_uart.pRxBuffPtr = (uint8_t *)&RxChar;
    if (dbg_uart.gState & HAL_UART_STATE_BUSY_TX)
    {
        dbg_uart.gState &= HAL_UART_STATE_BUSY_RX;
    }
    else
    {
        dbg_uart.gState = HAL_UART_STATE_BUSY_RX;
    }
    RxCharIndexIn = 0;
    __HAL_USART_ENABLE_IT(&dbg_uart, USART_IT_RXNE);
}

void dump_hex(char * buf,int len)
{
    int i=0;
    printf("%s\n",__func__);
    for(i=0;i<len;i++){
        if(!i%8)
            printf("\n");
        printf("%#2x ",buf[i]);
    }
    printf("\n");
}
#endif
void StartDebugUsartTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugUsartTask */

  /* Infinite loop */
  for(;;)
  {
      UsartMsgQueue commd_buf = {0};
      xQueueReceive( UsartMsgQueueHandle, &commd_buf,  portMAX_DELAY );
      switch(commd_buf.CmdQueue){
        case MSG_QUEUE_FRAME:
            parse_buf.cmd_length = filter_useless_string(parse_buf.cmd_buf, commd_buf.rec_buf.cmd_buf);
            if( get_queue_commd() != 0 ){
                if( xQueueSend(get_queue_commd(), &parse_buf, 1) !=pdPASS ){
                    uart_err("StartDebugUsartTask xQueueSend error!\n");
                }
            }
            break;
        case MSG_QUEUE_RXCHAR_FULL:

            RxCharIndexIn = 0;
            memset(RxChar,0,DBG_UART_RX_BUFFER_SIZE);
            HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar, DBG_UART_RX_BUFFER_SIZE);
            break;
        default :
            break;
      }

  }
  /* USER CODE END StartDebugUsartTask */
}
int dbg_uart_notify_callback(uint32_t _notify_flag, uint32_t _state, void *data)
{
    switch(_notify_flag){
        case PWRMGR_NOTIFY_STOP_STATE:
            if(_state == STOP_ENTER){
                //printf("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
                dbg_uart_disable();

                /* Enable Rx port interrupt */
                dbg_uart_wakeup_GPIO_Config();
            }
            else if(_state == STOP_LEAVE){
                dbg_uart_wakeup_GPIO_DeConfig();
                dbg_uart_enable();
                //printf("%s _notify_flag:%#x, state:%d\n",__func__, _notify_flag, _state);
            }
            else{
                uart_debug("I'm sorry for this\n");
            }
            break;
        case PWRMGR_NOTIFY_POWER_OFF:
            PWRMGR_SendNotifyAck(&PmNotifyData);
            break;
        case PWRMGR_NOTIFY_SYSTEM_RESET :
            PWRMGR_SendNotifyAck(&PmNotifyData);
            break;
        default:
            uart_debug("dbg_uart don't care this flag\n");
            break;
    }
    return 0;
}
void dbg_uart_postinit(void)
{
    HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar, DBG_UART_RX_BUFFER_SIZE);
    osThreadDef(dbg_uartTask, StartDebugUsartTask, osPriorityNormal, 0,
                                                configMINIMAL_STACK_SIZE*3);
    dbg_uartTaskHandle = osThreadCreate(osThread(dbg_uartTask), NULL);
    if(dbg_uartTaskHandle == NULL){
        uart_debug("dbg_uartTaskHandle is NULL\n");
    }

    osMessageQDef(UsartMsgQueue, 5, U_MsgQueue);
    UsartMsgQueueHandle = osMessageCreate(osMessageQ(UsartMsgQueue), NULL);

    PmNotifyData.func_name = "usart_drv";
    PmNotifyData.data = NULL;
    PmNotifyData.callback= dbg_uart_notify_callback;
    PmNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE/*|PWRMGR_NOTIFY_POWER_OFF|PWRMGR_NOTIFY_SYSTEM_RESET*/;
    PmNotifyData.func_level = PWRMGR_FUNC_CORE_LEVEL;
    PWRMGR_register_notify_func(&PmNotifyData);

    uart_info("dbg_uart_postinit success!\n");

}
static int SendMsgQueueFromISR(void)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(UsartMsgQueueHandle){
        if(xQueueIsQueueFullFromISR(UsartMsgQueueHandle)){
            return MSG_QUEUE_FULL;
        }
        if( xQueueSendFromISR( UsartMsgQueueHandle, &U_MsgQueue, &xHigherPriorityTaskWoken ) !=pdPASS ){
            //printf("dbg_uart_rec_buf xQueueSend error! \n");
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
        return MSG_QUEUE_SUCCESS;
    }
    return MSG_QUEUE_ERROR;
}

void dbg_uart_rec_buf(char data)
{
    static uint32_t rec_count=0;

    U_MsgQueue.rec_buf.cmd_buf[rec_count]=data;
    if(((0x0D==U_MsgQueue.rec_buf.cmd_buf[rec_count-1])&&(0x0A==U_MsgQueue.rec_buf.cmd_buf[rec_count])) || (0x0D==U_MsgQueue.rec_buf.cmd_buf[rec_count]))
    {
        U_MsgQueue.CmdQueue = MSG_QUEUE_FRAME;
        U_MsgQueue.rec_buf.cmd_length = rec_count+1;
        SendMsgQueueFromISR();

        rec_count=0;
    }
    else
    {
        rec_count++;
        if(rec_count>=CMD_BUF_LEN)
        {
            rec_count=0;
        }
    }
}

void dbg_uart_rx_handler(UART_HandleTypeDef * huart)
{
    if(huart->Instance == dbg_uart.Instance)
    {
        if (dbg_uart_rxne_flag == 1 && RxCharIndexIn < DBG_UART_RX_BUFFER_SIZE)
        {
            dbg_uart_rxne_flag = 0;

            huart->Instance->DR = RxChar[RxCharIndexIn];
            dbg_uart_rec_buf(RxChar[RxCharIndexIn++]);
        }

        if (dbg_uart_rx_cplt == 1){
            dbg_uart_rx_cplt = 0;
            U_MsgQueue.CmdQueue = MSG_QUEUE_RXCHAR_FULL;
            SendMsgQueueFromISR();

        }
    }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == dbg_uart.Instance)
    {
        dbg_uart_rx_cplt = 1;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == dbg_uart.Instance){
        RxCharIndexIn = 0;
        memset(RxChar,0,DBG_UART_RX_BUFFER_SIZE);
        HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar,
        DBG_UART_RX_BUFFER_SIZE);
    }
    uart_err("\r\nDebugUsart recv error ,pls retry!!\n");

}

void dbg_uart_set_rxne_flag(UART_HandleTypeDef *huart)
{
    if(huart->Instance == dbg_uart.Instance){
        dbg_uart_rxne_flag = 1;
    }

}

/**
*use SecureCRT serial transceivers tools,in the character stream,may contain
*a character that is not needed , such as backspace,move around.Before use the
*command line tool to parse the character,we must delete the invalid character.
* supported character:
*   move up:1B 5B 41
*   move down:1B 5B 42
*   move right:1B 5B 43
*   move left:1B 5B 44
*   carriage retuen and line feed:0D 0A
*   Backspace:08
*/
static uint32_t filter_useless_string(char *dest,const char *src)
{
    uint32_t dest_count=0;
    uint32_t src_count=0;
    while(src[src_count]!=0x0D && src[src_count+1]!=0x0A){
        //if(isprint(src[src_count])){
        if(src[src_count] >= 0x20 && src[src_count] <= 0x7E){
            dest[dest_count++]=src[src_count++];
        }
        else{
            switch(src[src_count]){
                case    0x08:                          //Backspace
                {
                    if(dest_count>0){
                        dest_count --;
                    }
                    src_count ++;
                }
                break;
                case    0x1B:
                {
                    if(src[src_count+1]==0x5B){
                        if(src[src_count+2]==0x41 || src[src_count+2]==0x42){
                            src_count +=3;              //move up and down
                        }
                        else if(src[src_count+2]==0x43){
                            dest_count++;               //move right
                            src_count+=3;
                        }
                        else if(src[src_count+2]==0x44){
                            if(dest_count >0)           //move left
                            {
                                dest_count --;
                            }
                            src_count +=3;
                        }
                        else{
                            src_count +=3;
                        }
                    }
                    else{
                        src_count ++;
                    }
                }
                break;
                default:
                {
                    src_count++;
                }
                break;
            }
        }
    }

    dest[dest_count++]=src[src_count++];
    dest[dest_count++]=src[src_count++];

    return dest_count;
}
void dbg_uart_wakeup_handler(void)
{
    /* Nothing to do at here */
}


