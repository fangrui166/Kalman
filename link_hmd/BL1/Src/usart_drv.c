#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usart_drv.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "shellTask.h"

IRQn_Type dbg_uart_irq_num;

#define DBG_UART_RX_BUFFER_SIZE  (256)
static uint8_t RxChar[DBG_UART_RX_BUFFER_SIZE] = {0};
static uint32_t RxCharIndexIn = 0;
UART_HandleTypeDef dbg_uart;

/* DBG_UART init function */
static void MX_DBG_UART_Init(pcbid_t pcb_id)
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
  if (HAL_UART_Init(&dbg_uart) != HAL_OK)
  {
    Error_Handler();
  }

}
void dbg_uart_init(pcbid_t pcb_id)
{
    MX_DBG_UART_Init(pcb_id);
    HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar, DBG_UART_RX_BUFFER_SIZE);

    HAL_NVIC_SetPriority(dbg_uart_irq_num, 5, 0);
    HAL_NVIC_EnableIRQ(dbg_uart_irq_num);
    RxCharIndexIn = 0;

}

void dbg_uart_rx_handler(UART_HandleTypeDef * huart)
{
    if(huart->Instance == dbg_uart.Instance){
        if (RxCharIndexIn < DBG_UART_RX_BUFFER_SIZE){
            huart->Instance->DR = RxChar[RxCharIndexIn];
            Shell_rec_buf(RxChar[RxCharIndexIn++]);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == dbg_uart.Instance){
        RxCharIndexIn = 0;
        memset(RxChar,0,DBG_UART_RX_BUFFER_SIZE);
        HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar,
        DBG_UART_RX_BUFFER_SIZE);
    }
    uart_debug("\r\n%s\n",__func__);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == dbg_uart.Instance){
        RxCharIndexIn = 0;
        memset(RxChar,0,DBG_UART_RX_BUFFER_SIZE);
        HAL_UART_Receive_IT(&dbg_uart, (uint8_t *)&RxChar,
        DBG_UART_RX_BUFFER_SIZE);
    }
    uart_err("\r\nUsart2 recv error ,pls retry!!\n");

}


