/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "flash_drv.h"
#include "bl1_check.h"
#include "htc_memory_define.h"
#include "misc_data.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define GPIO_XB_POWER_KEY                       GPIO_PIN_15  /* PA15 for power key detection interrupt*/
#define GPIO_PORT_XB_POWER_KEY                  GPIOA
#define GPIO_POWER_KEY                          GPIO_PIN_10  /* PC10 for power
key detection interrupt*/
#define GPIO_PORT_POWER_KEY                     GPIOC
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef dbg_uart;
static volatile uint8_t powerkey_kick_count = 0;
#define WAITE_KEY_KICK_TIME     200  //ms
pcbid_t pcb_id;

void my_printf(char * str)
{
	HAL_UART_Transmit(&dbg_uart, (uint8_t *)str, strlen(str), 0xFFFF);

}

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
typedef void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t  JumpAddress;

#pragma optimize=none
static void jump_and_execute(uint32_t address)
{
    /* Disable SysTick */
    SysTick->CTRL = 0x0;

    /* Disable all IRQ */
    __disable_irq();
    /* Jump code to external PSRAM to execut */
    JumpAddress = *(__IO uint32_t*) (address + 4);
    Jump_To_Application = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) address);
    Jump_To_Application();
}
static void jump_and_execute_sysmemDFU(void)
{
	memcpy(boot_type,"SYSDFU", strlen("SYSDFU"));
    __disable_irq();
    HAL_NVIC_SystemReset();
}
int main(void)
{
  uint32_t time_count = 0;
  uint32_t Dcount = 0;


  /* MCU Configuration----------------------------------------------------------*/
  //memcpy(boot_type,"SYSDFU", strlen("SYSDFU"));

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  flash_init();
  get_pcbid(&pcb_id);
  if(pcb_id == 0xFFFFFFFF){
    pcb_id = XA0n;
  }

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  xlog("Link BL0 build @ "__DATE__ " - "__TIME__ "\n");
  /* USER CODE BEGIN 2 */
  time_count = HAL_GetTick();
  while(1){

	Dcount = HAL_GetTick() - time_count;
  	if((Dcount >= WAITE_KEY_KICK_TIME) && (powerkey_kick_count == 0)){
		break;
  	}
	else if((Dcount >= (WAITE_KEY_KICK_TIME*2)) && (powerkey_kick_count <= 1)){
		break;
	}
	else if((Dcount >= (WAITE_KEY_KICK_TIME*3)) && (powerkey_kick_count <= 2)){
		break;
	}
	else if((Dcount >= (WAITE_KEY_KICK_TIME*4)) && (powerkey_kick_count <= 3)){
		break;
	}
	else if((Dcount >= (WAITE_KEY_KICK_TIME*5)) && (powerkey_kick_count <= 4)){
		break;
	}
	else if((Dcount >= (WAITE_KEY_KICK_TIME*5)) && (powerkey_kick_count >= 4)){
		jump_and_execute_sysmemDFU();
	}
  }


    if(isBL1FwOK()){
        if(XA0n == pcb_id){
            HAL_GPIO_DeInit(GPIO_PORT_POWER_KEY, GPIO_POWER_KEY);
        }
        else{
            HAL_GPIO_DeInit(GPIO_PORT_XB_POWER_KEY, GPIO_XB_POWER_KEY);
        }
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        jump_and_execute(IMAGE_START_BL1);
    }
    else{
    jump_and_execute_sysmemDFU();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	*/
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|
RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

	/**Initializes the CPU, AHB and APB busses clocks
	*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
	Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|
RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

	/**Configure the Systick interrupt time
	*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
    if(XA0n == pcb_id){
      dbg_uart.Instance = USART2;
    }
    else{
        dbg_uart.Instance = USART1;
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

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  if(XA0n == pcb_id){

      /* config power key PC10*/
      GPIO_InitStruct.Pin = GPIO_POWER_KEY;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_PORT_POWER_KEY, &GPIO_InitStruct);
  }
  else{
      /* config power key PC10*/
      GPIO_InitStruct.Pin = GPIO_XB_POWER_KEY;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_PORT_XB_POWER_KEY, &GPIO_InitStruct);
  }
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == GPIO_POWER_KEY)||(GPIO_Pin == GPIO_XB_POWER_KEY)){
		powerkey_kick_count ++;
		if(powerkey_kick_count >=5){
			jump_and_execute_sysmemDFU();
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
