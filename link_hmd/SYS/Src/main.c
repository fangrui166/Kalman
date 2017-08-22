/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "hlog_api.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "PowerManager.h"
#include "PowerManager_command.h"
#include "usart_drv.h"
#include "command.h"
#include "rtos_i2c_drv.h"
#include "x_pmic.h"
#include "gpio.h"
#include "proximity_task.h"
#include "audio_alc5665_driver.h"
#include "audio_alc4040_driver.h"
#include "sensor_task.h"
#include "usb_device.h"
#include "htc_usb_cdc_data_service.h"
#include "system_property.h"
#include "system_command.h"
#include "htc_audio_path_service.h"
#include "htc_3dof_transfer_service.h"
#include "gpio_exp.h"
#include "led_hal.h"
#include "htc_adc_func.h"
#include "display_drv.h"
#include "flash_drv.h"
#include "rtc_drv.h"
#include "key_drv.h"
#include "ccgx_ec.h"
#include "misc_data.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

#ifdef HAL_IWDG_MODULE_ENABLED
IWDG_HandleTypeDef hiwdg;
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
WWDG_HandleTypeDef hwwdg;
#endif
pcbid_t pcb_id;

osThreadId watchdog_th;

SPI_HandleTypeDef hspi2;

//HCD_HandleTypeDef hhcd_USB_OTG_FS;

osThreadId defaultTaskHandle;

unsigned int g_MFGModeFlag = 0;
extern char build_date[16];
extern char build_time[16];

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);

#ifdef HAL_IWDG_MODULE_ENABLED
void MX_IWDG_Init(void);
#endif
#ifdef HAL_WWDG_MODULE_ENABLED
void MX_WWDG_Init(void);
#endif

//static void MX_USB_OTG_FS_HCD_Init(void);
void StartDefaultTask(void const * argument);
void bl_update_process();
#define HTC_WWDG_TEST   0
static void watchdog_task( void const * pvParameters )
{
        TickType_t xLastWakeTime;
#if defined(HAL_IWDG_MODULE_ENABLED)
        const TickType_t xFrequency = portTICK_PERIOD_MS * 2000;
        MX_IWDG_Init();
        //HAL_IWDG_Start(&hiwdg);
#elif defined(HAL_WWDG_MODULE_ENABLED)
        const TickType_t xFrequency = portTICK_PERIOD_MS * 16;
        HAL_RCC_WWDG_CTRL(1);
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
                __HAL_RCC_CLEAR_RESET_FLAGS();
        MX_WWDG_Init();
        //HAL_WWDG_Start(&hwwdg);
#else
        const TickType_t xFrequency = portTICK_PERIOD_MS * 10000;

#endif

        (void)pvParameters;
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_LOG, "%s Init Done\n", __func__);

        xLastWakeTime = xTaskGetTickCount ();
        for( ;; )
        {
#if defined(HAL_IWDG_MODULE_ENABLED)
                HAL_IWDG_Refresh(&hiwdg);
#elif defined(HAL_WWDG_MODULE_ENABLED)
                HAL_WWDG_Refresh(&hwwdg);
#endif
#if HTC_WWDG_TEST == 1
                {
                        static int cnt;
                        if (cnt%100 == 0)
                              printf("WDG : cnt=%d\n", cnt);
                        if (cnt == 1000)
                                while(1)
                                    ;
                        cnt++;
                }
#endif
                vTaskDelayUntil( &xLastWakeTime, xFrequency );
        }
}

void watchdog_init(void)
{
#if defined(HAL_IWDG_MODULE_ENABLED) || defined(HAL_WWDG_MODULE_ENABLED)
	osThreadDef(watchdog, watchdog_task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE*2);
	watchdog_th = osThreadCreate(osThread(watchdog), NULL);
#endif
}

void watchdog_refresh(void)
{
#if defined(HAL_IWDG_MODULE_ENABLED)
        if(NULL == hiwdg.Instance)
                return;
        HAL_IWDG_Refresh(&hiwdg);
#elif defined(HAL_WWDG_MODULE_ENABLED)
        if(NULL == hwwdg.Instance)
                return;
        HAL_WWDG_Refresh(&hwwdg);
#endif
}
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int main(void)
{
    int ret = 0;
    BOOT_MODE boot_mode;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  flash_init();
  misc_data_init();
  ret = get_pcbid(&pcb_id);
  if(pcb_id == 0xFFFFFFFF){
    pcb_id = XA0n;
    set_pcbid(pcb_id);
  }

  /* Initialize all configured peripherals */
  gpio_init(pcb_id);
  dbg_uart_init(pcb_id);
  hlog_init();
  logi("Link HMD system build @ %s - %s\r\n",build_date,build_time);
  logi("Get PCB ID %s, <0x%08X>\r\n", (ret ? "failed":"success"), pcb_id);
  logi("boot_type:%s\r\n",boot_type);
  memset(boot_type, 0, sizeof(boot_type));
  memcpy(boot_type, "hTC_Link_hmd", strlen("hTC_Link_hmd"));
  boot_mode = get_bootmode();
  if(boot_mode == SYS_MFG_MODE){
    g_MFGModeFlag = 1;
    logi("boot_mode : SYS_MFG_MODE\r\n");
  }
  RTOS_I2C_Init();
  ioexp_drv_init();
  led_init();
  key_drv_init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  dbg_uart_postinit();
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  //bl_update_process();
  cmd_init();
  rtc_drv_init(&hrtc);
  bsp_pmic_init();
  PWRMGR_initial();

  htc_adc1_func_initial(&hadc1);
  if(pcb_id == XA0n){
    proximityTaskInit();
    /* Audio Driver */
    audio_alc4040_driver_init();
  }
  display_drv_init();
  /* Audio Driver */
  audio_alc5665_driver_init();
  ccgx_init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* Audio Path Service */
  audio_path_service_init();
  /* USB Service */
  usb_cdc_service_init();
  system_property_init();
  system_command_init();
  htc_3dof_transfer_service_initial();
  sensorTaskInit();
#if 0
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0,
  configMINIMAL_STACK_SIZE*2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
#endif
  SendLedEventToDeal(LED_EVENT_LOADING | LED_EVENT_OFF);
  PWRMGR_reboot_test();
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
#if 1 //WA for Sensor Calibration
	watchdog_init();
#endif
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
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

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_CLK48;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  //RTC_TimeTypeDef sTime;
  //RTC_DateTypeDef sDate;
  //RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
#if 0
    /**Initialize RTC and set the Time and Date
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm A
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm B
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the WakeUp
    */
    /*
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  */
#endif
}

#ifdef HAL_IWDG_MODULE_ENABLED
/* IWDG init function */
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init(&hiwdg);

}
/*
	Set IWDG count to max value when system enter suspend(sleep/stop)
*/
void MX_IWDG_Reinit_For_Suspend(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 32767;
  HAL_IWDG_Init(&hiwdg);

}
#else
void MX_IWDG_Init(void){}
void MX_IWDG_Reinit_For_Suspend(void){}
#endif

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

#ifdef HAL_WWDG_MODULE_ENABLED
/* WWDG init function */
void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
#if 0
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
#else
/*8/9 * 63*/
hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
hwwdg.Init.Window = 0x7f;
hwwdg.Init.Counter = 0x7e;
#endif
  HAL_WWDG_Init(&hwwdg);

}
#endif


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
#if 0
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  static int p_count = 0;

  /* Infinite loop */
  for(;;)
  {
    printf("Hello FreeRTOS(%5d)\r\n", p_count);
    switch (p_count % 4) {
    #if NUCLEO_BOARD_TEST
    case 0:
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_BLUE_Pin,GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);
        break;
    #endif
    default:
        break;
    }
    if (p_count == 100000)
        p_count = 0;
    else
        p_count++;
    osDelay(1000);

  }
}
#endif
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
