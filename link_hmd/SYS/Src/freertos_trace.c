
/*
*
*     本文件为实现 FreeRTOS 调试功能
*
*
*/


#include "FreeRTOS.h"
#include <stm32f4xx.h>
#include "freertos_trace.h"

#if defined(HTC_TRACE_DEBUG)

long Tim1RunCounter;
TIM_HandleTypeDef        htim1up_tim10;
#define MAX_TASK_NUM        30
TaskStatus_t pxTaskStatusArray[MAX_TASK_NUM];
void vConfigureTimerForRunTimeStats(void)
{
    //RCC_ClkInitTypeDef    clkconfig;
    //uint32_t              uwTimclock = 0;
    //uint32_t              uwPrescalerValue = 0;
    //uint32_t              pFLatency;
    Tim1RunCounter = 100;

    /*Configure the TIM1_UP_TIM10_IRQn IRQ priority */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0 ,0);

    /* Enable the TIM1_UP_TIM10_IRQn global Interrupt */
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /* Enable TIM1 clock */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Get clock configuration */
    //HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Compute TIM1 clock */
    //uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

    /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
    //uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);

    /* Initialize TIM1 */
    htim1up_tim10.Instance = TIM1;

    /* Initialize TIMx peripheral as follow:
    + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
    + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
    + ClockDivision = 0
    + Counter direction = Up
    */
    htim1up_tim10.Init.Period = 10000;
    htim1up_tim10.Init.Prescaler = 168;
    htim1up_tim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim1up_tim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&htim1up_tim10) == HAL_OK)
    {
      /* Start the TIM time Base generation in interrupt mode */
      HAL_TIM_Base_Start_IT(&htim1up_tim10);
    }
    /* Return function status */
}

void Tim1PeriodElapsedHandle(void)
{
    Tim1RunCounter ++;
}

unsigned int ValueTimerForRunTimeStats(void)
{
	return Tim1RunCounter ;
}


/* Get task information of OS */
int get_task_state(int argc, char *argv[], _command_source source)
{
    const char task_state[]={'r','R','B','S','D'};
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime,ulStatsAsPercentage;
    /* Get total task number */
    uxArraySize = uxTaskGetNumberOfTasks();
    if(uxArraySize>MAX_TASK_NUM){
        printf("Too much task at present :%d\n",uxArraySize);
    }
    /* Get state information of every task */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime );
    #if (configGENERATE_RUN_TIME_STATS==1)
    printf("TaskName        State    ID     Priority  HEAP    CPU-Usage\n");
    /* Avoid Divide By Zero Excepiton */
    if( ulTotalRunTime > 0 ){
        for( x = 0; x < ((uxArraySize < MAX_TASK_NUM)? uxArraySize : MAX_TASK_NUM); x++ ){
            char tmp[128];
            /* Calculate percentage of task run time and total run time */
            ulStatsAsPercentage =(uint64_t)(pxTaskStatusArray[ x ].ulRunTimeCounter)*100 / ulTotalRunTime;
            if( ulStatsAsPercentage > 0UL ){
                sprintf(tmp,"%-18s%-7c%-10d%-7d%-8d%d%%",
                    pxTaskStatusArray[ x ].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                    pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x ].uxCurrentPriority,
                    pxTaskStatusArray[ x ].usStackHighWaterMark,ulStatsAsPercentage);
            }
            else{
                /* The runing time of task less than 1% of total run time */
                sprintf(tmp,"%-18s%-7c%-10d%-7d%-8dt<1%%",
                    pxTaskStatusArray[ x ].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                    pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x ].uxCurrentPriority,
                    pxTaskStatusArray[ x ].usStackHighWaterMark);
            }
            printf("%s\n",tmp);
        }

    }
    printf("\nTaskState:   r-runing  R-Ready  B-Block  S-Suspend  D-Delete\n");
    #endif //#if (configGENERATE_RUN_TIME_STATS==1)
    return 0;
}

#endif
