/****************************************************************************
 * file name:   key_drv.c
 * description: button driver code
 * note: Need SW debounce, even HW had do the job.
 *-----------------            ---------------- GPIO HIGH
 *                |           |
 *                |           |
 *                -------------                 GPIO LOW
 *              press      release
 *****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "gpio.h"
#include "key_drv.h"
#include "led_hal.h"
#include "PowerManager.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_system.h"
#include "PowerManager_power.h"

btn_data_t btn_info[BTN_NUM];
static osMessageQId ButtonMsgQueueHandle;
static osMutexId keyIndexMutexId;
static uint32_t longPressFlag = 0;
static uint8_t longpress_counter = 0;
static TimerHandle_t longPressTimerHandle;
static osThreadId KeyIsrHandle;
extern pcbid_t pcb_id;

static void key_gpio_init(void)
{

    if(XA0n == pcb_id){
        btn_info[BTN_INDEX_POWER].keyGpioPin = GPIO_POWER_KEY;
        btn_info[BTN_INDEX_POWER].keyGpioPort = GPIO_PORT_POWER_KEY;
    }
    else{
        btn_info[BTN_INDEX_POWER].keyGpioPin = GPIO_XB_POWER_KEY;
        btn_info[BTN_INDEX_POWER].keyGpioPort = GPIO_XB_PORT_POWER_KEY;
    }
}

static int key_data_init(void)
{
    uint8_t i = 0;
    btn_info[BTN_INDEX_POWER].keyTimerName = "PowerKeyTimer";

    for(i=0; i<BTN_NUM; i++){
        btn_info[i].index = i;
        btn_info[i].debounce_time_old = 0;
        btn_info[i].PinState = GPIO_PIN_RESET;
        btn_info[i].s.KeyPress = 0xFF;
    }
    return 0;
}

static void key_longpress_timer_start(void)
{
    osTimerStart(longPressTimerHandle, LONG_PRESS_TIME_MS);
}

static void key_longpress_timer_exit(void)
{
    longpress_counter = 0;
    osTimerStop(longPressTimerHandle);
}

static void key_longpress_timer_callback(void const *argument)
{
    if(btn_info[BTN_INDEX_POWER].s.KeyPress &&
        (longPressFlag & BTN_PRESSED_BIT_POWER) ){
        longpress_counter++;
        key_log_d("Power Key pressed %ds\r\n",longpress_counter);

        if(longpress_counter >= 4){
            longPressFlag |= BTN_POWER_OFF_FLAG;
            key_log_d("Prepare to power off...\n");
            key_longpress_timer_exit();
            /* turn off(charger not in) blue led before shutdown */
            if(!isChargerIn()){
                SendLedEventToDeal(LED_EVENT_POWER_OFF | LED_EVENT_ON);
            }
            pwrmgr_system_send_message(PM_SYS_CMD_POWERKEY_LONG_PRESS, 0);
        }
        else{
            key_longpress_timer_start();
        }
    }
}

static void KeyIsrHandleTask(void const * argument)
{
    uint8_t index;
    GPIO_PinState PinState;

    for(;;){

        xQueueReceive( ButtonMsgQueueHandle, &index, portMAX_DELAY );
        if(index < BTN_NUM){
            PinState = HAL_GPIO_ReadPin(btn_info[index].keyGpioPort,
                btn_info[index].keyGpioPin);
        }
        else{
            key_log_e("key queue send or receive error! \n");
            continue;
        }

        if(GPIO_PIN_SET == PinState){
            if(btn_info[index].s.KeyPress == KeyRelease) continue;
            btn_info[index].s.KeyPress = KeyRelease;
            switch(index)
            {
                case BTN_INDEX_POWER:
                    longPressFlag &= ~BTN_PRESSED_BIT_POWER;
                    break;
                default:break;
            }
            key_log_i("button(index:%d) Released-------\r\n",index);
        }
        else{
            if(btn_info[index].s.KeyPress == KeyPress) continue;
            btn_info[index].s.KeyPress = KeyPress;
            switch(index)
            {
                case BTN_INDEX_POWER:
                    longPressFlag |= BTN_PRESSED_BIT_POWER;
                    break;
                default:break;
            }
            key_log_i("button(index:%d) Pressed++++++++\r\n",index);
        }

        /* start timer power key pressed */
        if(longPressFlag & BTN_PRESSED_BIT_POWER){
            longpress_counter = 0;
            key_longpress_timer_start();
        }
        else{
            /* stop longpress timer when key not pressed. */
            key_longpress_timer_exit();
        }
        pwrmgr_system_send_message(PM_SYS_CMD_POWERKEY_SHORT_PRESS,
                                            btn_info[index].s.KeyPress);

    }
}
static void key_queue_send(uint8_t index)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(ButtonMsgQueueHandle){
        if( xQueueSendFromISR( ButtonMsgQueueHandle, &index,
                                &xHigherPriorityTaskWoken ) != pdPASS ){
            key_log_e("key_rx_buf xQueueSend error! \n");
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void key_btn_isr(uint8_t key_index)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(keyIndexMutexId, &xHigherPriorityTaskWoken);
    uint8_t cur_key_index = key_index;
    xSemaphoreGiveFromISR(keyIndexMutexId, &xHigherPriorityTaskWoken);

    int debounce_time_new = xTaskGetTickCountFromISR();
    btn_info[cur_key_index].debounce_time_new = debounce_time_new;

    /* do debounce */
    if((debounce_time_new - btn_info[cur_key_index].debounce_time_old)
        >= DEBOUNCE_TIME_MS){
        btn_info[cur_key_index].debounce_time_old =
                            btn_info[cur_key_index].debounce_time_new;
        key_queue_send(cur_key_index);
    }
}

int key_drv_init(void)
{
    key_gpio_init();
    key_data_init();

    osMessageQDef(ButtonMsgQueue, 1, uint8_t);
    ButtonMsgQueueHandle = osMessageCreate(osMessageQ(ButtonMsgQueue), NULL);

    osThreadDef(key_isr_handle, KeyIsrHandleTask, osPriorityNormal, 0,
                                                configMINIMAL_STACK_SIZE*2);
    KeyIsrHandle = osThreadCreate(osThread(key_isr_handle), NULL);
    if(KeyIsrHandle == NULL){
        key_log_e("KeyIsrHandle is NULL\n");
    }

    osTimerDef(longPressTimer, key_longpress_timer_callback);
    longPressTimerHandle = osTimerCreate(osTimer(longPressTimer), osTimerOnce, NULL);
    if(longPressTimerHandle == NULL){
        key_log_e("%s: Long press Timer create fail!\r\n",__func__);
        return -2;
    }

    osMutexDef(key_index_mutex);
    keyIndexMutexId = osMutexCreate(osMutex(key_index_mutex));
    if(keyIndexMutexId == NULL){
        key_log_e("%s: key index mutex create fail!\r\n",__func__);
        return -3;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif
