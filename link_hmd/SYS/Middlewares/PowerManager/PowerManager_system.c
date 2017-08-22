#include "FreeRTOS.h"
#include "timers.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"
#include "gpio.h"
#include "led_hal.h"
#include "PowerManager.h"
#include "PowerManager_battery.h"
#include "PowerManager_system.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_notify_func.h"
#include "PowerManager_power.h"
#include "ccgx_ec.h"
#include "key_drv.h"

struct pwmgr_system_data{
    osThreadId pwrmgr_system_thread;
    osMessageQId PmSysMsgQueueHandle;
    PmSysMsgQueue_t pm_sys_msg_queue;
    osTimerId      wait_usb_timer;
    osTimerId      wait_video_timer;
};
extern pcbid_t pcb_id;
extern unsigned int g_MFGModeFlag;

static struct pwmgr_system_data ps_data = {0};
static struct pwrmgr_notify_func_data PmSysNotifyData = {0};

static void pwrmgr_system_try_to_poweroff(int type)
{
    if(PWRMGR_SYSTEM_CAN_BE_POWER_OFF()){
        pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_POWEROFF_MODE, type);
    }
    else{
        pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_STOP_MODE, type);
    }
}
static void pwrmgr_system_thread_func(void const * argument)
{
    struct pwmgr_system_data *data = &ps_data;
    PmSysMsgQueue_t PmSysMsg;
    static uint8_t key_state = 0;
    if(!isUSBDataIn()){
        osTimerStart(data->wait_usb_timer, WAITE_USB_DATA_IN_TIMEOUT);
        PWRMGR_SendNotify(PWRMGR_NOTIFY_USB_DP_HOLE, PLUG_OUT,
            LEVER_DESCEND_ORDER);
    }
    else{
        SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_ON);
        osTimerStart(data->wait_video_timer, WAITE_VIDEO_OUTPUT_TIMEOUT);
        PWRMGR_SendNotify(PWRMGR_NOTIFY_USB_DP_HOLE, PLUG_IN,
            LEVER_ASCEND_ORDER);
    }
    for(;;){
        if(data->PmSysMsgQueueHandle == NULL) continue;
        xQueueReceive( data->PmSysMsgQueueHandle, &PmSysMsg, portMAX_DELAY );
        switch(PmSysMsg.cmd){
            case PM_SYS_CMD_STOP_TIMEOUT:
                pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_POWEROFF_MODE,
                    PWRMGR_STOP_TIMEOUT);
                break;
            case PM_SYS_CMD_POWERKEY_LONG_PRESS:
                if(XA0n == pcb_id){
                    while(!HAL_GPIO_ReadPin(GPIO_PORT_POWER_KEY, GPIO_POWER_KEY)); // Wate power key release.
                }
                else{
                    while(!HAL_GPIO_ReadPin(GPIO_XB_PORT_POWER_KEY, GPIO_XB_POWER_KEY)); // Wate power key release.
                }
                key_state = 0;
                pwrmgr_system_try_to_poweroff(PWRMGR_LONG_KEY);
                break;
            case PM_SYS_CMD_USB_DATA_OUT:
                pwrmgr_warning("DP hole plug out\r\n");
                osTimerStart(data->wait_usb_timer, WAITE_USB_DATA_IN_TIMEOUT);
                osTimerStop(data->wait_video_timer);
				/* turn off CONNECTING/CONNECTING_FAIL led event */
                SendLedEventToDeal(LED_EVENT_CONNECTED | LED_EVENT_ON);
                PWRMGR_SendNotify(PWRMGR_NOTIFY_USB_DP_HOLE, PLUG_OUT,
                    LEVER_DESCEND_ORDER);
                break;
            case PM_SYS_CMD_USB_DATA_IN:
                pwrmgr_warning("DP hole plug in\r\n");
                if(PWRMGR_STAT_MACHINE_STOP_MODE==pwrmgr_state_going_get()){
                    pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_BOOTING_MODE,
                            PWRMGR_STOP_EXIT);
                }
                SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_ON);
                osTimerStop(data->wait_usb_timer);
                osTimerStart(data->wait_video_timer, WAITE_VIDEO_OUTPUT_TIMEOUT);
                PWRMGR_SendNotify(PWRMGR_NOTIFY_USB_DP_HOLE, PLUG_IN,
                    LEVER_ASCEND_ORDER);
                break;
            case PM_SYS_CMD_POWERKEY_SHORT_PRESS:
            {
                if(pcb_id == XA0n){
                    if((wakeup_source & WAKEUP_SOURCE_EXTI10) ==
                        WAKEUP_SOURCE_EXTI10) {
                        wakeup_source &= ~WAKEUP_SOURCE_EXTI10;
                        wakeup_source |= WAKEUP_SOURCE_POWERKEY;
                        break;
                    }
                }
                else{ /* for XB02, XC01, XC02*/
                    if((wakeup_source & WAKEUP_SOURCE_EXTI15) ==
                        WAKEUP_SOURCE_EXTI15) {
                        wakeup_source &= ~WAKEUP_SOURCE_EXTI15;
                        wakeup_source |= WAKEUP_SOURCE_POWERKEY;
                        break;
                    }
                }
                if(PmSysMsg.state == KeyPress){
                    key_state = 1;
                }
                if((PmSysMsg.state == KeyRelease) && (key_state)){
                    key_state = 0;
                    if(PWRMGR_STAT_MACHINE_STOP_MODE==pwrmgr_state_going_get()){
                        break;
                    }
                    pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_STOP_MODE,
                        PWRMGR_SHORT_KEY);
                }
            }
                break;
            case PM_SYS_CMD_PSENSOR_DET:
                /*
                if(PmSysMsg.state == 1){
                    pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_STOP_MODE,
                        PWRMGR_PSENSOR_AWAY);
                }
                */
                break;
            case PM_SYS_CMD_NO_USB_TIMEOUT:
                if(!g_MFGModeFlag){
                    pwrmgr_system_try_to_poweroff(PWRMGR_WAIT_USBIN_TIMEOUT);
                }
                break;
            case PM_SYS_CMD_SYS_WAKEUP:
                if(!isUSBDataIn()){
                    osTimerStart(data->wait_usb_timer, WAITE_USB_DATA_IN_TIMEOUT);
                }
                break;
            case PM_SYS_CMD_LOW_BATTERY:
                pwrmgr_system_try_to_poweroff(PWRMGR_BATTERY_LOW);
                break;
           case PM_SYS_CMD_RTC_UPDATE_SOC:
               pwrmgr_state_machine_set(PWRMGR_STAT_MACHINE_STOP_MODE,
                   PWRMGR_RTC_UPDATE_SOC);
                break;
            case PM_SYS_CMD_WAIT_VIDEO_TIMEOUT:
                pwrmgr_warning("Wait video output timeout\r\n");
                if(g_MFGModeFlag){
                    /* do a fake connected state*/
                    /* SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_OFF); */
                    SendLedEventToDeal(LED_EVENT_CONNECTED | LED_EVENT_ON);
                }
                else{
                    /* SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_OFF); */
                    SendLedEventToDeal(LED_EVENT_CONNECTING_FAIL | LED_EVENT_ON);
                    pwrmgr_system_try_to_poweroff(PWRMGR_WAIT_VIDEO_TIMEOUT);
                }
                break;
            case PM_SYS_CMD_DP_SYNC_STATE:
                pwrmgr_warning("DP Sync %s\r\n",(PmSysMsg.state == DP_SYNC_DET)
                ? "Detected":"Lost");

                if(PmSysMsg.state == DP_SYNC_DET){
                    osTimerStop(data->wait_video_timer);
                    /* SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_OFF); */
                    SendLedEventToDeal(LED_EVENT_CONNECTED | LED_EVENT_ON);
                }
                else if(PmSysMsg.state == DP_SYNC_LOST){
                    if(isUSBDataIn()){
                        osTimerStart(data->wait_video_timer,
                            WAITE_VIDEO_OUTPUT_TIMEOUT);
                        /* SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_OFF); */
                        SendLedEventToDeal(LED_EVENT_CONNECTING_FAIL | LED_EVENT_ON);
                    }
                }
                break;
            default:
                break;
        }
    }
}

int pwrmgr_system_send_message(PM_SYS_CMD_TYPE cmd, uint8_t state)
{
    struct pwmgr_system_data *data = &ps_data;
    PmSysMsgQueue_t *PmSysMsg = &data->pm_sys_msg_queue;
    PmSysMsg->cmd = cmd;
    PmSysMsg->state = state;
    if(data->PmSysMsgQueueHandle == NULL) return -2;
    if(xQueueSend(data->PmSysMsgQueueHandle, PmSysMsg, portMAX_DELAY) != pdPASS){
        pwrmgr_err("%s fail\n",__func__);
        return -1;
    }
    return 0;
}
#if !USB_HOLE_DET_BY_CCG4
void phone_vbus_det_irq(void)
{
    struct pwmgr_system_data *data = &ps_data;
    PmSysMsgQueue_t *PmSysMsg = &data->pm_sys_msg_queue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    GPIO_PinState PinState;
    if(pcb_id == XA0n){
        PinState = HAL_GPIO_ReadPin(GPIO_PORT_PHONE_DETC, GPIO_PHONE_DETC);
    }
    else if((pcb_id == XC01) || (XC02 == pcb_id)){
        PinState = HAL_GPIO_ReadPin(GPIO_XC_PORT_PHONE_DETC, GPIO_XC_PHONE_DETC);
    }
    else{
        PinState = HAL_GPIO_ReadPin(GPIO_XB_PORT_PHONE_DETC, GPIO_XB_PHONE_DETC);
    }
    if(PinState == GPIO_PIN_SET){
        PmSysMsg->cmd = PM_SYS_CMD_USB_DATA_OUT;
    }
    else{
        PmSysMsg->cmd = PM_SYS_CMD_USB_DATA_IN;
    }
    if(!data->PmSysMsgQueueHandle) return ;
    xQueueSendFromISR( data->PmSysMsgQueueHandle, PmSysMsg, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );

}
#endif
static void WaitUSBPulgInCallback(void const *argument)
{
    pwrmgr_system_send_message(PM_SYS_CMD_NO_USB_TIMEOUT, 0);

}
static void WaitVideoOutCallback(void const *argument)
{
    pwrmgr_system_send_message(PM_SYS_CMD_WAIT_VIDEO_TIMEOUT, 0);

}
static int pwrmgr_system_notify_callback(uint32_t _notify_flag,
    uint32_t _state, void *pdata)
{
    struct pwmgr_system_data *data = (struct pwmgr_system_data *) pdata;
    switch(_notify_flag){
        case PWRMGR_NOTIFY_STOP_STATE:
            if(_state == STOP_ENTER){
                //bsp_pmic_power_enable(V_VDD_1V8, POWER_DISABLE);
                //bsp_pmic_power_enable(V_AUD_3V3, POWER_DISABLE);
                bsp_pmic_power_enable(V_BOOST_5V_EN, POWER_DISABLE);
                bsp_pmic_power_enable(V_USB30_5V_EN,POWER_DISABLE);
                SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_OFF);
                osTimerStop(data->wait_usb_timer);
                osTimerStop(data->wait_video_timer);
            }
            else if(_state == STOP_LEAVE){
                //bsp_pmic_power_enable(V_VDD_1V8, POWER_ENABLE);
                //bsp_pmic_power_enable(V_AUD_3V3, POWER_ENABLE);
                bsp_pmic_power_enable(V_BOOST_5V_EN, POWER_ENABLE);
                bsp_pmic_power_enable(V_USB30_5V_EN,POWER_ENABLE);
                if(!isUSBDataIn()){
                    osTimerStart(data->wait_usb_timer, WAITE_USB_DATA_IN_TIMEOUT);
                }
                else{
                    SendLedEventToDeal(LED_EVENT_CONNECTING | LED_EVENT_ON);
                    osTimerStart(data->wait_video_timer, WAITE_VIDEO_OUTPUT_TIMEOUT);
                }
            }
            break;
    }
    return 0;
}
static int pwrmgr_system_register_notify(void)
{
    PmSysNotifyData.func_name = "pm_sys";
    PmSysNotifyData.data = (void *)&ps_data;
    PmSysNotifyData.callback= pwrmgr_system_notify_callback;
    PmSysNotifyData.notify_flag = PWRMGR_NOTIFY_STOP_STATE;
    PmSysNotifyData.func_level = PWRMGR_FUNC_APP_LEVEL;
    return PWRMGR_register_notify_func(&PmSysNotifyData);
}
int pwrmgr_system_init(void)
{
    struct pwmgr_system_data *data = &ps_data;
    osThreadDef(pm_sys_thread, pwrmgr_system_thread_func,
                osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    data->pwrmgr_system_thread = osThreadCreate(osThread(pm_sys_thread), NULL);
    if(data->pwrmgr_system_thread == NULL){
        pwrmgr_err("Creat pm_sys_thread fail\n");
        return -1;
    }
    osMessageQDef(PmSysMsgQueue, 1, PmSysMsgQueue_t);
    data->PmSysMsgQueueHandle = osMessageCreate(osMessageQ(PmSysMsgQueue), NULL);

    osTimerDef(WaitUSBDateIn, WaitUSBPulgInCallback);
    data->wait_usb_timer = osTimerCreate(osTimer(WaitUSBDateIn), osTimerOnce, NULL);

    osTimerDef(WaitVideoOut, WaitVideoOutCallback);
    data->wait_video_timer = osTimerCreate(osTimer(WaitVideoOut), osTimerOnce, NULL);

    pwrmgr_system_register_notify();

    return 0;
}

