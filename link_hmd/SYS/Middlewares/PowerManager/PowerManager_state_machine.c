#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

//#include "component.h"
#include "Stm32f4xx_hal.h"

#include "PowerManager.h"
#include "PowerManager_power.h"
#include "PowerManager_state_machine.h"
#include "PowerManager_notify_func.h"

#include "htc_usb_cdc_data_service.h"

//#include  "usbd_app.h"

static void pwrmgr_state_machine_thread_run(enum __pwrmgr_stat_machine_mode);

const char *__pwrmgr_stat_machine_mode_string[] = {
    "BOOTING",
    "IDLE",
    "RUNNING",
    "SLEEP",
    "STOP",
    "REBOOT",
    "POWEROFF",
};
const char *__pwrmgr_stat_change_reason_string[] = {
    [PWRMGR_NORMAL] = "NORMAL",
    [PWRMGR_LONG_KEY] = "LONG_KEY",
    [PWRMGR_SHORT_KEY] = "SHORT_KEY",
    [PWRMGR_BATTERY_LOW] = "BATTERY_LOW",
    [PWRMGR_STOP_TIMEOUT] = "STOP_TIMEOUT",
    [PWRMGR_I2C_FAIL] = "I2C_FAIL",
    [PWRMGR_SPI_FAIL] = "SPI_FAIL",
    [PWRMGR_CANTSTOP] = "CANTSTOP",
    [PWRMGR_SENSOR_FAIL] = "SENSOR_FAIL",
    [PWRMGR_CHANGE_BOOT_MODE] = "CHANGE_BOOT_MODE",
    [PWRMGR_FW_UPDATE] = "FW_UPDATE",
    [PWRMGR_MFG] = "MFG",
    [PWRMGR_DFU] = "DFU",
    [PWRMGR_SYSDFU] = "SYSDFU",
    [PWRMGR_USB_DATA_OUT] = "USB_DATA_OUT",
    [PWRMGR_PSENSOR_AWAY] = "PSENSOR_AWAY",
    [PWRMGR_WAIT_USBIN_TIMEOUT] = "WAIT_USBIN_TIMEOUT",
    [PWRMGR_WAIT_VIDEO_TIMEOUT] = "WAIT_VIDEO_TIMEOUT",
    [PWRMGR_RTC_UPDATE_SOC] = "UPDATE_SOC",
    [PWRMGR_STOP_EXIT] = "STOP_EXIT",
    [PWRMGR_PQM_TEST] = "PQM_TEST",
    [PWRMGR_MFG_TEST] = "MFG_TEST",
    [PWRMGR_ENG_TEST] = "ENG_TEST",
};
struct __pwrmgr_state_machine_data {
    uint8_t is_initialized;
    uint8_t is_enabled;
    osMessageQId state_machine_queue;
    enum __pwrmgr_stat_machine_mode current_state_machine_mode;
    enum __pwrmgr_stat_machine_mode last_state_at_suspend_mode_before;
    enum __pwrmgr_stat_machine_mode current_state_going_mode;
    osThreadId pwrmgr_sm_main_thread;
    osSemaphoreId change_sm_semaphore;
    int change_stat_reason;
    int change_stat_result;
};

struct __pwrmgr_state_machine_data __sm_data = { 0 };


int GetBitNum(uint32_t BitMask)
{
    register uint32_t index = BitMask ;
    index = (index-1) & (~index);
    index = (index & 0x55555555) + ((index >> 1) & 0x55555555);
    index = (index & 0x33333333) + ((index >> 2) & 0x33333333);
    index = (index & 0x0F0F0F0F) + ((index >> 4) & 0x0F0F0F0F);
    index = (index & 0xFF) + ((index & 0xFF00) >> 8) +
           ((index & 0xFF0000) >> 16) + ((index & 0xFF000000) >> 24);
    return (int)(index);
}


static int __pwrmgr_sm_enter_idle_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    PWRMGR_SYSTEM_SET_VOLTAGE_SCALING(PWRMGR_VOLTSCALING_120HZ);
    return 0;
}

static int __pwrmgr_sm_enter_running_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    PWRMGR_SYSTEM_SET_VOLTAGE_SCALING(PWRMGR_VOLTSCALING_168HZ);
    return 0;
}

static int __pwrmgr_sm_enter_stop_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System go to STOP mode ###\r\n");
    }
    pwrmgr_info("### System go to STOP mode <%s>###\r\n",
            __pwrmgr_stat_change_reason_string[sm_data->change_stat_reason]);
    sm_data->last_state_at_suspend_mode_before =
                    PWRMGR_STAT_MACHINE_BOOTING_MODE;
    PWRMGR_suspend_func_execute();

    PWRMGR_SYSTEM_POWER_ENTER_STOP_MODE();

    PWRMGR_SYSTEM_POWER_EXIT_STOP_MODE();

    PWRMGR_resume_func_execute();

    pwrmgr_state_machine_thread_run(
            sm_data->last_state_at_suspend_mode_before);

    pwrmgr_info("### System back from STOP mode <bit(%d)>###\r\n",
                    GetBitNum(wakeup_source));
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System back from STOP mode ###\r\n");
    }
    return 0;
}

static int __pwrmgr_sm_enter_sleep_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System go to SLEEP mode ###\r\n");
    }
    pwrmgr_info("### System go to SLEEP mode <%s>###\r\n",
            __pwrmgr_stat_change_reason_string[sm_data->change_stat_reason]);
    sm_data->last_state_at_suspend_mode_before =
                    PWRMGR_STAT_MACHINE_BOOTING_MODE;

    PWRMGR_suspend_func_execute();

    PWRMGR_SYSTEM_POWER_SLEEP_MODE();

    PWRMGR_SYSTEM_POWER_EXIT_SLEEP_MODE();

    PWRMGR_resume_func_execute();

    pwrmgr_state_machine_thread_run(
            sm_data->last_state_at_suspend_mode_before);

    pwrmgr_info("### System back from SLEEP mode <bit(%d)>###\r\n",
                    GetBitNum(wakeup_source));
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System back from SLEEP mode ###\r\n");
    }
    return 0;

}
static int __pwrmgr_sm_enter_reboot_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System Going to RESET!! ###\r\n");
    }
    pwrmgr_info("### System Going to RESET!! <%s>###\r\n",
            __pwrmgr_stat_change_reason_string[sm_data->change_stat_reason]);
    sm_data->last_state_at_suspend_mode_before =
                    sm_data->current_state_machine_mode;
    PWRMGR_SYSTEM_POWER_RESET(sm_data->change_stat_reason);

    /* Normaly we can't goto here , if power off fialed it will goto normal*/
    pwrmgr_info("*** reboot failed ***\r\n");
    pwrmgr_state_machine_thread_run(
            sm_data->last_state_at_suspend_mode_before);
    return 0;
}

static int __pwrmgr_sm_enter_poweroff_mode(
            struct __pwrmgr_state_machine_data *sm_data)
{
    if(sm_data->change_stat_reason == PWRMGR_MFG_TEST){
        usb_cdc_printf("### System Going to POWER OFF!! ###\r\n");
    }
    pwrmgr_info("### System Going to POWER OFF!! <%s>###\r\n",
            __pwrmgr_stat_change_reason_string[sm_data->change_stat_reason]);
    sm_data->last_state_at_suspend_mode_before =
                    sm_data->current_state_machine_mode;
    PWRMGR_SYSTEM_POWER_POWEROFF(sm_data->change_stat_reason);

    /* Normaly we can't goto here , if power off fialed it will goto normal*/
    pwrmgr_state_machine_thread_run(
            sm_data->last_state_at_suspend_mode_before);
    return 0;
}
void __pwrmgr_state_machine_switch(struct __pwrmgr_state_machine_data *sm_data,
                                            enum __pwrmgr_stat_machine_mode new_state)
{
    switch (new_state) {
    case PWRMGR_STAT_MACHINE_BOOTING_MODE:
    case PWRMGR_STAT_MACHINE_IDLE_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_idle_mode(sm_data);
        break;
    case PWRMGR_STAT_MACHINE_RUNNING_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_running_mode(sm_data);
        break;
    case PWRMGR_STAT_MACHINE_SLEEP_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_sleep_mode(sm_data);
        break;
    case PWRMGR_STAT_MACHINE_STOP_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_stop_mode(sm_data);
        break;
    case PWRMGR_STAT_MACHINE_REBOOT_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_reboot_mode(sm_data);
        break;
    case PWRMGR_STAT_MACHINE_POWEROFF_MODE:
        sm_data->change_stat_result =
            __pwrmgr_sm_enter_poweroff_mode(sm_data);
        break;
    default:
        sm_data->change_stat_result = -9;
        break;
    }

}
void __pwrmgr_state_machine_main_func(void const * argument)
{
    osEvent notify_data;
    enum __pwrmgr_stat_machine_mode new_state;
    struct __pwrmgr_state_machine_data *sm_data = &__sm_data;
    do {
        notify_data = osMessageGet(sm_data->state_machine_queue,
                            osWaitForever);
        osSemaphoreWait(sm_data->change_sm_semaphore, osWaitForever);
        new_state = (enum __pwrmgr_stat_machine_mode)
                        notify_data.value.signals;
        pwrmgr_info("get new state: %s\n",
            __pwrmgr_stat_machine_mode_string[new_state]);
        __pwrmgr_state_machine_switch(sm_data, new_state);

        if (sm_data->change_stat_result != 0) {
            pwrmgr_info("%s: change %s mode failed, old: %s!!\n",
            __func__,
            __pwrmgr_stat_machine_mode_string[
                    (enum __pwrmgr_stat_machine_mode)
                    notify_data.value.signals],
            __pwrmgr_stat_machine_mode_string[new_state]);
        } else {
            sm_data->current_state_machine_mode = new_state;
            sm_data->current_state_going_mode =
                        PWRMGR_STAT_MACHINE_BOOTING_MODE;
        }
        osSemaphoreRelease(sm_data->change_sm_semaphore);
    } while (1);
}

static void pwrmgr_state_machine_thread_run(
    enum __pwrmgr_stat_machine_mode new_state)
{
    struct __pwrmgr_state_machine_data *sm_data = &__sm_data;
    osMessagePut(sm_data->state_machine_queue, (uint32_t)new_state,
                        portTICK_PERIOD_MS * 500);
}

int pwrmgr_state_machine_set(enum __pwrmgr_stat_machine_mode new_state,
                                    int reason)
{
    struct __pwrmgr_state_machine_data *sm_data = &__sm_data;
    if (sm_data->is_initialized == 0) {
        pwrmgr_err("%s: function uninitialized\n", __func__);
        return -1;
    }
    if (new_state < PWRMGR_STAT_MACHINE_MODE_LAST) {
        sm_data->current_state_going_mode = new_state;
        if (new_state == sm_data->current_state_machine_mode) {
            pwrmgr_warning("already config state machine as: %s\n",
                __pwrmgr_stat_machine_mode_string[new_state]);
            return -2;
        }
        sm_data->change_stat_reason = reason;
        pwrmgr_state_machine_thread_run(new_state);
        pwrmgr_info("get state change result: %d\n",
                        sm_data->change_stat_result);
        return sm_data->change_stat_result;
    }else
        pwrmgr_warning("%s: unsupport machine state!\n", __func__);
    return -1;
}
enum __pwrmgr_stat_machine_mode pwrmgr_state_going_get(void)
{
    struct __pwrmgr_state_machine_data *sm_data = &__sm_data;
    return sm_data->current_state_going_mode;
}

void pwrmgr_state_machine_get(enum __pwrmgr_stat_machine_mode *mode)
{
    struct __pwrmgr_state_machine_data *sm_data = &__sm_data;
    if (sm_data->is_initialized == 0) {
        pwrmgr_warning("%s: function uninitialized\n", __func__);
        return;
    }
    osSemaphoreWait(sm_data->change_sm_semaphore, osWaitForever);
    *mode = sm_data->current_state_machine_mode;
    osSemaphoreRelease(sm_data->change_sm_semaphore);
}

static int __pwrmgr_create_state_machine_main_queue(
                struct __pwrmgr_state_machine_data *data)
{
    osMessageQDef(sm_queue_t, 8, uint32_t);
    data->state_machine_queue = osMessageCreate(osMessageQ(sm_queue_t),
                    data->pwrmgr_sm_main_thread);
    if (data->state_machine_queue == NULL)
        return -1;

    return 0;
}

static int __pwrmgr_create_state_machine_main_semaphore(
                struct __pwrmgr_state_machine_data *data)
{
    osSemaphoreDef(sm_change_sema);
    data->change_sm_semaphore = osSemaphoreCreate(
                        osSemaphore(sm_change_sema), 1);
    if (data->change_sm_semaphore == NULL)
        return -1;

    return 0;
}


int __pwrmgr_create_state_machine_main_thread(
                struct __pwrmgr_state_machine_data *sm_data)
{
    osThreadDef(sm_main_thread, __pwrmgr_state_machine_main_func,
            osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 4);
    sm_data->pwrmgr_sm_main_thread =
            osThreadCreate(osThread(sm_main_thread), sm_data);
    if (sm_data->pwrmgr_sm_main_thread == NULL)
        return -1;

    return 0;
}

int pwrmgr_state_machine_init(void)
{
    struct __pwrmgr_state_machine_data *data = &__sm_data;
    if (data->is_initialized != 0) {
        pwrmgr_info("state machine already initialized, exit\n");
        return 0;
    }

    if (__pwrmgr_create_state_machine_main_queue(data) != NULL)
        pwrmgr_warning("create state machine queue failed\n");

    if (__pwrmgr_create_state_machine_main_semaphore(data) != 0)
        pwrmgr_warning("create state machine change mutex failed\n");

    if (__pwrmgr_create_state_machine_main_thread(data) != 0)
        pwrmgr_warning("create state machine thread failed\n");

    data->is_initialized = 1;
    pwrmgr_info("%s: done\n", __func__);
    return 0;
}

