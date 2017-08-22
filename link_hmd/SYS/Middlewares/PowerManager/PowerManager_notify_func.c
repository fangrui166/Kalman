#include "FreeRTOS.h"
#include "timers.h"
#include "list.h"
#include "Portable.h"

#include "cmsis_os.h"

#include "string.h"
#include "stdio.h"
//#include "component.h"

#include "PowerManager.h"
#include "PowerManager_notify_func.h"

extern void watchdog_refresh(void);
struct __pwrmgr_notify_linkedlist_data {
    struct __pwrmgr_notify_linkedlist_data *next;
    struct __pwrmgr_notify_linkedlist_data *prev;
    struct pwrmgr_notify_func_data *func_data;
    int func_state;
};

struct __pwrmgr_notify_main_data {
    unsigned char initialized;
    struct  __pwrmgr_notify_linkedlist_data *linkedlist;
    unsigned int ll_num;
};

static struct __pwrmgr_notify_main_data __sr_data = { 0 };

static int __pwrmgr_sr_linkedlist_init(
                struct __pwrmgr_notify_main_data *sr_data)
{
    if (sr_data->initialized == 0) {
        sr_data->linkedlist = (struct __pwrmgr_notify_linkedlist_data *)
            pvPortMalloc(sizeof(struct __pwrmgr_notify_main_data));
        if (sr_data->linkedlist == NULL) {
            pwrmgr_err("malloc linked list head data failed\n");
            return -1;
        }
        sr_data->linkedlist->next = sr_data->linkedlist;
        sr_data->linkedlist->prev = sr_data->linkedlist;
        sr_data->linkedlist->func_data = NULL;
        sr_data->linkedlist->func_state = -1;
        sr_data->initialized = 1;
    }
    return 0;

}

static struct __pwrmgr_notify_linkedlist_data *__pwrmgr_sr_linkedlist_get_item(
                    const struct pwrmgr_notify_func_data *data)
{
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;

    for (curr = head->next; curr != head; curr = curr->next) {
        if (curr->func_data == data)
            return curr;
    }
    return NULL;
}

static struct __pwrmgr_notify_linkedlist_data *__pwrmgr_get_insert_node(
                    const struct pwrmgr_notify_func_data *data)
{
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;

    for (curr = head->next; curr != head; curr = curr->next) {

        if (curr->func_data->func_level < data->func_level){
            if((curr->next->func_data == NULL) ||
            (curr->next->func_data->func_level >= data->func_level)){
                return curr;
            }
        }

    }
    return head;
}

static int __pwrmgr_sr_linkedlist_add_item_tail(
                    struct pwrmgr_notify_func_data *data)
{
    struct __pwrmgr_notify_linkedlist_data *tmp_data;
    struct __pwrmgr_notify_linkedlist_data *head;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;

    if (__pwrmgr_sr_linkedlist_get_item(data) != NULL)
        return -2;

    tmp_data = pvPortMalloc(sizeof(struct __pwrmgr_notify_linkedlist_data));
    if (tmp_data == NULL)
        return -1;

    tmp_data->func_data = data;
    tmp_data->func_state = -1;

    head = __pwrmgr_get_insert_node(data);

    head->next->prev = tmp_data;
    tmp_data->next = head->next;
    tmp_data->prev = head;
    head->next = tmp_data;

    main_data->ll_num++;

    return 0;
}


static int __pwrmgr_sr_linkedlist_del_item(
                const struct pwrmgr_notify_func_data *data)
{
    struct __pwrmgr_notify_linkedlist_data *del_item;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;

    del_item = __pwrmgr_sr_linkedlist_get_item(data);
    if (del_item == NULL)
        return -1;

    del_item->prev->next = del_item->next;
    del_item->next->prev = del_item->prev;

    vPortFree(del_item);
    main_data->ll_num--;
    return 0;
}
int PWRMGR_SendNotify(uint32_t _notify_flag, uint32_t _state,
                                PWRMGR_NOTIFY_LEVEL_ORDER lever_order)
{
    int ret;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;
    struct pwrmgr_notify_func_data *callback_func_data;

    if (main_data->ll_num == 0)
        return 0;
    for (curr = (lever_order ? head->next : head->prev); curr != head;
        curr = (lever_order ? curr->next : curr->prev) ){
        callback_func_data = curr->func_data;
        if ((callback_func_data->callback == NULL) ||
            (curr->func_state == _state)){
            continue;
        }
        if(callback_func_data->notify_flag & _notify_flag){
            if (PWRMGR_NOTIFY_NEED_ACK(_notify_flag)){
                if (callback_func_data->notify_type == NOTIFY_AFTER_ACK)
                    continue;
                callback_func_data->notify_flag_need_ack |= _notify_flag;
            }
            ret = callback_func_data->callback(_notify_flag, _state, callback_func_data->data);
            if (ret != 0) {
                pwrmgr_err("PWRMGR_SendNotify: %s failed (%3d)\n",
                        callback_func_data->func_name, ret);
                continue;
            }
            curr->func_state = _state;
            watchdog_refresh();
            pwrmgr_info("PWRMGR_SendNotify\t <%d>: %s\r\n",
                _state, callback_func_data->func_name);
        }
    }

    return 0;

}

void PWRMGR_dump_notify_func(void)
{
    struct __pwrmgr_notify_linkedlist_data *curr;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    for (curr = head->next; curr != head; curr = curr->next)
        pwrmgr_info("func: %s(0x%8.8X)\n\tcallback: 0x%8.8X\n"
            "\tdata:    0x%8.8X\n"
            "\tlevel:   %d\n\tstate:   %d\n",
                curr->func_data->func_name,
                curr->func_data,
                curr->func_data->callback,
                curr->func_data->data,
                curr->func_data->func_level,
                curr->func_state);
}

int PWRMGR_register_notify_func(struct pwrmgr_notify_func_data *data)
{
    int ret;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    __pwrmgr_sr_linkedlist_init(main_data);

    if (data->func_name == NULL)
        return -1;

    if (__pwrmgr_sr_linkedlist_get_item(data) != NULL)
        return -2;

    ret = __pwrmgr_sr_linkedlist_add_item_tail(data);
    pwrmgr_info("register %s notify func %s\n", data->func_name,
                        (ret)? "failed" : "done");
    return ret;
}

int PWRMGR_unregister_notify_func(struct pwrmgr_notify_func_data *data)
{
    int ret;
    ret = __pwrmgr_sr_linkedlist_del_item(data);
    pwrmgr_info("unregister %s notify func %s\n", data->func_name,
                        (ret)? "failed" : "done");
    return ret;
}
static int PWRMGR_CheckNotifyAck (PWRMGR_NOTIFY_FLAG _notify_flag)
{
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;
    struct pwrmgr_notify_func_data *callback_func_data;

    if (main_data->ll_num == 0)
        return -1;


    if (!PWRMGR_NOTIFY_NEED_ACK (_notify_flag))
        return 0;

    for (curr = head->next; curr != head; curr = curr->next) {
        callback_func_data = curr->func_data;
        if ((callback_func_data->notify_flag & _notify_flag) &&
            (callback_func_data->notify_flag_need_ack & _notify_flag)){
            return -2;
        }
    }

    return 0;
}
int PWRMGR_SendNotifyAck (struct pwrmgr_notify_func_data *data)
{
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;
    //struct pwrmgr_notify_func_data *callback_func_data;

    if (main_data->ll_num == 0)
        return -1;

    if (!data || !data->callback || !IS_PWRMGR_NOTIFY (data->notify_flag))
        return -1;
    for (curr = head->next; curr != head; curr = curr->next) {
        if(data == curr->func_data){
            data->notify_flag_need_ack &= ~(data->notify_flag);
        }
    }
    return 0;
}
static int PWRMGR_SendNotifyAfterAck (uint32_t _notify_flag, uint32_t _state)
{
    int ret;
    struct __pwrmgr_notify_main_data *main_data = &__sr_data;
    struct __pwrmgr_notify_linkedlist_data *head = main_data->linkedlist;
    struct __pwrmgr_notify_linkedlist_data *curr;
    struct pwrmgr_notify_func_data *callback_func_data;

    if (main_data->ll_num == 0)
        return -1;

    for (curr = head->next; curr != head; curr = curr->next) {
        callback_func_data = curr->func_data;
        if (callback_func_data->notify_flag & _notify_flag &&
            callback_func_data->notify_type == NOTIFY_AFTER_ACK &&
            PWRMGR_NOTIFY_NEED_ACK(_notify_flag)){

            callback_func_data->notify_flag_need_ack |= _notify_flag;
            ret = callback_func_data->callback (_notify_flag, _state, callback_func_data->data);
            if (ret != 0)
                pwrmgr_err("PWRMGR_SendNotifyAfterAck fail : %X\n", ret);
        }
    }

    return ret;
}
static int PWRMGR_PreparePowerOff (PWRMGR_NOTIFY_FLAG _notify_flag, PWRMGR_POWEROFF_TYPE _type)
{
    int ret = 0;
    uint8_t i=0;
    uint8_t wait_ack_time = 0;
    PWRMGR_SendNotify(_notify_flag, _type, LEVER_DESCEND_ORDER);
    wait_ack_time = ((_type != PWRMGR_NORMAL) ?
        PWRMGR_POWER_OFF_ABNORMAL_TIMEOUT : PWRMGR_POWER_OFF_NORMAL_TIMEOUT);

    for (i = 0; i < wait_ack_time; ++i){
        if ((ret = PWRMGR_CheckNotifyAck(_notify_flag)) == 0 ) break;
        osDelay(1000);
    }
    if( i >= wait_ack_time){
        pwrmgr_err("wait ack timeout\n");
    }
    PWRMGR_SendNotifyAfterAck (_notify_flag, _type);
    for (i = 0; i < wait_ack_time; ++i){
        if ((ret = PWRMGR_CheckNotifyAck(_notify_flag)) == 0 ) break;
        osDelay(1000);
    }
    pwrmgr_info("%s done\n",__func__);
    return ret;
}
int PWRMGR_suspend_func_execute(void)
{
    return PWRMGR_SendNotify(PWRMGR_NOTIFY_STOP_STATE, STOP_ENTER, LEVER_DESCEND_ORDER);
}

int PWRMGR_resume_func_execute(void)
{
    return PWRMGR_SendNotify(PWRMGR_NOTIFY_STOP_STATE, STOP_LEAVE, LEVER_ASCEND_ORDER);
}
int PWRMGR_reboot_func_execute(PWRMGR_POWEROFF_TYPE _type)
{
    return PWRMGR_PreparePowerOff(PWRMGR_NOTIFY_SYSTEM_RESET, _type);
}
int PWRMGR_shutdown_func_execute(PWRMGR_POWEROFF_TYPE _type)
{
    return PWRMGR_PreparePowerOff(PWRMGR_NOTIFY_POWER_OFF, _type);
}
