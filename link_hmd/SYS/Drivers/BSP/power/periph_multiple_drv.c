#include "cmsis_os.h"
#include "gpio.h"
#include "periph_multiple_drv.h"
#include "x_pmic.h"
#include "PowerManager_notify_func.h"
#include "gpio_exp.h"

typedef struct {
    uint8_t cmd;
    uint8_t data;
}periph_multiple_msg;

struct periph_multiple{
    osThreadId multipleTaskHandle;
    osMessageQId multipleMaseage;
    periph_multiple_msg mesage;
    int phone_state;
};

typedef enum{
    USB_HUB_OAC,
}periph_multiple_msg_t;

static struct periph_multiple multiple_data = {0};
extern pcbid_t pcb_id;

void usb_hub_reset(void)
{
    if(XA0n == pcb_id){
        HAL_GPIO_WritePin(GPIO_PORT_USB_HUB_RST, GPIO_USB_HUB_RST, GPIO_PIN_RESET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIO_PORT_USB_HUB_RST, GPIO_USB_HUB_RST, GPIO_PIN_SET);
    }
    else{
        ioexp_gpio_set_value(IOEXP_REG_DIR, IOEXP_II_USBHUB_RST_N,
                    IOEXP_GPIO_OUTPUT);
        ioexp_gpio_set_value(IOEXP_REG_OUT,
                    IOEXP_II_USBHUB_RST_N, 0);
        osDelay(100);
        ioexp_gpio_set_value(IOEXP_REG_OUT,
                    IOEXP_II_USBHUB_RST_N, 1);
    }
}
void usb_hub_oac_det_irq(void)
{
    struct periph_multiple *data = &multiple_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    data->mesage.cmd = USB_HUB_OAC;
    if(!data->multipleMaseage) return ;
    xQueueSendFromISR( data->multipleMaseage, &data->mesage, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken );

}
void PeriphMultipleTask(void const * argument)
{
    int temp_state = 0;
    static int last_state;
    periph_multiple_msg mesage;
    struct periph_multiple *data = &multiple_data;

    usb_hub_reset();
    for(;;){
        if(!data->multipleMaseage) continue;
        xQueueReceive( data->multipleMaseage, &mesage,  portMAX_DELAY );
        switch(mesage.cmd){
            case USB_HUB_OAC:
                /* For debounce */
                osDelay(20);
                if(pcb_id == XA0n){
                    temp_state = HAL_GPIO_ReadPin(GPIO_PORT_USB_A_OC, GPIO_USB_A_OC);
                }
                else{/*XB02, XC01*/
                    temp_state = HAL_GPIO_ReadPin(GPIO_XB_PORT_USB_A_OC, GPIO_XB_USB_A_OC);
                }
                if(last_state == temp_state) break;
                if(temp_state){
                    pm_warning("USB Type A AOC disappear!\n");
                }
                else{
                    pm_err("USB Type A over current!!\n");
                    //1 TODO: usb hub over current .
                }
                last_state = temp_state;
                break;
        }

    }
}
void periph_multiple_init(void)
{
    struct periph_multiple *data = &multiple_data;

    osThreadDef(phoneDetTask, PeriphMultipleTask, osPriorityNormal,
    0,configMINIMAL_STACK_SIZE*3);
    data->multipleTaskHandle = osThreadCreate(osThread(phoneDetTask), NULL);
    if(!data->multipleTaskHandle){
        pwrmgr_err("phoneDetTask create failed \n");
    }
    osMessageQDef(phoneMsgQueue, 1, periph_multiple_msg);
    data->multipleMaseage = osMessageCreate(osMessageQ(phoneMsgQueue), NULL);

}

