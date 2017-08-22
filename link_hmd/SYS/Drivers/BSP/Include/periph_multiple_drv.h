#ifndef __PHONE_CHARGE_H__
#define __PHONE_CHARGE_H__

typedef int (*PHONE_VBUS_DET)(int);
typedef enum{
    PHONE_DET_NONE,
    PHONE_DET_IN,
    PHONE_DET_OUT
}PHONE_DET_STATE_t;
typedef enum{
    LIMIT_CURRENT_1000MA,
    LIMIT_CURRENT_1250MA,
    LIMIT_CURRENT_1500MA,
    LIMIT_CURRENT_2000MA,
}phone_charge_current_t;
void periph_multiple_init(void);
int phone_register_vbus_det_notify(int (*callback)(int));
void phone_charge_enable(uint8_t enable);

#endif

