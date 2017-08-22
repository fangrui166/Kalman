#ifndef _CCGX_H
#define _CCGX_H

#ifdef __cplusplus
extern "C" {
#endif
#include "hlog_api.h"
#include "cmsis_os.h"

#define _HLOG
#ifdef _HLOG
#define ccgx_pr_DEBUG(fmt, args...)   hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_CCGX, fmt, ##args)
#define ccgx_pr_INFO(fmt, args...)    hlog_printf(HLOG_LVL_INFO, HLOG_TAG_CCGX, fmt, ##args)
#define ccgx_pr_WARN(fmt, args...)    hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_CCGX, fmt, ##args)
#define ccgx_pr_ERROR(fmt, args...)   hlog_printf(HLOG_LVL_ERR, HLOG_TAG_CCGX, fmt, ##args)
#else

#define ccgx_pr_DEBUG(fmt, args...) printf("CCGX DEBUG in %s : " fmt, __func__, ##args)
#define ccgx_pr_INFO(fmt, args...) printf("CCGX INFO in %s : " fmt, __func__, ##args)
#define ccgx_pr_WARN(fmt, args...) printf("CCGX WARN in %s : " fmt, __func__, ##args)
#define ccgx_pr_ERROR(fmt, args...) printf("CCGX ERROR in %s : " fmt, __func__, ##args)
#endif

#define TRACE()         ccgx_pr_INFO("\n")

#define USB_HOLE_DET_BY_CCG4                0

int ccgx_init(void);
void ccgx_irq_handler(void);

void ccgx_test(void);
void ccg4_reset(void);
typedef enum{
		CHARGE_PORT_CONNECT=(0x09<<4)&0xf0,
		CHARGE_PORT_DISCONNECT=(0x00<<4)&0xf0,
		CHARGE_PORT_ERROR=(0x0f<<4)&0xf0,
		DATA_PORT_CONNECT=0x09&0x0f,
		DATA_PORT_DISCONNECT=0x00&0x0f,
		DATA_PORT_ERROR=0x0f&0x0f,
		PORT_ERROR=0xff
}enumPortStatus;

typedef enum{
		DATA_PORT=0,
		CHARGE_PORT,
		ERROR_PORT
}enumPortID;

typedef struct{
	osThreadId 			ccgx_th;
	osMessageQId 		ccgx_msg_handler;
	osTimerId 			ccgx_data_ovp_timer;
	osTimerDef_t 		ccgx_data_ovp_timer_callback;
	osTimerId 			ccgx_charge_ovp_timer;
	osTimerDef_t 		ccgx_charge_ovp_timer_callback;
	void* 				data;
	uint8_t 			current_port_status;
	uint8_t 			is_initialized;
}ccgx_device;

enumPortStatus get_port_status(enumPortID);
#ifdef __cplusplus
}
#endif
#endif
