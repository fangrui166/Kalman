#ifndef __HTC_3DOF_TRANSFER_SERVICE_H__
#define __HTC_3DOF_TRANSFER_SERVICE_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "hlog_api.h"
#include "component.h"

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

enum htc_3dof_tran_srv_state {
	DOF_TRAN_SRV_UNINITIALIZED = 0,
	DOF_TRAN_SRV_RUNNING,
	DOF_TRAN_SRV_SUSPEND,
};

#define HTC_3DOF_TRANSFER_SERVICE_HLOG_ENABLE

#ifdef HTC_3DOF_TRANSFER_SERVICE_HLOG_ENABLE
#define htc_3dof_tran_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_3dof_tran_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_3dof_tran_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_3dof_tran_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define htc_3dof_tran_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#else /* HTC_3DOF_TRANSFER_SERVICE_HLOG_ENABLE */
#define htc_3dof_tran_emerg(fmt, ...) \
	printf("[3DOF_T][EMR] :" fmt, ##__VA_ARGS__)
#define htc_3dof_tran_err(fmt, ...) \
	printf("[3DOF_T][ERR] :" fmt, ##__VA_ARGS__)
#define htc_3dof_tran_warning(fmt, ...) \
	printf("[3DOF_T][WARN]:" fmt, ##__VA_ARGS__)
#define htc_3dof_tran_info(fmt, ...) \
	printf("[3DOF_T][INFO]:" fmt, ##__VA_ARGS__)
#define htc_3dof_tran_debug(fmt, ...) \
	printf("[3DOF_T][DBG] :" fmt, ##__VA_ARGS__)
#endif /* HTC_3DOF_TRANSFER_SERVICE_HLOG_ENABLE */

#define HTC_3DOF_MAX_PACKET_SIZE	64
#define HTC_3DOF_MAX_SENSOR_DATA_SIZE	60

enum htc_3dof_system_state_flag {
	HTC_3DOF_SYSTEM_STATE_P_SENSOR_FLAG = 0,
};

int htc_3dof_transfer_service_initial(void);
int htc_3dof_tran_service_send_data(uint8_t *, uint32_t);

void htc_3dof_update_system_state_flag(
	enum htc_3dof_system_state_flag flag, uint8_t );


uint8_t *htc_3dof_get_recv_buffer(void);
void htc_3dof_recv_data_is_comming(uint8_t *, uint32_t);


void htc_3dof_tran_service_get_work_task_en(uint8_t *);
int htc_3dof_tran_service_set_work_task_en(uint8_t);


void htc_3dof_update_flashlog_4_dump();
void htc_3dof_update_vsync_status(uint8_t);
void htc_3dof_update_display_status(uint8_t);
void htc_3dof_update_psensor_key(uint8_t);
void htc_3dof_update_battery_state(uint8_t, uint8_t, uint8_t, uint8_t,
							uint8_t, uint8_t);

int htc_3dof_tran_service_set_send_work_task_en(uint8_t);

#ifdef __cplusplus
}
#endif

#endif /* __HTC_3DOF_TRANSFER_SERVICE_H__ */
