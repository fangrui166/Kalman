#ifndef __HTC_USB_CDC_SERVICE_H__
#define __HTC_USB_CDC_SERVICE_H__
#ifdef __cplusplus
 extern "C" {
#endif

enum
{
    USB_CDC_TX_IDLE = 0,
    USB_CDC_TX_BUSY
};

enum htc_cdc_service_state {
	HTC_CDC_SRV_UNINITIALIZED = 0,
	HTC_CDC_SRV_RUNNING,
	HTC_CDC_SRV_SUSPEND,
};

#define USB_CDC_SRV_HLOG_ENABLE
#ifdef USB_CDC_SRV_HLOG_ENABLE
#define ucdc_srv_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define ucdc_srv_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define ucdc_srv_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define ucdc_srv_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#define ucdc_srv_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_USB, fmt, ##__VA_ARGS__)
#else /* USB_CDC_SRV_HLOG_ENABLE */
#define ucdc_srv_emerg(fmt, ...) \
	printf("[USB][EMR] :" fmt, ##__VA_ARGS__)
#define ucdc_srv_err(fmt, ...) \
	printf("[USB][ERR] :" fmt, ##__VA_ARGS__)
#define ucdc_srv_warning(fmt, ...) \
	printf("[USB][WARN]:" fmt, ##__VA_ARGS__)
#define ucdc_srv_info(fmt, ...) \
	printf("[USB][INFO]:" fmt, ##__VA_ARGS__)
#define ucdc_srv_debug(fmt, ...) \
	printf("[USB][DBG] :" fmt, ##__VA_ARGS__)
#endif /* USB_CDC_SRV_HLOG_ENABLE */

enum {
	APP_ID_SYSPROP = 'p',
	APP_ID_SYSCMD = 'c',
	APP_ID_SHELLCMD = 'E',
	APP_ID_LOOPBACK = 'B',
};

int usb_cdc_service_init(void);
int usb_cdc_receive_data(uint8_t *, uint8_t);
int usb_cdc_transmit_data(uint8_t *, uint8_t);
int usb_cdc_printf(const char *, ...);
int set_htc_cdc_srv_control(enum htc_cdc_service_state);
enum htc_cdc_service_state get_htc_cdc_srv_control(void);
#ifdef __cplusplus
  }
#endif
#endif /* __HTC_USB_CDC_SERVICE_H__ */