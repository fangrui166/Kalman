#ifndef __DP2HDMI_PS176_H__
#define __DP2HDMI_PS176_H__ 
#ifdef __cplusplus
 extern "C" {
#endif


void dp2h_power_en_ps176(int enable);
void dp2h_gpio_config_ps176(void);
int disp_d2h_getid_ps176(uint8_t *id, int size);

#ifdef __cplusplus
}
#endif
#endif /* __DP2HDMI_PS176_H__ */
