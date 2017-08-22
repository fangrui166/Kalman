#ifndef __DP2HDMI_ANX7737_H__
#define __DP2HDMI_ANX7737_H__
#ifdef __cplusplus
 extern "C" {
#endif


void dp2h_power_en_anx7737(int enable);
int disp_d2h_getid_anx7737(uint8_t *id, int size);

#ifdef __cplusplus
}
#endif
#endif /* __DP2HDMI_ANX7737_H__ */
