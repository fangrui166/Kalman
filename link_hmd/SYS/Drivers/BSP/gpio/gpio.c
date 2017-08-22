#include "gpio.h"
#include "proximity_task.h"
#include "sensor_task.h"
#include "display_drv.h"
#include "key_drv.h"
#include "PowerManager_power.h"
#include "command.h"
#include "htc_audio_path_service.h"

extern pcbid_t pcb_id;
static void MX_XA_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIOA pins*/
    GPIO_InitStruct.Pin = GPIO_PHONE_CHG_EN|GPIO_V_DP_EN|GPIO_AUD_ALC4040_GPIO7|
                        GPIO_AUD_ALC4040_GPIO6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PHONE_DETC|GPIO_CHARGE_DETC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_BAT_ALRT|GPIO_USB_A_OC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIOB pins*/
    GPIO_InitStruct.Pin = GPIO_PMIC_EN|GPIO_IO_EXP_RST;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_GYRO_INT|GPIO_G_S_INT|GPIO_PS_INT|GPIO_CCG4_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PS_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIOC pins*/
    GPIO_InitStruct.Pin = GPIO_V_VDDI_L_EN|GPIO_V_VDDI_R_EN|GPIO_GREE_LED|
                        GPIO_V_HDMI_EN|GPIO_USB_HUB_RST|GPIO_V_BOOST_5V_EN|
                        GPIO_AUD_ALC4040_GPIO_ALO_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_ECOMP_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_CHG_INT|GPIO_IO_EXP_INT|
                            GPIO_AUD_CDC_ALC5665_IRQ;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_POWER_KEY;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure GPIO Default value */
    HAL_GPIO_WritePin(GPIOA, GPIO_PHONE_CHG_EN|GPIO_V_DP_EN|GPIO_AUD_ALC4040_GPIO7|
                            GPIO_AUD_ALC4040_GPIO6,
                            GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_IO_EXP_RST, GPIO_PIN_SET);

    /* Disable PMIC NCP6924 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PMIC_EN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOC, GPIO_V_VDDI_L_EN|GPIO_V_VDDI_R_EN|GPIO_GREE_LED|
                            GPIO_V_BOOST_5V_EN|GPIO_AUD_ALC4040_GPIO_ALO_1|GPIO_V_HDMI_EN,
                            GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOC, GPIO_USB_HUB_RST, GPIO_PIN_SET);

}
static void MX_XB_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIOA pins */
    GPIO_InitStruct.Pin = GPIO_XB_L_GI_FLM_1_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_IO_EXP_RST | GPIO_XB_PMIC_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_PHONE_DETC/*|GPIO_XB_INT_H_R_LS*/|GPIO_XB_POWER_KEY\
                        /*|GPIO_XB_HUB_VBUS_DET*/;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_INT_H_R_LS;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;	//GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_BAT_ALRT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIOB pins*/
    GPIO_InitStruct.Pin = GPIO_XB_CC_FLAG_OVP_PHONE;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_USB_A_OC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CC_FLAG_OVP_CHARG;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_G_S_INT|GPIO_XB_CCG4_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIOC pins*/
    GPIO_InitStruct.Pin = GPIO_XB_IO_EXP_II_RST|GPIO_XB_L_AMO_LCD_RST|
                        GPIO_XB_H2D_RST_N;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_INT_H_L_LS;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;	//GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CHG_INT|GPIO_XB_IO_EXP_INT\
                        /*|GPIO_XB_CC_FLAG_OVP_CHARG*/;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CHARGE_DETC
                        /*|GPIO_XB_CC_FLAG_OVP_CHARG*/;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_H2D_INT_0|GPIO_XB_AUD_CDC_ALC5665_IRQ;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_ERR_FG_L_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_R_GI_FLM_1_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Configure GPIO Default value */
    HAL_GPIO_WritePin(GPIOA, GPIO_XB_IO_EXP_RST|GPIO_XB_PMIC_EN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOC, GPIO_XB_IO_EXP_II_RST|GPIO_XB_L_AMO_LCD_RST|GPIO_XB_H2D_RST_N,
                        GPIO_PIN_SET);


}
static void MX_XC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIOA pins */
    GPIO_InitStruct.Pin = GPIO_XB_L_GI_FLM_1_IO|GPIO_XC_R_GI_FLM_1_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_IO_EXP_RST | GPIO_XB_PMIC_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XC_CC_FLAG_OVP_PHONE;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_POWER_KEY;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_BAT_ALRT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIOB pins*/

    GPIO_InitStruct.Pin = GPIO_XB_USB_A_OC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_G_S_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_G_S_INT|GPIO_XB_CCG4_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIOC pins*/
    GPIO_InitStruct.Pin = GPIO_XB_IO_EXP_II_RST|GPIO_XB_L_AMO_LCD_RST|
                        GPIO_XB_H2D_RST_N;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XC_PHONE_DETC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CHG_INT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CHARGE_DETC;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_H2D_INT_0|GPIO_XB_AUD_CDC_ALC5665_IRQ;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_ERR_FG_L_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    /* Configure GPIO Default value */
    HAL_GPIO_WritePin(GPIOA, GPIO_XB_IO_EXP_RST|GPIO_XB_PMIC_EN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOC, GPIO_XB_IO_EXP_II_RST|GPIO_XB_L_AMO_LCD_RST|GPIO_XB_H2D_RST_N,
                        GPIO_PIN_SET);


}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(pcbid_t pcb_id)
{
    if(pcb_id == XB02){
        MX_XB_GPIO_Init();
    }
    else if((pcb_id == XC01) || (pcb_id == XC02)){
        MX_XC_GPIO_Init();
    }
    else{
        MX_XA_GPIO_Init();
    }
}
void gpio_dump_status(void)
{
    int i = 0;
    uint16_t pa,pb,pc,pd;
    uint32_t cursor = 0x8000;
    char tempA[50];
    char tempB[50];
    char tempC[50];
    char tempD[50];
    pa = GPIOA->IDR;
    pb = GPIOB->IDR;
    pc = GPIOC->IDR;
    pd = GPIOD->IDR;
    shell_info("Group  F  E  D  C  B  A  9  8  7  6  5  4  3  2  1  0\n");
    for(i=0;i<16;i++){
        sprintf(tempA+i*3,"%-3d", !!(pa&cursor));
        sprintf(tempB+i*3,"%-3d", !!(pb&cursor));
        sprintf(tempC+i*3,"%-3d", !!(pc&cursor));
        sprintf(tempD+i*3,"%-3d", !!(pd&cursor));
        cursor >>= 1;
    }
    shell_info("PA     %s\n",tempA);
    shell_info("PB     %s\n",tempB);
    shell_info("PC     %s\n",tempC);
    shell_info("PD     %s\n",tempD);
}

/** NVIC Configuration
*/
static void MX_XA_NVIC_Init(void)
{
    /* EXTI9_5_IRQn interrupt configuration
    *  GPIO5 -- PHONE_DETC
    *  GPIO9 -- USB_A_OC
    *  GPIO7 -- BAT_ALRT
    *  GPIO6 -- CHG_INT
    */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    /*EXTI4_IRQn -- Proximity interrupt line*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    /*EXTI0_IRQn -- G sensor interrupt line*/
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    /*EXTI2_IRQn -- ecompass interrupt line*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    /*EXTI3_IRQn -- bq25896 interrupt line*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
static void MX_XB_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void MX_NVIC_Init(pcbid_t pcb_id)
{

    if(XA0n == pcb_id){
        MX_XA_NVIC_Init();
    }
    else{
        MX_XB_NVIC_Init();
    }

#ifdef	HAL_WWDG_MODULE_ENABLED
    HAL_NVIC_SetPriority(WWDG_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(WWDG_IRQn);
#endif

    __enable_irq();
}
extern void ccgx_irq_handler(void);
extern void phone_vbus_det_irq(void);
extern void usb_hub_oac_det_irq(void);
extern void bq2589x_intr_handler(uint16_t GPIO_Pin);
extern void max17050_intr_handler(uint16_t GPIO_Pin);
void XA_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PS_INT){
		proximitySensorIsr(GPIO_Pin);
	}else if((GPIO_Pin == GPIO_G_S_INT) || (GPIO_Pin == GPIO_ECOMP_INT)){
		sensorIsr(GPIO_Pin);
	}else if (GPIO_Pin == GPIO_PIN_9){
	    //Handle USB VBUS detection upon External interrupt
	    //no used
		//extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
		//HAL_PCDEx_BCD_VBUSDetect (&hpcd_USB_OTG_FS);
		//printf("%s: USB VBUSDetect\r\n", __func__);
		usb_hub_oac_det_irq();
	}else if (GPIO_Pin == GPIO_PIN_7) {
		max17050_intr_handler(GPIO_PIN_7);
	} else if ( (GPIO_Pin == GPIO_PIN_6) ||  (GPIO_Pin == GPIO_PIN_3)) {
		bq2589x_intr_handler(GPIO_Pin);
	} else if (H2D_INT_0_PIN == GPIO_Pin) {
        h2d_int_handle();
    }else if(GPIO_CCG4_INT==GPIO_Pin){
        ccgx_irq_handler();
    }
	if(GPIO_POWER_KEY == GPIO_Pin){

		key_btn_isr(BTN_INDEX_POWER);
	}
    if(GPIO_PIN_5 == GPIO_Pin){
        phone_vbus_det_irq();
    }

}

void panel_error_monitor(uint16_t GPIO_Pin);
void XB_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin){
        case GPIO_PIN_0:
            //GPIO_CC_FLAG_OVP_PHONE
			panel_error_monitor(GPIO_PIN_0); //TODO: INT_H_R_LS ??
            break;
        case GPIO_PIN_1:
            // GPIO_G_S_INT
            sensorIsr(GPIO_Pin);
            break;
        case GPIO_PIN_2:
			panel_error_monitor(GPIO_PIN_2); //TODO: INT_H_L_LS ??
            break;
        case GPIO_PIN_3:
        case GPIO_PIN_12:
            // GPIO_CHG_INT bq25896(pin 3)
            // GPIO_CHARGE_DETC (pin 12)
            bq2589x_intr_handler(GPIO_Pin);
            break;
        case GPIO_PIN_4:
            // AUD_ALC4040_DIN
            break;
        case GPIO_PIN_5:
            // GPIO_PHONE_DETC
            phone_vbus_det_irq();
            break;
        case GPIO_PIN_6:
            // INT_H_R_LS
            break;
        case GPIO_PIN_7:
            // GPIO_BAT_ALRT
            max17050_intr_handler(GPIO_PIN_7);
            break;
        case GPIO_PIN_8:
            // GPIO_CCG4_INT
            ccgx_irq_handler();
            break;
        case GPIO_PIN_9:
            // GPIO_USB_A_OC
            usb_hub_oac_det_irq();
            break;
        case GPIO_PIN_10:
            // AUD_ALC4040_BCLK
            break;
        case GPIO_PIN_11:
            // GPIO_IO_EXP_INT
            break;
        case GPIO_PIN_13:
            audio_aj_detect();
            // AUD_CDC_ALC5665_IRQ
            break;
        case GPIO_PIN_14:
            // H2D_INT_0
            h2d_int_handle();
            break;
        case GPIO_PIN_15:
            // GPIO_POWER_KEY
            key_btn_isr(BTN_INDEX_POWER);
            break;
    }

}
extern void l_gi_flm_1_int_handle(void);
extern void r_gi_flm_1_int_handle(void);

void XC_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin){
        case GPIO_PIN_0:
			panel_error_monitor(GPIO_PIN_0);
            break;
        case GPIO_PIN_1:
            // GPIO_G_S_INT
            sensorIsr(GPIO_Pin);
            break;
        case GPIO_PIN_2:
            break;
        case GPIO_PIN_3:
        case GPIO_PIN_12:
            // GPIO_CHG_INT bq25896(pin 3)
            // GPIO_CHARGE_DETC (pin 12)
            bq2589x_intr_handler(GPIO_Pin);
            break;
        case GPIO_PIN_4:
            // AUD_ALC4040_DIN
            break;
        case GPIO_PIN_5:
			panel_error_monitor(GPIO_PIN_5);
            break;
        case GPIO_PIN_6:
            break;
        case GPIO_PIN_7:
            // GPIO_BAT_ALRT
            max17050_intr_handler(GPIO_PIN_7);
            break;
        case GPIO_PIN_8:
            // GPIO_CCG4_INT
            ccgx_irq_handler();
            break;
        case GPIO_PIN_9:
            // GPIO_USB_A_OC
            usb_hub_oac_det_irq();
            break;
        case GPIO_PIN_10:
            // AUD_ALC4040_BCLK
            break;
        case GPIO_PIN_11:
            phone_vbus_det_irq();
            break;
        case GPIO_PIN_13:
            audio_aj_detect();
            // AUD_CDC_ALC5665_IRQ
            break;
        case GPIO_PIN_14:
            // H2D_INT_0
            h2d_int_handle();
            break;
        case GPIO_PIN_15:
            // GPIO_POWER_KEY
            key_btn_isr(BTN_INDEX_POWER);
            break;
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(wakeup_source == 0){
        wakeup_source |= GPIO_Pin;
    }
    if(pcb_id == XB02){
        XB_GPIO_EXTI_Callback(GPIO_Pin);
    }
    else if((pcb_id == XC01) || (XC02 == pcb_id)){
        XC_GPIO_EXTI_Callback(GPIO_Pin);
    }
    else{
        XA_GPIO_EXTI_Callback(GPIO_Pin);
    }

}

void gpio_init(pcbid_t pcb_id)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    MX_GPIO_Init(pcb_id);
    MX_NVIC_Init(pcb_id);

}

