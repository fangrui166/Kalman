#include "gpio.h"
#include "boot.h"
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

    GPIO_InitStruct.Pin = GPIO_XB_PHONE_DETC|GPIO_XB_INT_H_R_LS|GPIO_XB_POWER_KEY\
                        /*|GPIO_XB_HUB_VBUS_DET*/;
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
    GPIO_InitStruct.Pin = GPIO_XB_CC_FLAG_OVP_PHONE;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CC_FLAG_OVP_CHARG;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_CHG_INT|GPIO_XB_IO_EXP_INT|GPIO_XB_CHARGE_DETC\
                        /*|GPIO_XB_CC_FLAG_OVP_CHARG*/;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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

    GPIO_InitStruct.Pin = GPIO_XB_AUD_CDC_ALC5665_IRQ;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XB_ERR_FG_L_IO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_XC_INT_GYRO;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    /* Configure GPIO Default value */
    HAL_GPIO_WritePin(GPIOA, GPIO_XB_IO_EXP_RST|GPIO_XB_PMIC_EN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOC, GPIO_XB_IO_EXP_II_RST|GPIO_XB_L_AMO_LCD_RST|
GPIO_XB_H2D_RST_N,
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
    else if((pcb_id == XC01) || (XC02 == pcb_id)){
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
    printf("\r\nGroup  F  E  D  C  B  A  9  8  7  6  5  4  3  2  1  0\n");
    for(i=0;i<16;i++){
        sprintf(tempA+i*3,"%-3d", !!(pa&cursor));
        sprintf(tempB+i*3,"%-3d", !!(pb&cursor));
        sprintf(tempC+i*3,"%-3d", !!(pc&cursor));
        sprintf(tempD+i*3,"%-3d", !!(pd&cursor));
        cursor >>= 1;
    }
    printf("PA     %s\n",tempA);
    printf("PB     %s\n",tempB);
    printf("PC     %s\n",tempC);
    printf("PD     %s\n",tempD);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(pcbid_t pcb_id)
{
    /*enable powerkey waekup both XA and XB*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

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


