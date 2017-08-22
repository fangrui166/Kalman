#ifndef __GPIO_H__
#define __GPIO_H__
#include "stm32f4xx.h"
#include "misc_data.h"

#define GPIO_XC_R_GI_FLM_1_IO                   GPIO_PIN_5   /* PA5 for OLED current enable pin, to watch OLED function normal or not*/
#define GPIO_XC_PORT_R_GI_FLM_1_IO              GPIOA
#define GPIO_XC_CC_FLAG_OVP_PHONE               GPIO_PIN_6   /* PA6 for Phone CC/PD OVP start work Active:low interrupt*/
#define GPIO_XC_PORT_OVP_PHONE                  GPIOA
#define GPIO_XC_PHONE_DETC                      GPIO_PIN_11  /* PC11 for Phone vbus interrupt OTG_OK*/
#define GPIO_XC_PORT_PHONE_DETC                 GPIOC


#define GPIO_XB_DEBUG_UART_TX                   GPIO_PIN_9   /* PA9  for debug port Tx*/
#define GPIO_XB_DEBUG_UART_RX                   GPIO_PIN_10  /* PA10 for debug port Rx*/
#define GPIO_XB_PORT_DEBUG_UART                 GPIOA

#define GPIO_XB_L_GI_FLM_1_IO                   GPIO_PIN_0   /* PA0 for OLED current enable pin, to watch OLED function normal or not*/
#define GPIO_XB_PORT_L_GI_FLM_1_IO              GPIOA

#define GPIO_XB_MBAT_ID_ADC                     GPIO_PIN_1   /* PA1 for Battery ID detection*/
#define GPIO_XB_PORT_MBAT_ID_ADC                GPIOA
#define GPIO_XB_IO_EXP_RST                      GPIO_PIN_2   /* PA2 for expend gpio reset active low*/
#define GPIO_XB_PORT_IO_EXP_RST                 GPIOA
#define GPIO_XB_PMIC_EN                         GPIO_PIN_3   /* PA3 for PMIC NCP6924 enable*/
#define GPIO_XB_PORT_PMIC_EN                    GPIOA
//#define GPIO_XB_AUD_ALC4040_LRCK                GPIO_PIN_4   /* PA4 for Connect to Audio codec*/
#define GPIO_XB_PHONE_DETC                      GPIO_PIN_5   /* PA5 for Phone vbus interrupt OTG_OK*/
#define GPIO_XB_PORT_PHONE_DETC                 GPIOA
#define GPIO_XB_INT_H_R_LS                      GPIO_PIN_6   /* PA6 */
#define GPIO_XB_PORT_INT_H_R_LS                 GPIOA
#define GPIO_XB_BAT_ALRT                        GPIO_PIN_7   /* PA7 for Gauage alert interrupt*/
#define GPIO_XB_PORT_BAT_ALRT                   GPIOA
#define GPIO_XB_HUB_VBUS_DET                    GPIO_PIN_8   /* PA8 for HUB vbus detection*/
#define GPIO_XB_PORT_HUB_VBUS_DET               GPIOA
#define GPIO_XB_POWER_KEY                       GPIO_PIN_15  /* PA15 for power key detection interrupt*/
#define GPIO_XB_PORT_POWER_KEY                  GPIOA

#define GPIO_XB_CC_FLAG_OVP_PHONE               GPIO_PIN_0   /* PB0 for Phone CC/PD OVP start work Active:low interrupt*/
#define GPIO_XB_PORT_OVP_PHONE                  GPIOB
#define GPIO_XB_G_S_INT                         GPIO_PIN_1   /* PB1 for G-Sensor interrupt*/
#define GPIO_XB_PORT_G_S_INT                    GPIOB
//#define GPIO_XB_AUD_ALC4040_DIN                 GPIO_PIN_4   /* PB4  */
//#define GPIO_XB_AUD_ALC4040_DOUT                GPIO_PIN_5   /* PB5  */
#define GPIO_XB_CCG4_INT                        GPIO_PIN_8   /* PB8 for CCG4 interrupt*/
#define GPIO_XB_PORT_CCG4_INT                   GPIOB
#define GPIO_XB_USB_A_OC                        GPIO_PIN_9   /* PB9 for Type A overcurrent interrupt*/
#define GPIO_XB_PORT_USB_A_OC                   GPIOB
#define GPIO_XB_SPI2_CS                         GPIO_PIN_12  /* PB12 for SPI2 soft NSS */
#define GPIO_XB_PORT_SPI2_CS                    GPIOB

#define GPIO_XB_ERR_FG_R_IO                     GPIO_PIN_0   /* PC0 for L display VDD 1V8 enable*/
#define GPIO_XB_PORT_ERR_FG_R_IO                GPIOC
#define GPIO_XB_R_AMO_LCD_RST                   GPIO_PIN_1   /* PC1 for R display VDD 1V8 enable*/
#define GPIO_XB_PORT_R_AMO_LCD_RST              GPIOC
#define GPIO_XB_INT_H_L_LS                      GPIO_PIN_2   /* PC2  */
#define GPIO_XB_PORT_INT_H_L_LS                 GPIOC
#define GPIO_XB_CHG_INT                         GPIO_PIN_3   /* PC3 for Charger bq25896 interrupt*/
#define GPIO_XB_PORT_CHG_INT                    GPIOC
#define GPIO_XB_USB_AC_CON_TEMP                 GPIO_PIN_4   /* PC4 for USB AC temp detection ADC*/
#define GPIO_XB_PORT_USB_AC_CON_TEMP            GPIOC
#define GPIO_XB_ERR_FG_L_IO                     GPIO_PIN_5   /* PC5  for ESD detection monitor MIPI signal and OLED power source */
#define GPIO_XB_PORT_ERR_FG_L_IO                GPIOC
#define GPIO_XB_IO_EXP_II_RST                   GPIO_PIN_6   /* PC6  for EXT II expend gpio reset active low*/
#define GPIO_XB_PORT_IO_EXP_II_RST              GPIOC
#define GPIO_XB_CC_FLAG_OVP_CHARG               GPIO_PIN_8   /* PC8  for AC Charge CC/PD OVP start work Active:low interrupt*/
#define GPIO_XB_PORT_OVP_CHARG                  GPIOC
#define GPIO_XB_L_AMO_LCD_RST                   GPIO_PIN_9   /* PC9  for AMOLED L reset*/
#define GPIO_XB_PORT_L_AMO_LCD_RST              GPIOC
#define GPIO_XB_IO_EXP_INT                      GPIO_PIN_11  /* PC11 for Expend gpio interrupt*/
#define GPIO_XB_PORT_IO_EXP_INT                 GPIOC
#define GPIO_XB_CHARGE_DETC                     GPIO_PIN_12  /* PC12 for AC charger vbus interrupt AC_POWER_IN#*/
#define GPIO_XB_PORT_CHARGE_DETC                GPIOC
#define GPIO_XB_AUD_CDC_ALC5665_IRQ             GPIO_PIN_13  /* PC13 for LALC5665 IRQ Interrupt after detection, reserved*/
#define GPIO_XB_PORT_AUD_CDC_ALC5665_IRQ        GPIOC
#define GPIO_XB_H2D_INT_0                       GPIO_PIN_14  /* PC14 for TC358870 interrupt, active high*/
#define GPIO_XB_PORT_H2D_INT_0                  GPIOC
#define GPIO_XB_H2D_RST_N                       GPIO_PIN_15  /* PC15 for TC358870 reset*/
#define GPIO_XB_PORT_H2D_RST_N                  GPIOC

#define GPIO_XB_R_GI_FLM_1_IO                   GPIO_PIN_2   /* PA0 for OLED current enable pin, to watch OLED function normal or not*/
#define GPIO_XB_PORT_R_GI_FLM_1_IO              GPIOD

#define GPIO_DEBUG_UART_TX                   GPIO_PIN_2   /* PA2 for debug port Tx*/
#define GPIO_DEBUG_UART_RX                   GPIO_PIN_3   /* PA3 for debug port Rx*/
#define GPIO_PORT_DEBUG_UART                 GPIOA

#define GPIO_PHONE_CHG_EN                    GPIO_PIN_0   /* PA0 for phone charge enable*/
#define GPIO_PORT_PHONE_CHG_EN               GPIOA
#define GPIO_MBAT_ID_ADC                     GPIO_PIN_1   /* PA1 for Battery ID detection*/
#define GPIO_PORT_MBAT_ID_ADC                GPIOA
#define GPIO_V_DP_EN                         GPIO_PIN_4   /* PA4 for D2H Power enable*/
#define GPIO_PORT_V_DP_EN                    GPIOA
#define GPIO_PHONE_DETC                      GPIO_PIN_5   /* PA5 for Phone vbus interrupt OTG_OK*/
#define GPIO_PORT_PHONE_DETC                 GPIOA
#define GPIO_CHARGE_DETC                     GPIO_PIN_6   /* PA6 for AC charger vbus interrupt*/
#define GPIO_PORT_CHARGE_DETC                GPIOA
#define GPIO_BAT_ALRT                        GPIO_PIN_7   /* PA7 for Gauage alert interrupt*/
#define GPIO_PORT_BAT_ALRT                   GPIOA
#define GPIO_HUB_VBUS_DET                    GPIO_PIN_8   /* PA8 for HUB vbus detection*/
#define GPIO_PORT_HUB_VBUS_DET               GPIOA
#define GPIO_USB_A_OC                        GPIO_PIN_9   /* PA9 for Type A overcurrent interrupt*/
#define GPIO_PORT_USB_A_OC                   GPIOA
#define GPIO_AUD_ALC4040_GPIO7               GPIO_PIN_10  /* PA10 for */
#define GPIO_PORT_AUD_ALC4040_GPIO7          GPIOA
#define GPIO_AUD_ALC4040_GPIO6               GPIO_PIN_15  /* PA15 for */
#define GPIO_PORT_AUD_ALC4040_GPIO6          GPIOA

#define GPIO_GYRO_INT                        GPIO_PIN_0   /* PB0 for Gyro interrupt*/
#define GPIO_PORT_GYRO_INT                   GPIOB
#define GPIO_G_S_INT                         GPIO_PIN_1   /* PB1 for G-Sensor interrupt*/
#define GPIO_PORT_G_S_INT                    GPIOB
#define GPIO_PS_INT                          GPIO_PIN_4   /* PB4 for P-sensor interrupt*/
#define GPIO_PORT_PS_INT                     GPIOB
#define GPIO_PMIC_EN                         GPIO_PIN_5   /* PB5 for PMIC NCP6924 enable*/
#define GPIO_PORT_PMIC_EN                    GPIOB
#define GPIO_CCG4_INT                        GPIO_PIN_8   /* PB8 for CCG4 interrupt*/
#define GPIO_PORT_CCG4_INT                   GPIOB
#define GPIO_IO_EXP_RST                      GPIO_PIN_9   /* PB9 for expend gpio reset active low*/
#define GPIO_PORT_IO_EXP_RST                 GPIOB
#define GPIO_SPI2_CS                         GPIO_PIN_12  /* PB12 for SPI2 soft NSS */
#define GPIO_PORT_SPI2_CS                    GPIOB

#define GPIO_V_VDDI_L_EN                     GPIO_PIN_0   /* PC0 for L display VDD 1V8 enable*/
#define GPIO_PORT_V_VDDI_L_EN                GPIOC
#define GPIO_V_VDDI_R_EN                     GPIO_PIN_1   /* PC1 for R display VDD 1V8 enable*/
#define GPIO_PORT_V_VDDI_R_EN                GPIOC
#define GPIO_ECOMP_INT                       GPIO_PIN_2   /* PC2 for Ecompass interrupt*/
#define GPIO_PORT_ECOMP_INT                  GPIOC
#define GPIO_CHG_INT                         GPIO_PIN_3   /* PC3 for Charger bq25896 interrupt*/
#define GPIO_PORT_CHG_INT                    GPIOC
#define GPIO_USB_AC_CON_TEMP                 GPIO_PIN_4   /* PC4 for USB AC temp detection ADC*/
#define GPIO_PORT_USB_AC_CON_TEMP            GPIOC
//#define GPIO_RED_LED                         GPIO_PIN_4   /* PC4 for red LED on*/
//#define GPIO_PORT_RED_LED                    GPIOC
#define GPIO_GREE_LED                        GPIO_PIN_5   /* PC5 for green LED on*/
#define GPIO_PORT_GREE_LED                   GPIOC
#define GPIO_V_HDMI_EN                       GPIO_PIN_6   /* PC6 for */
#define GPIO_PORT_V_HDMI_EN                  GPIOC
#define GPIO_USB_HUB_RST                     GPIO_PIN_7   /* PC7 for USB HUB reset*/
#define GPIO_PORT_USB_HUB_RST                GPIOC
#define GPIO_V_BOOST_5V_EN                   GPIO_PIN_8   /* PC8 for */
#define GPIO_PORT_V_BOOST_5V_EN              GPIOC
#define GPIO_L_AMO_LCD_RST                   GPIO_PIN_9   /* PC9 for */
#define GPIO_PORT_L_AMO_LCD_RST              GPIOC
#define GPIO_POWER_KEY                       GPIO_PIN_10  /* PC10 for power key detection interrupt*/
#define GPIO_PORT_POWER_KEY                  GPIOC
#define GPIO_IO_EXP_INT                      GPIO_PIN_11  /* PC11 for Expend gpio interrupt*/
#define GPIO_PORT_IO_EXP_INT                 GPIOC
#define GPIO_AUD_ALC4040_GPIO_ALO_1          GPIO_PIN_12  /* PC12 for */
#define GPIO_PORT_AUD_ALC4040_GPIO_ALO_1     GPIOC
#define GPIO_AUD_CDC_ALC5665_IRQ             GPIO_PIN_13  /* PC13 for */
#define GPIO_PORT_AUD_CDC_ALC5665_IRQ        GPIOC
#define GPIO_H2D_INT_0                       GPIO_PIN_14  /* PC14 for */
#define GPIO_PORT_H2D_INT_0                  GPIOC
#define GPIO_H2D_RST_N                       GPIO_PIN_15  /* PC15 for */
#define GPIO_PORT_H2D_RST_N                  GPIOC

#define GPIO_PD1__                           GPIO_PIN_1   /* PD1 for */
#define GPIO_R_AMO_LCD_RST                   GPIO_PIN_2   /* PD2 for */
#define GPIO_PORT_R_AMO_LCD_RST              GPIOD

void gpio_init(pcbid_t pcb_id);
void gpio_dump_status(void);

#endif
