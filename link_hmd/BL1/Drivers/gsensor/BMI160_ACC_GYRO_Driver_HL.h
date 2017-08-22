/*! \file bmi160_support.h
    \brief BMI160 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMI160_SUPPORT_H__
#define __BMI160_SUPPORT_H__

//#include "component.h"
//#include "sensor.h"
//#include "gyroscope.h"
//#include "accelerometer.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>


typedef	signed char  int8_ts;/**< used for signed 8bit */
typedef	signed short int int16_ts;/**< used for signed 16bit */
typedef	signed int int32_ts;/**< used for signed 32bit */
typedef	signed long long int int64_ts;/**< used for signed 64bit */
 
/*unsigned integer types*/
typedef	unsigned char us8;/**< used for unsigned 8bit */
typedef	unsigned short int us16;/**< used for unsigned 16bit */
typedef	unsigned int us32;/**< used for unsigned 32bit */
typedef	unsigned long long int us64;/**< used for unsigned 64bit */



/*############################### SPI2 #######################################*/
#define INCLUDE_BMI160API
#define SPI_BUFFER_LEN 1024
#define BMI160_SPI2_TIMEOUT_MAX  1000

#define BMI160_SPI2                                       SPI2
#define BMI160_SPI2_CLK_ENABLE()                        __HAL_RCC_SPI2_CLK_ENABLE()

#define BMI160_SPI2_SCK_AF                              GPIO_AF5_SPI2
#define BMI160_SPI2_SCK_GPIO_PORT                       GPIOB
#define BMI160_SPI2_SCK_PIN                             GPIO_PIN_13
#define BMI160_SPI2_SCK_GPIO_CLK_ENABLE()               __GPIOB_CLK_ENABLE()
#define BMI160_SPI2_SCK_GPIO_CLK_DISABLE()              __GPIOB_CLK_DISABLE()

#define BMI160_SPI2_MISO_MOSI_AF                        GPIO_AF5_SPI2
#define BMI160_SPI2_MISO_MOSI_GPIO_PORT                 GPIOB
#define BMI160_SPI2_MISO_MOSI_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define BMI160_SPI2_MISO_MOSI_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()
#define BMI160_SPI2_MISO_PIN                            GPIO_PIN_14
#define BMI160_SPI2_MOSI_PIN                            GPIO_PIN_15



#define BMI160_CS_LOW()       HAL_GPIO_WritePin(BMI160_CS_GPIO_PORT, BMI160_CS_PIN, GPIO_PIN_RESET)
#define BMI160_CS_HIGH()      HAL_GPIO_WritePin(BMI160_CS_GPIO_PORT, BMI160_CS_PIN, GPIO_PIN_SET)
/**
  * @brief BMI160 Control Interface pins
  */
#define BMI160_CS_PIN                                 GPIO_PIN_12
#define BMI160_CS_GPIO_PORT                           GPIOB
#define BMI160_CS_GPIO_CLK_ENABLE()                 __GPIOB_CLK_ENABLE()
#define BMI160_CS_GPIO_CLK_DISABLE()                __GPIOB_CLK_DISABLE()


/*!
 *	@brief This function is an example for delay
 *	@param msek: delay in milli seconds
 *	@return : communication result
 */
void bmi160_delay_ms(uint32_t msek);
void bmi160_spi2_cs(void);
void  BMI160_IO_Init(void);

#ifdef INCLUDE_BMI160API
 /*!
 *	@brief : The function is used as I2C bus read
 *	@return : Status of the I2C read
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be read
 *	@param reg_data : This data read from the sensor,
 *	which is hold in an array
 *	@param cnt : The no of byte of data to be read
 */
int8_ts bmi160_spi_bus_read(us32 dev_addr, us8 reg_addr,us8 *reg_data, us8 cnt);
 /*!
 *	@brief : The function is used as I2C bus write
 *	@return : Status of the I2C write
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be written
 *	@param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	@param cnt : The no of byte of data to be write
 */
int8_ts  bmi160_spi_bus_write(us32 dev_addr, us8 reg_addr,us8 *reg_data, us8 cnt);

/*!
 *	@brief : The function is used as SPI bus read
 *	@return : Status of the SPI read
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be read
 *	@param reg_data : This data read from the sensor,
 *	which is hold in an array
 *	@param cnt : The no of byte of data to be read
 */

int8_ts  burst_read(us32 dev_addr, us8 reg_addr,us8 *reg_data, us32 cnt);
#endif
#endif
