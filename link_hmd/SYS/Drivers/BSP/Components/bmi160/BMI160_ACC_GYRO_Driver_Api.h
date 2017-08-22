#ifndef __BMI160_ACC_GYRO_DRIVER_H__
#define __BMI160_ACC_GYRO_DRIVER_H__
#include "BMI160_ACC_GYRO_Driver.h"


/**
 * @brief BMI160 accelero specific data internal structure definition
 */
#define BMI160_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define BMI160_ACC_SENSITIVITY_FOR_FS_16G  0.488 /**< Sensitivity value for 16g full scale [mg/LSB] */


#define BMI160_GYRO_SENSITIVITY_FOR_FS_125DPS   13.68	//0.0038   /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_250DPS   27.36 //0.0076  /**< Sensitivity value for 250 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_500DPS   54.72 //0.0153  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_1000DPS  109.44//0.0305  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define BMI160_GYRO_SENSITIVITY_FOR_FS_2000DPS  218.88//0.0610  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */


#define BMI160_ACC_GYRO_WHO_AM_I     0xD1
#define BMI160_ACC_GYRO_ADDRESS      0x68
#define BMI160_SENSORS_MAX_NUM       0x01 

#define C_BMI160_ZERO_U8X	0
#define C_BMI160_ONE_U8X 	1
#define C_BMI160_EIGHT_U8X	8
#define C_BMI160_HUNDRED_U8X	100



typedef struct
{
  uint8_t isEnabled;
  float   lastODR;
} BMI160_Combo_Data_t;

typedef struct
{
  float Previous_ODR;//default output data rate
  BMI160_Combo_Data_t *comboData;       /* Combo data to manage in software ODR of the combo sensors */
} BMI160_X_Data_t;

typedef struct
{
  float Previous_ODR;//default output data rate
  BMI160_Combo_Data_t *comboData;       /* Combo data to manage in software ODR of the combo sensors */
} BMI160_G_Data_t;

extern ACCELERO_Drv_t BMI160_X_Drv;
extern GYRO_Drv_t     BMI160_G_Drv;
extern BMI160_Combo_Data_t BMI160_Combo_Data[BMI160_SENSORS_MAX_NUM];

/*!
 *	@brief This function used for interrupt configuration
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_init_fifo(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_init_gpio(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_accel_mode(us8 accel_power_mode);
BMI160_RETURN_FUNCTION_TYPE bmi160_gyro_mode(us8 gyro_power_mode);
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_mode(us8 mag_power_mode);
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_flush(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_int_reset(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_softreset(void);
char BMI160_INIT_SENSOR_API(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_init(void);
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_write_reg(us8 v_addr_us8,us8 *v_data);
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_read_reg(us8 v_addr_us8,us8 *v_data,us8 v_len_us8 );
BMI160_RETURN_FUNCTION_TYPE  bmi160_mag_manual_disable(us8 v_addr_us8,us8 v_len_us8 );
BMI160_RETURN_FUNCTION_TYPE bmi160_mag_manual_enable(void);
#endif

