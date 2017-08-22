#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"

#define SENSOR_LATENCY_TEST     0
#define SENSOR_LATENCY_TEST_COUNT     200
#define SENSOR_LATENCY_TEST_POINT     4



/* Exported types ------------------------------------------------------------*/
typedef enum
{
	ACC = 0,     /* Acceleration sensor  */
	GYRO,        /* Gyroscope sensor. */
	MAG,	    /* magnetic sensor. */
} SENSOR_ID_t;

enum {
	MAG_SET_SUB_FUNC00 = 0,
};

enum {
	MAG_CALIBRATION_CANCEL = 0,
	MAG_CALIBRATION_START,
};

enum {
	MAG_GET_SUB_FUNC00 = 0,
};

//extern osThreadId sensorTaskHandle;
//extern BaseType_t xSensorTaskWoken;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void sensorTaskInit(void);
void sensorIsr(uint16_t GPIO_Pin);
void sensorEnable(SENSOR_ID_t sensor_id);
void sensorDisable(SENSOR_ID_t sensor_id);
void sensorSetOdr(SENSOR_ID_t sensor_id, SensorOdr_t odr);
void sensorGetOdr(SENSOR_ID_t sensor_id, float* odr);
int sensorSuspendResume(SensorMode_t Sensor_mode);
void magneticSetUserCalibration(uint8_t enable);
void magneticGetUserCalibration(uint8_t *info,uint8_t length);


#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TASK_H */
