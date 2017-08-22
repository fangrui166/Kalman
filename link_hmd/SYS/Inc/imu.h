#ifndef __IMU_H__
#define __IMU_H__

typedef struct {
    float       accel[3];
    float       gyro[3];
}Sensor_t;

typedef struct {
    Sensor_t *  pSensor;
    float       q[4];
}imudata;


#endif