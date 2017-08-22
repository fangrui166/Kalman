/* user defined code to be added here ... */
#ifndef __BMI160_Calibate_H__
#define __BMI160_Calibate_H__
#include "accelerometer.h"

#define BMI160_MAX 3 
#define BMI160_E 0.000000001  
#define ACC_READ_XUP "xup"
#define ACC_READ_XDOWN "xdown"
#define ACC_READ_YUP "yup"
#define ACC_READ_YDOWN "ydown"
#define ACC_READ_ZUP "zup"
#define ACC_READ_ZDOWN "zdown"




typedef enum
{
  GSENSORNONCAL = 0,
  GSENSORCAL_1,
  GSENSORCAL_6,
} gSensorCalTypeDef;


typedef struct
{
	char isAccValid;// 1 is valid
	char isGyroValid;// 1 is valid
	int gyro_x;
	int gyro_y;
	int gyro_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	float accel_k_xyz[BMI160_MAX][BMI160_MAX];
} BMI160_DataTypeDef;
typedef struct
{
  float AXIS_X;
  float AXIS_Y;
  float AXIS_Z;
} SensorAxes_t_k6;
#define accel_cal_count 3000
#define accel_cal_count 3000

#define ONEPACKAGESIZE (700)//BYTE
#define ONESIDESIZ (ONEPACKAGESIZE*30)
#define ACC_XUP_FLASHID 	0x08020000
#define ACC_XDOWN_FLASHID	  (ACC_XUP_FLASHID + ONESIDESIZ*1)
#define ACC_YUP_FLASHID		(ACC_XUP_FLASHID + ONESIDESIZ*2)
#define ACC_YDOWN_FLASHID	 (ACC_XUP_FLASHID + ONESIDESIZ*3)
#define ACC_ZUP_FLASHID		(ACC_XUP_FLASHID + ONESIDESIZ*4)
#define ACC_ZDOWN_FLASHID		(ACC_XUP_FLASHID + ONESIDESIZ*5)
//#define ACC_XDOWN_FLASHID	6

#define ACC_WRITEFLASH_SIZE 100
typedef struct
{
	int start_id;
	SensorAxesRaw_t accel[ACC_WRITEFLASH_SIZE];
	int end_id;
}Save_str_k6;

DrvStatusTypeDef Gyro_Calibate(void);
double calculate_A( double src[][BMI160_MAX], int n );
void calculate_A_adjoint( double src[BMI160_MAX][BMI160_MAX], double dst[BMI160_MAX][BMI160_MAX], int n );
int calculate_A_inverse( double src[BMI160_MAX][BMI160_MAX], double dst[BMI160_MAX][BMI160_MAX], int n );
void print_M( double M[][BMI160_MAX], int n );
int test_bmi160(void);
DrvStatusTypeDef ACC_Calibate_6(void);
DrvStatusTypeDef  read_accel_xup(void);
DrvStatusTypeDef read_accel_xdown(void);
DrvStatusTypeDef  read_accel_yup(void);
DrvStatusTypeDef read_accel_ydown(void);
DrvStatusTypeDef  read_accel_zup(void);
DrvStatusTypeDef read_accel_zdown(void);
DrvStatusTypeDef accel_k6andtest(void);
DrvStatusTypeDef ACC_Calibate(unsigned char *buf);
DrvStatusTypeDef ACC_fd_test(void);
DrvStatusTypeDef  test_gyro(void);
DrvStatusTypeDef  cal_accel_1axis(void);
#endif
