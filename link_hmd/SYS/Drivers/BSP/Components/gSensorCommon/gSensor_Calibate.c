#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "gSensor_Calibate.h"
#include "stm32f4xx_hal.h"
//#include "misc_data.h"
#include "flash_drv.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "misc_data.h"

extern pcbid_t pcb_id;
extern int8_t usb_cdc_transmit_data(uint8_t *buf, uint8_t len);
extern void watchdog_refresh();
extern BMI160_DataTypeDef bmi160_offset;
extern gSensorCalTypeDef gSensorCalStatus;
extern void *SENSOR_X_0_handle;
extern void *SENSOR_G_0_handle;

#define CAL_SENSOR_X_0_handle CAL_X_0_handle
#define CAL_SENSOR_G_0_handle CAL_G_0_handle

void* CAL_X_0_handle = NULL;
void* CAL_G_0_handle = NULL;

float xup_x = 0,xup_y = 0,xup_z = 0;//x up   x = -16384
float xdown_x = 0,xdown_y = 0,xdown_z = 0;//x down   x = 16384
float yup_x = 0,yup_y = 0,yup_z = 0;//y up   y= -16384
float ydown_x = 0,ydown_y = 0,ydown_z = 0;//y down   y= 16384
float zup_x = 0,zup_y = 0,zup_z = 0;//z up   z= -16384
float zdown_x = 0,zdown_y = 0,zdown_z = 0;//z down   z= 16384

#define accel_test_count 3000
#define GYRO_BASE_VALUE  5.5  //dps
#define ACCEL_BASE_VALUE 41  //mg  <= 40mg pass
#define ACCEL_1000MG     1001 //mg <= 1001mg pass



static DrvStatusTypeDef X_Get_Axes_k6( SensorAxes_t_k6 *acceleration, SensorAxesRaw_t *accel)
{

	  float sensitivity = 0;

	  /* Get BMI160 actual sensitivity. */
	  if ( BSP_GYRO_Get_Sensitivity(CAL_SENSOR_X_0_handle, &sensitivity ) != COMPONENT_OK )
	  {
		return COMPONENT_ERROR;
	  }
	  /* Read raw data from BMI160 output register. */
	  if (BSP_ACCELERO_Get_AxesRaw( CAL_SENSOR_X_0_handle, accel) != COMPONENT_OK)
	  {
		return COMPONENT_ERROR;
	  }



	  /* Calculate the data. */
	  acceleration->AXIS_X = ( (accel->AXIS_X) * (sensitivity) );
	  acceleration->AXIS_Y = ( (accel->AXIS_Y) * (sensitivity) );
	  acceleration->AXIS_Z = ( (accel->AXIS_Z) * (sensitivity) );

	  return COMPONENT_OK;
}
static DrvStatusTypeDef G_Get_Axes( SensorAxes_gyro_t *angular_velocity)
{
	  SensorAxes_t  sensor_datatmp;
	  /* Get BMI160 actual sensitivity. */
	  if ( BSP_GYRO_Get_Axes( CAL_SENSOR_G_0_handle, &sensor_datatmp ) != COMPONENT_OK )
	  {
		return COMPONENT_ERROR;
	  }


	  /* Calculate the data. */
        angular_velocity->AXIS_X = (float)sensor_datatmp.AXIS_X / 3600;
        angular_velocity->AXIS_Y = (float)sensor_datatmp.AXIS_Y / 3600;
        angular_velocity->AXIS_Z = (float)sensor_datatmp.AXIS_Z / 3600;

	  return COMPONENT_OK;
}
static DrvStatusTypeDef gSensorWDdogRefresh( void)
{
	watchdog_refresh();
	return COMPONENT_OK;
}
/**
 * @brief  Initialize accel
 * @param  None
 * @retval None
 */
DrvStatusTypeDef Cal_initializeAccSensor(void)
{
    uint8_t pBuffer = 0;

    if(SENSOR_X_0_handle == NULL)
    {
		return COMPONENT_ERROR;
    }

    CAL_X_0_handle = SENSOR_X_0_handle;
    bmi160_offset.isAccValid = 0;
    bmi160_offset.isGyroValid = 0;
    #if 0
    BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO,&CAL_SENSOR_X_0_handle) ;
    BSP_ACCELERO_Sensor_Enable(CAL_SENSOR_X_0_handle) ;
    BSP_ACCELERO_Set_ODR(CAL_SENSOR_X_0_handle, ODR_MID_HIGH) ;
    #endif

    Sensor_IO_SPI_Read( NULL, 0x0, &pBuffer, 1);
    if(pBuffer == 0xD1)
    {
    		pBuffer = 0;
		Sensor_IO_SPI_Write( NULL, 0x56, &pBuffer, 1);
    }else
    {
		Sensor_IO_SPI_Read( NULL, 0x0F, &pBuffer, 1);
		if(pBuffer == 0x6A)
		{
			pBuffer = 0;
			Sensor_IO_SPI_Write( NULL, 0x0D, &pBuffer, 1);
		}else
		{
			return COMPONENT_ERROR;
		}
   }


    return COMPONENT_OK;
}

/**
 * @brief  Initialize gyro
 * @param  None
 * @retval None
 */
DrvStatusTypeDef Cal_initializeGyroSensor(void)
{
    uint8_t pBuffer = 0;

    if(SENSOR_G_0_handle == NULL)
    {
		return COMPONENT_ERROR;
    }

    CAL_G_0_handle = SENSOR_G_0_handle;
    bmi160_offset.isAccValid = 0;
    bmi160_offset.isGyroValid = 0;
    #if 0
    BSP_GYRO_Init(GYRO_SENSORS_AUTO, &CAL_SENSOR_G_0_handle);
    BSP_GYRO_Sensor_Enable(CAL_SENSOR_G_0_handle);
    BSP_GYRO_Set_ODR(CAL_SENSOR_G_0_handle, ODR_MID_HIGH);
    #endif

    Sensor_IO_SPI_Read( NULL, 0x0, &pBuffer, 1);
    if(pBuffer == 0xD1)
    {
    		pBuffer = 0;
		Sensor_IO_SPI_Write( NULL, 0x56, &pBuffer, 1);
    }else
    {
		Sensor_IO_SPI_Read( NULL, 0x0F, &pBuffer, 1);
		if(pBuffer == 0x6A)
		{
			pBuffer = 0;
			Sensor_IO_SPI_Write( NULL, 0x0D, &pBuffer, 1);
		}else
		{
			return COMPONENT_ERROR;
		}
   }

    return COMPONENT_OK;
}

DrvStatusTypeDef Gyro_Calibate(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0;
	int gyro_x_tmp = 0,gyro_y_tmp = 0,gyro_z_tmp = 0;
	SensorAxesRaw_t gyro;
	BMI160_DataTypeDef gyro_calvalue = {0};

	if(Cal_initializeGyroSensor() != COMPONENT_OK) //bmi160 init
	{
		printf("Gyro Calibate fail,init fail!\n");
		return COMPONENT_ERROR;
	}

	for(i = 0 ; i < 3000 ; i++)
    	{
    		gSensorWDdogRefresh();
       	com_rslt = BSP_GYRO_Get_AxesRaw( CAL_SENSOR_G_0_handle, &gyro );
        	if (com_rslt != COMPONENT_OK)
        	{
          	  printf("Gyro Calibate fail,read rawdata fail!\n");
          	  return com_rslt;
       		}
        	gyro_x_tmp += gyro.AXIS_X;
        	gyro_y_tmp += gyro.AXIS_Y;
        	gyro_z_tmp += gyro.AXIS_Z;
        	HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}

	gyro_calvalue.gyro_x = gyro_x_tmp/i;
	gyro_calvalue.gyro_y = gyro_y_tmp/i;
	gyro_calvalue.gyro_z = gyro_z_tmp/i;

	saveGyroOffsetValue((void *)(&gyro_calvalue));
	printf("SaveBmi160Data:gyro_x = %d gyro_y = %d gyro_z = %d\n",
		gyro_calvalue.gyro_x,gyro_calvalue.gyro_y,gyro_calvalue.gyro_z);
	return COMPONENT_OK;
}

DrvStatusTypeDef  test_gyro(void)
{
	int i = 0;
	SensorAxes_gyro_t gyro_tmp = {0};
	BMI160_DataTypeDef bmi160_offset_tmp;
	int gyro_flag  = 0;
	uint8_t buffer[128];

	gyro_flag  = 0;


	bmi160_offset_tmp.isGyroValid = 0;
	getBmi160OffsetValue((void *)(&bmi160_offset_tmp));
	if(bmi160_offset_tmp.isGyroValid != 1)
	{
		printf("Gyro [has not  calibration]\r\n");
		return COMPONENT_ERROR;
	}
	for(i = 0 ; i < accel_test_count ; i++)
	{
		gSensorWDdogRefresh();
	      if(G_Get_Axes( &gyro_tmp) != 0)
	      {
			printf("gyro calibration fail\n");
			clearGyroValue();
			return COMPONENT_ERROR;
		}
		 //2000 dps  +/- 5.5 dps
		 if(((-GYRO_BASE_VALUE)<gyro_tmp.AXIS_X) &&(gyro_tmp.AXIS_X<GYRO_BASE_VALUE) && \
			((-GYRO_BASE_VALUE)<gyro_tmp.AXIS_Y) &&(gyro_tmp.AXIS_Y<GYRO_BASE_VALUE) && \
			 ((-GYRO_BASE_VALUE)<gyro_tmp.AXIS_Z) &&(gyro_tmp.AXIS_Z<GYRO_BASE_VALUE)    )
		{
					;//MFG_LOG_ERR(MFG_TAG_SENSOR,"bmi160 gyro calibration success\n");
		}else
		{
				gyro_flag++;
				memset(buffer,'\0',128);
		 		sprintf( (char *)buffer,"x = %.3f dps\ty =%.3f dps\tz=%.3f dps\r\n", gyro_tmp.AXIS_X, gyro_tmp.AXIS_Y, gyro_tmp.AXIS_Z); // C4996
		 		usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 		printf("%s",buffer);
				break;
		}
		HAL_Delay(2);/* bmi160_delay_ms in ms*/
	   }

	if(gyro_flag != 0)
	{
		clearGyroValue();
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;

}

DrvStatusTypeDef  cal_accel_1axis(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];
	float raw_x,raw_y,raw_z;
	BMI160_DataTypeDef acc_calvalue = {0};
	SensorAxes_t acceleration_tmp;
	int32_t x= 0,y = 0,z = 0;
	int32_t offset_max[1][3];
	int32_t cal_flag = 0;

	raw_x = 0;
	raw_y = 0;
	raw_z = 0;
	if(Cal_initializeAccSensor() != COMPONENT_OK) //gsensor init
	{
		printf("gsensor cal fail,init fail\n");
		return COMPONENT_ERROR;
	}
	if(VR_flash_erase(ACC_XUP_FLASHID,0x2)!=0)
	{
		return COMPONENT_ERROR;
	}

	 for(i = 0,j = 0; i < accel_cal_count ; i++)
        {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel rawdata fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_XUP_FLASHID;
			 accel_tmp.end_id = ACC_XUP_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_XUP_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           	   raw_x += acceleration.AXIS_X;
		   raw_y += acceleration.AXIS_Y;
		   raw_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}


	raw_x = raw_x/i;
	raw_y = raw_y/i;
	raw_z = raw_z/i;
	if(pcb_id == XA0n){
	/**************XA*****************************************************************************/
	if((800 > raw_x) ||(raw_x > 1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:x = %.1f y = %.1f z = %.1f\n",raw_x,raw_y,raw_z); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}

	acc_calvalue.accel_x = raw_x - 1000;
	acc_calvalue.accel_y = raw_y;
	acc_calvalue.accel_z = raw_z;
    	for( i = 0; i < 3; ++i )
    	{
        	for( j = 0; j < 3; ++j )
        	{
           		 acc_calvalue.accel_k_xyz[i][j] = 1;
        	}
    	}
	saveAccelOffsetValue((void *)(&acc_calvalue));
	printf("SaveData accel:offsetx = %f offsety = %f offsetz =%f\n",
		acc_calvalue.accel_x,acc_calvalue.accel_y,acc_calvalue.accel_z);

	if(getBmi160OffsetValue((void *)(&bmi160_offset)) != 0)//read accel and gyro offset value
	{
		return COMPONENT_ERROR;
	}
	if((bmi160_offset.accel_k_xyz[0][0] == 1) && (bmi160_offset.accel_k_xyz[0][1] == 1) && (bmi160_offset.accel_k_xyz[0][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[1][0] == 1) && (bmi160_offset.accel_k_xyz[1][1] == 1) && (bmi160_offset.accel_k_xyz[1][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[2][0] == 1) && (bmi160_offset.accel_k_xyz[2][1] == 1) && (bmi160_offset.accel_k_xyz[2][2] == 1) )
	{
		gSensorCalStatus = GSENSORCAL_1;
	}
	else
	{
		gSensorCalStatus = GSENSORCAL_6;
	}


	for(i = 0,j = 0; i < accel_cal_count ; i++)
       {
       	com_rslt = BSP_ACCELERO_Get_Axes( CAL_SENSOR_X_0_handle, &acceleration_tmp);
		if (com_rslt != COMPONENT_OK)
           	{
             		printf("read accel rawdata fail\n");
             		return COMPONENT_ERROR;
           	}

		if(x < abs(acceleration_tmp.AXIS_X - 1000))
		{
			offset_max[0][0] = acceleration_tmp.AXIS_X;
			x = abs(acceleration_tmp.AXIS_X - 1000);
		}
		if(y < abs(acceleration_tmp.AXIS_Y))
		{
			offset_max[0][1] = acceleration_tmp.AXIS_Y;
			y = abs(acceleration_tmp.AXIS_Y);
		}
		if(z < abs(acceleration_tmp.AXIS_Z))
		{
			offset_max[0][2] = acceleration_tmp.AXIS_Z;
			z = abs(acceleration_tmp.AXIS_Z);
		}

		//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
		if((((ACCEL_1000MG - ACCEL_BASE_VALUE))<acceleration_tmp.AXIS_X) && (acceleration_tmp.AXIS_X<((ACCEL_1000MG + ACCEL_BASE_VALUE))) && \
   	  		((-ACCEL_BASE_VALUE<acceleration_tmp.AXIS_Y)) && (acceleration_tmp.AXIS_Y<ACCEL_BASE_VALUE) && \
   	  		((-ACCEL_BASE_VALUE<acceleration_tmp.AXIS_Z)) && (acceleration_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
   		{

   		}
   		else
   		{
			cal_flag++;
   		}
		HAL_Delay(1);
	}}
	/**************XA END*******************************************************************/
	if((XB02 == pcb_id)||(XC01 == pcb_id) || (XC02 == pcb_id)){
	/**************XB*****************************************************************************/
	if((800 > raw_y) ||(raw_y > 1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:x = %.1f y = %.1f z = %.1f\n",raw_x,raw_y,raw_z); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}

	acc_calvalue.accel_x = raw_x;
	acc_calvalue.accel_y = raw_y - 1000;
	acc_calvalue.accel_z = raw_z;
    	for( i = 0; i < 3; ++i )
    	{
        	for( j = 0; j < 3; ++j )
        	{
           		 acc_calvalue.accel_k_xyz[i][j] = 1;
        	}
    	}
	saveAccelOffsetValue((void *)(&acc_calvalue));
	printf("SaveData accel:offsetx = %f offsety = %f offsetz =%f\n",
		acc_calvalue.accel_x,acc_calvalue.accel_y,acc_calvalue.accel_z);

	if(getBmi160OffsetValue((void *)(&bmi160_offset)) != 0)//read accel and gyro offset value
	{
		return COMPONENT_ERROR;
	}
	if((bmi160_offset.accel_k_xyz[0][0] == 1) && (bmi160_offset.accel_k_xyz[0][1] == 1) && (bmi160_offset.accel_k_xyz[0][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[1][0] == 1) && (bmi160_offset.accel_k_xyz[1][1] == 1) && (bmi160_offset.accel_k_xyz[1][2] == 1) &&
	    (bmi160_offset.accel_k_xyz[2][0] == 1) && (bmi160_offset.accel_k_xyz[2][1] == 1) && (bmi160_offset.accel_k_xyz[2][2] == 1) )
	{
		gSensorCalStatus = GSENSORCAL_1;
	}
	else
	{
		gSensorCalStatus = GSENSORCAL_6;
	}


	for(i = 0,j = 0; i < accel_cal_count ; i++)
       {
       	com_rslt = BSP_ACCELERO_Get_Axes( CAL_SENSOR_X_0_handle, &acceleration_tmp);
		if (com_rslt != COMPONENT_OK)
           	{
             		printf("read accel rawdata fail\n");
             		return COMPONENT_ERROR;
           	}

		if(x < abs(acceleration_tmp.AXIS_X))
		{
			offset_max[0][0] = acceleration_tmp.AXIS_X;
			x = abs(acceleration_tmp.AXIS_X);
		}
		if(y < abs(acceleration_tmp.AXIS_Y - 1000))
		{
			offset_max[0][1] = acceleration_tmp.AXIS_Y;
			y = abs(acceleration_tmp.AXIS_Y - 1000);
		}
		if(z < abs(acceleration_tmp.AXIS_Z))
		{
			offset_max[0][2] = acceleration_tmp.AXIS_Z;
			z = abs(acceleration_tmp.AXIS_Z);
		}

		//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
		if((((ACCEL_1000MG - ACCEL_BASE_VALUE))<acceleration_tmp.AXIS_Y) && (acceleration_tmp.AXIS_Y<((ACCEL_1000MG + ACCEL_BASE_VALUE))) && \
   	  		((-ACCEL_BASE_VALUE<acceleration_tmp.AXIS_X)) && (acceleration_tmp.AXIS_X<ACCEL_BASE_VALUE) && \
   	  		((-ACCEL_BASE_VALUE<acceleration_tmp.AXIS_Z)) && (acceleration_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
   		{

   		}
   		else
   		{
			cal_flag++;
   		}
		HAL_Delay(1);
	}}
	/**************XB END*******************************************************************/
	if(cal_flag !=0)
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"axis_1:   xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[0][0], offset_max[0][1], offset_max[0][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

	 	 clearAccValue();
		 return COMPONENT_ERROR;
	 }
	return COMPONENT_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/*  * * * * * * * * * * * * * * * * k  6 * * * * * *  * * * * * * * * *////////////////////////////////////

double calculate_A( double src[][BMI160_MAX], int n )
{
    int i,j,k,x,y;
    double tmp[BMI160_MAX][BMI160_MAX], t;
    double result = 0.0;

    if( n == 1 )
    {
        return src[0][0];
    }

    for( i = 0; i < n; ++i )
    {
        for( j = 0; j < n - 1; ++j )
        {
            for( k = 0; k < n - 1; ++k )
            {
                x = j + 1;
                y = k >= i ? k + 1 : k;

                tmp[j][k] = src[x][y];
            }
        }

        t = calculate_A( tmp, n - 1 );

        if( i % 2 == 0 )
        {
            result += src[0][i] * t;
        }
        else
        {
            result -= src[0][i] * t;
        }
    }

    return result;
}
void calculate_A_adjoint( double src[BMI160_MAX][BMI160_MAX], double dst[BMI160_MAX][BMI160_MAX], int n )
{
    int i, j, k, t, x, y;
    double tmp[BMI160_MAX][BMI160_MAX];

    if( n == 1 )
    {
        dst[0][0] = 1;
        return;
    }

    for( i = 0; i < n; ++i )
    {
        for( j = 0; j < n; ++j )
        {
            for( k = 0; k < n - 1; ++k )
            {
                for( t = 0; t < n - 1; ++t )
                {
                    x = k >= i ? k + 1 : k ;
                    y = t >= j ? t + 1 : t;

                    tmp[k][t] = src[x][y];
                }
            }

            dst[j][i]  =  calculate_A( tmp, n - 1 );

            if( ( i + j ) % 2 == 1 )
            {
                dst[j][i] = -1*dst[j][i];
            }
        }
    }
}
int calculate_A_inverse( double src[BMI160_MAX][BMI160_MAX], double dst[BMI160_MAX][BMI160_MAX], int n )
{
    double A = calculate_A( src, n );
    double tmp[BMI160_MAX][BMI160_MAX];
    int i, j;

    if ( fabs( A - 0 ) <= BMI160_E )
    {
        printf("Acc Calibate fail!\n");
        return 0;
    }

    calculate_A_adjoint( src, tmp, n );

    for( i = 0; i < n; ++i )
    {
        for( j = 0; j < n; ++j )
        {
            dst[i][j] = (double)( tmp[i][j] / A );
        }
    }

    return 1;
}
void print_M( double M[][BMI160_MAX], int n )
{
    int i, j;

    for ( i = 0; i < n; ++i )
    {
        for ( j = 0; j < n; ++j )
        {
           printf("%lf ", M[i][j]);
        }

        printf("\n");
    }
}
int test_bmi160(void)
{
    double test[BMI160_MAX][BMI160_MAX], dst[BMI160_MAX][BMI160_MAX];
    int n = 3;
    int is_exist;

    /**
     *  1, 0, 0,
     *  0, 2, 0,
     *  0, 0, 5
     */
    memset( test, 0, sizeof( test ) );

    test[0][0] = 1.0;
	test[0][1] = 2.0;
	test[0][2] = 3.0;

	test[1][0] = 2.0;
    test[1][1] = 2.0;
    test[1][2] = 1.0;

	test[2][0] = 3.0;
    test[2][1] = 4.0;
    test[2][2] = 3.0;




    is_exist = calculate_A_inverse( test, dst, n );

    if ( is_exist )
    {
        print_M(dst, n);
    }
    else
    {
        printf("Acc Calibate fail!\n");
    }

    return 0;
}

DrvStatusTypeDef  read_accel_xup(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];

	xup_x = 0;
	xup_y = 0;
	xup_z = 0;
	if(Cal_initializeAccSensor() != COMPONENT_OK) //gsensor init
	{
		printf("gsensor cal fail,init fail\n");
		return COMPONENT_ERROR;
	}
	if(VR_flash_erase(ACC_XUP_FLASHID,0x1)!=0)
	{
		return COMPONENT_ERROR;
	}

	 for(i = 0,j = 0; i < accel_cal_count ; i++)
        {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_xup fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_XUP_FLASHID;
			 accel_tmp.end_id = ACC_XUP_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_XUP_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           	   xup_x += acceleration.AXIS_X;
		   xup_y += acceleration.AXIS_Y;
		   xup_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}


	xup_x = xup_x/i;
	xup_y = xup_y/i;
	xup_z = xup_z/i;
	if((-800 < xup_x) ||(xup_x < -1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:xup_x = %.1f xup_y = %.1f xup_z = %.1f\n",xup_x,xup_y,xup_z); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}

	printf("accel:xup_x = %f xup_y = %f xup_z = %f\n",
		xup_x,xup_y,xup_z);
	return COMPONENT_OK;
}
DrvStatusTypeDef read_accel_xdown(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];

	xdown_x = 0;
	xdown_y = 0;
	xdown_z = 0;
	for(i = 0,j = 0; i < accel_cal_count ; i++)
       {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_xdown fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_XDOWN_FLASHID;
			 accel_tmp.end_id = ACC_XDOWN_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_XDOWN_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           xdown_x += acceleration.AXIS_X;
	    xdown_y += acceleration.AXIS_Y;
	    xdown_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}


	xdown_x = xdown_x/i;
	xdown_y = xdown_y/i;
	xdown_z = xdown_z/i;

	if((xdown_x < 800) ||(xdown_x >1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:xdown_x = %.1f xdown_y = %.1f xdown_z = %.1f\n",xdown_x,xdown_y,xdown_z);
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}
	printf("accel:xdown_x = %f xdown_y = %f xdown_z = %f\n",
		xdown_x,xdown_y,xdown_z);
	return COMPONENT_OK;
}

DrvStatusTypeDef  read_accel_yup(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];

	yup_x = 0;
	yup_y = 0;
	yup_z = 0;

	for(i = 0,j = 0; i < accel_cal_count ; i++)
       {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_yup fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_YUP_FLASHID;
			 accel_tmp.end_id = ACC_YUP_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_YUP_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           yup_x += acceleration.AXIS_X;
	    yup_y += acceleration.AXIS_Y;
	    yup_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}

	yup_x = yup_x/i;
	yup_y = yup_y/i;
	yup_z = yup_z/i;

	if((-800 < yup_y) ||(yup_y < -1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:yup_x = %.1f yup_y = %.1f yup_z = %.1f\n",yup_x,yup_y,yup_z);
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}
	printf("accel:yup_x = %f yup_y = %f yup_z = %f\n",
		yup_x,yup_y,yup_z);
	return COMPONENT_OK;
}
DrvStatusTypeDef read_accel_ydown(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];

	ydown_x = 0;
	ydown_y = 0;
	ydown_z = 0;

	  for(i = 0,j =0; i < accel_cal_count ; i++)
        {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_ydown fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_YDOWN_FLASHID;
			 accel_tmp.end_id = ACC_YDOWN_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_YDOWN_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           ydown_x += acceleration.AXIS_X;
	    ydown_y += acceleration.AXIS_Y;
	    ydown_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}

	ydown_x = ydown_x/i;
	ydown_y = ydown_y/i;
	ydown_z = ydown_z/i;
	if((ydown_y < 800) ||(ydown_y > 1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:ydown_x = %.1f ydown_y = %.1f ydown_z = %.1f\n",ydown_x,ydown_y,ydown_z);
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}
	printf("accel:ydown_x = %f ydown_y = %f ydown_z = %f\n",
		ydown_x,ydown_y,ydown_z);
	return COMPONENT_OK;
}

DrvStatusTypeDef  read_accel_zup(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];

	zup_x = 0;
	zup_y = 0;
	zup_z = 0;
	for(i = 0,j = 0; i < accel_cal_count ; i++)
       {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_zup fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_ZUP_FLASHID;
			 accel_tmp.end_id = ACC_ZUP_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_ZUP_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           zup_x += acceleration.AXIS_X;
	    zup_y += acceleration.AXIS_Y;
	    zup_z += acceleration.AXIS_Z;
          HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}

	zup_x = zup_x/i;
	zup_y = zup_y/i;
	zup_z = zup_z/i;

	if((-800 < zup_z) ||(zup_z < -1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:zup_x = %.1f zup_y = %.1f zup_z = %.1f\n",zup_x,zup_y,zup_z);
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}
	printf("accel:zup_x = %f zup_y = %f zup_z = %f\n",
		zup_x,zup_y,zup_z);
	return COMPONENT_OK;
}
DrvStatusTypeDef read_accel_zdown(void)
{
	DrvStatusTypeDef com_rslt = COMPONENT_ERROR;
	int i = 0,j = 0;
	SensorAxes_t_k6 acceleration;
	Save_str_k6	accel_tmp;
	uint8_t buffer[128];


	zdown_x = 0;
	zdown_y = 0;
	zdown_z = 0;
	for(i = 0 ; i < accel_cal_count ; i++)
       {
           j++;
           com_rslt = X_Get_Axes_k6(&acceleration,&accel_tmp.accel[j-1]);
           if (com_rslt != COMPONENT_OK)
           {
             printf("read accel_zup fail\n");
             return COMPONENT_ERROR;
           }
		   gSensorWDdogRefresh();
		   if(j%ACC_WRITEFLASH_SIZE == 0)
		   {
			 j = 0;
			 accel_tmp.start_id = ACC_ZDOWN_FLASHID;
			 accel_tmp.end_id = ACC_ZDOWN_FLASHID;
			 //write flash
			 if(VR_flash_write(&accel_tmp,ACC_ZDOWN_FLASHID + ((i+1)/ACC_WRITEFLASH_SIZE - 1)*ONEPACKAGESIZE ,sizeof(Save_str_k6))!=0)
			 {
				return COMPONENT_ERROR;
			 }
		   }
           zdown_x += acceleration.AXIS_X;
	    zdown_y += acceleration.AXIS_Y;
	    zdown_z += acceleration.AXIS_Z;
           HAL_Delay(2);/* bmi160_delay_ms in ms*/
    	}
	zdown_x = zdown_x/i;
	zdown_y = zdown_y/i;
	zdown_z = zdown_z/i;

	if(( zdown_z < 800) ||(zdown_z > 1200) )
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"accel:zdown_x = %.1f zdown_y = %.1f zdown_z = %.1f\n",zdown_x,zdown_y,zdown_z);
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		return COMPONENT_ERROR;
	}
	printf("accel:zdown_x = %f zdown_y = %f zdown_z = %f\n",
		zdown_x,zdown_y,zdown_z);
	return COMPONENT_OK;
}


DrvStatusTypeDef accel_k6andtest(void)
{
	if(ACC_Calibate_6() !=0)
	{
		usb_cdc_transmit_data("matrix fail\n",strlen("matrix fail\n"));
		return COMPONENT_ERROR;
	}

	//test
	printf("cal test\n");
	if(ACC_fd_test() !=0)
	{
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

DrvStatusTypeDef ACC_Calibate_6(void)
{
	BMI160_DataTypeDef acc_calvalue = {0};
	double test[BMI160_MAX][BMI160_MAX], dst[BMI160_MAX][BMI160_MAX];
	int is_exist = 0;
	int i = 0,j = 0;

	#if 0
	test[0][0] = (double)((xdown_x - xup_x)/2);// 1000mg  kxx
	test[0][1] = (double)((xdown_y - xup_y)/2);// kxy
	test[0][2] = (double)((xdown_z - xup_z)/2);// kxz
	acc_calvalue.accel_x= (xdown_x + xup_x)/2;

	test[1][0] = (double)((ydown_x - yup_x)/2);// kyx
	test[1][1] = (double)((ydown_y - yup_y)/2);// 1000mg  kyy
	test[1][2] = (double)((ydown_z - yup_z)/2);// kyz
	acc_calvalue.accel_y= (ydown_y + yup_y)/2;

	test[2][0] = (double)((zdown_x - zup_x)/2);// kzx
	test[2][1] = (double)((zdown_y - zup_y)/2);// kzy
	test[2][2] = (double)((zdown_z - zup_z)/2);// 1000mg  kzz
	acc_calvalue.accel_z= (zdown_z + zup_z)/2;
	#endif

	test[0][0] = (double)((xdown_x - xup_x)/2);// 1000mg  kxx
	test[1][0] = (double)((xdown_y - xup_y)/2);// kxy
	test[2][0] = (double)((xdown_z - xup_z)/2);// kxz
	acc_calvalue.accel_x= (xdown_x + xup_x)/2;

	test[0][1] = (double)((ydown_x - yup_x)/2);// kyx
	test[1][1] = (double)((ydown_y - yup_y)/2);// 1000mg  kyy
	test[2][1] = (double)((ydown_z - yup_z)/2);// kyz
	acc_calvalue.accel_y= (ydown_y + yup_y)/2;

	test[0][2] = (double)((zdown_x - zup_x)/2);// kzx
	test[1][2] = (double)((zdown_y - zup_y)/2);// kzy
	test[2][2] = (double)((zdown_z - zup_z)/2);// 1000mg  kzz
	acc_calvalue.accel_z= (zdown_z + zup_z)/2;

	gSensorWDdogRefresh();
	is_exist = calculate_A_inverse( test, dst, 3 );
	for( i = 0; i < 3; ++i )
    	{
        	for( j = 0; j < 3; ++j )
        	{
           		dst[i][j] = 1000 * dst[i][j];
        	}
    	}
	if ( is_exist )
    	{
        	print_M(dst, 3);
    	}
    	else
    	{
        	printf("Acc Calibate fail!\n");
		return COMPONENT_ERROR;
    	}
    	for( i = 0; i < 3; ++i )
    	{
        	for( j = 0; j < 3; ++j )
        	{
           		 acc_calvalue.accel_k_xyz[i][j] = dst[i][j];
        	}
    	}
	gSensorWDdogRefresh();
	saveAccelOffsetValue((void *)(&acc_calvalue));
	printf("SaveData accel:offsetx = %f offsety = %f offsetz =%f\n",
		acc_calvalue.accel_x,acc_calvalue.accel_y,acc_calvalue.accel_z);

	return COMPONENT_OK;
}
DrvStatusTypeDef ACC_fd_test(void)
{
	Save_str_k6	accel_k6;
	int i = 0,j = 0;
	SensorAxes_t accel_tmp = {0};
	BMI160_DataTypeDef bmi160_offset_tmp;
	float sensitivity = 0;
	int32_t x= 0,y = 0,z = 0;
	int32_t cal_flag = 0;
	int32_t offset_max[6][3];
	uint8_t buffer[128];
	#if 1
	bmi160_offset_tmp.isAccValid = 0;
	getBmi160OffsetValue((void *)(&bmi160_offset_tmp));
	if(bmi160_offset_tmp.isAccValid != 1)
	{
		printf("Acc [has not calibrated]\r\n");
		return COMPONENT_ERROR;
	 }
	#endif
	/* Get BMI160 actual sensitivity. */
	if ( BSP_GYRO_Get_Sensitivity(CAL_SENSOR_X_0_handle, &sensitivity ) != COMPONENT_OK )
	{
		return COMPONENT_ERROR;
	}


	 cal_flag = 0;
	//test xup
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_XUP_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_XUP_FLASHID) && (accel_k6.end_id== ACC_XUP_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X + 1000))
				{
					offset_max[0][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X + 1000);
				}
				if(y < abs(accel_tmp.AXIS_Y))
				{
					offset_max[0][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y);
				}
				if(z < abs(accel_tmp.AXIS_Z))
				{
					offset_max[0][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z);
				}

				//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
				if(((-(ACCEL_1000MG + ACCEL_BASE_VALUE))<accel_tmp.AXIS_X) && (accel_tmp.AXIS_X<(-(ACCEL_1000MG - ACCEL_BASE_VALUE))) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Y)) && (accel_tmp.AXIS_Y<ACCEL_BASE_VALUE) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Z)) && (accel_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
		   		{

		   		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}

	x = 0;
	y = 0;
	z = 0;
	//test xdown
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_XDOWN_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_XDOWN_FLASHID) && (accel_k6.end_id== ACC_XDOWN_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X - 1000))
				{
					offset_max[1][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X - 1000);
				}
				if(y < abs(accel_tmp.AXIS_Y))
				{
					offset_max[1][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y);
				}
				if(z < abs(accel_tmp.AXIS_Z))
				{
					offset_max[1][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z);
				}

				//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
				if((((ACCEL_1000MG - ACCEL_BASE_VALUE))<accel_tmp.AXIS_X) && (accel_tmp.AXIS_X<((ACCEL_1000MG + ACCEL_BASE_VALUE))) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Y)) && (accel_tmp.AXIS_Y<ACCEL_BASE_VALUE) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Z)) && (accel_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
		   		{

		  		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}
	x = 0;
	y = 0;
	z = 0;
	//test yup
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_YUP_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_YUP_FLASHID) && (accel_k6.end_id== ACC_YUP_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X ))
				{
					offset_max[2][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X );
				}
				if(y < abs(accel_tmp.AXIS_Y + 1000))
				{
					offset_max[2][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y +1000);
				}
				if(z < abs(accel_tmp.AXIS_Z))
				{
					offset_max[2][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z);
				}
				//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
				if(((-(ACCEL_1000MG + ACCEL_BASE_VALUE))<accel_tmp.AXIS_Y) && (accel_tmp.AXIS_Y<(-(ACCEL_1000MG - ACCEL_BASE_VALUE))) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_X)) && (accel_tmp.AXIS_X<ACCEL_BASE_VALUE) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Z)) && (accel_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
		   		{

		   		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}
	//test ydown
	x = 0;
	y = 0;
	z = 0;
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_YDOWN_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_YDOWN_FLASHID) && (accel_k6.end_id== ACC_YDOWN_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X ))
				{
					offset_max[3][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X );
				}
				if(y < abs(accel_tmp.AXIS_Y - 1000))
				{
					offset_max[3][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y -1000);
				}
				if(z < abs(accel_tmp.AXIS_Z))
				{
					offset_max[3][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z);
				}
				if((((ACCEL_1000MG - ACCEL_BASE_VALUE))<accel_tmp.AXIS_Y) && (accel_tmp.AXIS_Y<((ACCEL_1000MG + ACCEL_BASE_VALUE))) && \
		   	 		 ((-ACCEL_BASE_VALUE<accel_tmp.AXIS_X)) && (accel_tmp.AXIS_X<ACCEL_BASE_VALUE) && \
		   	  		 ((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Z)) && (accel_tmp.AXIS_Z<ACCEL_BASE_VALUE)    )
		   		{

		  		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}
	x = 0;
	y = 0;
	z = 0;
	//test zup
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_ZUP_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_ZUP_FLASHID) && (accel_k6.end_id== ACC_ZUP_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X ))
				{
					offset_max[4][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X );
				}
				if(y < abs(accel_tmp.AXIS_Y))
				{
					offset_max[4][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y );
				}
				if(z < abs(accel_tmp.AXIS_Z + 1000))
				{
					offset_max[4][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z +1000);
				}
				//printf("x = %d y = %d z = %d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z);
				if(((-(ACCEL_1000MG + ACCEL_BASE_VALUE))<accel_tmp.AXIS_Z) && (accel_tmp.AXIS_Z<(-(ACCEL_1000MG - ACCEL_BASE_VALUE))) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_X)) && (accel_tmp.AXIS_X<ACCEL_BASE_VALUE) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Y)) && (accel_tmp.AXIS_Y<ACCEL_BASE_VALUE)    )
		   		{

		   		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}
	//test zdown
	x = 0;
	y = 0;
	z = 0;
	for(i = 0;i < 30;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_ZDOWN_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.start_id == ACC_ZDOWN_FLASHID) && (accel_k6.end_id== ACC_ZDOWN_FLASHID))
		{
			for(j =0;j<100;j++)
			{
				accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  					bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   					bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

				if(x < abs(accel_tmp.AXIS_X ))
				{
					offset_max[5][0] = accel_tmp.AXIS_X;
					x = abs(accel_tmp.AXIS_X );
				}
				if(y < abs(accel_tmp.AXIS_Y))
				{
					offset_max[5][1] = accel_tmp.AXIS_Y;
					y = abs(accel_tmp.AXIS_Y );
				}
				if(z < abs(accel_tmp.AXIS_Z - 1000))
				{
					offset_max[5][2] = accel_tmp.AXIS_Z;
					z = abs(accel_tmp.AXIS_Z -1000);
				}
				  if((((ACCEL_1000MG - ACCEL_BASE_VALUE))<accel_tmp.AXIS_Z) && (accel_tmp.AXIS_Z<((ACCEL_1000MG + ACCEL_BASE_VALUE))) && \
		   	  		((-ACCEL_BASE_VALUE<accel_tmp.AXIS_X)) && (accel_tmp.AXIS_X<ACCEL_BASE_VALUE) && \
		   	 		 ((-ACCEL_BASE_VALUE<accel_tmp.AXIS_Y)) && (accel_tmp.AXIS_Y<ACCEL_BASE_VALUE)    )
		   		{

		   		}
		   		else
		   		{
					cal_flag++;
		   		}
			}
		}
		else
		{
			usb_cdc_transmit_data("read flash fail\n",strlen("read flash fail\n"));
			return COMPONENT_ERROR;
		}
	}


	if(cal_flag !=0)
	{
		 memset(buffer,'\0',128);
		 sprintf( (char *)buffer,"xup:   xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[0][0], offset_max[0][1], offset_max[0][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

		 sprintf( (char *)buffer,"xdown: xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[1][0], offset_max[1][1], offset_max[1][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

		 sprintf( (char *)buffer,"yup:   xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[2][0], offset_max[2][1], offset_max[2][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

		 sprintf( (char *)buffer,"ydown: xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[3][0], offset_max[3][1], offset_max[3][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

		 sprintf( (char *)buffer ,"zup:   xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[4][0], offset_max[4][1], offset_max[4][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);
		 memset(buffer,'\0',128);

		 sprintf( (char *)buffer,"zdown: xmax =%dmg\tymax=%dmg\tzmax =%dmg\r\n", offset_max[5][0], offset_max[5][1], offset_max[5][2]); // C4996
		 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		 printf("%s",buffer);

	 	 clearAccValue();
		return COMPONENT_ERROR;
	 }
	return COMPONENT_OK;
}

DrvStatusTypeDef dump_accdata(void)
{
	Save_str_k6	accel_k6;
	int i = 0,j = 0;
	SensorAxes_t accel_tmp = {0};
	BMI160_DataTypeDef bmi160_offset_tmp = {0};
	float sensitivity = 0;
	SensorAxes_t acc_rawdata;
	uint8_t  buffer[256];

	if(Cal_initializeAccSensor() != COMPONENT_OK) //gsensor init
	{
		printf("gsensor cal fail,init fail\n");
		return COMPONENT_ERROR;
	}
	#if 1
	bmi160_offset_tmp.isAccValid = 0;
	getBmi160OffsetValue((void *)(&bmi160_offset_tmp));
	#endif
	//Cal_initializeAccSensor();
	/* Get BMI160 actual sensitivity. */
	if ( BSP_GYRO_Get_Sensitivity(CAL_SENSOR_X_0_handle, &sensitivity ) != COMPONENT_OK )
	{
		printf("read accel sensitivity fail\n");
	}


	///////////////////////////////////////////
	 memset(buffer,'\0',256);
	 j = 0;
	 j  = sprintf( (char *)buffer,    "k 6_axis offset: x = %f y = %fz = %f\n",   bmi160_offset_tmp.accel_x,
											  					   bmi160_offset_tmp.accel_y,
											  					   bmi160_offset_tmp.accel_z);

	 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
	 //////////////////////////////////////////
	memset(buffer,'\0',256);
	 j = 0;
	 j  = sprintf( (char *)buffer,   "%f\t%f\t%f\n",bmi160_offset_tmp.accel_k_xyz[0][0],
	 									    bmi160_offset_tmp.accel_k_xyz[0][1],
	 									    bmi160_offset_tmp.accel_k_xyz[0][2]) ;

	 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
	 //////////////////////////////////////////
	memset(buffer,'\0',256);
	 j = 0;
	 j  = sprintf( (char *)buffer,   "%f\t%f\t%f\n",bmi160_offset_tmp.accel_k_xyz[1][0],
	 									    bmi160_offset_tmp.accel_k_xyz[1][1],
	 									    bmi160_offset_tmp.accel_k_xyz[1][2]) ;
	 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
	 //////////////////////////////////////////
	memset(buffer,'\0',256);
	 j = 0;
	 j  = sprintf( (char *)buffer,   "%f\t%f\t%f\n",bmi160_offset_tmp.accel_k_xyz[2][0],
	 									    bmi160_offset_tmp.accel_k_xyz[2][1],
	 									    bmi160_offset_tmp.accel_k_xyz[2][2]) ;
	 ////////////////////////////////////////////////////////////////////////////dump 1 axis
	 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
	 if((bmi160_offset_tmp.accel_k_xyz[0][0] == 1) && (bmi160_offset_tmp.accel_k_xyz[0][1] == 1) && (bmi160_offset_tmp.accel_k_xyz[0][2] == 1) &&
	    (bmi160_offset_tmp.accel_k_xyz[1][0] == 1) && (bmi160_offset_tmp.accel_k_xyz[1][1] == 1) && (bmi160_offset_tmp.accel_k_xyz[1][2] == 1) &&
	    (bmi160_offset_tmp.accel_k_xyz[2][0] == 1) && (bmi160_offset_tmp.accel_k_xyz[2][1] == 1) && (bmi160_offset_tmp.accel_k_xyz[2][2] == 1) )
	{
		for(i = 0;i < 30;i++)
		{
			gSensorWDdogRefresh();
			VR_flash_read(ACC_XUP_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
			if((accel_k6.start_id == ACC_XUP_FLASHID) && (accel_k6.end_id== ACC_XUP_FLASHID))
			{
				for(j =0;j<100;j++)
				{
					accel_tmp.AXIS_X = ( int32_ts )((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) ;

					accel_tmp.AXIS_Y = ( int32_ts )((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) ;

					accel_tmp.AXIS_Z = ( int32_ts )((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z);

					acc_rawdata.AXIS_X = ( int32_ts )( (accel_k6.accel[j].AXIS_X) * (sensitivity) );
			  		acc_rawdata.AXIS_Y = ( int32_ts )( (accel_k6.accel[j].AXIS_Y) * (sensitivity) );
			 		acc_rawdata.AXIS_Z = ( int32_ts )( (accel_k6.accel[j].AXIS_Z) * (sensitivity) );

					 //////////////////////////////////////////
					memset(buffer,'\0',256);
					sprintf( (char *)buffer,  "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z,
													   			   acc_rawdata.AXIS_X,acc_rawdata.AXIS_Y,acc_rawdata.AXIS_Z,
													   			   accel_k6.accel[j].AXIS_X,accel_k6.accel[j].AXIS_Y,accel_k6.accel[j].AXIS_Z);

					 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
			             //////////////////////////////////////////

				}

			}
			else
			{
				usb_cdc_transmit_data("Gsensor data is erased\r\n ",strlen("Gsensor data is erased\r\n"));
				return COMPONENT_ERROR;
			}
		}

		return COMPONENT_OK;
	}
	 //////////////////////////////////////////

	for(i = 0;i<180;i++)
	{
		gSensorWDdogRefresh();
		VR_flash_read(ACC_XUP_FLASHID + i*ONEPACKAGESIZE, &accel_k6, sizeof(Save_str_k6));
		if((accel_k6.end_id == ACC_XUP_FLASHID) || (accel_k6.end_id == ACC_XDOWN_FLASHID) ||(accel_k6.end_id == ACC_YUP_FLASHID) ||
		   (accel_k6.end_id == ACC_YDOWN_FLASHID) || (accel_k6.end_id == ACC_ZUP_FLASHID) ||(accel_k6.end_id == ACC_ZDOWN_FLASHID ))
		{
		}
		else
		{
			usb_cdc_transmit_data("Gsensor data is erased\r\n ",strlen("Gsensor data is erased\r\n"));
			return COMPONENT_ERROR;
		}
		for(j =0;j<100;j++)
		{
			accel_tmp.AXIS_X = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[0][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   			bmi160_offset_tmp.accel_k_xyz[0][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   			bmi160_offset_tmp.accel_k_xyz[0][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

			accel_tmp.AXIS_Y = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[1][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							  			bmi160_offset_tmp.accel_k_xyz[1][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							  			bmi160_offset_tmp.accel_k_xyz[1][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

			accel_tmp.AXIS_Z = ( int32_ts )(bmi160_offset_tmp.accel_k_xyz[2][0]*((accel_k6.accel[j].AXIS_X)*(sensitivity)-bmi160_offset_tmp.accel_x) + \
							   			bmi160_offset_tmp.accel_k_xyz[2][1]*((accel_k6.accel[j].AXIS_Y)*(sensitivity)-bmi160_offset_tmp.accel_y) + \
							   			bmi160_offset_tmp.accel_k_xyz[2][2]*((accel_k6.accel[j].AXIS_Z)*(sensitivity)-bmi160_offset_tmp.accel_z));

			acc_rawdata.AXIS_X = ( int32_ts )( (accel_k6.accel[j].AXIS_X) * (sensitivity) );
	  		acc_rawdata.AXIS_Y = ( int32_ts )( (accel_k6.accel[j].AXIS_Y) * (sensitivity) );
	 		acc_rawdata.AXIS_Z = ( int32_ts )( (accel_k6.accel[j].AXIS_Z) * (sensitivity) );

			 //////////////////////////////////////////
			memset(buffer,'\0',256);
			sprintf( (char *)buffer,  "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",accel_tmp.AXIS_X,accel_tmp.AXIS_Y,accel_tmp.AXIS_Z,
											   			   acc_rawdata.AXIS_X,acc_rawdata.AXIS_Y,acc_rawdata.AXIS_Z,
											   			   accel_k6.accel[j].AXIS_X,accel_k6.accel[j].AXIS_Y,accel_k6.accel[j].AXIS_Z);

			 usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
	             //////////////////////////////////////////

		}
	}
	return COMPONENT_OK;
}

int cmd_gsensor_check(char cmdsource)
{

	BMI160_DataTypeDef bmi160_offset_tmp = {0};
	uint32_t selftestvalue = 0;
	uint8_t buffer[128];
	getBmi160OffsetValue((void *)(&bmi160_offset_tmp));
	if(bmi160_offset_tmp.isAccValid == 1)
	{
		selftestvalue = getAccCalheaderValue();
		if(selftestvalue == 0x67676767)
		{
			memset(buffer,'\0',128);
			sprintf( (char *)buffer,"Acc [calibrated pass, x = %f, y = %f, z = %f]\n",
					bmi160_offset_tmp.accel_x, bmi160_offset_tmp.accel_y,bmi160_offset_tmp.accel_z); // C4996
			if(cmdsource ==1)
			{
				usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
			}
			else
			{
				printf("%s", buffer);
			}
		}
		else
		{
			memset(buffer,'\0',128);
			sprintf( (char *)buffer,"Acc [calibrated fail]\r\n"); // C4996
			if(cmdsource ==1)
			{
				usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
			}
			else
			{
				printf("%s", buffer);
			}
		}
	}
	else
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Acc [has not calibrated]\r\n"); // C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}
	selftestvalue = getAccSelfTestValue();
	if(selftestvalue == 0x67676767)
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Acc [self-tseted pass]\r\n"); // C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}
	else
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Acc [has not self-tseted]\r\n"); // C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}


    return 0 ;
}
int cmd_gyro_check(char cmdsource)
{

	BMI160_DataTypeDef bmi160_offset_tmp = {0};
	uint32_t selftestvalue = 0;
	uint8_t buffer[128];
	getBmi160OffsetValue((void *)(&bmi160_offset_tmp));
	if(bmi160_offset_tmp.isAccValid == 1)
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Gyro [calibrated pass, x = %d, y = %d, z = %d]\n",
					bmi160_offset_tmp.gyro_x, bmi160_offset_tmp.gyro_y,bmi160_offset_tmp.gyro_z); // C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}

	selftestvalue = getGyroSelfTestValue();
	if(selftestvalue == 0x67676767)
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Gyro [self-tseted pass]\r\n");// C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}
	else
	{
		memset(buffer,'\0',128);
		sprintf( (char *)buffer,"Gyro [has not self-tseted]\r\n");// C4996
		if(cmdsource ==1)
		{
			usb_cdc_transmit_data(buffer,strlen((char const *)buffer));
		}
		else
		{
			printf("%s", buffer);
		}
	}


    return 0 ;
}

DrvStatusTypeDef ACC_Calibate(unsigned char *buf)
{
	 if (!strcmp((char const*)buf, "xup")) //k_acc  xup
	{
		if(read_accel_xup() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
		}
	} else if (!strcmp((char const*)buf, "xdown"))//k_acc  x_down
	{
		if(read_accel_xdown() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
		}
	} else if (!strcmp((char const*)buf, "yup"))//k_acc  yup
	{
		if(read_accel_yup() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
		}
	} else if (!strcmp((char const*)buf, "ydown"))//k_acc  ydown
	{
		if(read_accel_ydown() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
		}
	} else if (!strcmp((char const*)buf, "zup"))//k_acc  zup
	{
		if(read_accel_zup() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
		}
	} else if (!strcmp((char const*)buf, "zdown"))//k_acc zdown
	{
		if(read_accel_zdown() == 0)
		{
			usb_cdc_transmit_data("OK\n",3);
		}
		else
		{
			usb_cdc_transmit_data("fail\n",5);
			return COMPONENT_ERROR;
		}

		if(accel_k6andtest() ==0)
		{
			usb_cdc_transmit_data("calibration pass\n",strlen("calibration pass\n"));
		}
		else
		{
			usb_cdc_transmit_data("calibration fail\n",strlen("calibration fail\n"));
		}

	} else if (!strcmp((char const*)buf, "dump"))//dump data
	{
		dump_accdata();
	}else if (!strcmp((char const*)buf, "accheck"))//
	{
		cmd_gsensor_check(1);
	}else if (!strcmp((char const*)buf, "gyroheck"))//
	{
		cmd_gyro_check(1);
	}else if (!strcmp((char const*)buf, "axis_1"))//
	{
		if(cal_accel_1axis() ==0)
		{
			usb_cdc_transmit_data("calibration pass\n",strlen("calibration pass\n"));
		}
		else
		{
			usb_cdc_transmit_data("calibration fail\n",strlen("calibration fail\n"));
		}
	}
	return COMPONENT_OK;
}

