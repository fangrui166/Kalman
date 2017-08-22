#include "gSensor_Command.h"
#include "LSM6DSM_ACC_GYRO_driver_HL.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "stm32f4xx_hal.h"
#include "SensorTile.h"



extern void *SENSOR_X_0_handle;
extern void *SENSOR_G_0_handle;
extern int8_t usb_cdc_transmit_data(uint8_t *buf, uint8_t len);
extern char gsensorDebugFlag;
int shell_lsm6dsm_commad(int argc, char * argv[], _command_source source)
{
	uint8_t i,pBuffer;
	uint8_t buffer[64];
	int j = 0;
	SensorAxes_t angular_velocity;
	uint8_t sensorFlag = 0;
	
	Sensor_IO_SPI_Read( NULL, 0x0, &pBuffer, 1);
	if(pBuffer == 0xD1)
	{
		sensorFlag = 1;
		//printf("G-sensor is bmi160\n");
	}else
	{
		Sensor_IO_SPI_Read( NULL, 0x0F, &pBuffer, 1);
		if(pBuffer == 0x6A)
		{
			sensorFlag = 2;
			//printf("G-sensor is lsm6dsm\n");
		}else
		{
			//printf("G-sensor Unknown\n");
		}
	}


	if(sensorFlag == 2)
	{
			if (argc == 2) {
		
			if (!strcmp((char const*)argv[1], "?"))
			{
				gsensor_info("**********	Command list	**********\n");
				gsensor_info("%s\n%s\n%s\n%s\n", COMMAND_GSENSOR_DUMPREG, COMMAND_GSENSOR_READCHIPID,
										COMMAND_GSESNOR_READACC,COMMAND_GSENSOR_READGYRO);
				gsensor_info("**********end**********\n");
				return 0;
			}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_DUMPREG))
			{
				for(i = 0x1;i<0x72;i++)
				{
					Sensor_IO_Read(SENSOR_X_0_handle,i, &pBuffer, 1);
					gsensor_info("[LSM6DSM] addr = 0x%x value = 0x%x\n",i,pBuffer);
					HAL_Delay(1);
				}
				return 0;
			}
			else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READCHIPID))
			{
				LSM6DSM_ACC_GYRO_R_WHO_AM_I(SENSOR_X_0_handle, &pBuffer);
				gsensor_info("[LSM6DSM] chipid = 0x%x\n",pBuffer);
				return 0;
			}else if (!strcmp((char const*)argv[1], COMMAND_GSESNOR_READACC))
			{
				BSP_ACCELERO_Get_Axes( SENSOR_X_0_handle, &angular_velocity );
				gsensor_info("[LSM6DSM] accel: x = %dmg y = %dmg z = %dmg\n",angular_velocity.AXIS_X,angular_velocity.AXIS_Y,angular_velocity.AXIS_Z);
				return 0;
			}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READGYRO))
			{
				BSP_GYRO_Get_Axes( SENSOR_G_0_handle, &angular_velocity);
				gsensor_info("[LSM6DSM] gyro: x = %dmdps y = %dmdps z = %dmdps\n",angular_velocity.AXIS_X,angular_velocity.AXIS_Y,angular_velocity.AXIS_Z);
				return 0;
			}else
			{
				gsensor_err("input arguments ERROR!\n");
			}
		}else if(argc == 3)
		{
			if (!strcmp((char const*)argv[1], "rMag"))
			{
				if (!strcmp((char const*)argv[2], "dumpreg"))
				{
					  if( !LSM6DSM_ACC_GYRO_W_EmbeddedAccess(SENSOR_X_0_handle, LSM6DSM_ACC_GYRO_EMBEDDED_ACCESS_ENABLED))
					  	return -1;
					 for(i = 0x1;i<0x60;i++)
					{
						Sensor_IO_Read(SENSOR_X_0_handle,i, &pBuffer, 1);
						gsensor_info("[BankA] addr = 0x%x value = 0x%x\n",i,pBuffer);
						HAL_Delay(1);
					}
					 if( !LSM6DSM_ACC_GYRO_W_EmbeddedAccess(SENSOR_X_0_handle, LSM6DSM_ACC_GYRO_EMBEDDED_ACCESS_DISABLED))
					 	return -1;
				}
				return 0;
			}
		}else {
			gsensor_err("input arguments ERROR!\n");
		}
  }else if(sensorFlag == 1)
  {
		if (argc == 2) {
	
		if (!strcmp((char const*)argv[1], "?"))
		{
			gsensor_info("**********	Command list	**********\n");
			gsensor_info("%s\n%s\n%s\n%s\n", COMMAND_GSENSOR_DUMPREG, COMMAND_GSENSOR_READCHIPID,
									COMMAND_GSESNOR_READACC,COMMAND_GSENSOR_READGYRO);
			gsensor_info("**********end**********\n");
			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_DUMPREG))
		{
			for(i = 0x0;i<0x7E;i++)
			{
				bmi160_read_reg(i, &pBuffer, 1);
				gsensor_emerg("[BMI160] addr = 0x%x value = 0x%x\n",i,pBuffer);
				HAL_Delay(1);
			}
			return 0;
		}
		else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READCHIPID))
		{
			bmi160_read_reg(0x0,&pBuffer, 1);
			gsensor_info("[BMI160] chipid = 0x%x\n",pBuffer);
			if(source == CMD_SOURCE_USBCDC){
				memset(buffer,0,64);
				j = 0;
				j  =  sprintf( (char *)buffer,       "[BMI160] chipid = 0x%x\n",pBuffer); // C4996
				usb_cdc_transmit_data(buffer,strlen((char const*)buffer));
			}
			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSESNOR_READACC))
		{
			BSP_ACCELERO_Get_Axes( SENSOR_X_0_handle, &angular_velocity );
			gsensor_info("[BMI160] accel: x = %dmg y = %dmg z = %dmg\n",angular_velocity.AXIS_X,angular_velocity.AXIS_Y,angular_velocity.AXIS_Z);
			if(source == CMD_SOURCE_USBCDC){
				memset(buffer,0,64);
				j = 0;
				j  =  sprintf( (char *)buffer,       "x=%dmg\t", angular_velocity.AXIS_X); // C4996
				j += sprintf( (char *)buffer + j, "y=%dmg\t", angular_velocity.AXIS_Y); // C4996
				j += sprintf( (char *)buffer + j, "z=%dmg\t", angular_velocity.AXIS_Z); // C4996
				j += sprintf( (char *)buffer + j, "\r\n"); // C4996
				usb_cdc_transmit_data(buffer,strlen((char const*)buffer));
			}

			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_READGYRO))
		{
			BSP_GYRO_Get_Axes( SENSOR_G_0_handle, &angular_velocity);
			gsensor_info("[BMI160] gyro: x = %.2fmdps y = %.2fmdps z = %.2fmdps\n",
				(float)angular_velocity.AXIS_X/3600,(float)angular_velocity.AXIS_Y/3600,(float)angular_velocity.AXIS_Z/3600);
			if(source == CMD_SOURCE_USBCDC){
				memset(buffer,0,64);
				j = 0;
				j  = sprintf( (char *)buffer,       "x=%.2fmdps\t", (float)angular_velocity.AXIS_X/3600); // C4996
				j += sprintf( (char *)buffer + j, "y=%.2fmdps\t", (float)angular_velocity.AXIS_Y/3600); // C4996
				j += sprintf( (char *)buffer + j, "z=%.2fmdps\t", (float)angular_velocity.AXIS_Z/3600); // C4996
				j += sprintf( (char *)buffer + j, "\r\n");
				usb_cdc_transmit_data(buffer,strlen((char const*)buffer));
			}

			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_DUMPDATA))
		{
			gsensorDebugFlag = 1;
			return 0;
		}else if (!strcmp((char const*)argv[1], COMMAND_GSENSOR_DISABLEDATA))
		{
			gsensorDebugFlag = 0;
			return 0;
		}else
		{
			gsensor_err("input arguments ERROR!\n");
		}
	}else {
		gsensor_err("input arguments ERROR!\n");
	}

  }


	return 0;
}





















