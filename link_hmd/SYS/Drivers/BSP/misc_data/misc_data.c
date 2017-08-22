#include <string.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "misc_data.h"
#include "flash_drv.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "component.h"
#include "cmsis_os.h"
#include "htc_memory_define.h"
#include "gSensor_Calibate.h"
#include "htc_version.h"
#include "sys_header.h"
#include "htc_usb_cdc_data_service.h"
//#include "usbd_app.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MISC_DataTypeDef MISC_DataStructure = {0};
static osMutexId misc_read_mutex;
static osMutexId misc_write_mutex;
extern float disp_fps_get(void);

int read_count = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
int misc_data_init(void)
{
	osMutexDef(read_mutex);
	osMutexDef(write_mutex);
	misc_read_mutex = osMutexCreate(osMutex(read_mutex));
	misc_write_mutex = osMutexCreate(osMutex(write_mutex));

	if (misc_read_mutex == NULL || misc_write_mutex == NULL) {
		misc_err("%s: create mutex failed\n", __func__);
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
}

void take_write_mutex(void){
	osMutexWait(misc_read_mutex, portMAX_DELAY);
	read_count++;
	if (read_count == 1) {
		osMutexWait(misc_write_mutex, portMAX_DELAY);
	}
	osMutexRelease(misc_read_mutex);
}
void release_write_mutex(void){
	osMutexWait(misc_read_mutex, portMAX_DELAY);
	read_count--;
	if (read_count == 0) {
		osMutexRelease(misc_write_mutex);
	}
	osMutexRelease(misc_read_mutex);
}

int loadMiscData(void)
{
	uint32_t ret;
	//misc_info("loadMiscData\n");
	ret = VR_flash_read(REGION_FLASH_SECTOR1, (void *)&MISC_DataStructure, sizeof(MISC_DataTypeDef));
	if (ret == sizeof(MISC_DataTypeDef)) {
		return 0;
	} else {
		misc_err("load misc data failed\n");
		return -1;
	}
}

int saveMiscData(void)
{
	uint32_t ret;
	misc_info("saveMiscData\n");
	ret = VR_flash_erase(REGION_FLASH_SECTOR1,sizeof(MISC_DataTypeDef));
	if (ret) {
		misc_err("flash erase failed\n");
		return -1;
	}
	ret = VR_flash_write(&MISC_DataStructure,REGION_FLASH_SECTOR1, sizeof(MISC_DataTypeDef));
	if (ret) {
		misc_err("flash write failed\n");
		return -1;
	}
	return 0;
}

static BOOL misc_is_ascii_char(char char_key)
{
	if (char_key >= 0x20 && char_key <= 0x7E)
		return TRUE;

	return FALSE;
}

int setDefaultData(pcbid_t pcbid){
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset(&MISC_DataStructure,0,sizeof(MISC_DataTypeDef));
		strcpy((char *)&(MISC_DataStructure.Prop_ProjectID), PROJECTID_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_CustomID), CUSTOMID_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_ModelName), MODELNAME_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_cProductDate), CPRODUCTDATE_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_ColorID), COLORID_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_TrackingSystemName), TRACKINGSYSTEMNAME_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_ManufacturerName), MANUFACTURENAME_DEFAULT_VALUE);
		strcpy((char *)&(MISC_DataStructure.Prop_HardwareRevision_string), XB02_DEFAULT_VALUE);

		MISC_DataStructure.Prop_dwMFGAreaSignature = DWMFGAREASIGNATURE_DEFAULT_VALUE;
		MISC_DataStructure.Prop_PCBID = XB02;
		MISC_DataStructure.Prop_EngineerID = 0;
		MISC_DataStructure.Prop_DeviceIsWireless = 0;
		MISC_DataStructure.Prop_DeviceIsCharging = 1;
		MISC_DataStructure.Prop_DeviceProvidesBatteryStatus = 1;
		MISC_DataStructure.Prop_DeviceCanPowerOff = 1;
		MISC_DataStructure.Prop_DeviceBatteryPercentage = 0;
		MISC_DataStructure.Prop_HardwareRevision_int = XB02;
		MISC_DataStructure.Prop_HasCamera = 0;
		MISC_DataStructure.Prop_Firmware_UpdateAvailable = 1;
		MISC_DataStructure.Prop_LensCenterLeftU = 0.5;
		MISC_DataStructure.Prop_LensCenterLeftV = 0.5;
		MISC_DataStructure.Prop_LensCenterRightU = 0.5;
		MISC_DataStructure.Prop_LensCenterRightV = 0.5;
		MISC_DataStructure.Prop_UserHeadToEyeDepthMeters = 0;
		MISC_DataStructure.Prop_UserIpdMeters = 0.052;
		MISC_DataStructure.Prop_ContainsProximitySensor = 0;
		MISC_DataStructure.Prop_ScreenshotHorizontalFieldOfViewDegrees = 90;
		MISC_DataStructure.Prop_ScreenshotVerticalFieldOfViewDegrees = 90;
		MISC_DataStructure.Prop_DeviceClass = 1;
		MISC_DataStructure.Prop_ContainsRecenter = 0;
		MISC_DataStructure.Prop_distanceEyeToLens = 16;
		MISC_DataStructure.Prop_distanceLensToScreen = 37;
		MISC_DataStructure.Prop_lensFocalLength = 38.526000;
		MISC_DataStructure.Prop_distanceScaleX = 1;
		MISC_DataStructure.Prop_distanceScaleY = 1;
		MISC_DataStructure.Prop_RenderOverfill = 1.0;
		MISC_DataStructure.Prop_isDisplayOnDevice = 1;
		memset(MISC_DataStructure.Prop_RecommendedRenderTargetSize,0,sizeof(MISC_DataStructure.Prop_RecommendedRenderTargetSize));
		MISC_DataStructure.Prop_RecommendedRenderTargetSize[0] = 2160;
		MISC_DataStructure.Prop_RecommendedRenderTargetSize[1] = 1200;
		memset(MISC_DataStructure.Prop_RealScreenSize,0,sizeof(MISC_DataStructure.Prop_RealScreenSize));
		MISC_DataStructure.Prop_RealScreenSize[0] = 126;
		MISC_DataStructure.Prop_RealScreenSize[1] = 68;
		memset(MISC_DataStructure.Prop_polynomialCoeffsRed,0,sizeof(MISC_DataStructure.Prop_polynomialCoeffsRed));
		MISC_DataStructure.Prop_polynomialCoeffsRed[1] = 1;
		MISC_DataStructure.Prop_polynomialCoeffsRed[3] = 0.377561;
		MISC_DataStructure.Prop_polynomialCoeffsRed[5] = 0.251940;
		memset(MISC_DataStructure.Prop_polynomialCoeffsGreen,0,sizeof(MISC_DataStructure.Prop_polynomialCoeffsGreen));
		MISC_DataStructure.Prop_polynomialCoeffsGreen[1] = 1;
		MISC_DataStructure.Prop_polynomialCoeffsGreen[3] = 0.414526;
		MISC_DataStructure.Prop_polynomialCoeffsGreen[5] = 0.251940;
		memset(MISC_DataStructure.Prop_polynomialCoeffsBlue,0,sizeof(MISC_DataStructure.Prop_polynomialCoeffsBlue));
		MISC_DataStructure.Prop_polynomialCoeffsBlue[1] = 1;
		MISC_DataStructure.Prop_polynomialCoeffsBlue[3] = 0.442044;
		MISC_DataStructure.Prop_polynomialCoeffsBlue[5] = 0.251940;

		memset(MISC_DataStructure.Prop_getSensorToHead,0,sizeof(MISC_DataStructure.Prop_getSensorToHead));
		MISC_DataStructure.Prop_getSensorToHead[0] = -1;
		MISC_DataStructure.Prop_getSensorToHead[1] = -1;
		MISC_DataStructure.Prop_getSensorToHead[2] = -1;

		memset(MISC_DataStructure.Prop_CameraToHeadTransform_Matrix34,0,sizeof(MISC_DataStructure.Prop_CameraToHeadTransform_Matrix34));
		memset(MISC_DataStructure.Prop_StatusDisplayTransform_Matrix34,0,sizeof(MISC_DataStructure.Prop_StatusDisplayTransform_Matrix34));
		if (pcbid == XA0n) {
			strcpy((char *)&(MISC_DataStructure.Prop_HardwareRevision_string), XA0N_DEFAULT_VALUE);
			MISC_DataStructure.Prop_PCBID = XA0n;
			MISC_DataStructure.Prop_HardwareRevision_int = XA0n;
			MISC_DataStructure.Prop_ContainsProximitySensor = 1;
		}
		if (pcbid == XC01) {
			strcpy((char *)&(MISC_DataStructure.Prop_HardwareRevision_string), XC01_DEFAULT_VALUE);
			MISC_DataStructure.Prop_PCBID = XC01;
			MISC_DataStructure.Prop_HardwareRevision_int = XC01;
		}
		if (pcbid == XC02) {
			strcpy((char *)&(MISC_DataStructure.Prop_HardwareRevision_string), XC02_DEFAULT_VALUE);
			MISC_DataStructure.Prop_PCBID = XC02;
			MISC_DataStructure.Prop_HardwareRevision_int = XC02;
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDefaultDate failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDefaultDate(0x%8.8X) success\n", pcbid);
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDefaultDate failed\n");
		return -1;
	}

}

int getDefaultData()
{
	take_write_mutex();
	if (!loadMiscData()) {
		misc_info("ProjectID = %s\r\n",MISC_DataStructure.Prop_ProjectID);
		misc_info("CustomID = %s\r\n",MISC_DataStructure.Prop_CustomID);
		misc_info("ModelName = %s\r\n",MISC_DataStructure.Prop_ModelName);
		misc_info("cProductDate = %s\r\n",MISC_DataStructure.Prop_cProductDate);
		misc_info("ColorID = %s\r\n",MISC_DataStructure.Prop_ColorID);
		misc_info("TrackingSystemName = %s\r\n",MISC_DataStructure.Prop_TrackingSystemName);
		misc_info("ManufacturerName = %s\r\n",MISC_DataStructure.Prop_ManufacturerName);
		misc_info("HardwareRevision_string = %s\r\n",MISC_DataStructure.Prop_HardwareRevision_string);
		misc_info("vendorPartNumber = %s\r\n",MISC_DataStructure.Prop_vendorPartNumber);

		misc_info("dwMFGAreaSignature = 0x%x\r\n",MISC_DataStructure.Prop_dwMFGAreaSignature);
		misc_info("PCBID = 0x%x\r\n",MISC_DataStructure.Prop_PCBID);
		misc_info("EngineerID = %d\r\n",MISC_DataStructure.Prop_EngineerID);
		misc_info("DeviceIsWireless = %d\r\n",MISC_DataStructure.Prop_DeviceIsWireless);
		misc_info("DeviceIsCharging = %d\r\n",MISC_DataStructure.Prop_DeviceIsCharging);
		misc_info("DeviceProvidesBatteryStatus = %d\r\n",MISC_DataStructure.Prop_DeviceProvidesBatteryStatus);
		misc_info("DeviceCanPowerOff = %d\r\n",MISC_DataStructure.Prop_DeviceCanPowerOff);
		misc_info("DeviceBatteryPercentage = %f\r\n",MISC_DataStructure.Prop_DeviceBatteryPercentage);
		misc_info("HardwareRevision_int = 0x%x\r\n",MISC_DataStructure.Prop_HardwareRevision_int);
		misc_info("HasCamera = %d\r\n",MISC_DataStructure.Prop_HasCamera);
		misc_info("FirmwareUpdateAvailable = %d\r\n",MISC_DataStructure.Prop_Firmware_UpdateAvailable);
		misc_info("LensCenterLeftU = %f\r\n",MISC_DataStructure.Prop_LensCenterLeftU);
		misc_info("LensCenterLeftV = %f\r\n",MISC_DataStructure.Prop_LensCenterLeftV);
		misc_info("LensCenterRightU = %f\r\n",MISC_DataStructure.Prop_LensCenterRightU);
		misc_info("LensCenterRightV = %f\r\n",MISC_DataStructure.Prop_LensCenterRightV);
		misc_info("UserHeadToEyeDepthMeters = %f\r\n",MISC_DataStructure.Prop_UserHeadToEyeDepthMeters);
		misc_info("UserIpdMeters = %f\r\n",MISC_DataStructure.Prop_UserIpdMeters);
		misc_info("ContainsProximitySensor = %d\r\n",MISC_DataStructure.Prop_ContainsProximitySensor);
		misc_info("DeviceClass = %d\r\n",MISC_DataStructure.Prop_DeviceClass);
		misc_info("DisplayFrequency = %f\r\n",getDisplayFrequency());
		misc_info("DistanceEyeToLens = %f\r\n",MISC_DataStructure.Prop_distanceEyeToLens);
		misc_info("ScreenshotHorizontalFieldOfViewDegrees = %f\r\n",MISC_DataStructure.Prop_ScreenshotHorizontalFieldOfViewDegrees);
		misc_info("ScreenshotVerticalFieldOfViewDegrees = %f\r\n",MISC_DataStructure.Prop_ScreenshotVerticalFieldOfViewDegrees);
		misc_info("ContainsRecenter = %d\r\n",MISC_DataStructure.Prop_ContainsRecenter);
		misc_info("DistanceLensToScreen = %f\r\n",MISC_DataStructure.Prop_distanceLensToScreen);
		misc_info("LensFocalLength = %f\r\n",MISC_DataStructure.Prop_lensFocalLength);
		misc_info("DistanceScaleX = %f\r\n",MISC_DataStructure.Prop_distanceScaleX);
		misc_info("DistanceScaleY = %f\r\n",MISC_DataStructure.Prop_distanceScaleY);
		misc_info("RenderOverfill = %f\r\n",MISC_DataStructure.Prop_RenderOverfill);
		misc_info("RecommendedRenderTargetSize = {%f,%f}\r\n",MISC_DataStructure.Prop_RecommendedRenderTargetSize[0],
			MISC_DataStructure.Prop_RecommendedRenderTargetSize[1]);
		misc_info("RealScreenSize = {%f,%f}\r\n",MISC_DataStructure.Prop_RealScreenSize[0],MISC_DataStructure.Prop_RealScreenSize[1]);
		misc_info("getSensorToHead = {%f,%f,%f}\r\n",MISC_DataStructure.Prop_getSensorToHead[0]
			,MISC_DataStructure.Prop_getSensorToHead[1],MISC_DataStructure.Prop_getSensorToHead[2]);
		misc_info("isDisplayOnDevice = %d\r\n",MISC_DataStructure.Prop_isDisplayOnDevice);
		misc_info("PolynomialCoeffsRed = {%f,%f,%f,%f,%f,%f,%f,%f}\r\n",MISC_DataStructure.Prop_polynomialCoeffsRed[0],MISC_DataStructure.Prop_polynomialCoeffsRed[1],
			MISC_DataStructure.Prop_polynomialCoeffsRed[2],MISC_DataStructure.Prop_polynomialCoeffsRed[3],MISC_DataStructure.Prop_polynomialCoeffsRed[4],
			MISC_DataStructure.Prop_polynomialCoeffsRed[5],MISC_DataStructure.Prop_polynomialCoeffsRed[6],MISC_DataStructure.Prop_polynomialCoeffsRed[7]);
		misc_info("PolynomialCoeffsGreen = {%f,%f,%f,%f,%f,%f,%f,%f}\r\n",MISC_DataStructure.Prop_polynomialCoeffsGreen[0],MISC_DataStructure.Prop_polynomialCoeffsGreen[1],
			MISC_DataStructure.Prop_polynomialCoeffsGreen[2],MISC_DataStructure.Prop_polynomialCoeffsGreen[3],MISC_DataStructure.Prop_polynomialCoeffsGreen[4],
			MISC_DataStructure.Prop_polynomialCoeffsGreen[5],MISC_DataStructure.Prop_polynomialCoeffsGreen[6],MISC_DataStructure.Prop_polynomialCoeffsGreen[7]);
		misc_info("PolynomialCoeffsBlue = {%f,%f,%f,%f,%f,%f,%f,%f}\r\n",MISC_DataStructure.Prop_polynomialCoeffsBlue[0],MISC_DataStructure.Prop_polynomialCoeffsBlue[1],
			MISC_DataStructure.Prop_polynomialCoeffsBlue[2],MISC_DataStructure.Prop_polynomialCoeffsBlue[3],MISC_DataStructure.Prop_polynomialCoeffsBlue[4],
			MISC_DataStructure.Prop_polynomialCoeffsBlue[5],MISC_DataStructure.Prop_polynomialCoeffsBlue[6],MISC_DataStructure.Prop_polynomialCoeffsBlue[7]);
		misc_info("getDefaultData success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getDefaultData failed\n");
		return -1;
	}
}

int dump_property_for_usb(void)
{
	int i;
	take_write_mutex();
	if (!loadMiscData()) {
		usb_cdc_printf("ProjectID:%s\r\n", MISC_DataStructure.Prop_ProjectID);
		usb_cdc_printf("CustomID:%s\r\n", MISC_DataStructure.Prop_CustomID);
		usb_cdc_printf("FirmwareMainVersion = %s\r\n", SW_SYS_VER);
		usb_cdc_printf("ModelName = %s\r\n", MISC_DataStructure.Prop_ModelName);
		usb_cdc_printf("dwMFGAreaSignature = 0x%x\r\n", MISC_DataStructure.Prop_dwMFGAreaSignature);
		usb_cdc_printf("cProductDate = %s\r\n", MISC_DataStructure.Prop_cProductDate);
		usb_cdc_printf("PCBID = 0x%x\r\n", MISC_DataStructure.Prop_PCBID);
		usb_cdc_printf("EngineerID = %d\r\n", MISC_DataStructure.Prop_EngineerID);
		usb_cdc_printf("MBSerialNumber = %s\r\n", MISC_DataStructure.Prop_MBSerialNumber);
		usb_cdc_printf("SerialNumber = %s\r\n", MISC_DataStructure.Prop_SerialNumber);
		usb_cdc_printf("ColorID = %s\r\n", MISC_DataStructure.Prop_ColorID);
		usb_cdc_printf("BootFlag = 0x%x\r\n", MISC_DataStructure.Boot_flag);
		usb_cdc_printf("TrackingSystemName = %s\r\n", MISC_DataStructure.Prop_TrackingSystemName);
		usb_cdc_printf("ManufacturerName = %s\r\n", MISC_DataStructure.Prop_ManufacturerName);
		usb_cdc_printf("HardwareRevision_string = %s\r\n", MISC_DataStructure.Prop_HardwareRevision_string);
		usb_cdc_printf("DeviceIsWireless = %d\r\n", MISC_DataStructure.Prop_DeviceIsWireless);
		usb_cdc_printf("DeviceIsCharging = %d\r\n", MISC_DataStructure.Prop_DeviceIsCharging);
		usb_cdc_printf("DeviceProvidesBatteryStatus = %d\r\n", MISC_DataStructure.Prop_DeviceProvidesBatteryStatus);
		usb_cdc_printf("DeviceCanPowerOff = %d\r\n", MISC_DataStructure.Prop_DeviceCanPowerOff);
		usb_cdc_printf("DeviceBatteryPercentage = %f\r\n", MISC_DataStructure.Prop_DeviceBatteryPercentage);
		usb_cdc_printf("HardwareRevision_int = 0x%x\r\n", MISC_DataStructure.Prop_HardwareRevision_int);
		usb_cdc_printf("HasCamera = %d\r\n", MISC_DataStructure.Prop_HasCamera);
		usb_cdc_printf("TrackingFirmwareVersion = %s\r\n", SW_SYS_VER);
		usb_cdc_printf("vendorPartNumber = %s\r\n",MISC_DataStructure.Prop_vendorPartNumber);

		usb_cdc_printf("FirmwareUpdateAvailable = %d\r\n", MISC_DataStructure.Prop_Firmware_UpdateAvailable);
		usb_cdc_printf("LensCenterLeftU = %f\r\n", MISC_DataStructure.Prop_LensCenterLeftU);
		usb_cdc_printf("LensCenterLeftV = %f\r\n", MISC_DataStructure.Prop_LensCenterLeftV);
		usb_cdc_printf("LensCenterRightU = %f\r\n", MISC_DataStructure.Prop_LensCenterRightU);
		usb_cdc_printf("LensCenterRightV = %f\r\n", MISC_DataStructure.Prop_LensCenterRightV);
		usb_cdc_printf("UserHeadToEyeDepthMeters = %f\r\n", MISC_DataStructure.Prop_UserHeadToEyeDepthMeters);
		usb_cdc_printf("UserIpdMeters = %f\r\n", MISC_DataStructure.Prop_UserIpdMeters);
		usb_cdc_printf("ContainsProximitySensor = %d\r\n", MISC_DataStructure.Prop_ContainsProximitySensor);
		usb_cdc_printf("DeviceClass = %d\r\n", MISC_DataStructure.Prop_DeviceClass);
		usb_cdc_printf("DisplayFrequency = %f\r\n",getDisplayFrequency());
		usb_cdc_printf("DistanceEyeToLens = %f\r\n",MISC_DataStructure.Prop_distanceEyeToLens);
		usb_cdc_printf("ScreenshotHorizontalFieldOfViewDegrees = %f\r\n",MISC_DataStructure.Prop_ScreenshotHorizontalFieldOfViewDegrees);
		usb_cdc_printf("ScreenshotVerticalFieldOfViewDegrees = %f\r\n",MISC_DataStructure.Prop_ScreenshotVerticalFieldOfViewDegrees);
		usb_cdc_printf("ContainsRecenter = %d\r\n",MISC_DataStructure.Prop_ContainsRecenter);
		usb_cdc_printf("DistanceLensToScreen = %f\r\n",MISC_DataStructure.Prop_distanceLensToScreen);
		usb_cdc_printf("LensFocalLength = %f\r\n",MISC_DataStructure.Prop_lensFocalLength);
		usb_cdc_printf("DistanceScaleX = %f\r\n",MISC_DataStructure.Prop_distanceScaleX);
		usb_cdc_printf("DistanceScaleY = %f\r\n",MISC_DataStructure.Prop_distanceScaleY);
		usb_cdc_printf("RenderOverfill = %f\r\n",MISC_DataStructure.Prop_RenderOverfill);
		usb_cdc_printf("RecommendedRenderTargetSize = {%f,%f}\r\n",MISC_DataStructure.Prop_RecommendedRenderTargetSize[0],MISC_DataStructure.Prop_RecommendedRenderTargetSize[1]);
		usb_cdc_printf("RealScreenSize = {%f,%f}\r\n",MISC_DataStructure.Prop_RealScreenSize[0],MISC_DataStructure.Prop_RealScreenSize[1]);
		usb_cdc_printf("isDisplayOnDevice = %d\r\n",MISC_DataStructure.Prop_isDisplayOnDevice);

		usb_cdc_printf("getSensorToHead :");
		for(i=0;i<3;i++) {
			if(i == 2) {
				usb_cdc_printf("%f\r\n",MISC_DataStructure.Prop_getSensorToHead[i]);
			}else {
				usb_cdc_printf("%f ",MISC_DataStructure.Prop_getSensorToHead[i]);
			}
		}

		usb_cdc_printf("PolynomialCoeffsRed :");
		for(i=0;i<8;i++) {
			if(i == 7) {
				usb_cdc_printf("%f\r\n",MISC_DataStructure.Prop_polynomialCoeffsRed[i]);
			}else {
				usb_cdc_printf("%f ",MISC_DataStructure.Prop_polynomialCoeffsRed[i]);
			}
		}

		usb_cdc_printf("PolynomialCoeffsGreen :");
		for(i=0;i<8;i++) {
			if(i == 7) {
				usb_cdc_printf("%f\r\n",MISC_DataStructure.Prop_polynomialCoeffsGreen[i]);
			}else {
				usb_cdc_printf("%f ",MISC_DataStructure.Prop_polynomialCoeffsGreen[i]);
			}
		}

		usb_cdc_printf("PolynomialCoeffsBlue :");
		for(i=0;i<8;i++) {
			if(i == 7) {
				usb_cdc_printf("%f\r\n",MISC_DataStructure.Prop_polynomialCoeffsBlue[i]);
			}else {
				usb_cdc_printf("%f ",MISC_DataStructure.Prop_polynomialCoeffsBlue[i]);
			}
		}

		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		usb_cdc_transmit_data("[misc] misc dump failed\r\n",strlen("[misc] misc dump failed\r\n"));
		return -1;
	}
}


int getCameraToHeadTransform_Matrix34()
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i=0;i<3;i++) {
			for (int j=0;j<4;j++) {
				if (j<3)
					misc_info("%f ",MISC_DataStructure.Prop_CameraToHeadTransform_Matrix34[i][j]);
				else
					misc_info("%f\r\n",MISC_DataStructure.Prop_CameraToHeadTransform_Matrix34[i][j]);
			}
		}
		misc_info("getCameraToHeadTransform_Matrix34 success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getCameraToHeadTransform_Matrix34 failed\n");
		return -1;
	}
}

int getStatusDisplayTransform_Matrix34()
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i=0;i<3;i++) {
			for (int j=0;j<4;j++) {
				if (j<3)
					misc_info("%f ",MISC_DataStructure.Prop_StatusDisplayTransform_Matrix34[i][j]);
				else
					misc_info("%f\r\n",MISC_DataStructure.Prop_StatusDisplayTransform_Matrix34[i][j]);
			}
		}
		misc_info("getStatusDisplayTransform_Matrix34 success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getStatusDisplayTransform_Matrix34 failed\n");
		return -1;
	}
}


int getBoardInformationMagicNumber(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_BoardInformationMagicNumber[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_BoardInformationMagicNumber);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getBoardInformationMagicNumber failed\n");
		return -1;
	}
}

int setBoardInformationMagicNumber(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_BoardInformationMagicNumber) - 1;
	if (length > __length)
		length = __length;
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_BoardInformationMagicNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_BoardInformationMagicNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_BoardInformationMagicNumber), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setBoardInformationMagicNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setBoardInformationMagicNumber success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setBoardInformationMagicNumber failed\n");
		return -1;
	}
}

int getCustomID(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_CustomID[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_CustomID);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getCustomID failed\n");
		return -1;
	}
}

int setCustomID(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_CustomID) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_CustomID), 0x0,
				sizeof(MISC_DataStructure.Prop_CustomID));
		strncpy((char *)&(MISC_DataStructure.Prop_CustomID), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setCustomID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setCustomID success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setCustomID failed\n");
		return -1;
	}
}


int set_skuid(int id)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (loadMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set_skuid failed\n");
		return -1;
	}

	MISC_DataStructure.Prop_SKUID = id;

	if (saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set_skuid failed\n");
		return -1;

	}

	osMutexRelease(misc_write_mutex);
	misc_info("set_skuid success\n");

	return 0;
}


int get_skubid(int *id)
{
	take_write_mutex();

	if (loadMiscData()) {
		release_write_mutex();
		return -1;
	}

	*id = MISC_DataStructure.Prop_SKUID;

	release_write_mutex();

	return 0;
}

int getFirmwareMainVersion(char *value)
{
    img_header_struct sys_header;
    if(get_image_header((void*)HTC_SYS_VERSION_LOCATION,&sys_header) == 0){
        strcpy(value,(const char *)&sys_header.version);
        return 0;
    } else {
    	return -1;
    }
}
#if 0
int setFirmwareMainVersion(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_FirmwareMainVersion) - 1;
	if (length > __length)
		length = __length;
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_FirmwareMainVersion), 0x0,
				sizeof(MISC_DataStructure.Prop_FirmwareMainVersion));
		strncpy((char *)&(MISC_DataStructure.Prop_FirmwareMainVersion), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmwareMainVersion failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setFirmwareMainVersion success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmwareMainVersion failed\n");
		return -1;
	}
}
#endif
int getModelName(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ModelName[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ModelName);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getModelName failed\n");
		return -1;
	}
}

int setModelName(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_ModelName) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ModelName), 0x0,
				sizeof(MISC_DataStructure.Prop_ModelName));
		strncpy((char *)&(MISC_DataStructure.Prop_ModelName), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setModelName failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setModelName success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setModelName failed\n");
		return -1;
	}
}

int getcProductDate(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_cProductDate[0])){
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_cProductDate);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getcProductDate failed\n");
		return -1;
	}
}

int setcProductDate(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_cProductDate) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_cProductDate), 0x0,
				sizeof(MISC_DataStructure.Prop_cProductDate));
		strncpy((char *)&(MISC_DataStructure.Prop_cProductDate), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setcProductDate failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setcProductDate success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setcProductDate failed\n");
		return -1;
	}
}

int getMBSerialNumber(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_MBSerialNumber[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_MBSerialNumber);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getMBSerialNumber failed\n");
		return -1;
	}
}

int setMBSerialNumber(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_MBSerialNumber) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_MBSerialNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_MBSerialNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_MBSerialNumber), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setMBSerialNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setMBSerialNumber success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setMBSerialNumber failed\n");
		return -1;
	}
}

int getColorID(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ColorID[0])){
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ColorID);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getColorID failed\n");
		return -1;
	}
}

int setColorID(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_ColorID) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ColorID), 0x0,
				sizeof(MISC_DataStructure.Prop_ColorID));
		strncpy((char *)&(MISC_DataStructure.Prop_ColorID), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setColorID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setColorID success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setColorID failed\n");
		return -1;
	}
}
int getSemiPara(SEMI_STATUS *value,char offset)
{
	take_write_mutex();
	if (!loadMiscData()) {
		switch(offset){
			case 0:
				strcpy(value,(const char *)&MISC_DataStructure.mfg_semi_para.semi_earpod);
				break;
			case 1:
				strcpy(value,(const char *)&MISC_DataStructure.mfg_semi_para.semi_jack);
				break;
			case 2:
				strcpy(value,(const char *)&MISC_DataStructure.mfg_semi_para.semi_dp);
				break;
			default:
				release_write_mutex();
				misc_err("get semi MFG status failed\n");
				return -1;

		}
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("get semi MFG Status failed\n");
		return -1;
	}
}

int setSemiPara(const SEMI_STATUS *value, uint32_t offset)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		switch(offset){
			case 0:
				memset((char *)&(MISC_DataStructure.mfg_semi_para.semi_earpod), 0x0,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_earpod));
				strncpy((char *)&(MISC_DataStructure.mfg_semi_para.semi_earpod), value,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_earpod));
				break;
			case 1:
				memset((char *)&(MISC_DataStructure.mfg_semi_para.semi_jack), 0x0,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_jack));
				strncpy((char *)&(MISC_DataStructure.mfg_semi_para.semi_jack), value,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_jack));
				break;
			case 2:
				memset((char *)&(MISC_DataStructure.mfg_semi_para.semi_dp), 0x0,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_dp));
				strncpy((char *)&(MISC_DataStructure.mfg_semi_para.semi_dp), value,
					sizeof(MISC_DataStructure.mfg_semi_para.semi_dp));
				break;
			default:
				release_write_mutex();
				misc_err("set semi MFG  failed\n");
				return -1;
        }

	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("set semi MFG failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("set semi MFG success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("set semi MFG failed\n");
		return -1;
	}
}
int getProjectID(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ProjectID[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ProjectID);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getProjectID failed\n");
		return -1;
	}
}

int setProjectID(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_ProjectID) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ProjectID), 0x0,
				sizeof(MISC_DataStructure.Prop_ProjectID));
		strncpy((char *)&(MISC_DataStructure.Prop_ProjectID), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setProjectID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setProjectID success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setProjectID failed\n");
		return -1;
	}
}


int getdwMFGAreaSignature(int *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		*value = MISC_DataStructure.Prop_dwMFGAreaSignature;
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getdwMFGAreaSignature failed\n");
		return -1;
	}
}

int setdwMFGAreaSignature(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_dwMFGAreaSignature = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setdwMFGAreaSignature failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setdwMFGAreaSignature success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setdwMFGAreaSignature failed\n");
		return -1;
	}
}

int setDeviceClass(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceClass = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceClass failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceClass success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceClass failed\n");
		return -1;
	}
}

int getDeviceClass(int *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		*value = MISC_DataStructure.Prop_DeviceClass;
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getDeviceClass failed\n");
		return -1;
	}
}


int getEngineerID(int *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		*value = MISC_DataStructure.Prop_EngineerID;
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getEngineerID failed\n");
		return -1;
	}
}

int setEngineerID(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_EngineerID = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setEngineerID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setEngineerID success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setEngineerID failed\n");
		return -1;
	}
}

int getTrackingSystemName(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_TrackingSystemName[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_TrackingSystemName);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getTrackingSystemName failed\n");
		return -1;
	}
}

int setTrackingSystemName(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_TrackingSystemName) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_TrackingSystemName), 0x0,
				sizeof(MISC_DataStructure.Prop_TrackingSystemName));
		strncpy((char *)&(MISC_DataStructure.Prop_TrackingSystemName), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setTrackingSystemName failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setTrackingSystemName success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setTrackingSystemName failed\n");
		return -1;
	}
}

int getSerialNumber(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_SerialNumber[0])){
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_SerialNumber);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getSerialNumber failed\n");
		return -1;
	}
}

int setSerialNumber(const char *value, int length)
{
	int __length = sizeof(MISC_DataStructure.Prop_SerialNumber) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_SerialNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_SerialNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_SerialNumber), value, length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setSerialNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setSerialNumber success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setSerialNumber failed\n");
		return -1;
	}
}

int getManufacturerName(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ManufacturerName[0])){
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ManufacturerName);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getManufacturerName failed\n");
		return -1;
	}
}

int setManufacturerName(const char *value, int length)
{
	int __length = sizeof(MISC_DataStructure.Prop_ManufacturerName) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ManufacturerName), 0x0,
				sizeof(MISC_DataStructure.Prop_ManufacturerName));
		strncpy((char *)&(MISC_DataStructure.Prop_ManufacturerName),
				value, length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setManufacturerName failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setManufacturerName success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setManufacturerName failed\n");
		return -1;
	}
}

int getHardwareRevision_string(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_HardwareRevision_string[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)
				&MISC_DataStructure.Prop_HardwareRevision_string);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getHardwareRevision failed\n");
		return -1;
	}
}

int setHardwareRevision_string(const char *value, int length)
{
	int __length = sizeof(MISC_DataStructure.Prop_HardwareRevision_string) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_HardwareRevision_string), 0x0,
				sizeof(MISC_DataStructure.Prop_HardwareRevision_string));
		strncpy((char *)&(MISC_DataStructure.Prop_HardwareRevision_string),value, length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setHardwareRevision success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;
	}
}
int getTrackingFirmwareVersion(char *value)
{
    img_header_struct sys_header;
    if(get_image_header((void*)HTC_SYS_VERSION_LOCATION,&sys_header) == 0){
      strcpy(value,(const char *)&sys_header.version);
      return 0;
    } else {
    	return -1;
    }
}

#if 0
int setTrackingFirmwareVersion(const char *value, int length)
{
	int __length = sizeof(MISC_DataStructure.Prop_TrackingFirmwareVersion) - 1;
	if (length > __length)
		length = __length;
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_TrackingFirmwareVersion), 0x0,
				sizeof(MISC_DataStructure.Prop_TrackingFirmwareVersion));
		strcpy((char *)&(MISC_DataStructure.Prop_TrackingFirmwareVersion),value);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setTrackingFirmwareVersion failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setTrackingFirmwareVersion success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setTrackingFirmwareVersion failed\n");
		return -1;
	}
}
#endif

BOOL getDeviceIsWireless(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_DeviceIsWireless;
	} else {
		release_write_mutex();
		misc_err("getDeviceIsWireless failed\n");
		return -1;
	}
}

int setDeviceIsWireless(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceIsWireless = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceIsWireless failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceIsWireless success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceIsWireless failed\n");
		return -1;
	}
}

BOOL getDeviceIsCharging(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_DeviceIsCharging;
	} else {
		release_write_mutex();
		misc_err("getDeviceIsCharging failed\n");
		return -1;
	}
}

int setDeviceIsCharging(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceIsCharging = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceIsCharging failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceIsCharging success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceIsCharging failed\n");
		return -1;
	}
}

BOOL getDeviceProvidesBatteryStatus(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_DeviceProvidesBatteryStatus;
	} else {
		release_write_mutex();
		misc_err("getDeviceProvidesBatteryStatus failed\n");
		return -1;
	}
}

int setDeviceProvidesBatteryStatus(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceProvidesBatteryStatus = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceProvidesBatteryStatus failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceProvidesBatteryStatus success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceProvidesBatteryStatus failed\n");
		return -1;
	}
}

BOOL getDeviceCanPowerOff(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_DeviceCanPowerOff;
	} else {
		release_write_mutex();
		misc_err("getDeviceCanPowerOff failed\n");
		return -1;
	}
}

int setDeviceCanPowerOff(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceCanPowerOff = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceCanPowerOff failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceCanPowerOff success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceCanPowerOff failed\n");
		return -1;
	}
}

BOOL getContainsProximitySensor(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_ContainsProximitySensor;
	} else {
		release_write_mutex();
		misc_err("getContainsProximitySensor failed\n");
		return -1;
	}
}

int setContainsProximitySensor(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_ContainsProximitySensor = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setContainsProximitySensor failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setContainsProximitySensor success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setContainsProximitySensor failed\n");
		return -1;
	}
}


BOOL getHasCamera(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_HasCamera;
	} else {
		release_write_mutex();
		misc_err("getHasCamera failed\n");
		return -1;
	}
}

int setHasCamera(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_HasCamera = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHasCamera failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setHasCamera success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHasCamera failed\n");
		return -1;
	}
}

BOOL getFirmware_UpdateAvailable(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_Firmware_UpdateAvailable;
	} else {
		release_write_mutex();
		misc_err("getFirmware_UpdateAvailable failed\n");
		return -1;
	}
}

int setFirmware_UpdateAvailable(BOOL value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_Firmware_UpdateAvailable = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmware_UpdateAvailable failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setFirmware_UpdateAvailable success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmware_UpdateAvailable failed\n");
		return -1;
	}
}

float getDeviceBatteryPercentage(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_DeviceBatteryPercentage;
	} else {
		release_write_mutex();
		misc_err("getDeviceBatteryPercentage failed\n");
		return -1;
	}
}

int setDeviceBatteryPercentage(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_DeviceBatteryPercentage = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceBatteryPercentage failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDeviceBatteryPercentage success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDeviceBatteryPercentage failed\n");
		return -1;
	}
}

int setUserIpdMeters(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_UserIpdMeters = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setUserIpdMeters failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setUserIpdMeters success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setUserIpdMeters failed\n");
		return -1;
	}
}


float getUserIpdMeters(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_UserIpdMeters;
	} else {
		release_write_mutex();
		misc_err("getUserIpdMeters failed\n");
		return -1;
	}
}

float getLensCenterLeftU(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_LensCenterLeftU;
	} else {
		release_write_mutex();
		misc_err("getLensCenterLeftU failed\n");
		return -1;
	}
}

int setLensCenterLeftU(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_LensCenterLeftU = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterLeftU failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setLensCenterLeftU success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterLeftU failed\n");
		return -1;
	}
}

float getLensCenterLeftV(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_LensCenterLeftV;
	} else {
		release_write_mutex();
		misc_err("getLensCenterLeftV failed\n");
		return -1;
	}
}

int setLensCenterLeftV(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_LensCenterLeftV = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterLeftV failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setLensCenterLeftV success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterLeftV failed\n");
		return -1;
	}
}

float getLensCenterRightU(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_LensCenterRightU;
	} else {
		release_write_mutex();
		misc_err("getLensCenterRightU failed\n");
		return -1;
	}
}

int setLensCenterRightU(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_LensCenterRightU = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterRightU failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setLensCenterRightU success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterRightU failed\n");
		return -1;
	}
}

float getLensCenterRightV(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_LensCenterRightV;
	} else {
		release_write_mutex();
		misc_err("getLensCenterRightV failed\n");
		return -1;
	}
}

int setLensCenterRightV(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_LensCenterRightV = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterRightV failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setLensCenterRightV success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensCenterRightV failed\n");
		return -1;
	}
}

float getUserHeadToEyeDepthMeters(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_UserHeadToEyeDepthMeters;
	} else {
		release_write_mutex();
		misc_err("getUserHeadToEyeDepthMeters failed\n");
		return -1;
	}
}

int setUserHeadToEyeDepthMeters(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_UserHeadToEyeDepthMeters = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setUserHeadToEyeDepthMeters failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setUserHeadToEyeDepthMeters success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setUserHeadToEyeDepthMeters failed\n");
		return -1;
	}
}

int getFirmwareVersion(void)
{
    img_header_struct sys_header;
    char buffer[10];
    int value;
    int i,j;
    if(get_image_header((void*)HTC_SYS_VERSION_LOCATION,&sys_header) == 0){
        j = 0;
    	for (i=0;i<strlen(sys_header.version);i++) {
            if(sys_header.version[i] == '.'){
                continue;
            } else {
                buffer[j] = sys_header.version[i];
                j++;
            }
        }
        buffer[j] = '\0';
        value = atoi((char const*)buffer);
        return value;
    } else {
      return -1;
    }
}

#if 0
int setFirmwareVersion(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_FirmwareVersion_int = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmwareVersion failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setFirmwareVersion success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setFirmwareVersion failed\n");
		return -1;
	}
}
#endif

int getHardwareRevision_int(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_HardwareRevision_int;
	} else {
		release_write_mutex();
		misc_err("getHardwareRevision failed\n");
		return -1;
	}
}

int setHardwareRevision_int(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_HardwareRevision_int = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setHardwareRevision success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;
	}
}


BOOT_MODE get_bootmode()
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Boot_flag;
	} else {
		release_write_mutex();
		misc_err("get_bootmode failed\n");
		return ERROR_MODE;
	}
}

int set_bootmode(BOOT_MODE mode)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Boot_flag = mode;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("set_bootmode failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("set_bootmode success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("set_bootmode failed\n");
		return -1;
	}
}

int set_pcbid(pcbid_t id)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (loadMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;
	}

	MISC_DataStructure.Prop_PCBID = id;

	if (saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("setHardwareRevision failed\n");
		return -1;

	}

	osMutexRelease(misc_write_mutex);
	//misc_info("setHardwareRevision success\n");

	return 0;
}


int get_pcbid(pcbid_t *id)
{
	take_write_mutex();

	if (loadMiscData()) {
		release_write_mutex();
		return -1;
	}

	*id = MISC_DataStructure.Prop_PCBID;

	release_write_mutex();

	return 0;
}
int saveAccelOffsetValue(void *buff)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	int i =0,j = 0;
	if (!loadMiscData()) {
		BMI160_DataTypeDef *accel_value = (BMI160_DataTypeDef *)buff;
		MISC_DataStructure.G_senser_calib_x_offset = accel_value->accel_x;
		MISC_DataStructure.G_senser_calib_y_offset = accel_value->accel_y;
		MISC_DataStructure.G_senser_calib_z_offset = accel_value->accel_z;
		for( i = 0; i < 3; ++i ) {
			for( j = 0; j < 3; ++j ) {
				MISC_DataStructure.G_sensor_calib_acc[i][j] = accel_value->accel_k_xyz[i][j];
			}
		}
		MISC_DataStructure.G_senser_calib_header = 0x67676767;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveAccelOffsetValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("saveAccelOffsetValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveAccelOffsetValue failed\n");
		return -1;
	}
}

int saveGyroOffsetValue(void *buff)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		BMI160_DataTypeDef *gyro_value = (BMI160_DataTypeDef *)buff;
		MISC_DataStructure.Gyro_calib_x_offset = gyro_value->gyro_x;
		MISC_DataStructure.Gyro_calib_y_offset = gyro_value->gyro_y;
		MISC_DataStructure.Gyro_calib_z_offset = gyro_value->gyro_z;
		MISC_DataStructure.Gyro_calib_header = 0x67676767;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveGyroOffsetValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("saveGyroOffsetValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveGyroOffsetValue failed\n");
		return -1;
	}
}

int clearAccValue(void)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.G_senser_calib_header = 0x0;
		//MISC_DataStructure.G_senser_selftest_header = 0x0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("clearAccValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("clearAccValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("clearAccValue failed\n");
		return -1;
	}
}
int saveChangeACCValue(void)
{
    osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
	    MISC_DataStructure.G_senser_calib_header = 0x76767676;
	} else {
        osMutexRelease(misc_write_mutex);
        misc_err("change ACCValue failed\n");
        return -1;
    }
    if (!saveMiscData()) {
       osMutexRelease(misc_write_mutex);
       misc_info("change ACCValue success\n");
       return 0;
    } else {
       osMutexRelease(misc_write_mutex);
       misc_err("change ACCValue failed\n");
       return -1;
    }
}
int clearGyroValue(void)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Gyro_calib_header = 0x0;
		//MISC_DataStructure.Gyro_selftest_header = 0x0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("clearGyroValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("clearGyroValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("clearGyroValue failed\n");
		return -1;
	}
}

int getBmi160OffsetValue(void *buff)
{
	take_write_mutex();
	int i = 0,j = 0;
	if (!loadMiscData()) {
		BMI160_DataTypeDef *bmi160_value = (BMI160_DataTypeDef *)buff;
		if ((MISC_DataStructure.G_senser_calib_header == 0x67676767) ||
			(MISC_DataStructure.G_senser_calib_header == 0x76767676)) {
			bmi160_value->accel_x = MISC_DataStructure.G_senser_calib_x_offset;
			bmi160_value->accel_y = MISC_DataStructure.G_senser_calib_y_offset;
			bmi160_value->accel_z = MISC_DataStructure.G_senser_calib_z_offset;
			for (i = 0; i < 3; ++i) {
				for (j = 0; j < 3; ++j) {
					bmi160_value->accel_k_xyz[i][j] =
						MISC_DataStructure.G_sensor_calib_acc[i][j];
				}
			}
			bmi160_value->isAccValid = 1;
		}
		if ((MISC_DataStructure.Gyro_calib_header== 0x67676767) ) {
			bmi160_value->gyro_x = MISC_DataStructure.Gyro_calib_x_offset;
			bmi160_value->gyro_y = MISC_DataStructure.Gyro_calib_y_offset;
			bmi160_value->gyro_z = MISC_DataStructure.Gyro_calib_z_offset;
			bmi160_value->isGyroValid = 1;
		}
		release_write_mutex();
		return 0;
	}else {
		release_write_mutex();
		misc_err("getBmi160OffsetValue failed\n");
		return -1;
	}
}

int saveAccSelfTestValue(void)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.G_senser_selftest_header = 0x67676767;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveAccSelfTestValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("saveAccSelfTestValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveAccSelfTestValue failed\n");
		return -1;
	}
}

int saveGyroSelfTestValue(void)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Gyro_selftest_header = 0x67676767;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveGyroSelfTestValue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("saveGyroSelfTestValue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("saveGyroSelfTestValue failed\n");
		return -1;
	}
}

uint32_t getGyroSelfTestValue(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Gyro_selftest_header;
	} else {
		release_write_mutex();
		misc_err("getGyroSelfTestValue failed\n");
		return -1;
	}
}

uint32_t getAccSelfTestValue(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.G_senser_selftest_header;
	} else {
		release_write_mutex();
		misc_err("getGyroSelfTestValue failed\n");
		return -1;
	}
}
uint32_t getAccCalheaderValue(void)
{
   	take_write_mutex();
	if (!loadMiscData()) {
	    release_write_mutex();
	    return MISC_DataStructure.G_senser_calib_header;
	} else {
		release_write_mutex();
		misc_err("getAccCalheaderValue failed\n");
		return -1;
	}
}
uint32_t getGyroCalheaderValue(void)
{
   	take_write_mutex();
	if (!loadMiscData()) {
	    release_write_mutex();
	    return MISC_DataStructure.Gyro_calib_header;
	} else {
		release_write_mutex();
		misc_err("getGyroCalheaderValue failed\n");
		return -1;
	}
}

int set_ps_threshold(uint16_t low_threshold,uint16_t high_threshold)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (loadMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set proximity threshold failed\n");
		return -1;
	}

	MISC_DataStructure.ps_threshold_low_value = low_threshold;
	MISC_DataStructure.ps_threshold_high_value = high_threshold;

	if (saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set proximity threshold failed\n");
		return -1;

	}

	osMutexRelease(misc_write_mutex);
	misc_info("set proximity threshold success\n");

	return 0;
}


int get_ps_threshold(uint16_t *low_threshold,uint16_t *high_threshold)
{
	take_write_mutex();

	if (loadMiscData()) {
		release_write_mutex();
		return -1;
	}

	*low_threshold = MISC_DataStructure.ps_threshold_low_value;
	*high_threshold = MISC_DataStructure.ps_threshold_high_value;

	release_write_mutex();

	return 0;
}

int set_akm_calibrate(struct AKL_NV_PRMS akm_calibrate)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (loadMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set akm calibrate value failed\n");
		return -1;
	}

	MISC_DataStructure.Akm_calibrate= akm_calibrate;

	if (saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set akm calibrate value  failed\n");
		return -1;

	}

	osMutexRelease(misc_write_mutex);
	misc_info("set akm calibrate value success\n");

	return 0;
}


int get_akm_calibrate(struct AKL_NV_PRMS *akm_calibrate)
{
	take_write_mutex();

	if (loadMiscData()) {
		release_write_mutex();
		return -1;
	}

	*akm_calibrate = MISC_DataStructure.Akm_calibrate;

	release_write_mutex();

	return 0;
}

int set_yas_calibrate(struct YAS_NV_PRMS yas_calibrate)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (loadMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set yas calibrate value failed\n");
		return -1;
	}

	MISC_DataStructure.Yas_calibrate= yas_calibrate;

	if (saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_err("set yas calibrate value  failed\n");
		return -1;

	}

	osMutexRelease(misc_write_mutex);
	misc_info("set yas calibrate value success\n");

	return 0;
}


int get_yas_calibrate(struct YAS_NV_PRMS *yas_calibrate)
{
	take_write_mutex();

	if (loadMiscData()) {
		release_write_mutex();
		return -1;
	}

	*yas_calibrate = MISC_DataStructure.Yas_calibrate;

	release_write_mutex();

	return 0;
}

float getDisplayFrequency(void)
{
	return disp_fps_get();
}

float getScreenshotHorizontalFOV(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_ScreenshotHorizontalFieldOfViewDegrees;
	} else {
		release_write_mutex();
		misc_err("getScreenshotHorizontalFOV failed\n");
		return -1;
	}
}

int setScreenshotHorizontalFOV(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_ScreenshotHorizontalFieldOfViewDegrees = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setScreenshotHorizontalFOV failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setScreenshotHorizontalFOV success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setScreenshotHorizontalFOV failed\n");
		return -1;
	}
}

float getScreenshotVerticalFOV(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_ScreenshotVerticalFieldOfViewDegrees;
	} else {
		release_write_mutex();
		misc_err("getScreenshotVerticalFOV failed\n");
		return -1;
	}
}

int setScreenshotVerticalFOV(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_ScreenshotVerticalFieldOfViewDegrees = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setScreenshotVerticalFOV failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setScreenshotVerticalFOV success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setScreenshotVerticalFOV failed\n");
		return -1;
	}
}

BOOL getContainsRecenter(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_ContainsRecenter;
	} else {
		release_write_mutex();
		misc_err("getContainsRecenter failed\n");
		return -1;
	}
}

int setContainsRecenter(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_ContainsRecenter = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setContainsRecenter failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setContainsRecenter success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setContainsRecenter failed\n");
		return -1;
	}
}

float getDistanceEyeToLens(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_distanceEyeToLens;
	} else {
		release_write_mutex();
		misc_err("getDistanceEyeToLens failed\n");
		return -1;
	}
}

int setDistanceEyeToLens(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_distanceEyeToLens = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceEyeToLens failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDistanceEyeToLens success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceEyeToLens failed\n");
		return -1;
	}
}

float getDistanceLensToScreen(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_distanceLensToScreen;
	} else {
		release_write_mutex();
		misc_err("getDistanceLensToScreen failed\n");
		return -1;
	}
}

int setDistanceLensToScreen(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_distanceLensToScreen = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceLensToScreen failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDistanceLensToScreen success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceLensToScreen failed\n");
		return -1;
	}
}

float getLensFocalLength(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_lensFocalLength;
	} else {
		release_write_mutex();
		misc_err("getLensFocalLength failed\n");
		return -1;
	}
}

int setLensFocalLength(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_lensFocalLength = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensFocalLength failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setLensFocalLength success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setLensFocalLength failed\n");
		return -1;
	}
}

float getDistanceScaleX(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_distanceScaleX;
	} else {
		release_write_mutex();
		misc_err("getDistanceScaleX failed\n");
		return -1;
	}
}

int setDistanceScaleX(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_distanceScaleX = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceScaleX failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDistanceScaleX success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceScaleX failed\n");
		return -1;
	}
}

float getDistanceScaleY(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_distanceScaleY;
	} else {
		release_write_mutex();
		misc_err("getDistanceScaleY failed\n");
		return -1;
	}
}

int setDistanceScaleY(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_distanceScaleY = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceScaleY failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setDistanceScaleY success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setDistanceScaleY failed\n");
		return -1;
	}
}

float getRenderOverfill(void)
{
	take_write_mutex();
	if (!loadMiscData()) {
		release_write_mutex();
		return MISC_DataStructure.Prop_RenderOverfill;
	} else {
		release_write_mutex();
		misc_err("getRenderOverfill failed\n");
		return -1;
	}
}

int setRenderOverfill(float value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_RenderOverfill = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRenderOverfill failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setRenderOverfill success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRenderOverfill failed\n");
		return -1;
	}
}

int getRecommendedRenderTargetSize(double *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 2; i++) {
			*(value + i) = MISC_DataStructure.Prop_RecommendedRenderTargetSize[i];
		}
		misc_info("getRecommendedRenderTargetSize success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getRecommendedRenderTargetSize failed\n");
		return -1;
	}
}

int setRecommendedRenderTargetSize(double value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i = 0; i < 2; i++) {
			MISC_DataStructure.Prop_RecommendedRenderTargetSize[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRecommendedRenderTargetSize failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setRecommendedRenderTargetSize success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRecommendedRenderTargetSize failed\n");
		return -1;
	}
}

int getRealScreenSize(float *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 2; i++) {
			*(value + i) = MISC_DataStructure.Prop_RealScreenSize[i];
		}
		misc_info("getRecommendedRenderTargetSize success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getRecommendedRenderTargetSize failed\n");
		return -1;
	}
}

int setRealScreenSize(float value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i = 0; i < 2; i++) {
			MISC_DataStructure.Prop_RealScreenSize[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRecommendedRenderTargetSize failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setRecommendedRenderTargetSize success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setRecommendedRenderTargetSize failed\n");
		return -1;
	}
}

int getPolynomialCoeffsRed(float *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			*(value + i) = MISC_DataStructure.Prop_polynomialCoeffsRed[i];
		}
		misc_info("getPolynomialCoeffsRed success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getPolynomialCoeffsRed failed\n");
		return -1;
	}
}

int setPolynomialCoeffsRed(float value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			MISC_DataStructure.Prop_polynomialCoeffsRed[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsRed failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setPolynomialCoeffsRed success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsRed failed\n");
		return -1;
	}
}


int getPolynomialCoeffsGreen(float *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			*(value + i) = MISC_DataStructure.Prop_polynomialCoeffsGreen[i];
		}
		misc_info("getPolynomialCoeffsGreen success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getPolynomialCoeffsGreen failed\n");
		return -1;
	}
}

int setPolynomialCoeffsGreen(float value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			MISC_DataStructure.Prop_polynomialCoeffsGreen[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsGreen failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setPolynomialCoeffsGreen success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsGreen failed\n");
		return -1;
	}
}

int getPolynomialCoeffsBlue(float *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			*(value + i) = MISC_DataStructure.Prop_polynomialCoeffsBlue[i];
		}
		misc_info("getPolynomialCoeffsBlue success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getPolynomialCoeffsBlue failed\n");
		return -1;
	}
}

int setPolynomialCoeffsBlue(float value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i = 0; i < 8; i++) {
			MISC_DataStructure.Prop_polynomialCoeffsBlue[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsBlue failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setPolynomialCoeffsBlue success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setPolynomialCoeffsBlue failed\n");
		return -1;
	}
}

int getDisplayOnDevice(int *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		*value = MISC_DataStructure.Prop_isDisplayOnDevice;
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getIsDisplayOnDevice failed\n");
		return -1;
	}
}

int setDisplayOnDevice(int value)
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		MISC_DataStructure.Prop_isDisplayOnDevice = value;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setisDisplayOnDevice failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setisDisplayOnDevice success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setisDisplayOnDevice failed\n");
		return -1;
	}
}

int getSensorToHead(float *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		for (int i = 0; i < 3; i++) {
			*(value + i) = MISC_DataStructure.Prop_getSensorToHead[i];
		}
		misc_info("getSensorToHead success\n");
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getSensorToHead failed\n");
		return -1;
	}
}

int setSensorToHead(float value[])
{
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		for (int i=0;i<3;i++) {
			MISC_DataStructure.Prop_getSensorToHead[i] = value[i];
		}
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setSensorToHead failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setSensorToHead success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setSensorToHead failed\n");
		return -1;
	}
}

int getVendorPartNumber(char *value)
{
	take_write_mutex();
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_vendorPartNumber[0])) {
			release_write_mutex();
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_vendorPartNumber);
		release_write_mutex();
		return 0;
	} else {
		release_write_mutex();
		misc_err("getVendorPartNumber failed\n");
		return -1;
	}
}

int setVendorPartNumber(const char *value, uint32_t length)
{
	int __length = sizeof(MISC_DataStructure.Prop_vendorPartNumber) - 1;
	if (length > __length) {
		length = __length;
		return -2;
	}
	osMutexWait(misc_write_mutex, portMAX_DELAY);
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_vendorPartNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_vendorPartNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_vendorPartNumber), value,
				length);
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setVendorPartNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		osMutexRelease(misc_write_mutex);
		misc_info("setVendorPartNumber success\n");
		return 0;
	} else {
		osMutexRelease(misc_write_mutex);
		misc_err("setVendorPartNumber failed\n");
		return -1;
	}
}


