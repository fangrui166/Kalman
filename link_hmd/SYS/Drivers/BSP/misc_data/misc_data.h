#ifndef __MISC_DATA_H
#define __MISC_DATA_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "integer.h"
#include "hlog_api.h"
#include "lib3d/akl_smart_compass.h"
#include "YAS537_MAG_driver_HL.h"
#define MISC_HLOG_ENABLE 1

#ifdef MISC_HLOG_ENABLE
#define misc_emerg(fmt, ...) \
	hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_MISC, fmt, ##__VA_ARGS__)
#define misc_err(fmt, ...) \
	hlog_printf(HLOG_LVL_ERR, HLOG_TAG_MISC, fmt, ##__VA_ARGS__)
#define misc_warning(fmt, ...) \
	hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_MISC, fmt, ##__VA_ARGS__)
#define misc_info(fmt, ...) \
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_MISC, fmt, ##__VA_ARGS__)
#define misc_debug(fmt, ...) \
	hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_MISC, fmt, ##__VA_ARGS__)
#else /* flash_HLOG_ENABLE */
#define misc_emerg(fmt, ...) \
	printf("[MISC][EMR] :" fmt, ##__VA_ARGS__)
#define misc_err(fmt, ...) \
	printf("[MISC][ERR] :" fmt, ##__VA_ARGS__)
#define misc_warning(fmt, ...) \
	printf("[MISC][WARN]:" fmt, ##__VA_ARGS__)
#define misc_info(fmt, ...) \
	printf("[MISC][INFO]:" fmt, ##__VA_ARGS__)
#define misc_debug(fmt, ...) \
	printf("[MISC][DBG] :" fmt, ##__VA_ARGS__)
#endif /* misc_HLOG_ENABLE */
/* Boolean type */
#define PROP_BOARDINFORMATIONMAGICNUMBER_LENTH     (16)
#define PROP_CUSTOMID_LENTH                        (16)
#define PROP_PROJECTID_LENTH                       (16)
#define PROP_RESERVED1_LENTH                       (12)
#define PROP_RESERVED2_LENTH                       (8)
#define PROP_RESERVED3_LENTH                       (12)
#define PROP_RESERVED4_LENTH                       (28)
#define PROP_RESERVED5_LENTH                       (0x6A0)
#define PROP_RESERVED6_LENTH                       (0x50)
#define PROP_FIRMWAREMAINVERSION_LENTH             (16)
#define PROP_CPRODUCTDATE_LENTH                    (16)
#define PROP_MODELNAME_LENTH                       (64)
#define PROP_MBSERIALNUMBER_LENTH                  (16)
#define PROP_SERIALNUMBER_LENTH                    (16)
#define PROP_COLORID_LENTH                         (16)
#define PROP_TRACKINGSYSTEMNAME_LENTH              (64)
#define PROP_MODELNUMBER_LENTH                     (64)
#define PROP_MANUFACTURENAME_LENTH                 (16)
#define PROP_HARDWAREREVISION_LENTH                (16)
#define PROP_TRACKINGFIRMWAREVERSION_LENTH         (16)
#define PROP_SUPERMID_LENTH                        (16)
#define PROP_NUM                                   (33)
#define PROP_ITEM_LENTH                            (64)
#define PROP_VENDORPARTNUMBER_LENTH                (64)




/* default String data*/
#define PROJECTID_DEFAULT_VALUE                     "LNK"
#define CUSTOMID_DEFAULT_VALUE                      "11111111"
#define MODELNAME_DEFAULT_VALUE                     "2Q25100"
#define CPRODUCTDATE_DEFAULT_VALUE                  "20170117"
#define COLORID_DEFAULT_VALUE                       "BLK01"
#define TRACKINGSYSTEMNAME_DEFAULT_VALUE            "HMD"
#define MANUFACTURENAME_DEFAULT_VALUE               "HTC"
#define EVT_DEFAULT_VALUE                           "0x000201FF"
#define EVT2_DEFAULT_VALUE                          "0x000002FF"
#define XA0N_DEFAULT_VALUE                          "0x00000001"
#define XB02_DEFAULT_VALUE                          "0x010102FF"
#define XC01_DEFAULT_VALUE                          "0x020201FF"
#define XC02_DEFAULT_VALUE                          "0x020202FF"







#define DWMFGAREASIGNATURE_DEFAULT_VALUE            (0x48772D52)


typedef enum{
	FOTA_MODE	= 0xFAFAFAFA,
	NORM_MODE	= 0xAEAEAEAE,
	BL_MFG_MODE	= 0xFEFEFEFE,
	SYS_MFG_MODE= 0xDEDEDEDE,
	DFU_MODE	= 0xDFDFDFDF,
	ERROR_MODE	= 0xEDEDEDED,
} BOOT_MODE;

typedef enum
{
    UNKNOWN = 0xFFFFFFFF,
    XA0n = 0x00000001,
    XB02 = 0x010102FF,
    XC01 = 0x020201FF,
    XC02 = 0x020202FF,
}  pcbid_t;

typedef enum
{
	SEMI_FINISH 	= 0x18,
	SEMI_UNFINISH	= 0x58,
}SEMI_STATUS;
struct MFG_semi_para{
	SEMI_STATUS semi_earpod;
	SEMI_STATUS semi_jack;
	SEMI_STATUS semi_dp;
};
typedef struct
{
    //Board Info Data Area
    char Prop_BoardInformationMagicNumber[PROP_BOARDINFORMATIONMAGICNUMBER_LENTH];
    char Prop_ProjectID[PROP_PROJECTID_LENTH];
    char Prop_CustomID[PROP_CUSTOMID_LENTH];
    int Prop_SKUID;
    char Prop_Reserved1[PROP_RESERVED1_LENTH];/*Reserved for further use*/

    //G-Senser Calibration Data Area
    float G_senser_calib_x_offset; /* X-offset */
    float G_senser_calib_y_offset; /* Y-offset */
    float G_senser_calib_z_offset; /* Z-offset */
    float G_sensor_calib_acc[3][3];
    uint32_t G_senser_calib_header;   /* Header should be 0x67676767 after calibration */
    uint32_t G_senser_selftest_header;/* Header should be 0x67676767 after selftest */
    char Prop_Reserved2[PROP_RESERVED2_LENTH];/*Reserved for further use*/

    //Gyro-Senser Calibration Data Area
    int32_t Gyro_calib_x_offset;     /* x -offset */
    int32_t Gyro_calib_y_offset;     /* y -offset */
    int32_t Gyro_calib_z_offset;     /* z -offset */
    uint32_t Gyro_calib_header;       /* Header should be 0x67676767 after calibration */
    uint32_t Gyro_selftest_header;    /* Header should be 0x67676767 after selftest */
    char Prop_Reserved3[PROP_RESERVED3_LENTH];/*Reserved for further use*/

    //p-Senser Calibration Data Area
    uint16_t ps_threshold_low_value;
    uint16_t ps_threshold_high_value;

    char Prop_Reserved4[PROP_RESERVED4_LENTH];/*Reserved for further other sensor use*/

    //MFG Customization Data Area
	char Prop_FirmwareMainVersion[PROP_FIRMWAREMAINVERSION_LENTH];
    char Prop_ModelName[PROP_MODELNAME_LENTH];
    int Prop_dwMFGAreaSignature;
    char Prop_cProductDate[PROP_CPRODUCTDATE_LENTH];
    pcbid_t Prop_PCBID;
    int Prop_EngineerID;
    char Prop_MBSerialNumber[PROP_MBSERIALNUMBER_LENTH];
    char Prop_SerialNumber[PROP_SERIALNUMBER_LENTH];
    char Prop_ColorID[PROP_COLORID_LENTH];
    BOOT_MODE Boot_flag;
    struct AKL_NV_PRMS Akm_calibrate;
    struct YAS_NV_PRMS Yas_calibrate;
    struct MFG_semi_para mfg_semi_para;
    char Prop_Reserved5[PROP_RESERVED5_LENTH-sizeof(struct AKL_NV_PRMS)-sizeof(struct YAS_NV_PRMS)-sizeof(struct MFG_semi_para)];/*Reserved for further use*/

    //HTC Propertry Customization Data Area
    char Prop_TrackingSystemName[PROP_TRACKINGSYSTEMNAME_LENTH];
    char Prop_Reserved6[PROP_RESERVED6_LENTH];/*Reserved for further use*/
    char Prop_ManufacturerName[PROP_MANUFACTURENAME_LENTH];
    char Prop_HardwareRevision_string[PROP_HARDWAREREVISION_LENTH];
    BOOL Prop_DeviceIsWireless;
    BOOL Prop_DeviceIsCharging;
    BOOL Prop_DeviceProvidesBatteryStatus;
    BOOL Prop_DeviceCanPowerOff;
    float Prop_DeviceBatteryPercentage;
    int Prop_HardwareRevision_int;
    BOOL Prop_HasCamera;
    char Prop_TrackingFirmwareVersion[PROP_TRACKINGFIRMWAREVERSION_LENTH];
    BOOL Prop_Firmware_UpdateAvailable;
    int Prop_FirmwareVersion_int;
    float Prop_LensCenterLeftU;
    float Prop_LensCenterLeftV;
    float Prop_LensCenterRightU;
    float Prop_LensCenterRightV;
    float Prop_UserHeadToEyeDepthMeters;
    float Prop_CameraToHeadTransform_Matrix34[3][4];
    float Prop_StatusDisplayTransform_Matrix34[3][4];
    float Prop_UserIpdMeters;
    BOOL Prop_ContainsProximitySensor;
    int Prop_DeviceClass;
    float Prop_DisplayFrequency;
    float Prop_ScreenshotHorizontalFieldOfViewDegrees;
    float Prop_ScreenshotVerticalFieldOfViewDegrees;
    BOOL Prop_ContainsRecenter;
    float Prop_distanceEyeToLens;
    float Prop_distanceLensToScreen;
    float Prop_lensFocalLength;
    float Prop_distanceScaleX;
    float Prop_distanceScaleY;
    float Prop_RenderOverfill;
    float Prop_polynomialCoeffsRed[8];
    float Prop_polynomialCoeffsGreen[8];
    float Prop_polynomialCoeffsBlue[8];
    double Prop_RecommendedRenderTargetSize[2];
    float Prop_RealScreenSize[2];
    int Prop_isDisplayOnDevice;
    float Prop_getSensorToHead[3];
    char Prop_vendorPartNumber[PROP_VENDORPARTNUMBER_LENTH];
} MISC_DataTypeDef;


int misc_data_init(void);
int loadMiscData(void);
int saveMiscData(void);
int setDefaultData(pcbid_t);
int getDefaultData();
int dump_property_for_usb(void);
int getCameraToHeadTransform_Matrix34();
int getStatusDisplayTransform_Matrix34();
int getBoardInformationMagicNumber(char *value);
int setBoardInformationMagicNumber(const char *value, uint32_t length);
int getCustomID(char *value);
int setCustomID(const char *value, uint32_t length);
int getFirmwareMainVersion(char *value);
//int setFirmwareMainVersion(const char *value, uint32_t length);
int getModelName(char *value);
int setModelName(const char *value, uint32_t length);
int getcProductDate(char *value);
int setcProductDate(const char *value, uint32_t length);
int getMBSerialNumber(char *value);
int setMBSerialNumber(const char *value, uint32_t length);
int getColorID(char *value);
int setColorID(const char *value, uint32_t length);
int getSerialNumber(char *value);
int setSerialNumber(const char *value, int length);
int getProjectID(char *value);
int setProjectID(const char *value, uint32_t length);
int getdwMFGAreaSignature(int *value);
int setdwMFGAreaSignature(int value);
int setDeviceClass(int value);
int getDeviceClass(int *value);
int getEngineerID(int *value);
int setEngineerID(int value);
BOOL getHasGsensor(void);
BOOL getHasGyrosensor(void);
BOOL getHasPsensor(void);
int getTrackingSystemName(char *value);
int setTrackingSystemName(const char *value, uint32_t length);
int getManufacturerName(char *value);
int setManufacturerName(const char *value, int length);
int getHardwareRevision_string(char *value);
int setHardwareRevision_string(const char *value, int length);
int getTrackingFirmwareVersion(char *value);
//int setTrackingFirmwareVersion(const char *value, int length);
BOOL getDeviceIsWireless(void);
int setDeviceIsWireless(BOOL value);
BOOL getDeviceIsCharging(void);
int setDeviceIsCharging(BOOL value);
BOOL getDeviceProvidesBatteryStatus(void);
int setDeviceProvidesBatteryStatus(BOOL value);
BOOL getDeviceCanPowerOff(void);
int setDeviceCanPowerOff(BOOL value);
BOOL getContainsProximitySensor(void);
int setContainsProximitySensor(int value);
BOOL getHasCamera(void);
int setHasCamera(BOOL value);
BOOL getFirmware_UpdateAvailable(void);
int setFirmware_UpdateAvailable(BOOL value);
float getDeviceBatteryPercentage(void);
int setDeviceBatteryPercentage(float value);
int setUserIpdMeters(float value);
float getUserIpdMeters(void);
float getLensCenterLeftU(void);
int setLensCenterLeftU(float value);
float getLensCenterLeftV(void);
int setLensCenterLeftV(float value);
float getLensCenterRightU(void);
int setLensCenterRightU(float value);
float getLensCenterRightV(void);
int setLensCenterRightV(float value);
float getUserHeadToEyeDepthMeters(void);
int setUserHeadToEyeDepthMeters(float value);
int getFirmwareVersion(void);
//int setFirmwareVersion(int value);
int getHardwareRevision_int(void);
int setHardwareRevision_int(int value);
BOOT_MODE get_bootmode();
int set_bootmode(BOOT_MODE mode);
int set_pcbid(pcbid_t id);
int get_pcbid(pcbid_t *id);
int saveAccelOffsetValue(void *buff);
int saveGyroOffsetValue(void *buff);
int clearAccValue(void);
int saveChangeACCValue(void);
int clearGyroValue(void);
int getBmi160OffsetValue(void *buff);
int saveAccSelfTestValue(void);
int saveGyroSelfTestValue(void);
uint32_t getGyroSelfTestValue(void);
uint32_t getAccSelfTestValue(void);
uint32_t getAccCalheaderValue(void);
uint32_t getGyroCalheaderValue(void);
int set_skuid(int id);
int get_skubid(int *id);
int set_ps_threshold(uint16_t low_threshold,uint16_t high_threshold);
int get_ps_threshold(uint16_t *low_threshold,uint16_t *high_threshold);

int set_akm_calibrate(struct AKL_NV_PRMS akm_calibrate);
int get_akm_calibrate(struct AKL_NV_PRMS *akm_calibrate);
int set_yas_calibrate(struct YAS_NV_PRMS yas_calibrate);
int get_yas_calibrate(struct YAS_NV_PRMS *yas_calibrate);
float getDisplayFrequency(void);
float getScreenshotHorizontalFOV(void);
int setScreenshotHorizontalFOV(float value);
float getScreenshotVerticalFOV(void);
int setScreenshotVerticalFOV(float value);
float getDistanceEyeToLens(void);
int setDistanceEyeToLens(float value);
float getDistanceLensToScreen(void);
int setDistanceLensToScreen(float value);
float getLensFocalLength(void);
int setLensFocalLength(float value);
float getDistanceScaleX(void);
int setDistanceScaleX(float value);
float getDistanceScaleY(void);
int setDistanceScaleY(float value);
float getRenderOverfill(void);
int setRenderOverfill(float value);
int getPolynomialCoeffsRed(float *value);
int setPolynomialCoeffsRed(float value[]);
int getPolynomialCoeffsGreen(float *value);
int setPolynomialCoeffsGreen(float value[]);
int getPolynomialCoeffsBlue(float *value);
int setPolynomialCoeffsBlue(float value[]);
int getRecommendedRenderTargetSize(double *value);
int setRecommendedRenderTargetSize(double value[]);
int getRealScreenSize(float *value);
int setRealScreenSize(float value[]);
BOOL getContainsRecenter(void);
int setContainsRecenter(int value);
int getDisplayOnDevice(int *value);
int setDisplayOnDevice(int value);
int getSensorToHead(float *value);
int setSensorToHead(float value[]);
int getVendorPartNumber(char *value);
int setVendorPartNumber(const char *value, uint32_t length);
int setSemiPara(const SEMI_STATUS *value, uint32_t offset);
int getSemiPara(SEMI_STATUS *value,char offset);






#ifdef __cplusplus
}
#endif

#endif /* __MiSC_DATA_H */

