#ifndef __MISC_DATA_H__
#define __MISC_DATA_H__
#include "stm32f4xx_hal.h"

#define BOOL int

/*magnetic sensor struct define start*/
#define AKM_DISABLE_DOEPLUS
//#define AKM_ENABLE_PDC
//#define AKSC_MATH_DOUBLE
typedef union _int16vec { // Three-dimensional vector constructed of signed 16 bits fixed point numbers
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } u;
    int16_t v[3];
} int16vec;

typedef union _int32vec { // Three-dimensional vector constructed of signed 32 bits fixed point numbers
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
    } u;
    int32_t v[3];
} int32vec;

typedef enum _AKSC_HDST {
    AKSC_HDST_UNSOLVED  = 0,           // Offset is not determined.
    AKSC_HDST_L0        = 1,           // Offset has been determined once or more with Level0 parameter
    AKSC_HDST_L1        = 2,           // Offset has been determined once or more with Level1 parameter
    AKSC_HDST_L2        = 3            // Offset has been determined once or more with Level2 parameter
} AKSC_HDST;

#ifdef AKSC_MATH_DOUBLE
typedef double AKSC_FLOAT;
#else
typedef float AKSC_FLOAT;
#endif

struct AKL_NV_PRMS {
    /*! This value is used to identify the data area is AKL_NV_PRMS.
     * This value should be #AKL_NV_MAGIC_NUMBER. */
    uint32_t     magic;

    int16vec   va_hsuc_ho;                  /*!< fine offset of magnetic vector */
    int16vec   va_hflucv_href;              /*!< rough value of magnetic vector */
    AKSC_HDST  a_hsuc_hdst;                 /*!< status of magnetic offset */
    int32vec   va_hsuc_hbase;               /*!< rough offset of magnetic vector */
#ifndef AKM_DISABLE_DOEPLUS
    AKSC_FLOAT a_doep_prms[AKSC_DOEP_SIZE]; /*!< a parameter for DOEPlus */
#endif
#ifdef AKM_ENABLE_PDC
    uint8_t      a_pdc[AKL_PDC_SIZE];         /*!< a parameter for PDC */
#endif
};

struct YAS_NV_PRMS
{
	uint32_t magic;
	int32_t offset[3];
	uint8_t accuracy;
};
/*magnetic sensor struct define end*/

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
    uint32_t G_senser_calib_header;   /* Header should be 0x67676767 after
calibration */
    uint32_t G_senser_selftest_header;/* Header should be 0x67676767 after
selftest */
    char Prop_Reserved2[PROP_RESERVED2_LENTH];/*Reserved for further use*/

    //Gyro-Senser Calibration Data Area
    int32_t Gyro_calib_x_offset;     /* x -offset */
    int32_t Gyro_calib_y_offset;     /* y -offset */
    int32_t Gyro_calib_z_offset;     /* z -offset */
    uint32_t Gyro_calib_header;       /* Header should be 0x67676767 after
calibration */
    uint32_t Gyro_selftest_header;    /* Header should be 0x67676767 after
selftest */
    char Prop_Reserved3[PROP_RESERVED3_LENTH];/*Reserved for further use*/

    //p-Senser Calibration Data Area
    uint16_t ps_threshold_low_value;
    uint16_t ps_threshold_high_value;

    char Prop_Reserved4[PROP_RESERVED4_LENTH];/*Reserved for further other
sensor use*/

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
}MISC_DataTypeDef;


int get_pcbid(pcbid_t *id);

#endif
