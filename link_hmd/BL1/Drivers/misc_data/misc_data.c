#include <string.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "misc_data.h"
#include "flash_drv.h"
#include "htc_memory_define.h"
#include "bl_header.h"
//#include "usbd_app.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MISC_DataTypeDef MISC_DataStructure = {0};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

int loadMiscData(void)
{
	uint32_t ret;
	misc_info("loadMiscData\n");
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
		misc_err("[misc_data] flash erase failed\n");
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

int getCustomID(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_CustomID[0])) {
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_CustomID);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_CustomID), 0x0,
				sizeof(MISC_DataStructure.Prop_CustomID));
		strncpy((char *)&(MISC_DataStructure.Prop_CustomID), value,
				length);
	} else {
		misc_err("setCustomID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setCustomID success\n");
		return 0;
	} else {
		misc_err("setCustomID failed\n");
		return -1;
	}
}

int set_skuid(int id)
{
	if (loadMiscData()) {
		misc_err("set_skuid failed\n");
		return -1;
	}

	MISC_DataStructure.Prop_SKUID = id;

	if (saveMiscData()) {
		misc_err("set_skuid failed\n");
		return -1;

	}

	misc_info("set_skuid success\n");

	return 0;
}

int get_skubid(int *id)
{

	if (loadMiscData()) {
		return -1;
	}

	*id = MISC_DataStructure.Prop_SKUID;

	return 0;
}
int getModelName(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ModelName[0])) {
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ModelName);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ModelName), 0x0,
				sizeof(MISC_DataStructure.Prop_ModelName));
		strncpy((char *)&(MISC_DataStructure.Prop_ModelName), value,
				length);
	} else {
		misc_err("setModelName failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setModelName success\n");
		return 0;
	} else {
		misc_err("setModelName failed\n");
		return -1;
	}
}

int getMBSerialNumber(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_MBSerialNumber[0])) {
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_MBSerialNumber);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_MBSerialNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_MBSerialNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_MBSerialNumber), value,
				length);
	} else {
		misc_err("setMBSerialNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setMBSerialNumber success\n");
		return 0;
	} else {
		misc_err("setMBSerialNumber failed\n");
		return -1;
	}
}

int getColorID(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ColorID[0])){
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ColorID);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ColorID), 0x0,
				sizeof(MISC_DataStructure.Prop_ColorID));
		strncpy((char *)&(MISC_DataStructure.Prop_ColorID), value,
				length);
	} else {
		misc_err("setColorID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setColorID success\n");
		return 0;
	} else {
		misc_err("setColorID failed\n");
		return -1;
	}
}

int getProjectID(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_ProjectID[0])) {
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_ProjectID);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_ProjectID), 0x0,
				sizeof(MISC_DataStructure.Prop_ProjectID));
		strncpy((char *)&(MISC_DataStructure.Prop_ProjectID), value,
				length);
	} else {
		misc_err("setProjectID failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setProjectID success\n");
		return 0;
	} else {
		misc_err("setProjectID failed\n");
		return -1;
	}
}

int getSerialNumber(char *value)
{
	if (!loadMiscData()) {
		if (!misc_is_ascii_char(MISC_DataStructure.Prop_SerialNumber[0])){
			return -1;
		}
		strcpy(value,(const char *)&MISC_DataStructure.Prop_SerialNumber);
		return 0;
	} else {
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
	if (!loadMiscData()) {
		memset((char *)&(MISC_DataStructure.Prop_SerialNumber), 0x0,
				sizeof(MISC_DataStructure.Prop_SerialNumber));
		strncpy((char *)&(MISC_DataStructure.Prop_SerialNumber), value, length);
	} else {
		misc_err("setSerialNumber failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("setSerialNumber success\n");
		return 0;
	} else {
		misc_err("setSerialNumber failed\n");
		return -1;
	}
}

int set_pcbid(pcbid_t id)
{
	if (loadMiscData()) {
		misc_err("set_pcbid failed\n");
		return -1;
	}

	MISC_DataStructure.Prop_PCBID = id;

	if (saveMiscData()) {
		misc_err("set_pcbid failed\n");
		return -1;

	}

	misc_info("set_pcbid success\n");

	return 0;
}

int get_pcbid(pcbid_t *id)
{
	if (loadMiscData()) {
		return -1;
	}

	*id = MISC_DataStructure.Prop_PCBID;

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


BOOT_MODE get_bootmode()
{
	if (!loadMiscData()) {
		return MISC_DataStructure.Boot_flag;
	} else {
		misc_err("get_bootmode failed\n");
		return ERROR_MODE;
	}
}

int set_bootmode(BOOT_MODE mode)
{
	if (!loadMiscData()) {
		MISC_DataStructure.Boot_flag = mode;
	} else {
		misc_err("set_bootmode failed\n");
		return -1;
	}
	if (!saveMiscData()) {
		misc_info("set_bootmode success\n");
		return 0;
	} else {
		misc_err("set_bootmode failed\n");
		return -1;
	}
}
