#include "misc_data.h"
#include "flash_drv.h"
#include "htc_memory_define.h"

MISC_DataTypeDef MISC_DataStructure = {0};

int loadMiscData(void)
{
	uint32_t ret;
	ret = VR_flash_read(REGION_FLASH_SECTOR1, (void *)&MISC_DataStructure, sizeof(MISC_DataTypeDef));
	if (ret == sizeof(MISC_DataTypeDef)) {
		return 0;
	} else {
		return -1;
	}
}

int get_pcbid(pcbid_t *id)
{
	if (loadMiscData()) {
		return -1;
	}
	*id = MISC_DataStructure.Prop_PCBID;
	return 0;
}

