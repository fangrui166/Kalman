#include <stdio.h>
#include <string.h>
#include "fotamode.h"
#include "Usbd_def.h"
#include "flash_drv.h"
#include "htc_memory_define.h"  
#include <string.h>
#include "bl_header.h"
#include "integer.h"
#include "ccgx_fw_io.h"
#include "ccgx_fw.h"

#define FOTA_REGION_START  0x08010000
#define FOTA_REGION_END 0x08040000

static int ccgx_fw_GetHexVal(char c)
{

	if ((c >= '0') && (c <= '9'))

		return (c - '0');

	if ((c >= 'A') && (c <= 'F'))

		return (10 + c - 'A');

	if ((c >= 'a') && (c <= 'f'))

		return (10 + c - 'a');

	return 0;

}

#if 0
//"fw1:4403:107522"
static int get_ccgx_img_info(int *fw_idx, uint8_t *ver, unsigned *size)
{
	int retval;
	unsigned offset;
	uint8_t buff[64];

	offset = HTC_FOTA_CCGX_IMG_VERSION;
	memset(buff, 0x00, sizeof(buff));
	retval = ccgx_fw_fgets(buff, sizeof(buff), &offset);
	if (retval < 0)
		return -1;

	printf("CCGX FOTA HEADER : %s\n", buff);

	if (buff[0] != 'f' ||  buff[1] != 'w' || buff[3] != ':')
		return -2;
	*fw_idx = buff[2] - '1';

	if (buff[8] != ':')
		return -3;
	ver[0] = ccgx_fw_GetHexVal(buff[4]) << 4 | ccgx_fw_GetHexVal(buff[5]);
	ver[1] = ccgx_fw_GetHexVal(buff[6]) << 4 | ccgx_fw_GetHexVal(buff[7]);

	*size = strtoul((char const *)buff + 9, NULL, 0);

	return offset;

}
#endif
void fota_move_image(uint16_t value,dfu_move_img_callback fota_callback,USBD_HandleTypeDef *cb_data){
    int state = 0;
    int fw_idx;
    unsigned fw_size;
    uint8_t cur_ver[8];
	uint8_t ver[2];
    int result=0;
    printf("[fota mode]%s,enter to move",__func__);
    if (ccgx_get_image_header((void *)(HTC_FOTA_CCGX_IMG_VERSION + 0x08000000), NULL) == 0) {

		printf("in %s : %s - %s\n", __func__, __DATE__, __TIME__);
#if 0
		fw_offset = get_ccgx_img_info(&fw_idx, img_ver + 6, &fw_size);
		if (fw_offset < 0) {
			fota_callback(cb_data, 0);
			return ;
		}
#endif
#define CCG4_FW1	("FW1")
#define CCG4_FW2	("FW2")

		if(strcmp(g_ccg4_header.header.fw_index,CCG4_FW1)==0){
			fw_idx=0;
		}
		else if(strcmp(g_ccg4_header.header.fw_index,CCG4_FW2)==0){
			fw_idx=1;
		}else {
			printf("in %s : fw_idx %d error\n", __func__, fw_idx + 1);
			fota_callback(cb_data,0);
			return;
		}
		printf("fw_idx = %d\n",fw_idx);
		printf("fw_projname = %s, sub_name = %s, version = %s,build_data = %s,build_time=%s,crc=%x,length = %d,index=%s,\n",
						g_ccg4_header.header.projectName,
						g_ccg4_header.header.subProjectName,
						g_ccg4_header.header.version,
						g_ccg4_header.header.build_date,
						g_ccg4_header.header.build_time,
						g_ccg4_header.header.crc,
						g_ccg4_header.header.length,
						g_ccg4_header.header.fw_index);

		ccgx_get_fw_version(fw_idx + 1, cur_ver);

		printf("UPgrade FW%d : 0x%02x 0x%02x -> %s\n", fw_idx + 1, cur_ver[6], cur_ver[7], g_ccg4_header.header.version);
		// compare version
		ver[0] = ccgx_fw_GetHexVal(g_ccg4_header.header.version[0]) << 4 | ccgx_fw_GetHexVal(g_ccg4_header.header.version[1]);
		ver[1] = ccgx_fw_GetHexVal(g_ccg4_header.header.version[2]) << 4 | ccgx_fw_GetHexVal(g_ccg4_header.header.version[3]);
		if(ver[0]==cur_ver[6]&&ver[1]==cur_ver[7])
		 {
			printf("version is the same as now ccg4 used.\r\n");
			fota_callback(cb_data, 1);
			return ;
		}
		fw_size = g_ccg4_header.header.length;
		printf("offset is %d,fw_size is %d",sizeof(ccg4_img_header_t),fw_size);
		ccgx_set_fw_info(fw_idx, sizeof(ccg4_img_header_t)+HTC_FOTA_CCGX_IMG_VERSION , fw_size);

		result=ccgx_upgrade_fw_bl(fw_idx,false);
		if(result > 0 ){//fail

			fota_callback(cb_data, result);
			return ;
		}
    }  else {
		printf("%s the package is wrong\n",__func__);
        // wrong
        fota_callback(cb_data,state);
		return;
	}
    if (ccgx_get_image_header((void *)(HTC_FOTA_CCGX_IMG_VERSION1 + 0x08000000), NULL) == 0) {

		printf("in %s : %s - %s\n", __func__, __DATE__, __TIME__);
#if 0
		fw_offset = get_ccgx_img_info(&fw_idx, img_ver + 6, &fw_size);
		if (fw_offset < 0) {
			fota_callback(cb_data, 0);
			return ;
		}
#endif
#define CCG4_FW1	("FW1")
#define CCG4_FW2	("FW2")

		if(strcmp(g_ccg4_header.header.fw_index,CCG4_FW1)==0){
			fw_idx=0;
		}
		else if(strcmp(g_ccg4_header.header.fw_index,CCG4_FW2)==0){
			fw_idx=1;
		}else return;

		printf("fw_idx = %d\n",fw_idx);
		printf("fw_projname = %s, sub_name = %s, version = %s,build_data = %s,build_time=%s,crc=%x,length = %d,index=%s,\n",
						g_ccg4_header.header.projectName,
						g_ccg4_header.header.subProjectName,
						g_ccg4_header.header.version,
						g_ccg4_header.header.build_date,
						g_ccg4_header.header.build_time,
						g_ccg4_header.header.crc,
						g_ccg4_header.header.length,
						g_ccg4_header.header.fw_index);

		ccgx_get_fw_version(fw_idx + 1, cur_ver);

		printf("UPgrade FW%d : 0x%02x 0x%02x -> %s\n", fw_idx + 1, cur_ver[6], cur_ver[7], g_ccg4_header.header.version);
		// compare version
		ver[0] = ccgx_fw_GetHexVal(g_ccg4_header.header.version[0]) << 4 | ccgx_fw_GetHexVal(g_ccg4_header.header.version[1]);
		ver[1] = ccgx_fw_GetHexVal(g_ccg4_header.header.version[2]) << 4 | ccgx_fw_GetHexVal(g_ccg4_header.header.version[3]);
		if(ver[0]==cur_ver[6]&&ver[1]==cur_ver[7])
		 {
			printf("version is the same as now ccg4 used.\r\n");
			fota_callback(cb_data, 1);
			return ;
		}
		fw_size = g_ccg4_header.header.length;
		printf("offset is %d,fw_size is %d",sizeof(ccg4_img_header_t),fw_size);
		ccgx_set_fw_info(fw_idx, sizeof(ccg4_img_header_t)+HTC_FOTA_CCGX_IMG_VERSION1 , fw_size);

		result=ccgx_upgrade_fw_bl(fw_idx,true);
		if(result > 0 ){//fail

			fota_callback(cb_data, result);
			return ;
		}
	} else {
		printf("%s the package is wrong\n",__func__);
        // wrong
        fota_callback(cb_data,state);
		return;
	}
	if(result == 0)
		fota_callback(cb_data, 1);
}
