#ifndef __BL_HEADER_H_
#define __BL_HEADER_H_
typedef struct {
	char version[16];
	char header_ID[16];
	char build_date[16];
	char build_time[16];
}img_header_struct;
typedef union {
	struct {
		char    projectName[16]; // Link_HMD
		char    subProjectName[16]; //sys¡bl0;bl1trackpad;ccg4;ble
		char    version[16]; //1.01.001.1
		char    build_date[16];
		char    build_time[16];
		int     crc;
		int     length;
		char    reserved[0]; //128 bytes reserve
	} header;
	char header_buf[128];
} img_header_t;

typedef union {
	struct {
		char    projectName[16]; // Link_HMD
		char    subProjectName[16]; //sys¡bl0;bl1trackpad;ccg4;ble
		char    version[16]; //1.01.001.1
		char    build_date[16];
		char    build_time[16];
		int     crc;
		int     length;
		char 	fw_index[16];//fw1,fw2
		char    reserved[0]; //128 bytes reserve
	} header;
	char header_buf[128];
} ccg4_img_header_t;

extern ccg4_img_header_t g_ccg4_header;
int get_image_header(void* header_addr,img_header_struct* header);
int ccgx_get_image_header(void* header_addr,img_header_struct* header);

#endif
