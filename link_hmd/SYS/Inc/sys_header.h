#ifndef __BL_HEADER_H_
#define __BL_HEADER_H_
typedef struct {
	char version[16];
	char header_ID[16];
	char build_date[16];
	char build_time[16];
}img_header_struct;

int get_image_header(void* header_addr,img_header_struct* header);
#endif
