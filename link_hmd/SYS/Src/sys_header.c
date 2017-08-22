/*
 * HTC Corporation Proprietary Rights Acknowledgment
 *
 * Copyright (C) 2013 HTC Corporation
 *
 * All Rights Reserved.
 *
 * The information contained in this work is the exclusive property of HTC Corporation
 * ("HTC").  Only the user who is legally authorized by HTC ("Authorized User") has
 * right to employ this work within the scope of this statement.  Nevertheless, the
 * Authorized User shall not use this work for any purpose other than the purpose
 * agreed by HTC.  Any and all addition or modification to this work shall be
 * unconditionally granted back to HTC and such addition or modification shall be
 * solely owned by HTC.  No right is granted under this statement, including but not
 * limited to, distribution, reproduction, and transmission, except as otherwise
 * provided in this statement.  Any other usage of this work shall be subject to the
 * further written consent of HTC.
 */
#include "htc_version.h"
#include "htc_memory_define.h"
#include "sys_header.h"
#include "flash_drv.h"

#pragma location = HTC_SYS_VERSION_LOCATION
__root const char version_string[16] = SW_SYS_VER;
#pragma location = (HTC_SYS_VERSION_LOCATION+0x10)
__root const char bl_header[16] = "systrtos";
#pragma location = (HTC_SYS_VERSION_LOCATION+0x20)
__root const char build_date[16] = __DATE__;
#pragma location = (HTC_SYS_VERSION_LOCATION+0x30)
__root const char build_time[16] = __TIME__;
int get_image_header(void* header_addr,img_header_struct* header)
{
	if(header_addr==0)
		return -1;

	if(header==0)
		return -1;

	VR_flash_read((uint32_t)header_addr,header,sizeof(img_header_struct));
	//printf(" version is %s, build date is %s,time is %s, header is %s \n",header->version,header->build_date,header->build_time,header->header_ID);
	return 0;
}
