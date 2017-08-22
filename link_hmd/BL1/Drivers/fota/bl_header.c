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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "htc_version.h"
#include "htc_memory_define.h"
#include "bl_header.h"
#include "flash_drv.h"

#pragma location = HTC_BOOT_VERSION_LOCATION
__root const char version_string[16] = SW_BL_VER;
#pragma location = (HTC_BOOT_VERSION_LOCATION+0x10)
__root const char bl_header[16] = "Link BL1";
#pragma location = (HTC_BOOT_VERSION_LOCATION+0x20)
__root const char build_date[16] = __DATE__;
#pragma location = (HTC_BOOT_VERSION_LOCATION+0x30)
__root const char build_time[16] = __TIME__;

int get_image_header(void* header_addr,img_header_struct* header)
{
	if(header_addr==0)
		return -1;

	if(header==0)
		return -1;

	VR_flash_read((uint32_t)header_addr,header,sizeof(img_header_struct));

	return 0;
}

ccg4_img_header_t g_ccg4_header= {0};
int ccgx_get_image_header(void* header_addr,img_header_struct* header)
{

	if(header_addr==0)
		return -1;
#define PROJECT_NAME 	("LINK_HMD")
#define CCG4_SUBPROJ_NAME ("ccg4")

    memset(&g_ccg4_header, 0x00, sizeof(ccg4_img_header_t));
	VR_flash_read((uint32_t)header_addr, &g_ccg4_header,sizeof(ccg4_img_header_t));

	if((strcmp(g_ccg4_header.header.projectName,PROJECT_NAME)==0)&& (strcmp(g_ccg4_header.header.subProjectName,CCG4_SUBPROJ_NAME)==0))
		return 0;
	return -1;
}

