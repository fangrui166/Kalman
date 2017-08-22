
// update bootloader in system.
#include <stdio.h>
#include <stdint.h>
#include "flash_drv.h"
#include "htc_memory_define.h"
#include <string.h>
#include "sys_header.h"
#include "stm32f4xx_hal.h"

typedef enum{
	FOTA_MODE=0xFAFAFAFA,
	NORM_MODE=0xAEAEAEAE,
	MFG_MODE=0xFEFEFEFE,
	DFU_MODE=0xDFDFDFDF,
	BL_UPDATE_MODE=0xBABABABA,
	ERROR_MODE=0xEDEDEDED,
} BOOT_MODE;

void bl_update_process()
{
	unsigned char buffer[256];
	char header[9]={0};
	unsigned dest_addr;
	unsigned source_addr;
	unsigned char bl_update_counter=0;
	img_header_struct bl_header;
    img_header_struct bl_header_pre;
	printf("%s enter\n",__func__);
	VR_flash_read(REGION_FLASH_SECTOR4, header, 4); // 4 represent fota header "fota"
	unsigned int bl_update_mode=0;

	if( strncmp(header,BL_FOTA,4)==0) {	//bootloader fota update, need compare version
		bl_update_mode = FOTA_MODE;

		if( get_image_header((void*)(HTC_BOOT_FOTA_VERSION_LOCATION+4),&bl_header)==0 ){

			if(strncmp(bl_header.header_ID,BL_HEADER,8)==0) {
				// compare version
			    if( get_image_header((void*)(HTC_BOOT_VERSION_LOCATION),&bl_header_pre)==0){
                              if(strncmp(bl_header.version,bl_header_pre.version,16) !=0){
                                    goto bl_update;
                              }
                               else{
                                    printf("bootloader is the newest \n");
                                    return;
                               }
                        }
                       else{
                             printf("image build error\n");
                             return;
                        }
			}else {
				// image error, should never happen
				printf("image build error\n");
				return;
			}
		}
	}else {

		if( get_image_header((void*)HTC_BOOT_FOTA_VERSION_LOCATION,&bl_header)==0 ){

			// RD update bootloader, no need to compare version.
			if(strncmp(bl_header.header_ID,BL_HEADER,8)==0) {
				printf("start update bootloader\n");
				goto bl_update;
			}else {
				//no need to update bootloader.
				return;
			}
		}else {
			return;
		}
	}

bl_update:
	// repeat 3 times at most.
	if(bl_update_counter<3) {

		if(VR_flash_erase(REGION_FLASH_START,0x00008000)==0){
			dest_addr = REGION_FLASH_SECTOR0;

			if(bl_update_mode==FOTA_MODE)
				source_addr = REGION_FLASH_SECTOR4+4; //add header "fota"
			else
				source_addr = REGION_FLASH_SECTOR4; //add header

			while( dest_addr<REGION_FLASH_SECTOR2 ){
				VR_flash_read(source_addr, buffer, 256);

				if(VR_flash_write(buffer,dest_addr,256)==0) {
					source_addr+=256;
					dest_addr+=256;
				}else{//write fail
					bl_update_counter++;
					goto bl_update;
				}
			}
			//verify
			if( dest_addr==REGION_FLASH_SECTOR2 ){
				dest_addr = REGION_FLASH_SECTOR0-REGION_FLASH_START;

				if(bl_update_mode==FOTA_MODE)
					source_addr = REGION_FLASH_SECTOR4+4-REGION_FLASH_START; //add header "fota"
				else
					source_addr = REGION_FLASH_SECTOR4-REGION_FLASH_START; //add header

				while(dest_addr<(REGION_FLASH_SECTOR2-REGION_FLASH_START)){

					if( *((unsigned int*)(source_addr))!=*((unsigned int*)(dest_addr))){ //verify error
						bl_update_counter++;
						goto bl_update;
					}
					source_addr+=4;
					dest_addr+=4;
				}
			}
		}else{ //erase failt.
			bl_update_counter++;
			goto bl_update;
		}
		// update success, ereas fota partition
		VR_flash_erase(REGION_FLASH_SECTOR4,0x00008000);
		printf("update bootloader success\n");
		HAL_Delay(1000U);
		HAL_NVIC_SystemReset();
	}else{
		printf("flash error, please contact vendor for help\n");
	}


	return;
}
