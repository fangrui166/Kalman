

#ifndef BOOTLOADER_INC_BOOTMODE_H_
#define BOOTLOADER_INC_BOOTMODE_H_
typedef enum{
	FOTA_MODE	= 0xFAFAFAFA,
	NORM_MODE	= 0xAEAEAEAE,
	BL_MFG_MODE	= 0xFEFEFEFE,
	SYS_MFG_MODE= 0xDEDEDEDE,
	DFU_MODE	= 0xDFDFDFDF,
	ERROR_MODE	= 0xEDEDEDED,
} BOOT_MODE;

extern volatile unsigned int blRunMode[2];

BOOT_MODE check_mode();
void  enter_dfu_mode();
#endif /* BOOTLOADER_INC_BOOTMODE_H_ */
