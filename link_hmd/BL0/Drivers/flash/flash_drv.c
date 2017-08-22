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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "flash_drv.h"
#include "htc_memory_define.h"
#include "string.h"


/* Private variables */

/* Private functions */

/*
 * flash init sequence without ThreadX components
 */
int  flash_init(void)
{
    //xlog(LOG_TAG, HTCLOG_INFO, "flash driver init start\n");

    /* enable flash write protect */
    /* Lock the Option Bytes */
    HAL_FLASH_OB_Lock();
    /* Lock the Flash Program Erase controller */
    HAL_FLASH_Lock();
    return 0;
}

// get the address in which sector
static FLASH_PART get_erase_sector(uint32_t addr,uint32_t len)
{
	// for our partition table
#if 0
	if(addr>=REGION_FLASH_SECTOR0&&(addr+len-1)<REGION_FLASH_SECTOR2){
		return PART_BL0;
	}
	else if(addr>=REGION_FLASH_SECTOR2&&(addr+len-1)<REGION_FLASH_SECTOR3 ){
		return PART_MISC_DATA;
	}
	else if(addr>=REGION_FLASH_SECTOR3&&(addr+len-1)<REGION_FLASH_SECTOR4 ){
		return PART_LOG;
	}
	else if(addr>=REGION_FLASH_SECTOR5&&(addr+len-1)<REGION_FLASH_SECTOR6 ){
		return PART_BL1;
	}
	else if(addr>=REGION_FLASH_SECTOR6&&(addr+len-1)<REGION_FLASH_SECTOR8 ){
		return PART_RTOS;
	}
	else if(addr>=REGION_FLASH_SECTOR8&&(addr+len-1)<REGION_FLASH_SECTOR10 ){
		return PART_FOTA;
	}
#else
    if(addr>=REGION_FLASH_SECTOR0&&(addr+len-1)<REGION_FLASH_SECTOR1){
        return PART_BL0;
    }
    else if(addr>=REGION_FLASH_SECTOR1&&(addr+len-1)<REGION_FLASH_SECTOR2 ){
        return PART_MISC_DATA;
    }
    else if(addr>=REGION_FLASH_SECTOR2&&(addr+len-1)<REGION_FLASH_SECTOR5 ){
        return PART_MFG_BL1;
    }
    else if(addr>=REGION_FLASH_SECTOR5&&(addr+len-1)<REGION_FLASH_SECTOR6 ){
        return PART_FOTA_LOG;
    }
    else if(addr>=REGION_FLASH_SECTOR6&&(addr+len-1)<REGION_FLASH_END ){
        return PART_RTOS;
    }
#endif
	else
		return PART_ERROR;
}
uint32_t VR_flash_read(uint32_t addr, void *buff, unsigned size)
{
	if(buff==NULL){
		//printf("flash read erro due to buffer is null\n");
		return 0;
	}
	addr = addr - REGION_FLASH_START;
    memcpy(buff, (void*)(addr), size);
    return size;
}

/*
 * Erase flash
 */
uint32_t VR_flash_erase(uint32_t addr, uint32_t length)
{
    volatile uint32_t NbrOfPage = 0x0;
    volatile HAL_StatusTypeDef FLASHStatus = HAL_OK;
    uint32_t end_addr = 0;
    FLASH_EraseInitTypeDef EraseInfo;
    uint32_t EraseErrorPage = 0;
    FLASH_PART curr_sec;
    //printf("[FLASH_DRV] flash_erase addr=0x%X, length=%u\r\n", addr, length);

    /* Check write range */
    if (length == 0)
    {
        //printf("[FLASH_DRV] invalid length 0x%08X\r\n", length);
        return 1;
    }

    if (!IS_FLASH_PROGRAM_ADDRESS(addr))
    {
        //printf("[FLASH_DRV] invalid start_addr 0x%X\r\n", addr);
        return 1;
    }
    end_addr = addr + length;
    if (!IS_FLASH_PROGRAM_ADDRESS(end_addr - 1))
    {
        //printf("[FLASH_DRV] invalid end_addr 0x%X\r\n", end_addr);
        return 1;
    }
    curr_sec = get_erase_sector(addr,length);
    if( curr_sec == PART_ERROR){
        //printf("[FLASH_DRV] invalid sector to erase start 0x%x, end 0x%x\r\n",addr, end_addr);
        return 1;
    }
    // now erase can only erase one sector a time.
    EraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
    __disable_irq();
    /* Unlock flash */
    HAL_FLASH_Unlock();
    /* Clear ERR flags */
    __HAL_FLASH_CLEAR_FLAG(  FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR|FLASH_SR_PGPERR );
    switch(curr_sec){
#if 0
	case PART_BL0:
		EraseInfo.Sector= FLASH_SECTOR_0;
		EraseInfo.NbSectors = 2;
		break;
	case PART_MISC_DATA:
		EraseInfo.Sector= FLASH_SECTOR_2;
		EraseInfo.NbSectors = 1;
		break;
	case PART_LOG:
		EraseInfo.Sector= FLASH_SECTOR_3;
		EraseInfo.NbSectors = 1;
		break;
    case PART_BL1:
        EraseInfo.Sector= FLASH_SECTOR_5;
        EraseInfo.NbSectors = 1;
        break;
    case PART_RTOS:
        EraseInfo.Sector= FLASH_SECTOR_6;
        EraseInfo.NbSectors = 2;
        break;
	case PART_FOTA:
		EraseInfo.Sector= FLASH_SECTOR_8;
		EraseInfo.NbSectors = 2;
		break;
#else
    case PART_BL0:
        EraseInfo.Sector= FLASH_SECTOR_0;
        EraseInfo.NbSectors = 1;
        break;
    case PART_MISC_DATA:
        EraseInfo.Sector= FLASH_SECTOR_1;
        EraseInfo.NbSectors = 1;
        break;
    case PART_MFG_BL1:
        EraseInfo.Sector= FLASH_SECTOR_2;
        EraseInfo.NbSectors = 3;
        break;
    case PART_FOTA_LOG:
        EraseInfo.Sector= FLASH_SECTOR_5;
        EraseInfo.NbSectors = 1;
        break;
    case PART_RTOS:
        EraseInfo.Sector= FLASH_SECTOR_6;
        EraseInfo.NbSectors = 2;
        break;
#endif
	default:
	        //printf("[FLASH_DRV] invalid sector to erase start 0x%x, end 0x%x\r\n",addr, end_addr);
	        return 1;
    }
    // only has bank1

    EraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	FLASHStatus = HAL_FLASHEx_Erase(&EraseInfo, &EraseErrorPage);

	if (FLASHStatus != HAL_OK)
    {
        /* Lock flash */
        HAL_FLASH_Lock();
        __enable_irq();

        //printf("[FLASH_DRV] flash_erase failed !! status=%d, address = 0x%08X\r\n", FLASHStatus, addr );

        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();
    __enable_irq();
    //printf("[FLASH_DRV] flash_erase success !!\r\n");

    return 0;
}


/*
 * Write to flash without erase
 */
uint32_t VR_flash_write(void * source, uint32_t addr, uint32_t length)
{
    uint8_t alignment_bytes = 0x0;
    HAL_StatusTypeDef FLASHStatus = HAL_OK;
    uint32_t end_addr = 0;
    uint32_t iterator = 0x0;
    uint64_t data = 0;
    uint8_t alignment=0;

    //printf("[FLASH_DRV] flash_write offset=0x%X, length=%u\r\n", addr, length);
    /* Check write range */

    if (!IS_FLASH_PROGRAM_ADDRESS(addr)){
        //printf("[FLASH_DRV] invalid start_addr %u\r\n", addr);
        return 1;
    }
    end_addr = addr + length;
    if (!IS_FLASH_PROGRAM_ADDRESS(end_addr)){
        //printf("[FLASH_DRV] invalid end_addr %u\r\n", end_addr);
        return 1;
    }

    if( addr%4 == 0){
	alignment = FLASH_TYPEPROGRAM_WORD;
	alignment_bytes = 4;
    } else if( addr%2 == 0){
	alignment = FLASH_TYPEPROGRAM_HALFWORD;
	alignment_bytes = 2;
    } else{
	alignment = FLASH_TYPEPROGRAM_BYTE;
	alignment_bytes = 1;
    }

    __disable_irq();
    /* Unlock flash */
    HAL_FLASH_Unlock();

    /* Clear ERR flags */
    __HAL_FLASH_CLEAR_FLAG(   FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR|FLASH_SR_PGPERR );
    /* Write loop */
    iterator = 0x0;


    while((iterator+alignment_bytes) <= length)
    {
		if(alignment == FLASH_TYPEPROGRAM_DOUBLEWORD){
			data = * ((uint64_t *)(source)+ (iterator/alignment_bytes));
				FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (addr + iterator), data);

		} else if( alignment == FLASH_TYPEPROGRAM_WORD){
				data = * ((uint32_t *)(source)+ (iterator/alignment_bytes));
				FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (addr + iterator), data);

		} else if( alignment == FLASH_TYPEPROGRAM_HALFWORD){
				data = * ((uint16_t *)(source)+ (iterator/alignment_bytes));
				FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (addr + iterator), data);

		}else if( alignment == FLASH_TYPEPROGRAM_BYTE){
				data = * ((uint8_t *)(source)+ (iterator/alignment_bytes));
				FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (addr + iterator), data);

		}
		if (FLASHStatus != HAL_OK){
			break;
		}
			iterator += alignment_bytes;
    }
    // flash left data
    while(iterator<length){
		data = * ((uint8_t *)(source)+ (iterator));
		FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (addr + iterator), data);
		if (FLASHStatus != HAL_OK){
			break;
		}
		iterator += 1;
    }
    if (FLASHStatus != HAL_OK)
    {
        /* Lock flash */
        HAL_FLASH_Lock();

        __enable_irq();


        //printf("[FLASH_DRV] flash_write failed !! status=%d, iterator = %u\r\n", FLASHStatus, iterator);

        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();
    __enable_irq();

    //printf("[FLASH_DRV] flash_write success !!\r\n");

    return 0;
}






/**
  * @brief  Enable NOR flash OB WP
  * @param  None
  * @retval None
  */
uint32_t flash_ob_wp_enable(FLASH_WP_AREA wp_area)
{
    FLASH_OBProgramInitTypeDef OBS_1A ;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t ret_val = 0;

    /* Init OB structure */
    __disable_irq();

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_SOP);
    HAL_FLASH_OB_Unlock();

    if(wp_area== FLASH_WP_BL ){
        /* Get current OB */
        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_ENABLE;
        OBS_1A.WRPSector = OB_WRP_SECTOR_0|OB_WRP_SECTOR_1;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);


    } else if(wp_area==FLASH_WP_SYS) {

        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_ENABLE;
        OBS_1A.WRPSector = OB_WRP_SECTOR_6|OB_WRP_SECTOR_7;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);

        if(status != HAL_OK) { ret_val = 1; }

    } else {
        ret_val = 5;
        goto FLASH_OB_WP_EXIT;
    }



FLASH_OB_WP_EXIT:
    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    __enable_irq();

    if (ret_val > 0) {
        //printf("[FLASH] ERROR: flash_ob_wp_enable ret=%u status=%u \r\n", ret_val, status);
    }
    else{
        //printf("[FLASH] OB WRP set success\r\n");
    }

    return ret_val;
}


/**
  * @brief  Disable NOR flash OB  WP
  * @param  None
  * @retval None
  */
uint32_t flash_ob_wp_disable(FLASH_WP_AREA wp_area)
{
    FLASH_OBProgramInitTypeDef OBS_1A = {0};
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t ret_val = 0;

    /* Init OB structure */

    __disable_irq();

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_SOP);
    HAL_FLASH_OB_Unlock();

    if(wp_area== FLASH_WP_BL ){

        /* Get current OB */
        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_DISABLE;
        OBS_1A.WRPSector =OB_WRP_SECTOR_0|OB_WRP_SECTOR_1;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);

        if(status != HAL_OK) { ret_val = 1; }

    } else if(wp_area==FLASH_WP_SYS) {

        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_DISABLE;
        OBS_1A.WRPSector = OB_WRP_SECTOR_6|OB_WRP_SECTOR_7;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);

        if(status != HAL_OK) { ret_val = 1; }

    } else {
        ret_val = 5;
        goto FLASH_OB_WP_EXIT;
    }

FLASH_OB_WP_EXIT:
    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    __enable_irq();

    if (ret_val > 0) {
        //printf("[FLASH] ERROR: flash_ob_wp_disable ret=%u status=%u \r\n", ret_val, status);
    }
    else{
        //printf("[FLASH] OB WRP Disable success\r\n");
    }

    return 0;
}


