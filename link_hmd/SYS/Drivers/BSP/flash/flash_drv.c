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
#include "FreeRTOS.h"
#include "semphr.h"
#include "component.h"
#include "string.h"

/*
 * flash init sequence without ThreadX components
 */
void dump_flash_register()
{
	flash_debug("[FLASH_DRV] flash_opt reg 0x%x\r\n",FLASH->OPTCR);
	flash_debug("[FLASH_DRV] flash_opt reg 0x%x\r\n",FLASH->CR);
	flash_debug("[FLASH_DRV] flash_opt reg 0x%x\r\n",FLASH->SR);
	flash_debug("[FLASH_DRV] flash_opt reg 0x%x\r\n",FLASH->ACR);
	flash_debug("[FLASH_DRV] flash_opt reg 0x%x\r\n",FLASH->KEYR);

}
static SemaphoreHandle_t flash_mutex;
int  flash_init(void)
{
    /* enable flash write protect */
    /* Lock the Option Bytes */
    HAL_FLASH_OB_Lock();
    /* Lock the Flash Program Erase controller */
    HAL_FLASH_Lock();
    flash_mutex = xSemaphoreCreateMutex();

    if (flash_mutex == NULL) {
		flash_err("%s: create mutex failed\n", __func__);
		return COMPONENT_ERROR;
	}
	return COMPONENT_OK;
    //xlog(LOG_TAG, HTCLOG_INFO, "flash driver init done\n");
}

// get the address in which sector
static FLASH_PART get_erase_sector(uint32_t addr,uint32_t len)
{
	// for our partition table
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
        return PART_FOTA;
    }
    else if(addr>=REGION_FLASH_SECTOR6&&(addr+len-1)<REGION_FLASH_SECTOR10 ){
        return PART_RTOS;
    }
    else if(addr>=REGION_FLASH_SECTOR11&&(addr+len-1)<=REGION_FLASH_END ){
        return PART_LOG;
	}
	else
		return PART_ERROR;
}
uint32_t VR_flash_read(uint32_t addr, void *buff, unsigned size)
{
		if(buff==NULL){
			flash_err("buff is NULL in function %s\n",__func__);
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
    flash_info("[FLASH_DRV] flash_erase addr=0x%X, length=%u\r\n", addr, length);

    /* Check write range */
    if (length == 0){
    	flash_warning("[FLASH_DRV] invalid length 0x%08X\r\n", length);
        return 1;
    }

    if (!IS_FLASH_PROGRAM_ADDRESS(addr)){
    	flash_warning("[FLASH_DRV] invalid start_addr 0x%X\r\n", addr);
        return 1;
    }
    end_addr = addr + length;
    if (!IS_FLASH_PROGRAM_ADDRESS(end_addr - 1)){
    	flash_warning("[FLASH_DRV] invalid end_addr 0x%X\r\n", end_addr);
        return 1;
    }
    curr_sec = get_erase_sector(addr,length);
    if( curr_sec == PART_ERROR){
    	flash_err("[FLASH_DRV] invalid sector to erase start 0x%x, end 0x%x\r\n",addr, end_addr);
        return 1;
    }
    // now erase can only erase one sector a time.
    EraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
    xSemaphoreTake(flash_mutex, portMAX_DELAY);
    /* Unlock flash */
    HAL_FLASH_Unlock();
    /* Clear ERR flags */
    __HAL_FLASH_CLEAR_FLAG(  FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR|FLASH_SR_PGPERR );

    switch(curr_sec){
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
    case PART_FOTA:
        EraseInfo.Sector= FLASH_SECTOR_5;
        EraseInfo.NbSectors = 1;
        break;
    case PART_RTOS:
        EraseInfo.Sector= FLASH_SECTOR_6;
        EraseInfo.NbSectors = 3;
        break;
	case PART_LOG:
        EraseInfo.Sector= FLASH_SECTOR_11;
        EraseInfo.NbSectors = 1;
        break;

    default:
        flash_warning("[FLASH_DRV] invalid sector to erase start 0x%x, end 0x%x\r\n",addr, end_addr);
        return 1;
    }
    // only has bank1

    EraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	FLASHStatus = HAL_FLASHEx_Erase(&EraseInfo, &EraseErrorPage);

	if (FLASHStatus != HAL_OK){
        /* Lock flash */
        HAL_FLASH_Lock();
        xSemaphoreGive(flash_mutex);
        flash_err("[FLASH_DRV] flash_erase failed !! status=%d, address = 0x%08X\r\n", FLASHStatus, addr );
        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();
    xSemaphoreGive(flash_mutex);
    flash_info("[FLASH_DRV] flash_erase success !!\r\n");

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

    flash_info("[FLASH_DRV] flash_write offset=0x%X, length=%u\r\n", addr, length);
    /* Check write range */

    if (!IS_FLASH_PROGRAM_ADDRESS(addr)){
    	flash_warning("[FLASH_DRV] invalid start_addr %u\r\n", addr);
        return 1;
    }
    end_addr = addr + length;
    if (!IS_FLASH_PROGRAM_ADDRESS(end_addr)){
    	flash_warning("[FLASH_DRV] invalid end_addr %u\r\n", end_addr);
        return 1;
    }
    /* Check write start offset. to decide the write alignment types */
   // nwrite = ((addr ^ (addr - 1)) + 1) >> 1;

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

    xSemaphoreTake(flash_mutex, portMAX_DELAY);
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
        if (FLASHStatus != HAL_OK)
        { break; }
        iterator += alignment_bytes;
    }
    // flash left data
    while( iterator<length ){
    	data = * ((uint8_t *)(source)+ (iterator));
    	FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (addr + iterator), data);

    	if (FLASHStatus != HAL_OK)
        { break; }
    	iterator += 1;
    }
    if (FLASHStatus != HAL_OK){
        /* Lock flash */
        HAL_FLASH_Lock();

        xSemaphoreGive(flash_mutex);
        flash_err("[FLASH_DRV] flash_write failed !! status=%d, iterator = %u\r\n", FLASHStatus, iterator);

        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();
    xSemaphoreGive(flash_mutex);

    flash_info("[FLASH_DRV] flash_write success !!\r\n");

    return 0;
}




/**
  * @brief  NOR flash OB status
  * @param  None
  * @retval None
  */
uint32_t flash_ob_status(void)
{
    FLASH_OBProgramInitTypeDef OBS_1A ;

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_SOP);
    HAL_FLASH_OB_Unlock();

    /* Get current OB */
    HAL_FLASHEx_OBGetConfig(&OBS_1A);

    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    /* Print the status */
    flash_info("  WRPSector 0x%x,  RDPLevel 0x%x, SPR mode 0x%x \r\n",\
    		OBS_1A.WRPSector,OBS_1A.RDPLevel,  (uint8_t)((*(__IO uint8_t *)OPTCR_BYTE3_ADDRESS) & 0x80));
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

        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_WP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_WP_EXIT;}


    } else if(wp_area==FLASH_WP_SYS) {

        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_ENABLE;
        OBS_1A.WRPSector = OB_WRP_SECTOR_6|OB_WRP_SECTOR_7;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);
        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_WP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_WP_EXIT;}

    } else {
        ret_val = 5;
        goto FLASH_OB_WP_EXIT;
    }



FLASH_OB_WP_EXIT:
    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    __enable_irq();

    if (ret_val > 0){
    	flash_err("[FLASH] ERROR: flash_ob_wp_enable ret=%u status=%u \r\n", ret_val, status);
    }
    else{
    	flash_info("[FLASH] OB WRP set success\r\n");
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
        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_WP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_WP_EXIT;}

    } else if(wp_area==FLASH_WP_SYS) {

        HAL_FLASHEx_OBGetConfig(&OBS_1A);
        OBS_1A.OptionType = OPTIONBYTE_WRP;
        OBS_1A.WRPState = OB_WRPSTATE_DISABLE;
        OBS_1A.WRPSector = OB_WRP_SECTOR_6|OB_WRP_SECTOR_7;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);
        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_WP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_WP_EXIT;}

    } else {
        ret_val = 5;
        goto FLASH_OB_WP_EXIT;
    }

FLASH_OB_WP_EXIT:
    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    __enable_irq();

    if (ret_val > 0){
    	flash_err("[FLASH] ERROR: flash_ob_wp_disable ret=%u status=%u \r\n", ret_val, status);
    }
    else{
    	flash_info("[FLASH] OB WRP Disable success\r\n");
    }

    return 0;
}


/**
  * @brief  NOR flash OB WP  status
  * @param  None
  * @retval None
  */
uint32_t flash_ob_wp_is_BL_protected(void)
{
    FLASH_OBProgramInitTypeDef OBS_1A = {0};

    /* Init OB structure */

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_SOP);
    HAL_FLASH_OB_Unlock();

    /* Get current OB */
    HAL_FLASHEx_OBGetConfig(&OBS_1A);

    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    if (((OBS_1A.WRPSector&OB_WRP_SECTOR_0)==0) && ((OBS_1A.WRPSector& OB_WRP_SECTOR_1)== 0) ){
    	flash_info("FLASH_WBL1_Protected\r\n");
        return 1;
    }
    else{
    	flash_info("FLASH_WBL1_UnProtected\r\n");
        return 0;
    }
}
/**
  * @brief  Set flash OB Read-out protection level
  * @param  level 0 to 1 (2 not supported)
  * @retval None
  */
uint32_t flash_ob_rdp_level_set(uint32_t level)
{
    FLASH_OBProgramInitTypeDef OBS_1A = {0};
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t ret_val = 0;

    __disable_irq();

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    OBS_1A.OptionType = OPTIONBYTE_RDP;

    if (level == 1){
        OBS_1A.RDPLevel = OB_RDP_LEVEL_1;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);
        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_RDP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_RDP_EXIT;}
    } else if( level == 0 ) {
        OBS_1A.RDPLevel = OB_RDP_LEVEL_0;
        status = HAL_FLASHEx_OBProgram(&OBS_1A);
        if(status != HAL_OK) { ret_val = 1; goto FLASH_OB_RDP_EXIT;}
        status = HAL_FLASH_OB_Launch();
        if(status != HAL_OK) { ret_val = 2; goto FLASH_OB_RDP_EXIT;}
    }
FLASH_OB_RDP_EXIT:
    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    __enable_irq();

    if (ret_val > 0) {
        flash_err("[FLASH] ERROR: flash_ob_rdp_level_set ret=%u status=%u\r\n", ret_val, status);
    } else {
        flash_info("[FLASH] OB RDP set success (%u)\r\n", level);
    }

    return ret_val;
}


/**
  * @brief  Get flash OB Read-out protection level
  * @param  None
  * @retval level
  */
uint32_t flash_ob_rdp_level_get(void)
{
    uint32_t ret_val = 5;
    FLASH_OBProgramInitTypeDef OBS_1A = {0};

    /* Init OB structure */

    /* Unlock the Options Bytes */
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_SOP);
    HAL_FLASH_OB_Unlock();

    /* Get current OB */
    HAL_FLASHEx_OBGetConfig(&OBS_1A);

    /* Lock the Options Bytes */
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    if(OBS_1A.RDPLevel == OB_RDP_LEVEL_0)
    { ret_val = 0; }

    else if(OBS_1A.RDPLevel == OB_RDP_LEVEL_2)
    { ret_val = 2; }

    else if(OBS_1A.RDPLevel == OB_RDP_LEVEL_1)
    { ret_val = 1; }

    if (ret_val > 2) {
        flash_err("[FLASH] ERROR: flash_ob_rdp_level_get = 0x%02X \r\n", OBS_1A.RDPLevel );
    }

    return ret_val;
}

