#include "hlog_inc.h"
#include "htc_memory_define.h"


#define DUMP_LOG_CDC_SIZE  (64)
int dbg_cdc_write(char *log_buff, unsigned size)
{
	int count = 0;
	int remain_size = size;
	int pose = 0;
	int retry = 0;
	int error = 0;

	while (remain_size > 0) {
		count = remain_size < DUMP_LOG_CDC_SIZE ?
					remain_size : DUMP_LOG_CDC_SIZE;
		HAL_Delay(2);
		if (usb_cdc_transmit_data((uint8_t*)log_buff + pose,
								count) == 0){
			pose += count;
			remain_size -= count;
		} else {
			if (++retry > 5) {
				hlog_printf(HLOG_LVL_ERR, HLOG_TAG_LOG,
				"%s : dump log error! %d %d %d %d\n",
				__func__, count, pose, remain_size, size);
				error = -1;
				break;
			}
		}
	}

	HAL_Delay(2);

	return error;
}

int dbg_cdc_write_result(int result)
{
	uint8_t* str_done = "dump_mcu_log_done";
	uint8_t* str_failed = "dump_mcu_log_failed";

	if (result == 0) {
		if(usb_cdc_printf("%s", str_done) < 0)
			return -1;
	} else {
		if(usb_cdc_printf("%s", str_failed) < 0)
			return -1;
	}
	return 0;
}

int dbg_uart_write(char *buff, unsigned size)
{
        if (g_dbg_uart_en)
#if 0
                HAL_UART_Transmit(&dbg_uart, (uint8_t *)buff, size, HAL_MAX_DELAY);
#else
                HAL_DBG_UART_Transmit(&dbg_uart, (uint8_t *)buff, size, HAL_MAX_DELAY);
#endif
        return size;
}

int flash_read(uint32_t addr, char *buff, unsigned size)
{
        memcpy(buff, (void*)(addr), size);
        return size;
}

int flash_write(uint32_t addr, char *buff, unsigned size)
{
    uint8_t alignment_bytes = 0x0;
    HAL_StatusTypeDef FLASHStatus = HAL_OK;
    uint32_t end_addr = 0;
    uint32_t iterator = 0x0;
    uint64_t data = 0;
    uint8_t alignment=0;

    hlog_printf(HLOG_LVL_INFO,HLOG_TAG_LOG,"[FLASH_DRV] flash_write offset=0x%X, length=%u\r\n", addr, size);
    /* Check write range */

    if (!IS_FLASH_PROGRAM_ADDRESS(addr)){
        hlog_printf(HLOG_LVL_WARNING,HLOG_TAG_LOG,"[FLASH_DRV] invalid start_addr %u\r\n", addr);
        return 1;
    }
    end_addr = addr + size;
    if (!IS_FLASH_PROGRAM_ADDRESS(end_addr)){
        hlog_printf(HLOG_LVL_WARNING,HLOG_TAG_LOG,"[FLASH_DRV] invalid end_addr %u\r\n", end_addr);
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

    /* Unlock flash */
    HAL_FLASH_Unlock();

    /* Clear ERR flags */
    __HAL_FLASH_CLEAR_FLAG(   FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR|FLASH_SR_PGPERR );
    /* Write loop */
    iterator = 0x0;


    while((iterator+alignment_bytes) <= size)
    {
	    if(alignment == FLASH_TYPEPROGRAM_DOUBLEWORD){
	        data = * ((uint64_t *)(buff)+ (iterator/alignment_bytes));
            FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (addr + iterator), data);

        } else if( alignment == FLASH_TYPEPROGRAM_WORD){
            data = * ((uint32_t *)(buff)+ (iterator/alignment_bytes));
            FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (addr + iterator), data);

        } else if( alignment == FLASH_TYPEPROGRAM_HALFWORD){
            data = * ((uint16_t *)(buff)+ (iterator/alignment_bytes));
            FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (addr + iterator), data);

        }else if( alignment == FLASH_TYPEPROGRAM_BYTE){
            data = * ((uint8_t *)(buff)+ (iterator/alignment_bytes));
            FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (addr + iterator), data);

        }
        watchdog_refresh();
        if (FLASHStatus != HAL_OK)
        { break; }
        iterator += alignment_bytes;
    }
    // flash left data
    while( iterator<size ){
        data = * ((uint8_t *)(buff)+ (iterator));
        FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (addr + iterator), data);
        watchdog_refresh();

        if (FLASHStatus != HAL_OK)
        { break; }
        iterator += 1;
    }
    if (FLASHStatus != HAL_OK){
        /* Lock flash */
        HAL_FLASH_Lock();

        hlog_printf(HLOG_LVL_ERR,HLOG_TAG_LOG,"[FLASH_DRV] flash_write failed !! status=%d, iterator = %u\r\n", FLASHStatus, iterator);

        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();

    hlog_printf(HLOG_LVL_INFO,HLOG_TAG_LOG,"[FLASH_DRV] flash_write success !!\r\n");
    return 0;
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
int flash_erase(uint32_t addr, unsigned size)
{
    volatile HAL_StatusTypeDef FLASHStatus = HAL_OK;
    FLASH_EraseInitTypeDef EraseInfo;
    uint32_t EraseErrorPage;
    FLASH_PART curr_sec;

    curr_sec = get_erase_sector(addr,size);
    if( curr_sec != PART_LOG){
        hlog_printf(HLOG_LVL_ERR,HLOG_TAG_LOG,"[FLASH_DRV] invalid sector to erase start 0x%x\r\n",addr);
        return 1;
    }
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
	        return 1;
    }

    // now erase can only erase one sector a time.
    EraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
    /* Unlock flash */
    HAL_FLASH_Unlock();
    /* Clear ERR flags */
    __HAL_FLASH_CLEAR_FLAG(  FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR|FLASH_SR_PGPERR );
    // only has bank1

    EraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    FLASHStatus = HAL_FLASHEx_Erase(&EraseInfo, &EraseErrorPage);

	if (FLASHStatus != HAL_OK){
        /* Lock flash */
        HAL_FLASH_Lock();
        hlog_printf(HLOG_LVL_ERR,HLOG_TAG_LOG,"[FLASH_DRV] flash_erase failed !! status=%d\r\n", FLASHStatus );
        return 1;
    }

    /* Lock flash */
    HAL_FLASH_Lock();
    hlog_printf(HLOG_LVL_INFO,HLOG_TAG_LOG,"[FLASH_DRV] flash_erase success !!\r\n");

    return 0;
}



#if defined(HAL_RTC_MODULE_ENABLED)
int get_calendar_time(int *year, int *mon, int *day, int *hour, int *min, int *sec)
{
        RTC_DateTypeDef ymd;
        RTC_TimeTypeDef hms;

        if (NULL == MVR_RTC_HANDLE)
                goto out;

        memset(&ymd, 0, sizeof(ymd));
        memset(&hms, 0, sizeof(hms));

        HAL_RTC_GetTime(MVR_RTC_HANDLE, &hms, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(MVR_RTC_HANDLE, &ymd, RTC_FORMAT_BIN);

out:
        *year = ymd.Year;
        *mon = ymd.Month;
        *day = ymd.Date;
        *hour = hms.Hours;
        *min = hms.Minutes;
        *sec = hms.Seconds;

        return 0;
}
#else
int get_calendar_time(int *year, int *mon, int *day, int *hour, int *min, int *sec)
{
        *year = 0;
        *mon = 0;
        *day = 0;
        *hour = 0;
        *min = 0;
        *sec = 0;

        return 0;
}
#endif


int dump_flash_raw(uint32_t size, int type)
{
    int n;
    int m;
    uint32_t offs = 0;
    char buff[FLASH_ACCESS_STACK_SIZE];

	while (size > 0) {
		n = size < FLASH_ACCESS_STACK_SIZE ? size : FLASH_ACCESS_STACK_SIZE;
		memset(buff, 0xff, FLASH_ACCESS_STACK_SIZE);
		flash_read(CONFIG_LOG_SAVE_ADDR + offs, buff, n);

		m = n - 1;
		while (m >= 0) {
			if (0xff != buff[m])
				break;
			m --;
		}
		m++;
		
		if(type == MCULOG_TYPE_UART){
			dbg_uart_write(buff, m);
		}else if(type == MCULOG_TYPE_CDC){
			//hlog_printf(HLOG_LVL_INFO,HLOG_TAG_LOG,"__jh__ len=%d data=%s\n",m,buff);
			int ret =usb_cdc_transmit_data((uint8_t*)buff,m);
			HAL_Delay(2);
			if(0 != ret){
				return -1;
			}
		}
		
		if ((n -m) > 8)
		        break;

		offs += n;
		size -= n;
	}

	return 0;
}

int is_buffdata_same(uint8_t *buff, unsigned size, uint8_t val)
{
#define BUFFDATA_SAME_BITS 9
     int i;
     unsigned s;

     s = 1;
#if 1
        if (size > (1 << BUFFDATA_SAME_BITS))
                s = size >> BUFFDATA_SAME_BITS;
#endif

     for(i = 0; i < size; i += s)
		if(buff[i] != val)
			return 0;
     return 1;
}

int CIRC_CNT_TO_END(int head, int tail, unsigned size)
{
        int end = (size) - (tail);
        int n = ((head) + end) & ((size)-1);
        return n < end ? n : end;
}

int CIRC_SPACE_TO_END(int head, int tail, unsigned size)
{
        int end = (size) - 1 - (head);
        int n = (end + (tail)) & ((size)-1);
        return n <= end ? n : end+1;
}

