#include "hlog_inc.h"

unsigned g_hlog_ready = 0;
unsigned g_dbg_uart_en = 1;

static unsigned hlog_ts_cnt = 0;
#if ((LOG_BUF_RUN_TIME_TEST))
static uint8_t log_buf_full_flag = 0;
#endif
static unsigned cur_log_level = CONFIG_LOG_LEVEL_DEFAULT;
static unsigned cur_log_tags = CONFIG_LOG_TAGS_DEFAULT;
static char *level_names[HLOG_LVL_MAX] = {"emerg", "err", "warn", "info", "debug"};
static char *tag_names[HLOG_TAG_MAX] = {
        [HLOG_TAG_FLASH] = "FLASH",
        [HLOG_TAG_SENSOR] = "SENSOR",
        [HLOG_TAG_PMIC] = "PMIC",
        [HLOG_TAG_AUDIO] = "AUDIO",
        [HLOG_TAG_RTC] = "RTC",
        [HLOG_TAG_LED] = "LED",
        [HLOG_TAG_BT] = "BT",
        [HLOG_TAG_USB] = "USB",
        [HLOG_TAG_BUTTON] = "BTN",
        [HLOG_TAG_TRACKPAD] = "TRKP",
        [HLOG_TAG_CCGX] = "CCGX",
        [HLOG_TAG_DISP] = "DISP",
        [HLOG_TAG_FG] = "FG",
        [HLOG_TAG_CHG] = "CHG",
        [HLOG_TAG_I2C] = "I2C",
        [HLOG_TAG_LOG] = "LOG",
        [HLOG_TAG_FOTA] = "FOTA",
        [HLOG_TAG_IOEXP] = "IOEXP",
        [HLOG_TAG_FUSION] = "FUSION",
        [HLOG_TAG_OS] = "OS",
        [HLOG_TAG_APP] = "APP",
        [HLOG_TAG_NTF] = "NTF",
        [HLOG_TAG_ALARM] = "ALARM",
        [HLOG_TAG_PWRMGR] = "PWRMGR",
        [HLOG_TAG_SYNCSRV] = "SYNCSRV",
        [HLOG_TAG_MISC] = "MISC",
        [HLOG_TAG_COMMON] = "COMMON",
        [HLOG_TAG_SHELL] = "SHELL",
        [HLOG_TAG_ADC] = "ADC",
};

__no_init static volatile int cursor_offset@0x20002010;
__no_init static char log_buf[CONFIG_LOG_BUF_SIZE]@0x20002018;


enum mculog_data_flag {
	LOGFLAG_RAM_LOG = 0,
	LOGFLAG_FLASH_LOG,
};


struct __mculog_data {
	osThreadId mculog_thread;
	uint8_t log_flag;
};
static struct __mculog_data g_mculog_data = { 0 };
void dump_mcu_log_func(void const *);

static int mculog_create_thread(
				struct __mculog_data *mculog_data)
{

	osThreadDef(dump_mculog_work_task, dump_mcu_log_func,
			osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
	mculog_data->mculog_thread =
		osThreadCreate(osThread(dump_mculog_work_task), mculog_data);
	if (mculog_data->mculog_thread == NULL){
		return -1;
	}
	return 0;
}

static int logbuf_wrap_around(void)
{
    int wrap_around = 1;
    int cursor = cursor_offset;
    if(cursor < CONFIG_LOG_BUF_SIZE){
        if (is_buffdata_same((uint8_t *)log_buf + cursor, CONFIG_LOG_BUF_SIZE - cursor, 0x00))
            wrap_around = 0;
        else if (is_buffdata_same((uint8_t *)log_buf + cursor, CONFIG_LOG_BUF_SIZE - cursor, 0xff))
            wrap_around = 0;
    }
    return wrap_around;
}
int __write_logbuf_force(char *buff, int size)
{
    int tmp_size = 0;
    int cursor = cursor_offset;
    if(size > (CONFIG_LOG_BUF_SIZE-cursor)){
        tmp_size = (CONFIG_LOG_BUF_SIZE-cursor);
        memcpy(log_buf+cursor, buff, tmp_size);
        memcpy(log_buf, buff+tmp_size, size-tmp_size);
        cursor = size-tmp_size;
        #if ((LOG_BUF_RUN_TIME_TEST))
        log_buf_full_flag = 1;
        #endif
    }
    else{
        memcpy(log_buf+cursor,buff, size);
        cursor += size;
    }
    cursor_offset = cursor;
    return cursor;
}

char *hlog_level_number2name(int level_number)
{
	if (level_number < HLOG_LVL_EMERG || level_number >= HLOG_LVL_MAX)
		return NULL ;
	return level_names[level_number];
}

int hlog_level_name2number(char *name)
{
	int level_number;
	for (level_number = HLOG_LVL_EMERG; level_number < HLOG_LVL_MAX; level_number++)
		if (!strcmp(name, level_names[level_number]))
			return level_number;
	return -1;
}

char *hlog_tag_number2name(int tag_number)
{
	if (tag_number < 0 || tag_number >= HLOG_TAG_MAX)
		return "NA" ;
	return tag_names[tag_number];
}

int hlog_tag_name2number(char *name)
{
	int tag_number;
	for (tag_number = 0; tag_number < HLOG_TAG_MAX; tag_number++)
		if (!strcmp(name, tag_names[tag_number]))
			return tag_number;
	return -1;
}

int hlog_init(void)
{
        if (g_hlog_ready)
                return 0;
        cursor_offset = 0;
        memset(log_buf, 0, CONFIG_LOG_BUF_SIZE);
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_LOG, "log_part_offs = 0x%x, log_part_size = 0x%x\r\n", CONFIG_LOG_PART_ADDR, CONFIG_LOG_BUF_SIZE);

        cur_log_level = CONFIG_LOG_LEVEL_DEFAULT;
        cur_log_tags = CONFIG_LOG_TAGS_DEFAULT;


        g_dbg_uart_en = 1;
        g_hlog_ready=1;

        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_LOG, "__stack_start__ = 0x%08x, __stack_end__ = 0x%08x\n", __main_stack_start__, __main_stack_end__);
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_LOG, "%s Done @ %s %s\n", __func__, __DATE__, __TIME__);

        mculog_create_thread(&g_mculog_data);
        return 0;
}

void hlog_exit(void)
{
     g_hlog_ready = 0;
     return ;
}

int hlog_printf(int level, int tag, char *fmt, ...)
{
     int len = 0;
     int ts_len = 0;
     unsigned tick;
     int year, mon, day, hour, min, sec;
     va_list     vp;
     char logmsg_buf[CONFIG_LOGMSG_BUF_SIZE];

     if (level < HLOG_LVL_LVL0 || level > HLOG_LVL_MAX)
          return -1;
     if ((level != HLOG_LVL_MAX) && (level > cur_log_level))
          return 0;

     if (tag < 0 || tag > HLOG_TAG_MAX)
          return -1;
     if ((tag != HLOG_TAG_MAX) && !(cur_log_tags & (1 << tag)))
          return 0;

     tick = hlog_get_tick();
     if (((hlog_ts_cnt & (CONFIG_HLOG_TS_INTERVAL - 1)) == 0) && g_hlog_ready) {
          get_calendar_time(&year, &mon, &day, &hour, &min, &sec);
          len += snprintf(logmsg_buf + len, sizeof(logmsg_buf)-len, LOG_META_FMT,
                        tick/CONFIG_HLOG_HZ, tick % CONFIG_HLOG_HZ,
                        hlog_level_number2name(HLOG_LVL_INFO),
                        hlog_tag_number2name(HLOG_TAG_LOG));
          len += snprintf(logmsg_buf + len, sizeof(logmsg_buf)-len,
                        "RTC %04d%02d%02d %02d:%02d:%02d TICK %d\r\n",
                        year, mon, day, hour, min, sec, tick);
          ts_len = len;
     }
     hlog_ts_cnt ++;

     len += snprintf(logmsg_buf + len, sizeof(logmsg_buf)-len, LOG_META_FMT,
                    tick/CONFIG_HLOG_HZ, tick%CONFIG_HLOG_HZ,
                    hlog_level_number2name(level), hlog_tag_number2name(tag));
     #if ((LOG_BUF_RUN_TIME_TEST))
     if(log_buf_full_flag){
        static uint8_t count = 0;
        len += snprintf(logmsg_buf + len, sizeof(logmsg_buf)-len,
                        "\r\nlogbuf was Full\r\n");
        if(++count >= 1){
            vTaskSuspendAll();
        }
        else{
            log_buf_full_flag = 0;
        }
     }
     #endif
     va_start(vp,fmt);
     len += vsnprintf(logmsg_buf + len, sizeof(logmsg_buf)-len, fmt, vp);
     va_end(vp);

     if (len < CONFIG_LOGMSG_BUF_SIZE) {
          len++;
     } else {
          len = CONFIG_LOGMSG_BUF_SIZE;
          logmsg_buf[len -1] = '\0';
     }


    __write_logbuf_force(logmsg_buf, len-1);

    if (g_hlog_ready)
        dbg_uart_write(logmsg_buf, len-1);

     return len -ts_len;
}

int hlog_set_level(int level)
{
	if (level < HLOG_LVL_EMERG || level >= HLOG_LVL_MAX)
		return -1;
	cur_log_level = level;
	return 0;
}

int hlog_get_level(void)
{
	return cur_log_level;
}

int hlog_set_tag(int tag)
{
	if (tag < 0 || tag >= HLOG_TAG_MAX)
		return -1;
	cur_log_tags |= (1 << tag);
	return 0;
}

int hlog_clear_tag(int tag)
{
	if (tag < 0 || tag >= HLOG_TAG_MAX)
		return -1;
	cur_log_tags &= ~(1 << tag);
	return 0;
}

void hlog_reset_tags(void)
{
	cur_log_tags = CONFIG_LOG_TAGS_DEFAULT;
}

unsigned hlog_get_tags(void)
{
	return cur_log_tags;
}
void hlog_set_tags(unsigned tags)
{
	cur_log_tags = tags;
}

int hlog_cdc_dump_flash_log(void)
{
	struct __mculog_data *mculog_data = &g_mculog_data;
	mculog_data->log_flag=LOGFLAG_FLASH_LOG;
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, "%s\n",__func__);
	osSignalSet(mculog_data->mculog_thread, 1);
	return 0;
}
int hlog_cdc_clear_flash_log(void)
{
	hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, "%s\n",__func__);
	hlog_erase_flashlog();
	return 0;
}

void dump_mcu_log_func(void const * argument)
{
	int wrap_around;
	int cdc_ret = 0;
	struct __mculog_data *data =(struct __mculog_data *)argument;
	do {
		osSignalWait(0xFFFFFFFF, osWaitForever);
		HAL_Delay(10);

		if(data->log_flag == LOGFLAG_RAM_LOG){
			wrap_around = logbuf_wrap_around();
			const int cur_offset =cursor_offset;
			if (wrap_around) {
				cdc_ret = dbg_cdc_write(log_buf + cur_offset,CONFIG_LOG_BUF_SIZE - cur_offset);
				if (cdc_ret != 0) {
					dbg_cdc_write_result(cdc_ret);
					continue;
				}
			}
			cdc_ret = dbg_cdc_write(log_buf, cur_offset);
			dbg_cdc_write_result(cdc_ret);
		}else if( data->log_flag == LOGFLAG_FLASH_LOG ){
			cdc_ret=dump_flash_raw(CONFIG_LOG_BUF_SIZE,MCULOG_TYPE_CDC);
			hlog_printf(HLOG_LVL_INFO, HLOG_TAG_USB, "%s ret=%d\n",__func__,cdc_ret);
			dbg_cdc_write_result(cdc_ret);
		}
	} while (1);
}

int hlog_cdc_dump_logbuf(void)
{
	struct __mculog_data *mculog_data = &g_mculog_data;
	mculog_data->log_flag=LOGFLAG_RAM_LOG;
	osSignalSet(mculog_data->mculog_thread, 1);
	return 0;
}

int hlog_cdc_clear_logbuf(void)
{
	uint8_t *str_clear = "clear_mcu_log_done";
	hlog_clear_logbuf();
	if( usb_cdc_transmit_data(str_clear,
					strlen((char const*)str_clear)) == 0)
		return 0;

	return -1;
}


void hlog_clear_logbuf(void)
{
    cursor_offset = 0;
    memset(log_buf, 0, CONFIG_LOG_BUF_SIZE);

    return ;
}

int hlog_flush_logbuf(int count)
{
        return 0;
}
int hlog_dump_logbuf(void)
{
        int wrap_around = 0;
        int cursor = cursor_offset;

        wrap_around = logbuf_wrap_around();
        if (wrap_around)
            dbg_uart_write(log_buf + cursor, CONFIG_LOG_BUF_SIZE - cursor);
        dbg_uart_write(log_buf, cursor);

        return 0;
}

int hlog_save_logbuf(void)
{
        int wrap_around;
        flash_log_head_t log_head;
        int cursor = cursor_offset;

        wrap_around = logbuf_wrap_around();

        log_head.log_exit_flag = 1;
        log_head.log_pending_type = PENDING_BY_EXP;
        if(wrap_around){
            log_head.log_size = CONFIG_LOG_BUF_SIZE;
        }
        else{
            log_head.log_size = cursor;
        }
        hlog_erase_flashlog();
        watchdog_refresh();
        flash_write(CONFIG_LOG_HEAD_ADDR, (char*)&log_head, sizeof(flash_log_head_t));

        if (wrap_around) {
                flash_write(CONFIG_LOG_SAVE_ADDR, log_buf + cursor, CONFIG_LOG_BUF_SIZE - cursor);
                flash_write((CONFIG_LOG_SAVE_ADDR+(CONFIG_LOG_BUF_SIZE - cursor)), log_buf, cursor);
        } else {
                flash_write(CONFIG_LOG_SAVE_ADDR, log_buf, cursor);
        }

        return 0;
}

int hlog_send_logbuf(void *hd)
{
        return 0;
}

int hlog_erase_flashlog(void)
{
    flash_erase(CONFIG_LOG_PART_ADDR, CONFIG_LOG_BUF_SIZE+CONFIG_LOG_HEAD_SIZE);
    return 0;
}

int hlog_dump_flashlog(void)
{
    dump_flash_raw(CONFIG_LOG_BUF_SIZE,MCULOG_TYPE_UART);

    return 0;
}
int hlog_get_flashlog_head(flash_log_head_t * log_head)
{
    return flash_read(CONFIG_LOG_HEAD_ADDR, (char*)log_head, sizeof(flash_log_head_t));
}
int hlog_send_flashlog(void *hd, int log_type)
{
        if (log_type & ~FLASHLOG_TYPE_MASK)
                return -1;

        return 0;
}

#if 0
unsigned hlog_get_nlog_part_offs(void)
{
	struct partition_t* ptr;
	ptr = get_partition("logdump");

	if ((ptr->start_block != 0x00) && (ptr->start_block != -1))
		return (unsigned)ptr->start_block;
	else if (BRD_XA == g_product_id)
		return 0x442000;
	else
		return 0x800000;
}

unsigned hlog_get_nlog_part_size(void)
{
	struct partition_t* ptr;
	ptr = get_partition("logdump");

	if ((ptr->size != 0x00) && (ptr->size != -1))
		return (unsigned)ptr->size;
	else
		return 0x80000;
}

unsigned hlog_get_elog_part_offs(void)
{
	struct partition_t* ptr;
	ptr = get_partition("crash_logdump");

	if ((ptr->start_block != 0x00) && (ptr->start_block != -1))
		return (unsigned)ptr->start_block;
	else if (BRD_XA == g_product_id)
		return 0x4c2000;
	else
		return 0xE01000;
}

unsigned hlog_get_elog_part_size(void)
{
	struct partition_t* ptr;
	ptr = get_partition("crash_logdump");

	if ((ptr->size != 0x00) && (ptr->size != -1))
		return (unsigned)ptr->size;
	else
		return 0x20000;
}
#endif

