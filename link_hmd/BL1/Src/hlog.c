#include "hlog.h"
#include "flash_drv.h"
#include "stm32f4xx_hal.h"

__no_init static volatile int cursor_offset@0x20002010;
__no_init static char log_buf[CONFIG_LOG_BUF_SIZE]@0x20002018;

int is_buffdata_same(uint8_t *buff, unsigned size, uint8_t val)
{
#define BUFFDATA_SAME_BITS 9
     int i;
     unsigned s;

     s = 1;
    if (size > (1 << BUFFDATA_SAME_BITS))
        s = size >> BUFFDATA_SAME_BITS;

     for(i = 0; i < size; i += s)
        if(buff[i] != val)
            return 0;
     return 1;
}
static int logbuf_wrap_around(void)
{
    int wrap_around = 1;
    int cursor = cursor_offset;
    if(cursor < CONFIG_LOG_BUF_SIZE){
        if (is_buffdata_same((uint8_t *)log_buf + cursor,
                CONFIG_LOG_BUF_SIZE - cursor, 0x00))
            wrap_around = 0;
        else if (is_buffdata_same((uint8_t *)log_buf + cursor,
                CONFIG_LOG_BUF_SIZE - cursor, 0xff))
            wrap_around = 0;
    }

    return wrap_around;
}

int hlog_erase_flashlog(void)
{
    VR_flash_erase(CONFIG_LOG_PART_ADDR,
        CONFIG_LOG_BUF_SIZE+CONFIG_LOG_HEAD_SIZE);
    return 0;
}
extern UART_HandleTypeDef dbg_uart;

int dbg_uart_write(char *buff, unsigned size)
{
    HAL_UART_Transmit(&dbg_uart, (uint8_t *)buff, size, HAL_MAX_DELAY);
    return size;
}

int hlog_save_logbuf(void)
{
    int wrap_around;
    flash_log_head_t log_head;
    int cursor = cursor_offset;

    wrap_around = logbuf_wrap_around();

    log_head.log_exit_flag = 1;
    log_head.log_pending_type = PENDING_BY_WDG;
    if(wrap_around){
        log_head.log_size = CONFIG_LOG_BUF_SIZE;
    }
    else{
        log_head.log_size = cursor;
    }
    hlog_erase_flashlog();

    VR_flash_write((char*)&log_head, CONFIG_LOG_HEAD_ADDR,
        sizeof(flash_log_head_t));

    if (wrap_around) {
        VR_flash_write(log_buf + cursor,CONFIG_LOG_SAVE_ADDR,
            CONFIG_LOG_BUF_SIZE - cursor);
        VR_flash_write(log_buf, (CONFIG_LOG_SAVE_ADDR+(
            CONFIG_LOG_BUF_SIZE - cursor)), cursor);
    } else {
        VR_flash_write(log_buf, CONFIG_LOG_SAVE_ADDR, cursor);
    }

    return 0;
}

int hlog_dump_logbuf(void)
{
    int wrap_around;
    int cursor = cursor_offset;

    wrap_around = logbuf_wrap_around();
    if (wrap_around)
        dbg_uart_write(log_buf + cursor, CONFIG_LOG_BUF_SIZE - cursor);
    dbg_uart_write(log_buf, cursor);

    return 0;
}

