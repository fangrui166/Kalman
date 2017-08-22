#ifndef _HLOG_UTILS_H
#define _HLOG_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))
#if 0
#define CIRC_CNT_TO_END(head,tail,size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})
#define CIRC_SPACE_TO_END(head,tail,size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})
#else
int CIRC_CNT_TO_END(int head, int tail, unsigned size);
int CIRC_SPACE_TO_END(int head, int tail, unsigned size);
#endif
#define CIRC_ADD(idx, n, size)	(((idx) + (n)) & ((size) - 1))


int dbg_cdc_write(char *log_buff, unsigned size);
int dbg_cdc_write_result(int);

int dbg_uart_write(char *buff, unsigned size);

int flash_read(uint32_t addr, char *buff, unsigned size);
int flash_write(uint32_t addr, char *buff, unsigned size);
int flash_erase(uint32_t addr, unsigned size);

int air_send(unsigned air_hdl, char *buff, unsigned size);
int air_recv(unsigned air_hdl, char *buff, unsigned size);

int get_calendar_time(int *year, int *mon, int *day, int *hour, int *min, int *sec);


#define CONFIG_FLASHLOG_CACHE_SIZE     (1 << 8)

int dump_flash_raw(uint32_t size, int type);
int is_buffdata_same(uint8_t *buff, unsigned size, uint8_t val);

#if defined(HAL_RTC_MODULE_ENABLED)
extern RTC_HandleTypeDef hrtc;
#define MVR_RTC_HANDLE           (&hrtc)
#else
#define MVR_RTC_HANDLE           NULL
#endif

#define CONFIG_HLOG_HZ            ((unsigned)configTICK_RATE_HZ)
#define hlog_get_tick()                  ((unsigned)xTaskGetTickCount())

#define HLOG_FLASH_OFFS2ADDR(offs)      ((offs) + FLASH_BASE)
#define HLOG_FLASH_ADDR2OFFS(addr)     ((addr) - FLASH_BASE)

union flash_data_desc {
        uint64_t dw;
        uint32_t w;
        uint16_t h;
        uint8_t  b;
        uint8_t data[sizeof(uint64_t)];
};

#define readl(addr)        (*(volatile unsigned long *)(addr))

extern char CSTACK$$Base[];
extern char CSTACK$$Limit[];

#define __main_stack_start__ ((unsigned long)CSTACK$$Base)
#define __main_stack_end__   ((unsigned long)CSTACK$$Limit)

#ifdef __cplusplus
}
#endif

#endif

