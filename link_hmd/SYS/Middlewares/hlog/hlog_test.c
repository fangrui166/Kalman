#include "hlog_inc.h"

static TaskHandle_t hlog_test_task_handle;

void hlog_test_func(void * args)
{
     unsigned cnt = 0;
     
     while(1) {
          cnt ++;
          hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_LOG, "in %s, cnt = %u\n", __func__, cnt);
          if (cnt%300 == 0) {
             *(volatile unsigned *)0 = 0xdead;
          }     
     }

}

int hlog_test_init(void)
{
	xTaskCreate(hlog_test_func, "hlog_test", 128, 0, osPriorityHigh, &hlog_test_task_handle);
	return 0;
}

void hlog_test_exit(void)
{

}

