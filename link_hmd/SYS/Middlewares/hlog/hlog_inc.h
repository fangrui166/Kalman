#ifndef _HLOG_INC_H
#define _HLOG_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdarg.h"
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#if defined(HAL_RTC_MODULE_ENABLED)
#include "rtc_drv.h"
#endif

#include "hlog_api.h"
#include "hlog_utils.h"
#include "hlog_main.h"

#ifdef __cplusplus
}
#endif

#endif

