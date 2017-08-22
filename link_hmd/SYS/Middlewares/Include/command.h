#ifndef __COMMAND_H__
#define __COMMAND_H__
#include "cmsis_os.h"

#define SHELL_HLOG_ENABLE 1

#if SHELL_HLOG_ENABLE
#define shell_emerg(fmt, ...) \
    hlog_printf(HLOG_LVL_EMERG, HLOG_TAG_SHELL, fmt, ##__VA_ARGS__)
#define shell_err(fmt, ...) \
    hlog_printf(HLOG_LVL_ERR, HLOG_TAG_SHELL, fmt, ##__VA_ARGS__)
#define shell_warning(fmt, ...) \
    hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_SHELL, fmt, ##__VA_ARGS__)
#define shell_info(fmt, ...) \
    hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SHELL, fmt, ##__VA_ARGS__)
#define shell_debug(fmt, ...) \
    hlog_printf(HLOG_LVL_DEBUG, HLOG_TAG_SHELL, fmt, ##__VA_ARGS__)
#else /* SHELL_HLOG_ENABLE */
#define shell_emerg(fmt, ...) \
    printf("[SHELL][EMR] :" fmt, ##__VA_ARGS__)
#define shell_err(fmt, ...) \
    printf("[SHELL][ERR] :" fmt, ##__VA_ARGS__)
#define shell_warning(fmt, ...) \
    printf("[SHELL][WARN]:" fmt, ##__VA_ARGS__)
#define shell_info(fmt, ...) \
    printf("[SHELL][INFO]:" fmt, ##__VA_ARGS__)
#define shell_debug(fmt, ...) \
    printf("[SHELL][DBG] :" fmt, ##__VA_ARGS__)
#endif /* SHELL_HLOG_ENABLE */

typedef enum{
    CMD_SOURCE_UART,
    CMD_SOURCE_USBCDC,
}_command_source;
#define MISC_PARAMETER_SIZE 20


typedef struct _MONITOR_COMMAND MONITOR_COMMAND, *PMONITOR_COMMAND;
typedef int     (*PFUNC_COMMAND)(int, char *argv[], _command_source);
typedef void    (*PFUNC_HELP)(const unsigned char *, unsigned int);

struct _MONITOR_COMMAND {
    unsigned char   *command;
    PFUNC_COMMAND   pFunc;
};


void cmd_init(void);
osMessageQId get_queue_commd(void);
int32_t USBShell_ProcessRcvData(uint8_t *, uint8_t);
int ParseCommandAndData(unsigned char *, int *, unsigned char **, int);
int DoCommand(int argn,unsigned char **, _command_source);
int cmd_mfg_bmi160_k_acc(int argc, char * argv[], _command_source source);
int cmd_mfg_bmi160_k_gyro(int argc, char * argv[], _command_source source);
int cmd_mfg_bmi160_test_gyro(int argc, char * argv[], _command_source source);
#endif
