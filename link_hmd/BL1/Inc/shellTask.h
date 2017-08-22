#ifndef __SHELLTASK_H__
#define __SHELLTASK_H__

#include "task.h"

#define CMD_BUF_LEN    100
#define CMD_CACHE_LEN  10
#define MAX_ARGS       88
#define MISC_PARAMETER_SIZE 20

typedef enum{
    CMD_SOURCE_UART,
    CMD_SOURCE_USBCDC,
}_command_source;


typedef struct _MONITOR_COMMAND MONITOR_COMMAND, *PMONITOR_COMMAND;
typedef int     (*PFUNC_COMMAND)(int, char *argv[], _command_source);
typedef void    (*PFUNC_HELP)(const unsigned char *, unsigned int);

struct _MONITOR_COMMAND {
    unsigned char   *command;
    PFUNC_COMMAND   pFunc;
};

void Shell_init(void);
TASKDFUNC(Shell)

void Shell_rec_buf(char data);
void ShellRecvProcess(void);

#endif
