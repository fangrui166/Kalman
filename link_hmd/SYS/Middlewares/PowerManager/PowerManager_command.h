#ifndef __POWERMANAGER_COMMAND_H__
#define __POWERMANAGER_COMMAND_H__

#include "FreeRTOS.h"
#include "timers.h"
#include "main.h"
#include "command.h"

#ifndef PARAMETER_SIZE
#define PARAMETER_SIZE 100
#endif

/*
typedef struct _POWER_COMMAND POWER_COMMAND;
typedef int     (*PFUNC_CMD)(int, char **, _command_source);

struct _POWER_COMMAND {
    unsigned char   *command;
    PFUNC_CMD   pFunc;
};
*/
int PowerDoCommand(int argc, char **argv, _command_source source);
void PWRMGR_reboot_test(void);

#endif /*__POWERMANAGER_COMMAND_H__*/
