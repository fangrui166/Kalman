
#ifndef __LSM6DSMCOMMAND_H
#define __LSM6DSMCOMMAND_H
#include "command.h"

#define COMMAND_GSENSOR_DUMPREG "dumpreg"
#define COMMAND_GSENSOR_READCHIPID "readchipid"
#define COMMAND_GSESNOR_READACC "readacc"
#define COMMAND_GSENSOR_READGYRO "readgyro"
#define COMMAND_GSENSOR_DUMPDATA "dumpdata"
#define COMMAND_GSENSOR_DISABLEDATA "disabledata"


int shell_lsm6dsm_commad(int argc, char * argv[], _command_source source);

#endif /* __LSM6DSMCOMMAND_H */
