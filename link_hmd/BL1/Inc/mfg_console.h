#ifndef __MFG_COMMANDS_H
#define __MFG_COMMANDS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"

enum {
        MFG_TAG_FLASH,
        MFG_TAG_SENSOR,
        MFG_TAG_PMIC,
        MFG_TAG_RTC,
        MFG_TAG_LED,
        MFG_TAG_BT,
        MFG_TAG_USB,
        MFG_TAG_BUTTON,
        MFG_TAG_TRACKPAD,
        MFG_TAG_APP,
        MFG_TAG_NTF, //notification
        MFG_TAG_ALARM,
        MFG_TAG_PWRMGR, //power manager
        MFG_TAG_SYNCSRV, //sync service
        MFG_TAG_MISC,
        MFG_TAG_COMMON,
        MFG_TAG_CCG4,
        MFG_TAG_MUX,
        MFG_TAG_AUDIO,
        MFG_TAG_MAX //ensure MFG_TAG_MAX <= 32
};

typedef enum {
	MFG_PASS,
	MFG_ERROR,
	MFG_WARNING,
}MFG_RESULT;


#define MFG_RELEASE 0
#if MFG_RELEASE
#define MFG_LOG_ERR(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Error:]"fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_WARNING(MFG_TAG_LVL,fmt,...)		        printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Warning:]"fmt,##__VA_ARGS__); printf("\r\n")
#define MFG_LOG_INFO(MFG_TAG_LVL,fmt,...)	 		//printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Info:]",fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_DEBUG(MFG_TAG_LVL,fmt,...)	 		//printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Debug:]",fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_PASS(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Pass:]"fmt,##__VA_ARGS__);printf("\r\n")
#else
#define MFG_LOG_ERR(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Error:]"fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_WARNING(MFG_TAG_LVL,fmt,...)	 	        printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Warning:]"fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_INFO(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Info:]"fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_DEBUG(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Debug:]"fmt,##__VA_ARGS__);printf("\r\n")
#define MFG_LOG_PASS(MFG_TAG_LVL,fmt,...)	 		printf(g_mfg_tag_names[MFG_TAG_LVL]);printf("[Pass:]"fmt,##__VA_ARGS__);printf("\r\n")
#endif
#ifdef __cplusplus
}
#endif

#endif /* __MFG_COMMANDS_H */
