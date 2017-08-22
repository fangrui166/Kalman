#ifndef _HLOG_API_H
#define _HLOG_API_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
     HLOG_LVL_EMERG,
     HLOG_LVL_ERR,
     HLOG_LVL_WARNING,
     HLOG_LVL_INFO,
     HLOG_LVL_DEBUG,
     HLOG_LVL_MAX
};
#define HLOG_LVL_LVL0 HLOG_LVL_EMERG

enum {
        HLOG_TAG_FLASH,
        HLOG_TAG_SENSOR,
        HLOG_TAG_PMIC,
        HLOG_TAG_AUDIO,
        HLOG_TAG_RTC,
        HLOG_TAG_LED,
        HLOG_TAG_BT,
        HLOG_TAG_USB,
        HLOG_TAG_BUTTON,
        HLOG_TAG_TRACKPAD,
        HLOG_TAG_CCGX,
        HLOG_TAG_DISP,
        HLOG_TAG_FG,
        HLOG_TAG_CHG,

        HLOG_TAG_LOG,
        HLOG_TAG_FOTA,
        HLOG_TAG_I2C,
        HLOG_TAG_PWRMGR, //power manager
        HLOG_TAG_IOEXP,
        HLOG_TAG_SHELL,
        HLOG_TAG_MISC,//misc data
        HLOG_TAG_FUSION,

        HLOG_TAG_OS,
        HLOG_TAG_APP,
        HLOG_TAG_NTF, //notification
        HLOG_TAG_ALARM,
        HLOG_TAG_SYNCSRV, //sync service
        HLOG_TAG_COMMON,
        HLOG_TAG_ADC,


        HLOG_TAG_MAX //ensure HLOG_TAG_MAX <= 32
};

#define FLASHLOG_TYPE_NORMAL       0x01
#define FLASHLOG_TYPE_EXCEP          0x02
#define FLASHLOG_TYPE_MASK           0x03

char *hlog_level_number2name(int level_number);
int hlog_level_name2number(char *name);

char *hlog_tag_number2name(int tag_number);
int hlog_tag_name2number(char *name);

int hlog_init(void);
void hlog_exit(void);

int hlog_set_level(int level);
int hlog_get_level(void);

int hlog_set_tag(int tag);
int hlog_clear_tag(int tag);
void hlog_reset_tags(void);
unsigned hlog_get_tags(void);
void hlog_set_tags(unsigned tags);
int __write_logbuf_force(char *buff, int size);

/**
 * hlog_printf - Format and print a string
 * @level: Log level
 * @tag: Log tag
 * @fmt: The format string to use
 * @...: Arguments for the format string
 *
 * Returns the number of characters printed upon success, < 0 otherwise
 */
int hlog_printf(int level, int tag, char *fmt, ...);


int hlog_cdc_dump_logbuf(void);
int hlog_cdc_clear_logbuf(void);
void hlog_clear_logbuf(void);
int hlog_flush_logbuf(int count);
int hlog_dump_logbuf(void);
int hlog_save_logbuf(void);
int hlog_send_logbuf(void *hd);


int hlog_erase_flashlog(void);
int hlog_dump_flashlog(void);
int hlog_send_flashlog(void *hd, int log_type);

int hlog_exp_handler(void *args);

inline void hlog_shell_cmd_init(void);

int hlog_test_init(void);

#ifdef __cplusplus
}
#endif

#endif

