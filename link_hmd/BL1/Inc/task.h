#ifndef __TASK_H__
#define __TASK_H__

typedef void			TASK;
typedef unsigned char* 	bp;
typedef void 			(*voidFuncPtr)(void);
typedef void			(*voidProcPtr)(bp bpv);
#define TASK_NUM	5	// define max task num , max:254

#define __KERNEL_VERSION		"V1.02"
extern char TaskInitFlag;

#ifndef NULL
#define NULL 0
#endif

#define TASKCFUNC(A) unsigned char A##TaskID;\
    TASK A##Task(void)

#define TASKDFUNC(A) extern unsigned char A##TaskID;\
    void A##Task(void);

void sch_scheduler_init(void);
void sch_scheduler(void);
TASK undefTask(void);
char CreatTask(unsigned char *TaskID, voidFuncPtr TFunc);
char ChangeTask(unsigned char TaskID);
char DeleteTask(unsigned char *TaskID);
unsigned char GetCurTaskID(void);


#endif
