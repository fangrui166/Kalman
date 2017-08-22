#include "task.h"
#include "initTask.h"

char TaskInitFlag;

volatile static voidFuncPtr TASKFunc[TASK_NUM+1];
volatile static unsigned char curTaskID;

void sch_scheduler_init(void)
{
	unsigned char chNum;

	// make sure task func array was initilized
	for (chNum = 0; chNum < TASK_NUM+1; chNum++)
		TASKFunc[chNum] = 0;

	TASKFunc[0] = undefTask;
	curTaskID = 0;

	// TODO  Add task witch created by user at here
	CreatTask(&InitTaskID, InitTask); // create initialize task

	// TODO	Add task's initialize

	// TODO	Start tasks
	ChangeTask(InitTaskID);
}

void sch_scheduler(void)
{
	for (; ;)
	{
		// schedule task
		if (curTaskID < TASK_NUM+1)
		{
			TASKFunc[curTaskID]();
		}
		else
			;	// invalid schedule

		// TODO	start IdelTask
		IdelTask();
	}
}

char CreatTask(unsigned char *TaskID, voidFuncPtr TFunc)
{
	unsigned char chNum;

	if (0 == TFunc)
		return 0;

	for (chNum = 1; chNum < TASK_NUM+1; chNum++)
	{
		if (0 == TASKFunc[chNum])
		{
			TASKFunc[chNum] = TFunc;
			*TaskID = chNum;
			return 1;
		}
	}

	return 0;
}

char DeleteTask(unsigned char *TaskID)
{
	if (*TaskID < TASK_NUM+1)
	{
		if (curTaskID == *TaskID)	// disable delete itself
			return 0;
		else
		{
			if (0 == TASKFunc[*TaskID]) // task not exist, no need delete
				return 0;
			else
			{
				TASKFunc[*TaskID] = 0;	// recovery resource
				*TaskID = 255;			// confirm not schedule any more
				return 1;
			}
		}
	}
	else
		return 0;
}

char ChangeTask(unsigned char TaskID)
{
	if (TaskID < TASK_NUM+1)
	{
		curTaskID = TaskID;
		TaskInitFlag = 0;
		return 1;
	}
	else
		return 0;
}

unsigned char GetCurTaskID(void)
{
	return (curTaskID);
}

TASK undefTask(void)
{

}

