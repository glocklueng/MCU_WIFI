//#define DEBUG
#include "api.h"
#include "debug.h"
#include "task.h"

int thread_create(void(*task)(void *p_arg), void *p_arg, unsigned int prio, unsigned int *pbos, unsigned int stk_size, char *name)
{
	INT8U ret, need_free_stack = 0;
	if (pbos == 0)
	{
		need_free_stack = 1;
		pbos = (OS_STK*)mem_calloc(stk_size, sizeof(OS_STK));
		if (!pbos)
			return OS_ERR;
	}

	ret = OSTaskCreateExt(task, p_arg, &pbos[stk_size - 1], prio, 
	                      prio, pbos, stk_size, NULL, 
	                      (INT16U)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR), need_free_stack);

	if (ret == OS_ERR_NONE)
	{
		OSTaskNameSet(prio, (INT8U*)name, (INT8U*) &ret);
		return prio;
	}

	return  - 1;
}

int thread_exit(int thread_id)
{
	int ret;
	ret = OSTaskDel(thread_id);
	return ret;
}

//返回任务自身的ID号
int thread_myself()
{
	return OSTCBCur->OSTCBId;
}
