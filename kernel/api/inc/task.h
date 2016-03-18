#ifndef _TASK_H_
#define _TASK_H_
#include "api.h"

int thread_create(void(*task)(void *p_arg), void *p_arg, unsigned int prio, unsigned int *pbos, unsigned int stk_size, char *name);

int thread_exit(int thread_id);
int thread_myself(void);
extern volatile INT32U tick_ms;

#endif
