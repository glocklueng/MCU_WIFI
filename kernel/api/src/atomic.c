#include "api.h"
#include "atomic.h"


int atomic_test_set(atomic *at, int val)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();
	if (at->val)
	{
		OS_EXIT_CRITICAL();
		return 1;
	}
	at->val = val;
	OS_EXIT_CRITICAL();
	return 0;
}

void atomic_set(atomic *at, int val)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();
	at->val = val;
	OS_EXIT_CRITICAL();
}
