#ifndef _ATOMIC_H_
#define _ATOMIC_H_
#include "api.h"

#include "os_cpu.h"

typedef struct _atomic
{
	volatile unsigned char val;
} atomic;

__inline unsigned int local_irq_save(void)
{
	#if OS_CRITICAL_METHOD == 3u                     /* Allocate storage for CPU status register           */
	OS_CPU_SR cpu_sr = 0u;
	#endif
	cpu_sr = OS_CPU_SR_Save();

	return (unsigned int)cpu_sr;
}

__inline void local_irq_restore(unsigned int cpu_sr)
{
	OS_CPU_SR_Restore(cpu_sr);
}

__inline void enter_critical()
{
	OSSchedLock();
}

__inline void exit_critical()
{
	OSSchedUnlock();
}

int atomic_test_set(atomic *at, int val);

void atomic_set(atomic *at, int val);

#endif
