#include "api.h"
#include "debug.h"
#include "timer.h"


timer_t *timer_setup(int time_val, int type, timer_callback_func callback, void *callback_arg)
{
	INT8U perr;
	OS_TMR *tmr;

	if (time_val < 100)
		time_val = 100;

	time_val = time_val * OS_TMR_CFG_TICKS_PER_SEC / 1000;

	if ((callback == 0))
	{
		p_err("setup_timer err arg\n");
		return 0;
	}
	if (type)
	//== 1 repeat
	{
		p_dbg("repeat:%d\n", time_val);
		tmr = OSTmrCreate(time_val, time_val, OS_TMR_OPT_PERIODIC, (OS_TMR_CALLBACK)callback, callback_arg, "", &perr);
	}
	else
	{
		p_dbg("one short:%d\n", time_val);
		tmr = OSTmrCreate(time_val, time_val, OS_TMR_OPT_ONE_SHOT, (OS_TMR_CALLBACK)callback, callback_arg, "", &perr);
	}

	if (perr != OS_ERR_NONE)
	{
		tmr = 0;
		p_err("setup_timer err\n");
	}
	return (timer_t*)tmr;
}

int timer_pending(timer_t *tmr)
{
	OS_TMR *tmr_t = (OS_TMR*)tmr;

	if (tmr_t == 0)
	{
		p_err("timer_pending tmr err");
		return  - 1;
	}

	if (tmr_t->OSTmrState == OS_TMR_STATE_RUNNING)
	{
		return 1;
	}

	return 0;
}

//expires ----n*ms
int mod_timer(timer_t *tmr, unsigned int expires)
{
	INT8U perr, ret;
	OS_TMR *tmr_t = tmr;

	if (tmr_t == 0)
	{
		p_err("mod_timer tmr err");
		return  - 1;
	}

	if (expires < 100)
		expires = 100;

	expires = expires * OS_TMR_CFG_TICKS_PER_SEC / 1000;

	tmr_t->OSTmrDly = expires;
	tmr_t->OSTmrPeriod = expires;
	ret = OSTmrStart(tmr_t, &perr);
	if (ret == OS_TRUE && perr == OS_ERR_NONE)
	{
		return OS_ERR_NONE;
	}

	p_err("mod_timer err %d", perr);
	return  - 1;
}

int add_timer(timer_t *tmr)
{
	INT8U perr, ret;
	OS_TMR *tmr_t = tmr;

	if (tmr_t == 0)
	{
		p_err("add_timer tmr err");
		return  - 1;
	}
	ret = OSTmrStart(tmr_t, &perr);
	if (ret == OS_TRUE && perr == OS_ERR_NONE)
	{
		return OS_ERR_NONE;
	}

	p_err("add_timer err %d", perr);
	return  - 1;
}

//停止定时器
int del_timer(timer_t *tmr)
{
	INT8U perr, ret;
	OS_TMR *tmr_t = tmr;

	if (tmr_t == 0)
	{
		p_err("del_timer tmr err");
		return  - 1;
	}
	ret = OSTmrStop(tmr_t, OS_TMR_OPT_NONE, 0, &perr);
	if (ret == OS_TRUE)
	{
		return OS_ERR_NONE;
	}

	p_err("del_timer err %d", perr);
	return  - 1;
}

//释放删除定时器
int timer_free(timer_t *tmr)
{
	INT8U perr, ret;
	OS_TMR *tmr_t = tmr;

	ret = OSTmrDel(tmr_t, &perr);

	if ((ret == OS_TRUE) && (perr == OS_ERR_NONE))
		return OS_ERR_NONE;

	p_err("timer_free err %d", perr);
	return  - 1;
}

void sleep(int ms)
{
	if (ms < 1000uL / OS_TICKS_PER_SEC)
	//最小睡眠分辨率
	{
		ms = 1000uL / OS_TICKS_PER_SEC;
	}
	ms = ms * OS_TICKS_PER_SEC / 1000uL;
	OSTimeDly(ms);
}

unsigned int os_time_get(void)
{
	return (unsigned int)(OSTimeGet()*(1000uL / OS_TICKS_PER_SEC));
}
