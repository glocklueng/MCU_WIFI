#define DEBUG
#include "debug.h"
#include "drivers.h"
#include "stdio.h"
#include "api.h"
#include "lwip/stats.h"
#include "lwip/sockets.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/arch/sys_arch.h"
//#include <absacc.h>

void dump_hex(const char *tag, unsigned char *buff, int size)
{
	int i;
	printf("%s[", tag);
	for (i = 0; i < size; i++)
	{
		printf("%02x ", buff[i]);
	}
	printf("]\r\n");
}

#if SYS_MONITOR

#pragma section = "monitor_item_table"

char *p_globle_buff = 0;
#define GLOBLE_TEST_BUFF_SIZE 1024

typedef enum
{
	STAT_TYPE_SYS = 1, STAT_TYPE_TASK, STAT_TYPE_WIFI, STAT_TYPE_SOCKET, STAT_TYPE_MAX_VALUE
} STAT_TYPE;



void send_test_pkg(int cmd, const void *data, int size)
{
	static uint8_t g_seq = 0;
	uint8_t *pkg;
	uint8_t *p_data = (uint8_t*)data;
	int i;
	uint8_t sum = 0;
	if ((size > 1024) || (size && (p_data == 0)))
	{
		return ;
	}

	pkg = (uint8_t*)mem_malloc(size + 7);
	if (pkg == 0)
	{
		p_err("send_pkg mem_malloc err, size:%d\n", size + 7);
		return ;
	}
	pkg[0] = 0xaa;
	pkg[1] = (uint8_t)cmd;
	pkg[2] = (uint8_t)((size + 7) % 256);
	pkg[3] = (uint8_t)((size + 7) / 256);
	pkg[4] = (uint8_t)g_seq++;
	sum ^= pkg[1];
	sum ^= pkg[2];
	sum ^= pkg[3];
	sum ^= pkg[4];
	for (i = 0; i < size; i++)
	{
		pkg[i + 5] = p_data[i];
		sum ^= p_data[i];
	}
	pkg[i + 5] = sum;
	pkg[i + 6] = 0x55;

//	uart1_send(pkg, i + 7);

	mem_free(pkg);
}

DECLARE_MONITOR_ITEM("monitor item example", monitor_send_cnt);

char *err = "test_buff err";

#define CHECK_AND_ADD_BUFF	{	\
tmp[127] = 0;\
cnt += strlen(tmp);\
if(cnt > (GLOBLE_TEST_BUFF_SIZE - 64)){\
p_err(err);\
goto end;\
}\
strcat(p_globle_buff, tmp);\
}

#define CHECK_AND_ADD_LINE	{	\
sprintf(tmp, "\r\n");	\
tmp[127] = 0;\
cnt += strlen(tmp);\
if(cnt > (GLOBLE_TEST_BUFF_SIZE - 64)){\
p_err(err);\
goto end;\
}\
strcat(p_globle_buff, tmp);\
}


extern const uint32_t monitor_item_table$$Base;
extern const uint32_t monitor_item_table$$Limit;

extern char STACK$$Base[];
#define STACK_SIZE 0X200

uint32_t get_msp_size()
{
	return (uint32_t)STACK_SIZE;
}

uint32_t get_msp_free()
{
	int i;
	uint32_t *stack_base = (uint32_t*)STACK$$Base;
	uint32_t stack_size = STACK_SIZE;
	stack_size = stack_size / 4;
	for (i = 0; i < stack_size; i++)
	{
		if (stack_base[i])
			break;
	}

	return i *4;
}

void send_sys_stat_data()
{
	char *tmp;
	int cnt = 0, i;
	uint32_t start, end;

	tmp = (char*)mem_malloc(128);
	if (!tmp)
		return ;

	INC_MONITOR_ITEM_VALUE(monitor_send_cnt);

	send_test_pkg(STAT_TYPE_SYS, 0, 0);

	memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);

	snprintf(tmp, 127, "CPUʹ����:%u\r\n", OSCPUUsage);
	CHECK_AND_ADD_BUFF;

	snprintf(tmp, 127, "�����л���:%u\r\n", OSCtxSwCtr);
	CHECK_AND_ADD_BUFF;


	snprintf(tmp, 127, "������:%d/%d\r\n", OSTaskCtr, OS_MAX_TASKS);
	CHECK_AND_ADD_BUFF;

	for (i = 0; i < OS_MAX_EVENTS; i++)
	{
		if (OSEventFreeList == (OSEventTbl + i))
			break;
	}
	snprintf(tmp, 127, "event used:%d/%d\r\n", i, OS_MAX_EVENTS);
	CHECK_AND_ADD_BUFF;

	snprintf(tmp, 127, "timer used:%d/%d\r\n", OSTmrUsed, OS_TMR_CFG_MAX);
	CHECK_AND_ADD_BUFF;

	snprintf(tmp, 127, "main stack free:%d/%d\r\n", get_msp_free(), get_msp_size());
	CHECK_AND_ADD_BUFF;

	for (i = 0; i < OS_MAX_QS; i++)
	{
		if (OSQFreeList == (OSQTbl + i))
			break;
	}
	snprintf(tmp, 127, "qs used:%d/%d\r\n", i, OS_MAX_QS);
	CHECK_AND_ADD_BUFF;

	snprintf(tmp, 127, "����ʱ��:%d s\r\n", os_time_get() / 1000);
	CHECK_AND_ADD_BUFF;

	CHECK_AND_ADD_LINE;

	p_globle_buff[cnt] = 0;
	send_test_pkg(STAT_TYPE_SYS, p_globle_buff, cnt + 1);

	for (i = 0; i < OS_MAX_QS; i++)
	{
		if (OSQFreeList == (OSQTbl + i))
			break;
	}

	start = (uint32_t) &monitor_item_table$$Base;
	end = (uint32_t) &monitor_item_table$$Limit;

	cnt = 0;
	memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);
	for (; start < end; start += sizeof(struct dev_monitor_item))
	{
		struct dev_monitor_item *item = (struct dev_monitor_item*)start;

		//logd("%s:%d\r\n", item->name, item->value);
		snprintf(tmp, 127, "%s:%d\r\n", item->name, item->value);
		CHECK_AND_ADD_BUFF;

		p_globle_buff[cnt] = 0;

		if (cnt > 512)
		{
			send_test_pkg(STAT_TYPE_SYS, p_globle_buff, cnt + 1);
			memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);
			cnt = 0;
		}
	}

	if (cnt)
	{
		send_test_pkg(STAT_TYPE_SYS, p_globle_buff, cnt + 1);
	}

	end: mem_free(tmp);
}


void send_task_stat_data()
{
	OS_TCB *ptcb = 0,  *tmp_tcp = 0;
	INT8U prio;
	uint32_t irq_flag;
	char *tmp;
	int cnt = 0;

	tmp = (char*)mem_malloc(128);
	if (!tmp)
		goto end;

	send_test_pkg(STAT_TYPE_TASK, 0, 0);

	tmp_tcp = (OS_TCB*)mem_malloc(sizeof(OS_TCB));
	if (!tmp_tcp)
		goto end;

	for (prio = 0; prio <= OS_TASK_IDLE_PRIO; prio++)
	{
		ptcb = OSTCBPrioTbl[prio];
		if (ptcb != (OS_TCB*)0)
		{
			if (ptcb != OS_TCB_RESERVED)
			{
				memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);

				irq_flag = local_irq_save();
				*tmp_tcp =  *ptcb;
				local_irq_restore(irq_flag);
				cnt = 0;

				snprintf(tmp, 127, "%s\r\n", ptcb->OSTCBTaskName);
				CHECK_AND_ADD_BUFF;

				snprintf(tmp, 127, "  ID��:%d\r\n", tmp_tcp->OSTCBId);
				CHECK_AND_ADD_BUFF;
				snprintf(tmp, 127, "  ���ȼ�:%d\r\n", tmp_tcp->OSTCBPrio);
				CHECK_AND_ADD_BUFF;

				snprintf(tmp, 127, "  ջ�׵�ַ0x%08x\r\n", tmp_tcp->OSTCBStkBottom);
				CHECK_AND_ADD_BUFF;
				snprintf(tmp, 127, "  ��ջ��С%d WORD\r\n", tmp_tcp->OSTCBStkSize);
				CHECK_AND_ADD_BUFF;
				snprintf(tmp, 127, "  ��ʹ�ö�ջ%d WORD\r\n", tmp_tcp->OSTCBStkUsed / 4);
				CHECK_AND_ADD_BUFF;
				/*
				snprintf(tmp,127, "  ״̬:%02x\r\n",tmp_tcp->OSTCBStat);
				CHECK_AND_ADD_BUFF;
				snprintf(tmp,127, "  ����״̬:%02x\r\n",tmp_tcp->OSTCBStatPend);
				CHECK_AND_ADD_BUFF;
				 */
				p_globle_buff[cnt] = 0;
				send_test_pkg(STAT_TYPE_TASK, p_globle_buff, cnt + 1);
			}
		}
	}
	end: if (tmp_tcp)
		mem_free(tmp_tcp);
	if (tmp)
		mem_free(tmp);
}


extern struct lwip_sock sockets[NUM_SOCKETS];

#define send_lwip_stat(X)	memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);	\
snprintf(tmp,127, ""###X##"\r\n  xmit:%d,recv:%d,drop:%d,memerr:%d,chkerr:%d,lenerr:%d,rterr:%d,proterr:%d,opterr:%d,err:%d\r\n", \
lwip_stats.X.xmit,lwip_stats.X.recv,lwip_stats.X.drop,lwip_stats.X.memerr,lwip_stats.X.chkerr,lwip_stats.X.lenerr,lwip_stats.X.rterr,lwip_stats.X.proterr,lwip_stats.X.opterr,lwip_stats.X.err);\
CHECK_AND_ADD_BUFF;\
send_test_pkg(STAT_TYPE_SOCKET, p_globle_buff, cnt + 1);

void send_socket_stat_data()
{
	int i, cnt = 0;
	char *tmp;
	tmp = (char*)mem_malloc(128);
	if (!tmp)
		goto end;

	send_test_pkg(STAT_TYPE_SOCKET, 0, 0);
	#if LINK_STATS
//	send_lwip_stat(link);
	#endif
	#if ETHARP_STATS
//	send_lwip_stat(etharp);
	#endif
	#if IPFRAG_STATS
//	send_lwip_stat(ip_frag);
	#endif
	#if IP_STATS
//	send_lwip_stat(ip);
	#endif
	#if ICMP_STATS
//	send_lwip_stat(icmp);
	#endif
	#if UDP_STATS
//	send_lwip_stat(udp);
	#endif
	#if TCP_STATS
//	send_lwip_stat(tcp);
	#endif

	for (i = 0; i < MEMP_NUM_NETCONN; i++)
	{

/*		if (sockets[i].conn)
		{
			memset(p_globle_buff, 0, GLOBLE_TEST_BUFF_SIZE);

			CHECK_AND_ADD_LINE;

			snprintf(tmp, 127, "socket:%d\r\n", i);
			CHECK_AND_ADD_BUFF;

			snprintf(tmp, 127, "  send_window:%d\r\n", sockets[i].conn->pcb.tcp->snd_wnd);
			CHECK_AND_ADD_BUFF;

			snprintf(tmp, 127, "  send_bufsize:%d\r\n", sockets[i].conn->pcb.tcp->snd_buf);
			CHECK_AND_ADD_BUFF;

			snprintf(tmp, 127, "  recv_window:%d\r\n", sockets[i].conn->pcb.tcp->rcv_wnd);
			CHECK_AND_ADD_BUFF;

			send_test_pkg(STAT_TYPE_SOCKET, p_globle_buff, cnt + 1);
		}*/
	}
	end: if (tmp)
		mem_free(tmp);
}


void handle_sys_monitor()
{
	p_globle_buff = (char*)mem_malloc(GLOBLE_TEST_BUFF_SIZE);
	if (!p_globle_buff)
		return ;
	//logd_trace;
	send_sys_stat_data();
	send_task_stat_data();
	send_socket_stat_data();
	//send_device_stat_data();
	mem_free(p_globle_buff);
}

void dev_monitor_task(void *arg)
{
	sleep(2000);
	while (1)
	{
		sleep(1000);
		handle_sys_monitor();
	}
}

#endif


#if OS_APP_HOOKS_EN > 0u
void App_TaskCreateHook(OS_TCB *ptcb)
{
	ptcb = ptcb;
}

void App_TaskDelHook(OS_TCB *ptcb)
{
	ptcb = ptcb;
}

void App_TaskReturnHook(OS_TCB *ptcb)
{
	ptcb = ptcb;
}

void App_TCBInitHook(OS_TCB *ptcb)
{
	ptcb = ptcb;
}

void App_TaskSwHook(void){

}

void App_TimeTickHook(void){}

//uC/OS-II Stat�߳��е��ô˺�����ÿ100MSһ��
void App_TaskStatHook()
{
	#if USE_MEM_DEBUG
	mem_slide_check(0);
	#endif
	//button_stat_callback();
}

#endif

void HardFault_Handler()
{
	static int once = 0;
	if (!once)
	{
		once = 1;
		mem_slide_check(0);
	}
	p_err("%s\n", __FUNCTION__);
	while (1)
		;
}
