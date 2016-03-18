#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "type.h"


#define ENABLE_MP3					1	//������63k flash

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
* UCOSÿ���߳����ȼ�������ͬ����������ͳһ����
*********************************************************************************************************
*/

enum TASK_PRIO{
TASK_TCPIP_THREAD_PRIO = 10,
TASK_IMG_SEND_PRIO,
RT_USBCMD_PRIO,
MLME_PRIO,
OS_TASKLET_PRIO,
RTMP_TIMER_PRIO,
TASK_MAIN_PRIO,
TASK_ADC_RECV_PKG_PRIO,
TASK_TCP_RECV_PRIO,
TASK_TCP_ACCEPT_PRIO,
TASK_WPS_PRIO,

TASK_MONITOR_PRIO,


OS_TASK_TMR_PRIO
};

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*��λWORD
*********************************************************************************************************
*/
#define RT_USBCMD_STACK_SIZE				256
#define TASK_TCPIP_THREAD_STACK_SIZE		512
#define MLME_STACK_SIZE						512
#define OS_TASKLET_STACK_SIZE				300
#define RTMP_TIMER_STACK_SIZE				384
//���϶�ջ�������޸�

#define TASK_MAIN_STACK_SIZE				512
#define TASK_ADC_RECV_PKG_STACK_SIZE		256
#if ENABLE_MP3
#define TASK_TCP_RECV_STACK_SIZE			(1024 + 512)
#else
#define TASK_TCP_RECV_STACK_SIZE			512
#endif
#define TASK_MONITOR_STACK_SIZE				128
#define TASK_ACCEPT_STACK_SIZE				512
#define TASK_IMG_SEND_STACK_SIZE			512
/*
*********************************************************************************************************
*                               		IRQ_PRIORITY
*����жϺ�������Ҫ��ӡ��Ϣ������ж����ȼ�Ҫ����uart�ж�
*********************************************************************************************************
*/

enum IRQ_PRIORITY{
	OTG_HS_IRQn_Priority = 1,
	USART1_IRQn_Priority,
	TIM2_IRQn_Priority,
	USART3_IRQn_Priority,
	DMA2_Stream5_IRQn_Priority,
	DMA2_Stream7_IRQn_Priority,
	ADC_IRQn_Priority,
	DMA1_Stream5_IRQn_Priority,
	SPI3_IRQn_Priority,
	DMA2_Stream0_IRQn_Priority,
	TIM3_IRQn_Priority,
	TIM4_IRQn_Priority,
	DMA2_Stream1_IRQn_Priority,
	DCMI_IRQn_Priority,
	SysTick_IRQn_Priority
};

/*
*********************************************************************************************************
*                                      kernel ���泣����Դֵ����
*********************************************************************************************************
*/
#define EVENTS_MAX		40		//EVENTS_MAX�������¼�������֮�ͣ��������䡢��Ϣ���С��ź�������������
#define TIMER_MAX       	40		//��ʱ��
#define MSG_QUEUE_MAX	16		//��Ϣ����
#define TASKS_MAX		14		//����


/*
*********************************************************************************************************
*                                      �����ж��������ַ
* ���16k��ַ����bootloader�����ǵĴ�����Ҫ��16k��ʼ��������Ҫ������ʱ���ж���������ӳ�䵽16k��ַ
*********************************************************************************************************
*/
#define IVECTOR_ADDR 					(16*1024)


#endif
