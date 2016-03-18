#define DEBUG

#include "app.h"
#include "api.h"
#include "bsp.h"
#include "drivers.h"
#include "test.h"
#include "lwip/netif.h"
//#include <absacc.h>


void delay_1us()
{
	int i = 25;
	while (i--)
		;
}

void delay_us(uint32_t us)
{
	while (us--)
		delay_1us();
}


void init_rng()
{
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	RNG_Cmd(ENABLE);
}

/**
 *读一个随机数
 */
uint32_t get_random(void)
{
	return RNG_GetRandomNumber();
}

void init_systick()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SysTick_Config(SystemCoreClock / (OS_TICKS_PER_SEC *10));
}

int is_hw_ok()
{
	return p_usb_dev->host.ConnSts;
}

uint8_t led_value = 0;
uint8_t led_bright_value = 10;

void led_switch(uint8_t value)
{
	p_dbg("led_switch:%x", value);
	led_value = value;
}

void led_bright(uint8_t value)
{
	p_dbg("led_bright:%d", value);
	if (value > 99)
		value = 99;

	led_bright_value = value / 10;
}

/*
在SysTick_Handler里面调用，每1ms调用一次，用于刷新led状态
 */
void led_scan()
{
	static int led_bright_cnt = 0;

	if (led_bright_cnt++ >= 10)
		led_bright_cnt = 0;

	if (led_bright_cnt == 0 && led_value)
	{
#if !USE_I2S_MODE
		if (led_value &(1 << 0))
			GPIO1_SET;
		if (led_value &(1 << 1))
			GPIO2_SET;
		if (led_value &(1 << 2))
			GPIO3_SET;
#endif
		if (led_value &(1 << 3))
			GPIO4_SET;
		if (led_value &(1 << 4))
			GPIO5_SET;
		if (led_value &(1 << 5))
			GPIO6_SET;
	}

	if (led_bright_cnt == led_bright_value)
	{
#if !USE_I2S_MODE
		GPIO1_CLR;
		GPIO2_CLR;
		GPIO3_CLR;
#endif
		GPIO4_CLR;
		GPIO5_CLR;
		GPIO6_CLR;
	}
}

void gpio_cfg(uint32_t group, uint32_t pin, uint8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pin;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	if (mode == GPIO_Mode_AIN)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else if (mode == GPIO_Mode_IN_FLOATING)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else
	if (mode == GPIO_Mode_IPD)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	}
	else
	if (mode == GPIO_Mode_IPU)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	}
	else
	if (mode == GPIO_Mode_Out_OD)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else
	if (mode == GPIO_Mode_Out_PP)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else
	if (mode == GPIO_Mode_AF_OD)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else
	if (mode == GPIO_Mode_AF_PP)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	}
	else
	if (mode == GPIO_Mode_AF_IF)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	else
	if (mode == GPIO_Mode_AF_IPU)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	}
	GPIO_Init((GPIO_TypeDef*)group, &GPIO_InitStructure);
}


SWITCH_EVENT g_switch = SWITCH_EVENT_OFF;
void button_stat_callback()
{
	static uint8_t last_stat = 1;
	BUTTON_EVENT button_event = BUTTON_EVENT_IDLE;

	if (BUTTON_STAT && !last_stat)
	{
		last_stat = BUTTON_STAT;
		button_event = BUTTON_EVENT_UP;
		GPIO6_SET;
		//p_dbg("button up");
	}

	if (!BUTTON_STAT && last_stat)
	{
		last_stat = BUTTON_STAT;
		button_event = BUTTON_EVENT_DOWN;
		GPIO6_CLR;
		//p_dbg("button down");
	}

	if (button_event == BUTTON_EVENT_UP)
	{
		g_switch = !g_switch;
		//camera_button_event(g_switch);
		audio_button_event(g_switch);
	}
}

void usr_gpio_init()
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //外部中断需要用到

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	gpio_cfg((uint32_t)BUTTON_PORT, BUTTON_PIN, GPIO_Mode_IPU);

#if !USE_I2S_MODE
	GPIO1_OUTPUT;
	GPIO2_OUTPUT;
	GPIO3_OUTPUT;
#endif
	GPIO4_OUTPUT;
	GPIO5_OUTPUT;
	GPIO6_OUTPUT;

	LED_OFF;
}


int check_rst_stat()
{
	uint32_t stat;
	stat = RCC->CSR;
	RCC->CSR = 1L << 24; //清除复位标志

	p_err("reset:");
	if (stat &(1UL << 31))
	// 低功耗复位
	{
		p_err("low power\n");
	}
	if (stat &(1UL << 30))
	//窗口看门狗复位
	{
		p_err("windw wdg\n");
	}
	if (stat &(1UL << 29))
	//独立看门狗复位
	{
		p_err("indep wdg\n");
	}
	if (stat &(1UL << 28))
	//软件复位
	{
		p_err("soft reset\n");
	}
	if (stat &(1UL << 27))
	//掉电复位
	{
		p_err("por reset\n");
	}
	if (stat &(1UL << 26))
	//rst复位
	{
		p_err("user reset\n");
	}

	return 0;
}

void assert_failed(uint8_t *file, uint32_t line)
{
	p_err("assert_failed in:%s,line:%d \n", file ? file : "n", line);
	while (1)
		;
}


void misc_init()
{
	init_rng();
	init_systick();
}


void show_sys_info()
{

	p_dbg("mac:%02x-%02x-%02x-%02x-%02x-%02x", p_netif->hwaddr[0], p_netif->hwaddr[1], p_netif->hwaddr[2], p_netif->hwaddr[3], p_netif->hwaddr[4], p_netif->hwaddr[5]);

	show_tcpip_info();
}
