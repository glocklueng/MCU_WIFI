#ifndef _MISC_H
#define _MISC_H
#include "api.h"
//#include "audio_low_level.h"

#define UART1_RX_PORT_GROUP 		GPIOA
#define UART1_TX_PORT_GROUP 		GPIOA
#define UART1_TX_PIN 				GPIO_Pin_9
#define UART1_RX_PIN 				GPIO_Pin_10
#define UART1_TX_PIN_SOURSE		GPIO_PinSource9
#define UART1_RX_PIN_SOURSE		GPIO_PinSource10

#define UART3_RX_PORT_GROUP 		GPIOB
#define UART3_TX_PORT_GROUP 		GPIOB
#define UART3_TX_PIN 				GPIO_Pin_10
#define UART3_RX_PIN 				GPIO_Pin_11
#define UART3_TX_PIN_SOURSE		GPIO_PinSource10
#define UART3_RX_PIN_SOURSE		GPIO_PinSource11


//define DCMI PORT
#define DCMI_HS_PORT_GROUP 	GPIOA
#define DCMI_HS_PIN 				GPIO_Pin_4
#define DCMI_HS_SOURCE 		GPIO_PinSource4

#define DCMI_VS_PORT_GROUP 	GPIOB
#define DCMI_VS_PIN 				GPIO_Pin_7
#define DCMI_VS_SOURCE 		GPIO_PinSource7

#define DCMI_PCLK_PORT_GROUP GPIOA
#define DCMI_PCLK_PIN 			GPIO_Pin_6
#define DCMI_PCLK_SOURCE 		GPIO_PinSource6

#define DCMI_D0_PORT_GROUP 	GPIOC
#define DCMI_D0_PIN 				GPIO_Pin_6
#define DCMI_D0_SOURCE 			GPIO_PinSource6

#define DCMI_D1_PORT_GROUP 	GPIOC
#define DCMI_D1_PIN 				GPIO_Pin_7
#define DCMI_D1_SOURCE 			GPIO_PinSource7

#define DCMI_D2_PORT_GROUP 	GPIOC
#define DCMI_D2_PIN 				GPIO_Pin_8
#define DCMI_D2_SOURCE 			GPIO_PinSource8

#define DCMI_D3_PORT_GROUP 	GPIOC
#define DCMI_D3_PIN 				GPIO_Pin_9
#define DCMI_D3_SOURCE 			GPIO_PinSource9

#define DCMI_D4_PORT_GROUP 	GPIOC
#define DCMI_D4_PIN 				GPIO_Pin_11
#define DCMI_D4_SOURCE 			GPIO_PinSource11

#define DCMI_D5_PORT_GROUP 	GPIOB
#define DCMI_D5_PIN 				GPIO_Pin_6
#define DCMI_D5_SOURCE 			GPIO_PinSource6

#define DCMI_D6_PORT_GROUP 	GPIOB
#define DCMI_D6_PIN 				GPIO_Pin_8
#define DCMI_D6_SOURCE 			GPIO_PinSource8

#define DCMI_D7_PORT_GROUP 	GPIOB
#define DCMI_D7_PIN 				GPIO_Pin_9
#define DCMI_D7_SOURCE 			GPIO_PinSource9

#define DCMI_D8_PORT_GROUP 	GPIOC
#define DCMI_D8_PIN 				GPIO_Pin_10
#define DCMI_D8_SOURCE 			GPIO_PinSource10

#define DCMI_D9_PORT_GROUP 	GPIOC
#define DCMI_D9_PIN 				GPIO_Pin_12
#define DCMI_D9_SOURCE 			GPIO_PinSource12

#define MCO_PORT_GROUP 		GPIOA
#define MCO_PIN 					GPIO_Pin_8

#define DCMI_RST_PORT_GROUP 	GPIOA
#define DCMI_RST_PIN 			GPIO_Pin_12

#define DCMI_PWD_PORT_GROUP 	GPIOA
#define DCMI_PWD_PIN 			GPIO_Pin_11

#define SIO_C_PORT_GROUP 		GPIOC
#define SIO_C_PIN 				GPIO_Pin_14

#define SIO_D_PORT_GROUP 		GPIOC
#define SIO_D_PIN 				GPIO_Pin_13

#define SIO_D_OUT		gpio_cfg((uint32_t)SIO_D_PORT_GROUP, SIO_D_PIN, GPIO_Mode_Out_OD);
#define SIO_D_IN			gpio_cfg((uint32_t)SIO_D_PORT_GROUP, SIO_D_PIN, GPIO_Mode_IPU);

//end define DCMI PORT


#define GPIO_SET(port,pin) (port->BSRRL = pin)
#define GPIO_CLR(port,pin) (port->BSRRH = pin)
#define GPIO_STAT(port,pin) (port->IDR & pin)

#if !USE_I2S_MODE
#define GPIO1_GROUP	GPIOA
#define GPIO2_GROUP	GPIOC
#define GPIO3_GROUP	GPIOC
#endif
#define GPIO4_GROUP	GPIOB
#define GPIO5_GROUP	GPIOB
#define GPIO6_GROUP	GPIOB

#if !USE_I2S_MODE
#define GPIO1_PIN	GPIO_Pin_7
#define GPIO2_PIN	GPIO_Pin_4
#define GPIO3_PIN	GPIO_Pin_5
#endif
#define GPIO4_PIN	GPIO_Pin_0
#define GPIO5_PIN	GPIO_Pin_1
#define GPIO6_PIN	GPIO_Pin_2

#define LED_ON 		GPIO_SET(GPIO6_GROUP, GPIO6_PIN)
#define LED_OFF 		GPIO_CLR(GPIO6_GROUP, GPIO6_PIN)

#if !USE_I2S_MODE
#define GPIO1_OUTPUT	gpio_cfg((uint32_t)GPIO1_GROUP, GPIO1_PIN, GPIO_Mode_Out_PP);
#define GPIO1_INPUT		gpio_cfg((uint32_t)GPIO1_GROUP, GPIO1_PIN, GPIO_Mode_IPU);
#define GPIO1_SET 		GPIO_SET(GPIO1_GROUP,GPIO1_PIN)
#define GPIO1_CLR 		GPIO_CLR(GPIO1_GROUP,GPIO1_PIN)

#define GPIO2_OUTPUT	gpio_cfg((uint32_t)GPIO2_GROUP, GPIO2_PIN, GPIO_Mode_Out_PP);
#define GPIO2_INPUT		gpio_cfg((uint32_t)GPIO2_GROUP, GPIO2_PIN, GPIO_Mode_IPU);
#define GPIO2_SET 		GPIO_SET(GPIO2_GROUP,GPIO2_PIN)
#define GPIO2_CLR 		GPIO_CLR(GPIO2_GROUP,GPIO2_PIN)

#define GPIO3_OUTPUT	gpio_cfg((uint32_t)GPIO3_GROUP, GPIO3_PIN, GPIO_Mode_Out_PP);
#define GPIO3_INPUT		gpio_cfg((uint32_t)GPIO3_GROUP, GPIO3_PIN, GPIO_Mode_IPU);
#define GPIO3_SET 		GPIO_SET(GPIO3_GROUP,GPIO3_PIN)
#define GPIO3_CLR 		GPIO_CLR(GPIO3_GROUP,GPIO3_PIN)
#endif

#define GPIO4_OUTPUT	gpio_cfg((uint32_t)GPIO4_GROUP, GPIO4_PIN, GPIO_Mode_Out_PP);
#define GPIO4_INPUT		gpio_cfg((uint32_t)GPIO4_GROUP, GPIO4_PIN, GPIO_Mode_IPU);
#define GPIO4_SET 		GPIO_SET(GPIO4_GROUP,GPIO4_PIN)
#define GPIO4_CLR 		GPIO_CLR(GPIO4_GROUP,GPIO4_PIN)

#define GPIO5_OUTPUT	gpio_cfg((uint32_t)GPIO5_GROUP, GPIO5_PIN, GPIO_Mode_Out_PP);
#define GPIO5_INPUT		gpio_cfg((uint32_t)GPIO5_GROUP, GPIO5_PIN, GPIO_Mode_IPU);
#define GPIO5_SET 		GPIO_SET(GPIO5_GROUP,GPIO5_PIN)
#define GPIO5_CLR 		GPIO_CLR(GPIO5_GROUP,GPIO5_PIN)


#define GPIO6_OUTPUT	gpio_cfg((uint32_t)GPIO6_GROUP, GPIO6_PIN, GPIO_Mode_Out_PP);
#define GPIO6_INPUT		gpio_cfg((uint32_t)GPIO6_GROUP, GPIO6_PIN, GPIO_Mode_IPU);
#define GPIO6_SET 		GPIO_SET(GPIO6_GROUP,GPIO6_PIN)
#define GPIO6_CLR		GPIO_CLR(GPIO6_GROUP,GPIO6_PIN)


#define BUTTON_PORT		GPIOA
#define BUTTON_PIN			GPIO_Pin_1
#define BUTTON_SOURCE		GPIO_PinSource1
#define BUTTON_EXTI_LINE 	EXTI_Line1

#define BUTTON_STAT 	GPIO_STAT(BUTTON_PORT, BUTTON_PIN)

typedef enum
{
	BUTTON_EVENT_IDLE, BUTTON_EVENT_UP, BUTTON_EVENT_DOWN, BUTTON_EVENT_TOGGLE_ON, BUTTON_EVENT_TOGGLE_OFF
} BUTTON_EVENT;

typedef enum
{
	SWITCH_EVENT_OFF, SWITCH_EVENT_ON
} SWITCH_EVENT;

void gpio_cfg(uint32_t group, uint32_t pin, uint8_t mode);
void usr_gpio_init(void);
int check_rst_stat(void);
void delay_us(uint32_t us);
void init_systick(void);
void misc_init(void);
int is_hw_ok(void);
void led_switch(uint8_t value);
void led_bright(uint8_t value);
uint32_t get_random(void);
void button_stat_callback(void);

#endif
