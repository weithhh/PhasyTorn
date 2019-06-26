#include "debug/serial.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_usart.h"


void debug_serial_hw_init() {
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_4;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &gpio_init);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_7);

	USART_InitTypeDef usart_init;
	usart_init.USART_BaudRate = 1152000;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &usart_init);
	USART_Cmd(USART1, ENABLE);
}

void debug_serial_send_halfword(uint16_t halfword) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, (uint8_t)(halfword >> 8));
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, (uint8_t)halfword);
}
