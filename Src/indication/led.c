#include "indication/led.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"


void indication_led_hw_init() {
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &gpio_init);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_2);

	TIM_TimeBaseInitTypeDef tim_base_init;
	tim_base_init.TIM_Prescaler = 0;
	tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_base_init.TIM_Period = SystemCoreClock / 10000 - 1;
	tim_base_init.TIM_ClockDivision = 0;
	tim_base_init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &tim_base_init);

	TIM_OCInitTypeDef tim_oc_init;
	tim_oc_init.TIM_OCMode = TIM_OCMode_PWM2;
	tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
	tim_oc_init.TIM_OutputNState = TIM_OutputNState_Disable;
	tim_oc_init.TIM_Pulse = 0;
	tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;
	tim_oc_init.TIM_OCNPolarity = TIM_OCNPolarity_High;
	tim_oc_init.TIM_OCIdleState = TIM_OCIdleState_Set;
	tim_oc_init.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &tim_oc_init);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
