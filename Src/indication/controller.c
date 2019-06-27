#include "indication/controller.h"

#include <stdlib.h>

#include "stm32f30x_syscfg.h"
#include "stm32f30x_exti.h"

#include "indication/led.h"



static uint16_t pk2pk_base_value = 0;
static uint16_t pk2pk_last_value = 0;

static uint16_t pk2pk_noise_dead_zone = 35;
const uint16_t pk2pk_max_deviation = 100;

void indication_controller_hw_init() {
	indication_led_hw_init();

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_IN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio_init);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line = EXTI_Line0;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	NVIC_EnableIRQ(EXTI0_IRQn);
}

uint16_t indication_controller_pk2pk_update(uint16_t value) {
	uint16_t deviation = abs(value - pk2pk_base_value);

	int32_t deviation_minus_dead_zone = deviation - pk2pk_noise_dead_zone;
	if (deviation_minus_dead_zone > 0) {
		uint8_t percentage = (float)deviation_minus_dead_zone / pk2pk_max_deviation * 100;
		indication_led_set_brightness(percentage);
	} else {
		indication_led_set_brightness(0);
	}

	pk2pk_last_value = value;
	return deviation;
}


/* IRQ Handlers */

void EXTI0_IRQHandler(void) {
	pk2pk_base_value = pk2pk_last_value;
	EXTI_ClearITPendingBit(EXTI_Line0);
}

/* IRQ Handlers */
