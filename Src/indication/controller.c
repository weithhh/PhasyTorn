#include "indication/controller.h"

#include <stdlib.h>

#include "stm32f30x_syscfg.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_dma.h"

#include "indication/led.h"
#include "indication/audio.h"
#include "utils.h"


static uint16_t pk2pk_base_value = 0;
static uint16_t pk2pk_last_value = 0;

//static uint16_t pk2pk_noise_dead_zone = 30;
//const uint16_t pk2pk_max_deviation = 100;

static uint16_t pk2pk_adjust[2];  // Noise dead zone and deviation scale

void exti_hw_init() {  // pk2pk_base_value reset on USER button press
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

void adc_hw_init() {  // Noise and sensitivity pots
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio_init);

	ADC_VoltageRegulatorCmd(ADC4, ENABLE);
	sleep(10);

	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC4);
	while (ADC_GetCalibrationStatus(ADC4) != RESET);


	DMA_InitTypeDef dma_init;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC4->DR;
	dma_init.DMA_MemoryBaseAddr = (uint32_t)&pk2pk_adjust;
	dma_init.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma_init.DMA_BufferSize = 2;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_M2M = DMA_M2M_Disable;
	DMA_DeInit(DMA2_Channel2);
	DMA_Init(DMA2_Channel2, &dma_init);


	ADC_CommonInitTypeDef adc_common_init;
	ADC_CommonStructInit(&adc_common_init);
	adc_common_init.ADC_Clock = ADC_Clock_AsynClkMode;
	adc_common_init.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	adc_common_init.ADC_DMAMode = ADC_DMAMode_Circular;
	adc_common_init.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC4, &adc_common_init);

	ADC_InitTypeDef adc_init;
	ADC_StructInit(&adc_init);
	adc_init.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	adc_init.ADC_Resolution = ADC_Resolution_12b;
	adc_init.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	adc_init.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	adc_init.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init.ADC_OverrunMode = ADC_OverrunMode_Enable;
	adc_init.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	adc_init.ADC_NbrOfRegChannel = 2;
	ADC_Init(ADC4, &adc_init);

	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_4, 2, ADC_SampleTime_601Cycles5);
	ADC_DMAConfig(ADC4, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC4, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	while (!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));
	DMA_Cmd(DMA2_Channel2, ENABLE);

	ADC_StartConversion(ADC4);
}

void indication_controller_hw_init() {
	indication_led_hw_init();
	indication_audio_hw_init();
	exti_hw_init();
	adc_hw_init();
}

uint16_t indication_controller_pk2pk_update(uint16_t value) {
	uint16_t deviation = abs(value - pk2pk_base_value);

	int32_t deviation_minus_dead_zone = deviation - (pk2pk_adjust[0] / 10);
	if (deviation_minus_dead_zone > 0) {
		uint8_t percentage = (float)deviation_minus_dead_zone / (pk2pk_adjust[1] / 10) * 100;
		indication_led_set_brightness(percentage);
		indication_audio_set_frequency(percentage);
	} else {
		indication_led_set_brightness(0);
		indication_audio_set_frequency(0);
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
