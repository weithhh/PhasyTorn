#include <cmath>
#include <stdio.h>

#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_dma.h"

#define DAC_DHR12R1_OFFSET 0x08

const uint16_t SAMPLES_COUNT = 255;
uint16_t sine_table[SAMPLES_COUNT];

void fill_dac_sine_table(uint16_t *table, const uint16_t samples_count, const uint16_t max_value, const uint16_t min_value = 0) {
	float step = 2 * M_PI / samples_count;
	int range = (max_value - min_value) / 2.f;

	for (uint16_t i  = 0; i < samples_count; i++) {
		float y = sin(i * step) + 1;
		table[i] = round(y * range + min_value);
	}
}

void setup_indicator_led() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &gpio_init);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_2);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

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
	tim_oc_init.TIM_Pulse = 100 * (tim_base_init.TIM_Period - 1) / 1000;
	tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;
	tim_oc_init.TIM_OCNPolarity = TIM_OCNPolarity_High;
	tim_oc_init.TIM_OCIdleState = TIM_OCIdleState_Set;
	tim_oc_init.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &tim_oc_init);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void setup_sine_generator(const uint16_t *table, const uint16_t samples_count, const uint32_t freq) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_5;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	DAC_DeInit(DAC1);
	DAC_InitTypeDef dac_init;
	dac_init.DAC_Trigger = DAC_Trigger_T2_TRGO;
	dac_init.DAC_WaveGeneration = DAC_WaveGeneration_None;
	dac_init.DAC_Buffer_Switch = DAC_BufferSwitch_Enable;
	dac_init.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_Init(DAC1, DAC_Channel_1, &dac_init);
	DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);

	DMA_DeInit(DMA2_Channel3);
	DMA_InitTypeDef dma_init;
	dma_init.DMA_PeripheralBaseAddr = DAC_BASE + DAC_DHR12R1_OFFSET;
	dma_init.DMA_MemoryBaseAddr = (uint32_t)table;
	dma_init.DMA_DIR = DMA_DIR_PeripheralDST;
	dma_init.DMA_BufferSize = samples_count;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel3, &dma_init);

	DMA_Cmd(DMA2_Channel3, ENABLE);
	DAC_DMACmd(DAC1, DAC_Channel_1, ENABLE);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef tim_init;
	TIM_TimeBaseStructInit(&tim_init);
	tim_init.TIM_Period = SystemCoreClock / SAMPLES_COUNT / freq;
	tim_init.TIM_Prescaler = 0x0;
	tim_init.TIM_ClockDivision = 0x0;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_init);

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	TIM_Cmd(TIM2, ENABLE);
}

int main(void) {
	setup_indicator_led();

	fill_dac_sine_table(sine_table, SAMPLES_COUNT, 2047);
	setup_sine_generator((uint16_t*)&sine_table, SAMPLES_COUNT, 10000);

	while (true) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
	while (1);
}
#endif
