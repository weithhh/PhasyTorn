#include "signals/generator.h"

#include <math.h>

#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_dma.h"


void fill_sine_lookup_table(uint16_t *table, const uint16_t samples_count, const uint16_t max_value, const uint16_t min_value);

void signals_generator_hw_init(uint16_t *lookup_table, const uint16_t samples, const uint32_t freq) {
	fill_sine_lookup_table(lookup_table, samples, 4095, 0);

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_5;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);


	DAC_InitTypeDef dac_init;
	dac_init.DAC_Trigger = DAC_Trigger_T2_TRGO;
	dac_init.DAC_WaveGeneration = DAC_WaveGeneration_None;
	dac_init.DAC_Buffer_Switch = DAC_BufferSwitch_Enable;
	dac_init.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_DeInit(DAC1);
	DAC_Init(DAC1, DAC_Channel_1, &dac_init);

	DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);


	DMA_InitTypeDef dma_init;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)&DAC1->DHR12R1;
	dma_init.DMA_MemoryBaseAddr = (uint32_t)lookup_table;
	dma_init.DMA_DIR = DMA_DIR_PeripheralDST;
	dma_init.DMA_BufferSize = samples;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_M2M = DMA_M2M_Disable;
	DMA_DeInit(DMA2_Channel3);
	DMA_Init(DMA2_Channel3, &dma_init);

	DMA_Cmd(DMA2_Channel3, ENABLE);
	DAC_DMACmd(DAC1, DAC_Channel_1, ENABLE);


	TIM_TimeBaseInitTypeDef tim_init;
	TIM_TimeBaseStructInit(&tim_init);
	tim_init.TIM_Period = SystemCoreClock / samples / freq;
	tim_init.TIM_Prescaler = 0x0;
	tim_init.TIM_ClockDivision = 0x0;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_init);

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIM2, ENABLE);
}

void fill_sine_lookup_table(uint16_t *table, const uint16_t samples_count, const uint16_t max_value, const uint16_t min_value) {
	float step = 2 * M_PI / samples_count;
	int range = (max_value - min_value) / 2.f;

	for (uint16_t i  = 0; i < samples_count; i++) {
		float y = sin(i * step) + 1;
		table[i] = round(y * range + min_value);
	}
}
