#include "signals/receiver.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_opamp.h"

#include "utils.h"


void signals_receiver_hw_init(uint16_t *circular_buffer, uint16_t samples, uint16_t clock_prescaler, uint32_t freq) {
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_7;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);


	OPAMP_InitTypeDef opamp_init;
	opamp_init.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO1;
	opamp_init.OPAMP_InvertingInput = OPAMP_InvertingInput_PGA;
	OPAMP_Init(OPAMP_Selection_OPAMP1, &opamp_init);

	OPAMP_PGAConfig(OPAMP_Selection_OPAMP1, OPAMP_OPAMP_PGAGain_8, OPAMP_PGAConnect_No);
	OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);

	// ADC calibration

	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	sleep(10);

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) != RESET);


	DMA_InitTypeDef dma_init;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	dma_init.DMA_MemoryBaseAddr = (uint32_t)circular_buffer;
	dma_init.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma_init.DMA_BufferSize = samples;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_M2M = DMA_M2M_Disable;
	DMA_DeInit(DMA1_Channel1);
	DMA_Init(DMA1_Channel1, &dma_init);


	ADC_CommonInitTypeDef adc_common_init;
	ADC_CommonStructInit(&adc_common_init);
	adc_common_init.ADC_Mode = ADC_Mode_Independent;
	adc_common_init.ADC_Clock = ADC_Clock_AsynClkMode;
	adc_common_init.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	adc_common_init.ADC_DMAMode = ADC_DMAMode_Circular;
	adc_common_init.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &adc_common_init);

	ADC_InitTypeDef adc_init;
	ADC_StructInit(&adc_init);
	adc_init.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	adc_init.ADC_Resolution = ADC_Resolution_12b;
	adc_init.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_7;
	adc_init.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	adc_init.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init.ADC_OverrunMode = ADC_OverrunMode_Disable;
	adc_init.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	adc_init.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &adc_init);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_61Cycles5);
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	DMA_Cmd(DMA1_Channel1, ENABLE);


	TIM_TimeBaseInitTypeDef tim_base_init;
	tim_base_init.TIM_Prescaler = clock_prescaler - 1;
	tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_base_init.TIM_Period = SystemCoreClock / samples / freq;
	tim_base_init.TIM_ClockDivision = 0;
	tim_base_init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &tim_base_init);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR1); // From
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
}

void signals_receiver_start() {
	ADC_StartConversion(ADC1);
}

void signals_receiver_stop() {
	ADC_StopConversion(ADC1);
}
