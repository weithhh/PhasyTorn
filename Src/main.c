#include <math.h>
#include <stdio.h>

#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_opamp.h"

#include "indication/led.h"
#include "utils.h"


#define TX_SAMPLES_COUNT 256
#define RX_SAMPLES_COUNT 1024
#define RX_CLOCK_DIVIDER 7  // + 1
#define FREQ_HZ 5000

static uint16_t tx_sine_table[TX_SAMPLES_COUNT];
static uint16_t rx_circular_buffer[RX_SAMPLES_COUNT];


void fill_dac_sine_table(uint16_t *table, const uint16_t samples_count, const uint16_t max_value, const uint16_t min_value) {
	float step = 2 * M_PI / samples_count;
	int range = (max_value - min_value) / 2.f;

	for (uint16_t i  = 0; i < samples_count; i++) {
		float y = sin(i * step) + 1;
		table[i] = round(y * range + min_value);
	}
}

void setup_sine_generator(const uint16_t *table, const uint16_t samples_count, const uint32_t freq) {
	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_5;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);

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
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)&DAC1->DHR12R1;
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


	TIM_TimeBaseInitTypeDef tim_init;
	TIM_TimeBaseStructInit(&tim_init);
	tim_init.TIM_Period = SystemCoreClock / samples_count / freq;
	tim_init.TIM_Prescaler = 0x0;
	tim_init.TIM_ClockDivision = 0x0;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tim_init);

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIM2, ENABLE);
}

void setup_uart() {
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

void setup_adc(uint16_t *circular_buffer, uint16_t samples_count, uint16_t clock_prescaler, uint32_t freq) {
	// GPIIO init

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_1;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);

	// OPAMP init
	OPAMP_InitTypeDef opamp_init;
	opamp_init.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
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

	// DMA init

	DMA_DeInit(DMA1_Channel1);
	DMA_InitTypeDef dma_init;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	dma_init.DMA_MemoryBaseAddr = (uint32_t)circular_buffer;
	dma_init.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma_init.DMA_BufferSize = samples_count;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &dma_init);

	// ADC init

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
	ADC_StartConversion(ADC1);

	//ADC timer init

	TIM_TimeBaseInitTypeDef tim_base_init;
	tim_base_init.TIM_Prescaler = clock_prescaler;
	tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_base_init.TIM_Period = SystemCoreClock / samples_count / freq;
	tim_base_init.TIM_ClockDivision = 0;
	tim_base_init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &tim_base_init);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR1);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
}

int main(void) {
	// Enable clocking for all required peripherals

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  // OPAMP

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Main

	utils_hw_init();
	indication_led_hw_init();

	fill_dac_sine_table(tx_sine_table, TX_SAMPLES_COUNT, 4095, 0);
	setup_sine_generator((uint16_t*)&tx_sine_table, TX_SAMPLES_COUNT, FREQ_HZ);

	setup_adc((uint16_t*)&rx_circular_buffer, RX_SAMPLES_COUNT, RX_CLOCK_DIVIDER, FREQ_HZ);
	setup_uart();

	while (1) {
//		ADC_StopConversion(ADC1);
//		for (uint16_t i = 0; i < RX_SAMPLES_COUNT; i++) {
//			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//			USART_SendData(USART1, (uint8_t)rx_circular_buffer[i]);
//			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//			USART_SendData(USART1, (uint8_t)(rx_circular_buffer[i] >> 8));
//		}
//		ADC_StartConversion(ADC1);

		ADC_StopConversion(ADC1);
		uint16_t high = rx_circular_buffer[0];
		uint16_t low = rx_circular_buffer[0];

		for (uint16_t i = 0; i < RX_SAMPLES_COUNT; i++) {
			if (rx_circular_buffer[i] > high) high = rx_circular_buffer[i];
			if (rx_circular_buffer[i] < low) low = rx_circular_buffer[i];
		}
		ADC_StartConversion(ADC1);

		uint16_t pk2pk = high - low;
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USART1, (uint8_t)(pk2pk >> 8));
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USART1, (uint8_t)pk2pk);

		sleep(1000000.f / FREQ_HZ * 4 + 100); // 1 / FREQ * 4 periods + 100 uS
	}
}



