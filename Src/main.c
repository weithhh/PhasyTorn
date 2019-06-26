#include "stm32f30x_rcc.h"

#include "indication/led.h"
#include "signals/generator.h"
#include "signals/receiver.h"
#include "debug/serial.h"
#include "utils.h"

#define TX_SAMPLES_COUNT 256
#define RX_SINE_PERIODS 2
#define RX_SAMPLES_COUNT TX_SAMPLES_COUNT * RX_SINE_PERIODS
#define RX_CLOCK_DIVIDER 2 * RX_SINE_PERIODS
#define FREQ_HZ 5000


static uint16_t tx_sine_table[TX_SAMPLES_COUNT];
static uint16_t rx_circular_buffer[RX_SAMPLES_COUNT];

void rcc_hw_init() {
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
}

int main(void) {
	rcc_hw_init();
	utils_hw_init();
	indication_led_hw_init();
	signals_generator_hw_init((uint16_t*)&tx_sine_table, TX_SAMPLES_COUNT, FREQ_HZ);
	signals_receiver_hw_init((uint16_t*)&rx_circular_buffer, RX_SAMPLES_COUNT, RX_CLOCK_DIVIDER, FREQ_HZ);
	debug_serial_hw_init();

	while (1) {
		ADC_StopConversion(ADC1);
		uint16_t high = rx_circular_buffer[0];
		uint16_t low = rx_circular_buffer[0];

		for (uint16_t i = 0; i < RX_SAMPLES_COUNT; i++) {
			if (rx_circular_buffer[i] > high) high = rx_circular_buffer[i];
			if (rx_circular_buffer[i] < low) low = rx_circular_buffer[i];
		}
		ADC_StartConversion(ADC1);

		debug_serial_send_halfword(high - low);
		sleep(1000000.f / FREQ_HZ * RX_SINE_PERIODS + 100);
	}
}



