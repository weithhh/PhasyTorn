#include "signals/dsp.h"


static uint16_t circular_buffer[SIGNALS_DSP_PK2PK_AVERAGING_WINDOW];
static uint16_t buffer_head = 0;

uint16_t signals_dsp_pk2pk_measure(uint16_t *values, uint16_t length) {
	uint16_t high = values[0];
	uint16_t low = values[0];

	for (uint16_t i = 0; i < length; i++) {
		if (values[i] > high) high = values[i];
		if (values[i] < low) low = values[i];
	}

	return high - low;
}

uint16_t signals_dsp_pk2pk_average(uint16_t new_value) {
	buffer_head++;
	if (buffer_head == SIGNALS_DSP_PK2PK_AVERAGING_WINDOW) buffer_head = 0;
	circular_buffer[buffer_head] = new_value;

	uint32_t sum = 0;
	for (uint16_t i = 0; i < SIGNALS_DSP_PK2PK_AVERAGING_WINDOW; i++) {
		sum += circular_buffer[i];
	}

	return sum / SIGNALS_DSP_PK2PK_AVERAGING_WINDOW;
}
