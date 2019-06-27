#ifndef SIGNALS_DSP_H_
#define SIGNALS_DSP_H_

#include <stdint.h>

#define PK2PK_AVERAGING_WINDOW 128 // Samples


uint16_t signals_dsp_pk2pk_measure(uint16_t *values, uint16_t length);
uint16_t signals_dsp_pk2pk_average(uint16_t new_value);

#endif /* SIGNALS_DSP_H_ */
