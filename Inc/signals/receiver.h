#ifndef SIGNALS_RECEIVER_H_
#define SIGNALS_RECEIVER_H_

#include <stdint.h>


void signals_receiver_hw_init(uint16_t *circular_buffer, uint16_t samples, uint16_t clock_prescaler, uint32_t freq);

#endif /* SIGNALS_RECEIVER_H_ */
