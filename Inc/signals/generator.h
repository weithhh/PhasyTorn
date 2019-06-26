#ifndef SIGNALS_GENERATOR_H_
#define SIGNALS_GENERATOR_H_

#include <stdint.h>


void signals_generator_hw_init(uint16_t *lookup_table, const uint16_t samples, const uint32_t freq);

#endif /* SIGNALS_GENERATOR_H_ */
