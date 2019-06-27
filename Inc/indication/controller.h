#ifndef INDICATION_CONTROLLER_H_
#define INDICATION_CONTROLLER_H_

#include <stdint.h>


void indication_controller_hw_init();
uint16_t indication_controller_pk2pk_update(uint16_t value);

#endif /* INDICATION_CONTROLLER_H_ */
