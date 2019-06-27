#ifndef DEBUG_SERIAL_H_
#define DEBUG_SERIAL_H_

#include <stdint.h>


void debug_serial_hw_init();
void debug_serial_send_halfword(uint16_t halfword);

#endif /* DEBUG_SERIAL_H_ */
