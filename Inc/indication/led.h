#ifndef INDICATION_LED_H_
#define INDICATION_LED_H_

#include <stdint.h>


void indication_led_hw_init();
void indication_led_set_brightness(uint8_t percentage);

#endif /* INDICATION_LED_H_ */
