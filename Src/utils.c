#include "utils.h"

#include "stm32f30x.h"


volatile uint32_t systick_delay = 0;

void utils_hw_init() {
	// Configure SysTick interrupts with 1 uSec resolution
	if (SysTick_Config(SystemCoreClock / 1000000)) while (1);
}

void sleep(uint32_t usec) {
	systick_delay = usec;
	while (systick_delay != 0);
}


/* IRQ Handlers */

void SysTick_Handler(void) {
	systick_delay--;
}

/* IRQ Handlers */


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
	while (1);
}
#endif
