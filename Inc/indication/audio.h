#ifndef INDICATION_AUDIO_H_
#define INDICATION_AUDIO_H_

#include <stdint.h>

#define INDICATION_AUDIO_FREQ_MIN 500
#define INDICATION_AUDIO_FREQ_MAX 10000


void indication_audio_hw_init();
void indication_audio_set_frequency(uint8_t percentage);

#endif /* INDICATION_AUDIO_H_ */
