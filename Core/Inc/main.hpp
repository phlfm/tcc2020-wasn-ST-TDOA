#ifndef __MAIN_H
#define __MAIN_H

#include "HAL_initializers.hpp"
#include "debug_config.hpp"
#include "TDOA.hpp"
/** ADC Buffer Size -- Number of samples the buffer can hold
 * Each sample is 2 bytes (16 bit) in memory (but the ADC samples @ 12bit max. So we can use 4 bits per sample for parity checks)
 *
 * If buffer size = 30720 samples it will use (30720 samples * 2 bytes/sample) 61440 bytes and hold (30720/3) 10240 samples per channel
 * |------> 10240 samples per channel @ 40 kSps represents a (256 ms) window of recording. In 256 ms sound @ 340 m/s travels 87 meters.
 *
 *  buffer size = 46080 samples ---> 92160 bytes in memory = 15360 samples per channel = 384 ms = 131 m
 *  buffer size = 60000 samples ---> 120 kb in memory = 20k samples per channel = 500 ms = 170 m
 *
**/
#define ADC_BUFFER_SIZE	48000 // Number of samples the buffer can hold
#define CHANNEL_COUNT	3 // number of channels / microphones
#define SPACE_DIMENSIONS 2




#endif /* __MAIN_H */
