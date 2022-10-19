

#ifndef __NUM_H
#define __NUM_H

#include <stdint.h>

uint16_t single2half(float number);
float half2single(uint16_t number);

uint16_t limitUint16(int32_t value);
float constrain(float value, const float minVal, const float maxVal);
float deadband(float value, const float threshold);

#endif
