#ifndef ANALOG_IO_H
#define ANALOG_IO_H

#include "common.h"

#define MID_VALUE 128

extern volatile uint8_t audio_sample;

void init_analog_io();

#endif // ANALOG_IO_H