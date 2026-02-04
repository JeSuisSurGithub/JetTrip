#include "analog_io.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main()
{
    init_analog_io();

    sei();
    while (true) {
        _delay_ms(10);
    }
}