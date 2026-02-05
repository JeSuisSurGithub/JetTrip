#include "analog_io.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define MAX_DIVIDER 16
#define LFO_UPPER 4095
#define LFO_LOWER 0

// 0->4096 LFO, 24K/16 ~=0.183Hz
// f_lfo = f_sample / 2 (up+down ramp) * divider * (lfo_up-lf_low)
volatile uint8_t divider = 0;
volatile uint16_t lfo_value = 0;
volatile bool lfo_direction = 0;
volatile int16_t delay = 0;

volatile uint8_t samples[SAMPLE_COUNT];
volatile uint16_t wr_idx = 0;

static inline uint8_t spi_io(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

void dac_write12(uint16_t value) {
    PORTB &= ~(1 << PORT_DAC_CS);

    // Gain x1, ~SHDN=1
    spi_io(0b00110000 | ((value >> 8) & 0x0F));
    spi_io(value & 0xFF);

    PORTB |= (1 << PORT_DAC_CS);
}

uint16_t adc_read10(void) {
    uint8_t hi, lo;

    PORTB &= ~(1 << PORT_ADC_CS);

    // Start bit
    // Single ended, Channel 0
    // Output data
    spi_io(0b00000001);
    hi = spi_io(0b10000000);
    lo = spi_io(0x0);

    PORTB |= (1 << PORT_ADC_CS);

    return ((hi & 0b00000011) << 8) | lo;
}

void init_analog_io()
{
    for (int16_t i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = 128;
    }

    // PB2 needs to be set as output
    // DDRB |= (1 << PB2);
    // PORTB |= (1 << PB2);

    // Enable CS MOSI SCK
    // Enable SPI in Master mode
    // 8MHz -> 4MHz speed, least divider
    DDRB |= (1 << PORT_ADC_CS) | (1 << PORT_DAC_CS) | (1 << PORT_MOSI) | (1 << PORT_SCK);
    SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (0 << SPR0);
    SPSR = (1 << SPI2X);
    PORTB |= (1 << PORT_ADC_CS) | (1 << PORT_DAC_CS);

    // Timer1
    // Normal mode
    // CTC, prescaler=1
    // 24KHz, compare match interrupt
    // Enable interrupt
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS10);
    OCR1A = (F_CPU / SAMPLE_RATE) - 1;
    TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    divider++;
    if (divider == MAX_DIVIDER) {
        divider = 0;
        if (!lfo_direction) {
            lfo_direction = (++lfo_value > LFO_UPPER - 1);
        } else {
            lfo_direction = !(--lfo_value < LFO_LOWER + 1);
        }

        // /8 + 16, 0-4096 -> 16-528
        delay = ((lfo_value >> 3) + 16);
    }
    uint8_t adc = (adc_read10() >> 2);

    // % 1024
    uint8_t delayed = samples[(wr_idx - delay) & 0x3FF];

    // /2
    uint8_t mix = (adc >> 1) + (delayed >> 1);

    samples[wr_idx] = adc + ((mix -MID_VALUE) >> 2);

    wr_idx = (wr_idx + 1) & 0x3FF;
    dac_write12(((uint16_t)mix << 4));
}