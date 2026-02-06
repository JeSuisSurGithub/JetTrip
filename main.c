#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdint.h>

#define PORT_DAC_CS PB1
#define PORT_ADC_CS PB2
#define PORT_SCK PB5
#define PORT_MOSI PB3

#define SAMPLE_RATE 24000 // 32000
#define SAMPLE_COUNT 1024
#define SAMPLE_MID_VAL 128
#define SAMPLE_MIN_DELAY 16 // 32 // 256

#define LFO_UPPER 4095
#define LFO_LOWER 0
#define LFO_DIVIDER 16 // 8 // 24

volatile uint8_t samples[SAMPLE_COUNT];
volatile uint16_t wr_idx = 0;
volatile int16_t delay = 0;

// f_lfo = f_sample / 2 (up+down ramp) * divider * (lfo_up-lf_low)
// f_sample=24K, divider=16, (lfo_up-lfo_low)=4095, => 0.183Hz
volatile uint16_t lfo_value = 0;
volatile bool lfo_direction = 0;
volatile uint8_t lfo_divider = 0;

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
int main()
{
    for (int16_t i = 0; i < SAMPLE_COUNT; i++) { samples[i] = 128; }

    // PB2 needs to be set as output to have master spi
    // DDRB |= (1 << PB2);
    // PORTB |= (1 << PB2);

    // Enable CS MOSI SCK
    // Enable SPI in Master mode
    // 8MHz -> 4MHz
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

    sei();
    while (true) {
        _delay_ms(10);
    }
}

ISR(TIMER1_COMPA_vect)
{
    lfo_divider++;
    if (lfo_divider == LFO_DIVIDER) {
        lfo_divider = 0;

        if (!lfo_direction) {
            lfo_direction = (++lfo_value > LFO_UPPER - 1);
        } else {
            lfo_direction = !(--lfo_value < LFO_LOWER + 1);
        }

        // /8 + 16, 0-4096 -> 16-528
        delay = ((lfo_value >> 3) + SAMPLE_MIN_DELAY);
    }
    uint8_t adc = (adc_read10() >> 2);

    // % 1024
    uint8_t delayed = samples[(wr_idx - delay) & 0x3FF];

    // /2
    uint8_t mix = (adc >> 1) + (delayed >> 1);

    // ADC + Ac(Mix) / 4
    samples[wr_idx] = adc + ((mix - SAMPLE_MID_VAL) >> 2);

    wr_idx = (wr_idx + 1) & 0x3FF;
    dac_write12(((uint16_t)mix << 4));
}
