/*
 * BellaBaby.cpp
 * Datasheet for Attiny20: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8235-8-bit-AVR-Microcontroller-ATtiny20_Datasheet.pdf
 *
 * Pin Configuration:
 * PA0      : Use as GND for voltage divider
 * PA1/ADC1 : BatLevel, voltage divider on battery
 * PA2      : PowerGood signal from BQ24040
 * PA3      : ChargeOK signal from BQ24040
 * PA4      : RED LED anode status indicator
 * PA5      : GREEN LED anode status indicator
 * PA6      : NC
 * PA7/OC0B : AP5724 Enable pin for brightness control
 * PB0      : TPICLK, TPI Header H3-3
 * PB1      : TPIDATA, TPI Header H3-1
 * PB2      : PwrButton, pulled high, active low
 * PB3      : Reset, pulled high, TPI header H3-5
 *
 * Created: 6/21/2017 2:54:38 PM
 * Author : George
 */ 

 #define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// 8.0 MHz, calibrated internal 8 MHz oscillator

#define LED_RED PA4
#define LED_GREEN PA5
#define LED_PWM PA7
#define SWITCH PB2

ISR(PCINT0_vect) {
	if (PINB & _BV(SWITCH))
	PORTB ^= _BV(LED_GREEN);
}

void pwm_setup (void)
{
	// Set Timer 0 prescaler to clock/8.
	// At 8.0 MHz this is 1.0 MHz.
	// See ATtiny20 datasheet, Table 11.9.
	TCCR0B |= (1 << CS01);
	
	// Set to 'Fast PWM' mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	
	// Clear OC0B output on compare match, upwards counting.
	TCCR0A |= (1 << COM0B1);
}

void pwm_write (int val)
{
	OCR0B = val;  //LED Control signal on PA7
}

void adc_setup (void)
{
	// Set the ADC input to PB2/ADC1
	ADMUX |= (1 << MUX0); //BatLevel on PA1
	ADMUX |= (1 << ADLAR);

	// Set the prescaler to clock/128 & enable ADC
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
}

int adc_read (void)
{
	// Start the conversion
	ADCSRA |= (1 << ADSC);

	// Wait for it to finish - blocking
	while (ADCSRA & (1 << ADSC));

	return ADCH;
}

int main(void)
{
    int adc_in;
    
    // LED is an output.
    DDRA |= (1 << LED_PWM);
    
    adc_setup();
    pwm_setup();

	DDRA |= _BV(LED_GREEN);           // Set port PA5 as output (all others are input)
	
	PCMSK1 |= _BV(SWITCH);            // Set pin change interrupt mask to listen to port PB2
	
	MCUCR = _BV(ISC01) | _BV(ISC00);  // Set interrupt on INT0 pin falling edge (high pulled to low)
	
	GIMSK |= _BV(PCIE1);              // Enable PCINT interrupt

	sei();  	                      // Global interrupts

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    while (1) 
    {
		adc_in = adc_read(); // 1/2 battery voltage

		// Update Output Compare Register (PWM 0-100%)
		for (int i = 0; i < 255 ; i++ ){
			OCR0B = 1;
			_delay_ms(5);
		}

		// Update Output Compare Register (PWM 100%-0)
		for (int i = 255 ; i > 0 ; i-- ){
			OCR0A = i;
			_delay_ms(5);
		}
		//pwm_write(adc_in);
    }
}

