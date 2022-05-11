#include <avr\io.h>
#include <avr\interrupt.h>
#include <defines.h>

volatile static uint16_t adc_reading = 0;
volatile static uint8_t adc_current_channel = 0;
volatile static uint8_t adc_interrupt_counter = 0;
volatile static uint8_t adc_finished = FALSE;               // TODO: Remove from main
volatile static uint16_t ADC_CURRENT_MV_VALUES[AXES_NUM];


void InitAdc(void)
{
	ADMUX = 0;
	ADMUX &= ~_BV(REFS1) & ~_BV(REFS0); // Reference voltage from AREF pin
	ADMUX &= ~_BV(MUX3) & ~_BV(MUX4);	// ADC multiplexer, channel choice
	ADCSRA = _BV(ADIE) | _BV(ADATE);	// Free running mode; interrupt enabled
	ADCSRA |= _BV(ADEN);				// ADC enabled
	ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);
	// ADC sampling prescaler ATmega8 Datasheet p.208
	// x = ADPS2|ADPS1|ADPS0 -> 2^x (exception: 000 -> 2)
	// ADMUX |= _BV(ADLAR);					// ADC Left Adjust Result: 8 MSB of 10 measured -> ADCH
	ADCSRA |= _BV(ADSC); // Free running mode, but needed to first convertion
	DIDR0 = 0x0F;		 // Digital Input Disable Register - PA0-PA4 DI Disabled
}

void GetAdcMvValues(uint16_t ARRAY[])
{
    for (uint8_t i = 0; i < AXES_NUM; i++)
	{
		ARRAY[i] = (uint32_t)ADC_CURRENT_MV_VALUES[i] * VREF / 1024;
	}
}

// Interrupt handler on ADC data received
ISR(ADC_vect)
{
	// Interrupt occurs 11059200/128 = 86400 times per second
	// Interrupt counter reduce measurement by 25 so it goes 3456 times per second (1152 for each channel)
	adc_reading = ADC;
	adc_interrupt_counter++;
	adc_interrupt_counter %= 25;
	if (adc_interrupt_counter == 0)
	{
		if (adc_finished == TRUE)
		{
			return;
		}
		else
		{
			ADC_CURRENT_MV_VALUES[adc_current_channel] = adc_reading;
			switch (adc_current_channel)
			{
			case 0:
				// Next measurement on channel 1
				ADMUX = 0b00000001;
				adc_current_channel = 1;
				break;
			case 1:
				// Next measurement on channel 2
				ADMUX = 0b00000010;
				adc_current_channel = 2;
				break;
			case 2:
				// Next measurement on channel 0; all channels complete
				ADMUX = 0b00000000;
				adc_current_channel = 0;
				adc_finished = 1;
				break;
			}
		}
	}
}