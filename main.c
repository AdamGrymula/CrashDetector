/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *					Gdansk University of Technology, September 2014									 *
 * 																		 							 *
 * Master's thesis:	Single track vehicle crash detector alarm with GPS locator						 *
 * 																		 							 *
 * Author:			Adam Grymula, 125085, Department of Automatic Control 							 *
 * Supervisor:		dr hab. inz. Janusz Smulko, Department of Optoelectronics and Electronic Systems *
 * Consultant:		mgr inz. Sylwia Babicz, Department of Optoelectronics and Electronic Systems	 *
 * 																		 							 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <defines.h>
#include <util\delay.h>
#include <avr\io.h>
#include <avr\pgmspace.h>
#include <avr\interrupt.h>
#include <avr\iom164.h>
#include <hd44780.h>

#include <buttons.h>

// Przenieść do innego pliku -- docelowo nie może być main.h
static int test = 6;
void SetTest(int val)
{
	test = val;
}
int GetTest(void)
{
	return test;
}

volatile uint16_t ADC_CURRENT_MV_VALUES[3];
uint16_t CURRENT_MV_VALUES[3];
uint16_t NORM_MV_VALUES[3] = {1650, 1650, 1650}; // According to datasheet
volatile int16_t TEMP_MG_VALUES[3];
volatile int16_t CURRENT_MG_VALUES[3];
volatile int16_t X_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile int16_t Y_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile int16_t Z_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile int16_t NEW_X_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile int16_t NEW_Y_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile int16_t NEW_Z_MG_TRIGGER_VALUES[2] = {-100, 100};
volatile uint8_t adc_finished = 0;
volatile uint8_t adc_current_channel = 0;
volatile uint8_t adc_interrupt_counter = 0;
volatile uint8_t alarm_delay = 10;
volatile uint8_t new_alarm_delay = 10;

char IntToChar(uint8_t integer_digit)
{
	return (char)(((uint8_t)'0') + integer_digit);
}

void AdcInit(void)
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

void SetOrigin(void)
{
	// Copying 6 bytes of memory from CURRENT_MV_VALUES to NORM_MV_VALUES
	memcpy(NORM_MV_VALUES, CURRENT_MV_VALUES, 6);
}

void LcdClear(void)
{
	hd44780_outcmd(HD44780_CLR);
	hd44780_wait_ready(1);
}
void LcdInit(void)
{
	hd44780_init();
	hd44780_outcmd(HD44780_CLR);
	hd44780_wait_ready(1);
	hd44780_outcmd(HD44780_ENTMODE(1, 0));
	hd44780_wait_ready(1);
	hd44780_outcmd(HD44780_DISPCTL(1, 0, 0));
	hd44780_wait_ready(1);
}
void LcdTimerInit(void)
{
	/* TIMER2: CS22, CS21, CS20 - DIFFERENT THAN IN TIMER0! ATmega164 datasheet p.156
	 *  0     0     0   - Timer/Counter2 Disabled
	 *  0     0     1   - CLK/1
	 *  0     1     0   - CLK/8
	 *  0     1     1   - CLK/32
	 *  1     0     0   - CLK/64
	 *  1     0     1   - CLK/128
	 *  1     1     0   - CLK/256
	 *  1     1     1   - CLK/1024
	 * Fovf = Fcpu/256 (0-255); timer2 8-bit
	 * source: https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
	 */
	TCCR2B |= _BV(CS22) | _BV(CS20) | _BV(CS21);

	/* TIMSK - "Timer Interrupt Mask Register"
	 * | --- | --- | --- | --- | --- | OCIE2B | OCIE2A | TOIE2 |
	 * TOIEn  - Timer_n Overflow Interrupt Enable
	 * OCIEnx - Timer_n Output Compare Match Interrupt Enable
	 */
	TIMSK2 |= _BV(TOIE2);

	// Setting the value of timer_n (register TCNTn)
	TCNT2 = 0;
}
void LcdPutChar(char c)
{
	hd44780_outdata(c);
	hd44780_wait_ready(1);
}
void LcdGoTo(uint8_t x, uint8_t y)
{
	hd44780_outcmd(HD44780_DDADDR(0x40 * y + x)); // sends cursor to y line and x position
	hd44780_wait_ready(1);						  // first sign is on 0,0 position
}
void LcdCursorOff(void)
{
	hd44780_outcmd(HD44780_DISPCTL(1, 0, 0));
	hd44780_wait_ready(1);
}
void LcdCursorOn(uint8_t x, uint8_t y)
{
	LcdGoTo(x, y);
	hd44780_outcmd(HD44780_DISPCTL(1, 1, 1));
	hd44780_wait_ready(1);
}
void LcdPutTextP(prog_char *txt, uint8_t x, uint8_t y)
{
	LcdGoTo(x, y);
	char ch;
	while ((ch = pgm_read_byte(txt)))
	{
		LcdPutChar(ch);
		txt++;
	}
}
void LcdPutFixedPoint(int16_t mili_value, uint8_t fractional_digits)
{
	/*
	 * If mili_value lower than 0 then put sign "-"
	 * otherwise put sign "+"
	 */
	if (mili_value < 0)
		LcdPutChar('-');
	else
		LcdPutChar('+');
	/*
	 * If third fractional digit is >= 5 and one wants to show two fractional digits
	 * then increase mili_value by 10 (if mili_value <0 then decrease by 10)
	 * it provides proper values ex. 1,999 -> 2,009 shown as 2,00 (cuts last digit)
	 */
	if ((abs(mili_value) % 10) >= 5 && fractional_digits == 2 && mili_value < 0)
	{
		mili_value -= 10;
	}
	else if ((abs(mili_value) % 10) >= 5 && fractional_digits == 2 && mili_value > 0)
	{
		mili_value += 10;
	}
	/*
	 * If second fractional digit is >= 5 and one wants to show just one fractional digit
	 * then increase mili_value by 100 (if mili_value<0 then decrease)
	 * it provides proper values ex. 1,999 -> 2,099 shown as 2,0 (cuts last two digits)
	 */
	if (((abs(mili_value) / 10) % 10) >= 5 && fractional_digits != 2 && fractional_digits != 3 && mili_value < 0)
		mili_value -= 100;
	if (((abs(mili_value) / 10) % 10) >= 5 && fractional_digits != 2 && fractional_digits != 3 && mili_value > 0)
		mili_value += 100;
	/*
	 * Puts integer number and then the dot "."
	 */
	LcdPutChar(IntToChar(abs(mili_value) / 1000));
	LcdPutChar('.');
	/*
	 * Depending on how many fractional digits one can show puts one or two or three
	 * fractional digits after dot. Setting fractional_digits other than two or three
	 * gives one fractional digit
	 */
	switch (fractional_digits)
	{
	case 2:
		LcdPutChar(IntToChar((abs(mili_value) / 100) % 10));
		LcdPutChar(IntToChar((abs(mili_value) / 10) % 10));
		break;
	case 3:
		LcdPutChar(IntToChar((abs(mili_value) / 100) % 10));
		LcdPutChar(IntToChar((abs(mili_value) / 10) % 10));
		LcdPutChar(IntToChar(abs(mili_value) % 10));
		break;
	default:
		LcdPutChar(IntToChar((abs(mili_value) / 100) % 10));
		break;
	}
}

enum unit
{
	volts,
	gs
};
void LcdShowMeasurement(uint8_t unit, uint8_t fractional_digits)
{
	LcdClear();
	LcdPutTextP(PSTR("X:"), 0, 0);
	switch (unit)
	{
	case volts:
		LcdPutFixedPoint(CURRENT_MV_VALUES[0], fractional_digits);
		LcdPutTextP(PSTR("Y:"), 8, 0);
		LcdPutFixedPoint(CURRENT_MV_VALUES[1], fractional_digits);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(CURRENT_MV_VALUES[2], fractional_digits);
		LcdPutTextP(PSTR("[V]"), 11, 1);
		break;
	case gs:
		LcdPutFixedPoint(CURRENT_MG_VALUES[0], fractional_digits);
		LcdPutTextP(PSTR("Y:"), 8, 0);
		LcdPutFixedPoint(CURRENT_MG_VALUES[1], fractional_digits);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(CURRENT_MG_VALUES[2], fractional_digits);
		LcdPutTextP(PSTR("[g]"), 11, 1);
		break;
	}
}

void USART0Init(uint16_t ubrr_value)
{
	/* Set Baud rate
	 * USART Baud Rate Register -> http://www.wormfood.net/avrbaudcalc.php/
	 * ubrr_value = Fosc/16/baud-1 (round)
	 */
	UBRR0L = ubrr_value;
	UBRR0H = (ubrr_value >> 8);

	/* UCSRnC - n-th USART Control and Status Register C:
	 * UMSELn1 | UMSELn0 -> 00 for asynchronous mode; 01 for synchronous mode
	 *   UPMn1 | UPMn0	 -> 00 for non parity mode, 10 for even parity, 11 for odd parity
	 *			 USBSn	 -> 0 for 1 stop bit, 1 for 2 stop bits
	 *	UCSZn1 | UCSZn0	 -> 11 for 8-bit char size; 00 for 5-bit; 01 for 6-bit; 10 for 7-bit
	 *			 UCPOLn	 -> 0 for TX data change on rising XCK edge and RX data change on falling
	 *			 			1 for TX data change on falling XCK edge and RX data change on rising
	 */
	UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);
	/* UCSRnB - n-th USART Control and Status Register B:
	 * RXCIEn | TXCIEn -> 11 for receive|transmit completed interrupt enabled
	 * 			UDRIEn -> 1 for usart data register empty interrupt enabled
	 * 					  (!) if receiver interrupt enabled one MUST READ the UDR register
	 * 					  (!) in order to clear the interrupt flag
	 *  RXENn | TXENn  -> 1 for enabling receiving|transmitting; 0 for disabling
	 * 		    UCSZn2 -> 1 for 9-bit char size (only with UCSZn1 and UCSZn0 = 11)
	 *  RXB8n | TXB8n  -> 9-th bits of received|transmitted data if 9-bit char mode used
	 */
	UCSR0B = (1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (1 << RXEN0) | (1 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);

	/* UCSRnA - n-th USART Control and Status Register A:
	 * UCSRnA: RXCn | TXCn | UDREn | FEn | DORn | UPEn | U2Xn | MPCMn
	 *
	 * RXCn  -> receive complete; set to 1 when received data available in UDRn
	 * TXCn  -> transmission complete; set to 1 when no other data to send
	 * UDREn -> data register empty; set to 1 when UDRn is empty
	 * FEn   -> frame error; set to 1 when receive buffer has a frame error; cleared when UDR read
	 * DORn  -> data over run; set to 1 when receive buffer is full (2 characters); cleared -||-
	 * UPEn  -> parity error; set to 1 when parity error occurred; cleared when UDR read
	 * U2Xn  -> double the usart transmission speed
	 * MPCMn -> multi-processor communication mode
	 */
}
/* void USART0WriteChar(unsigned char data)
{
   //Wait until the transmitter is ready
   while(!(UCSR0A & (1<<UDRE0)))
   {
	  //Do nothing
   }
   //Now write the data to USART buffer
   UDR0 = data;
} */
volatile unsigned char usart0_received_data;
volatile unsigned char GPS_RECEIVED_DATA[90];
volatile unsigned char GPS_RMC_TIME[6];
volatile unsigned char GPS_RMC_LATITUDE[10];
volatile unsigned char GPS_RMC_LONGITUDE[11];
volatile unsigned char GPS_RMC_DATE[6];
volatile uint8_t gps_array_pointer = 0;
volatile uint8_t gps_overwrite_allowed = 1;
volatile uint8_t gps_rmc_valid = 0;
void LcdShowGPSTime(void)
{
	LcdClear();
	LcdPutTextP(PSTR("Date: "), 0, 0);
	LcdPutChar(GPS_RMC_DATE[0]);
	LcdPutChar(GPS_RMC_DATE[1]);
	LcdPutChar('.');
	LcdPutChar(GPS_RMC_DATE[2]);
	LcdPutChar(GPS_RMC_DATE[3]);
	LcdPutTextP(PSTR(".20"), 11, 0);
	LcdPutChar(GPS_RMC_DATE[4]);
	LcdPutChar(GPS_RMC_DATE[5]);
	LcdPutTextP(PSTR(" UTC: "), 0, 1);
	LcdPutChar(GPS_RMC_TIME[0]);
	LcdPutChar(GPS_RMC_TIME[1]);
	LcdPutChar(':');
	LcdPutChar(GPS_RMC_TIME[2]);
	LcdPutChar(GPS_RMC_TIME[3]);
	LcdPutChar(':');
	LcdPutChar(GPS_RMC_TIME[4]);
	LcdPutChar(GPS_RMC_TIME[5]);
}
void LcdShowGPSPosition(void)
{
	LcdClear();
	LcdPutTextP(PSTR("Lt: "), 0, 0);
	LcdPutChar(GPS_RMC_LATITUDE[0]);
	LcdPutChar(GPS_RMC_LATITUDE[1]);
	LcdPutChar(223);
	for (uint8_t i = 2; i < 9; i++)
	{
		LcdPutChar(GPS_RMC_LATITUDE[i]);
	}
	LcdPutChar(39);
	LcdPutChar(GPS_RMC_LATITUDE[9]);
	LcdPutTextP(PSTR("Ln:"), 0, 1);
	if (GPS_RMC_LONGITUDE[0] != '0')
		LcdPutChar(GPS_RMC_LONGITUDE[0]);
	else
		LcdPutChar(' ');
	LcdPutChar(GPS_RMC_LONGITUDE[1]);
	LcdPutChar(GPS_RMC_LONGITUDE[2]);
	LcdPutChar(223);
	for (uint8_t i = 3; i < 10; i++)
	{
		LcdPutChar(GPS_RMC_LONGITUDE[i]);
	}
	LcdPutChar(39);
	LcdPutChar(GPS_RMC_LONGITUDE[10]);
}

volatile uint8_t timer2_ovf_counter = 0;

void USART1Init(uint16_t ubrr_value)
{
	/* Set Baud rate
	 * USART Baud Rate Register -> http://www.wormfood.net/avrbaudcalc.php/
	 * ubrr_value = Fosc/16/baud-1 (round)
	 */
	UBRR1L = ubrr_value;
	UBRR1H = (ubrr_value >> 8);

	/* UCSRnC - n-th USART Control and Status Register C:
	 * UMSELn1 | UMSELn0 -> 00 for asynchronous mode; 01 for synchronous mode
	 *   UPMn1 | UPMn0	 -> 00 for non parity mode, 10 for even parity, 11 for odd parity
	 *			 USBSn	 -> 0 for 1 stop bit, 1 for 2 stop bits
	 *	UCSZn1 | UCSZn0	 -> 11 for 8-bit char size; 00 for 5-bit; 01 for 6-bit; 10 for 7-bit
	 *			 UCPOLn	 -> 0 for TX data change on rising XCK edge and RX data change on falling
	 *			 			1 for TX data change on falling XCK edge and RX data change on rising
	 */
	UCSR1C = (0 << UMSEL11) | (0 << UMSEL10) | (0 << UPM11) | (0 << UPM10) | (0 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10) | (0 << UCPOL1);
	/* UCSRnB - n-th USART Control and Status Register B:
	 * RXCIEn | TXCIEn -> 11 for receive|transmit completed interrupt enabled
	 * 			UDRIEn -> 1 for usart data register empty interrupt enabled
	 * 					  (!) if receiver interrupt enabled one MUST READ the UDR register
	 * 					  (!) in order to clear the interrupt flag
	 *  RXENn | TXENn  -> 1 for enabling receiving|transmitting; 0 for disabling
	 * 		    UCSZn2 -> 1 for 9-bit char size (only with UCSZn1 and UCSZn0 = 11)
	 *  RXB8n | TXB8n  -> 9-th bits of received|transmitted data if 9-bit char mode used
	 */
	UCSR1B = (0 << RXCIE1) | (0 << TXCIE1) | (0 << UDRIE1) | (1 << RXEN1) | (1 << TXEN1) | (0 << UCSZ12) | (0 << RXB81) | (0 << TXB81);

	/* UCSRnA - n-th USART Control and Status Register A:
	 * UCSRnA: RXCn | TXCn | UDREn | FEn | DORn | UPEn | U2Xn | MPCMn
	 *
	 * RXCn  -> receive complete; set to 1 when received data available in UDRn
	 * TXCn  -> transmission complete; set to 1 when no other data to send
	 * UDREn -> data register empty; set to 1 when UDRn is empty
	 * FEn   -> frame error; set to 1 when receive buffer has a frame error; cleared when UDR read
	 * DORn  -> data over run; set to 1 when receive buffer is full (2 characters); cleared -||-
	 * UPEn  -> parity error; set to 1 when parity error occurred; cleared when UDR read
	 * U2Xn  -> double the usart transmission speed
	 * MPCMn -> multi-processor communication mode
	 */
}
void USART1WriteChar(unsigned char data)
{
	// Wait until the transmitter is ready
	while (!(UCSR1A & (1 << UDRE1)))
	{
		// Do nothing
	}
	// Now write the data to USART buffer
	UDR1 = data;
}
volatile uint8_t PHONE_NUMBER[9] = {6, 6, 7, 8, 1, 8, 6, 5, 2};
volatile uint8_t NEW_PHONE_NUMBER[9] = {6, 6, 7, 8, 1, 8, 6, 5, 2};
volatile unsigned char SMS_TEXT_MODE[10] = {'A', 'T', '+', 'C', 'M', 'G', 'F', '=', '1', 13};
volatile unsigned char SMS_SET_RECEIVER_NUMBER[12] = {'A', 'T', '+', 'C', 'M', 'G', 'S', '=', '"', '+', '4', '8'};
volatile unsigned char SMS_TEST_TEXT[17] = {'C', 'u', 'r', 'r', 'e', 'n', 't', ' ', 'P', 'o', 's', 'i', 't', 'o', 'n', ':', ' '};
volatile unsigned char SMS_TEST_ALARM[19] = {'C', 'r', 'a', 's', 'h', ' ', 'd', 'e', 't', 'e', 'c', 't', 'e', 'd', ' ', 'a', 't', ':', ' '};
enum sms_type
{
	test,
	alarm
};
void SendSMS(uint8_t sms_type)
{
	cli();
	for (uint8_t i = 0; i < 10; i++)
		USART1WriteChar(SMS_TEXT_MODE[i]);
	_delay_ms(200);
	for (uint8_t i = 0; i < 12; i++)
		USART1WriteChar(SMS_SET_RECEIVER_NUMBER[i]);
	for (uint8_t i = 0; i < 9; i++)
		USART1WriteChar(IntToChar(NEW_PHONE_NUMBER[i]));
	USART1WriteChar('"');
	USART1WriteChar(13);
	_delay_ms(200);
	switch (sms_type)
	{
	case test:
		for (uint8_t i = 0; i < 17; i++)
			USART1WriteChar(SMS_TEST_TEXT[i]);
		break;

	case alarm:
		for (uint8_t i = 0; i < 19; i++)
			USART1WriteChar(SMS_TEST_ALARM[i]);
		break;
	}
	USART1WriteChar(GPS_RMC_LATITUDE[0]);
	USART1WriteChar(GPS_RMC_LATITUDE[1]);
	USART1WriteChar('d');
	USART1WriteChar('e');
	USART1WriteChar('g');
	for (uint8_t i = 2; i < 9; i++)
		USART1WriteChar(GPS_RMC_LATITUDE[i]);
	USART1WriteChar(39);
	USART1WriteChar(GPS_RMC_LATITUDE[9]);
	USART1WriteChar(' ');
	if (GPS_RMC_LONGITUDE[0] != '0')
		USART1WriteChar(GPS_RMC_LONGITUDE[0]);
	USART1WriteChar(GPS_RMC_LONGITUDE[1]);
	USART1WriteChar(GPS_RMC_LONGITUDE[2]);
	USART1WriteChar('d');
	USART1WriteChar('e');
	USART1WriteChar('g');
	for (uint8_t i = 3; i < 10; i++)
		USART1WriteChar(GPS_RMC_LONGITUDE[i]);
	USART1WriteChar(39);
	USART1WriteChar(GPS_RMC_LONGITUDE[10]);
	USART1WriteChar(26);
	sei();
}

volatile uint16_t menu_position = 100;
void LcdShowMenu(uint16_t menu)
{
	switch (menu)
	{
	case 100:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_run_detector, 1, 0);
		LcdPutTextP(txt_calibration, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 110:
		LcdClear();
		LcdCursorOff();
		LcdShowMeasurement(gs, 2);
		break;
	case 200:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_calibration, 1, 0);
		LcdPutTextP(txt_gps_data, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 210:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_set_the_origin, 1, 0);
		LcdPutTextP(txt_run_calibration, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 211:
		LcdClear();
		LcdCursorOff();
		LcdShowMeasurement(gs, 2);
		break;
	case 220:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_run_calibration, 1, 0);
		LcdPutTextP(txt_alarm_triggers, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 221:
		LcdClear();
		LcdPutTextP(txt_calibration, 0, 0);
		LcdPutTextP(txt_calibration_in_progress, 0, 1);
		LcdCursorOn(14, 1);
		break;
	case 230:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_alarm_triggers, 1, 0);
		LcdPutTextP(txt_alarm_delay, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 231:
		LcdClear();
		LcdPutTextP(PSTR("X:"), 0, 0);
		LcdPutFixedPoint(NEW_X_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_X_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Y:"), 0, 1);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(6, 0);
		break;
	case 232:
		LcdClear();
		LcdPutTextP(PSTR("X:"), 0, 0);
		LcdPutFixedPoint(NEW_X_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_X_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Y:"), 0, 1);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(14, 0);
		break;
	case 233:
		LcdClear();
		LcdPutTextP(PSTR("Y:"), 0, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(6, 0);
		break;
	case 234:
		LcdClear();
		LcdPutTextP(PSTR("Y:"), 0, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(14, 0);
		break;
	case 235:
		LcdClear();
		LcdPutTextP(PSTR("Y:"), 0, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(6, 1);
		break;
	case 236:
		LcdClear();
		LcdPutTextP(PSTR("Y:"), 0, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 0);
		LcdPutFixedPoint(NEW_Y_MG_TRIGGER_VALUES[1], 3);
		LcdPutTextP(PSTR("Z:"), 0, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[0], 3);
		LcdPutTextP(PSTR("; "), 8, 1);
		LcdPutFixedPoint(NEW_Z_MG_TRIGGER_VALUES[1], 3);
		LcdCursorOn(14, 1);
		break;
	case 240:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_alarm_triggers, 1, 0);
		LcdPutTextP(txt_alarm_delay, 1, 1);
		LcdGoTo(0, 1);
		LcdPutChar('>');
		break;
	case 241:
		LcdClear();
		if (new_alarm_delay / 100 > 0)
			LcdPutChar(IntToChar(new_alarm_delay / 100));
		else
			LcdGoTo(1, 0);
		LcdPutChar(IntToChar((new_alarm_delay / 10) % 10));
		LcdPutChar(IntToChar(new_alarm_delay % 10));
		LcdPutTextP(PSTR(" s"), 3, 0);
		LcdCursorOn(2, 0);
		break;
	case 300:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gps_data, 1, 0);
		LcdPutTextP(txt_gsm_module, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 310:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gps_data_time, 1, 0);
		LcdPutTextP(txt_gps_data_position, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 311:
		LcdClear();
		LcdCursorOff();
		if (gps_rmc_valid == 1)
			LcdShowGPSTime();
		else
			LcdPutTextP(txt_gps_data_invalid, 0, 0);
		break;
	case 320:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gps_data_time, 1, 0);
		LcdPutTextP(txt_gps_data_position, 1, 1);
		LcdGoTo(0, 1);
		LcdPutChar('>');
		break;
	case 321:
		LcdClear();
		LcdCursorOff();
		if (gps_rmc_valid == 1)
			LcdShowGPSPosition();
		else
			LcdPutTextP(txt_gps_data_invalid, 0, 0);
		break;
	case 400:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gps_data, 1, 0);
		LcdPutTextP(txt_gsm_module, 1, 1);
		LcdGoTo(0, 1);
		LcdPutChar('>');
		break;
	case 410:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gsm_alarms_receiver, 1, 0);
		LcdPutTextP(txt_gsm_send_test_sms, 1, 1);
		LcdGoTo(0, 0);
		LcdPutChar('>');
		break;
	case 411:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(0, 0);
		break;
	case 412:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(1, 0);
		break;
	case 413:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(2, 0);
		break;
	case 414:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(3, 0);
		break;
	case 415:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(4, 0);
		break;
	case 416:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(5, 0);
		break;
	case 417:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(6, 0);
		break;
	case 418:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(7, 0);
		break;
	case 419:
		LcdClear();
		for (uint8_t i = 0; i < 9; i++)
		{
			LcdPutChar(IntToChar(NEW_PHONE_NUMBER[i]));
		}
		LcdCursorOn(8, 0);
		break;
	case 420:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_gsm_alarms_receiver, 1, 0);
		LcdPutTextP(txt_gsm_send_test_sms, 1, 1);
		LcdGoTo(0, 1);
		LcdPutChar('>');
		break;
	case 421:
		LcdClear();
		LcdCursorOff();
		LcdPutTextP(txt_check_your_inbox, 0, 0);
		break;
	default:
		break;
	}
}

volatile unsigned char keys = 0x0F;
volatile uint8_t prev_key_cancell = 0;
volatile uint8_t alarm_1st_grade = 0;
volatile uint8_t alarm_2nd_grade = 0;
volatile uint8_t alarm_sent = 0;
volatile unsigned char prevkeys = 0x0F;

void Key0Pressed(void)
{
	if (menu_position == 110)
	{
		new_alarm_delay = alarm_delay;
		menu_position = 100;
	}
	else if (menu_position == 210 || menu_position == 220 || menu_position == 230 || menu_position == 240)
		menu_position = 200;
	else if (menu_position == 211 || menu_position == 421 || menu_position == 311 || menu_position == 321)
		menu_position--;
	else if (menu_position == 221 || menu_position == 231)
	{
		menu_position--;
		NEW_X_MG_TRIGGER_VALUES[0] = X_MG_TRIGGER_VALUES[0];
		NEW_X_MG_TRIGGER_VALUES[1] = X_MG_TRIGGER_VALUES[1];
		NEW_Y_MG_TRIGGER_VALUES[0] = Y_MG_TRIGGER_VALUES[0];
		NEW_Y_MG_TRIGGER_VALUES[1] = Y_MG_TRIGGER_VALUES[1];
		NEW_Z_MG_TRIGGER_VALUES[0] = Z_MG_TRIGGER_VALUES[0];
		NEW_Z_MG_TRIGGER_VALUES[1] = Z_MG_TRIGGER_VALUES[1];
	}
	else if (menu_position >= 232 && menu_position <= 236)
		menu_position--;
	else if (menu_position == 241)
	{
		menu_position--;
		new_alarm_delay = alarm_delay;
	}
	else if (menu_position == 411)
	{
		menu_position--;
		for (uint8_t i = 0; i < 9; i++)
		{
			NEW_PHONE_NUMBER[i] = PHONE_NUMBER[i];
		}
	}
	else if (menu_position >= 412 && menu_position <= 419)
		menu_position--;
	else if (menu_position == 310 || menu_position == 320)
		menu_position = 300;
	else if (menu_position == 410 || menu_position == 420)
		menu_position = 400;
	else if (menu_position == 200 || menu_position == 300 || menu_position == 400)
		;
	else
		menu_position = 100;
}
void Key0Released(void)
{
}
void Key1Pressed(void)
{
	switch (menu_position)
	{
	case 200:
		menu_position = 100;
		break;
	case 220:
		menu_position = 210;
		break;
	case 230:
		menu_position = 220;
		break;
	case 231:
		if (NEW_X_MG_TRIGGER_VALUES[0] < 3000)
			NEW_X_MG_TRIGGER_VALUES[0] += 10;
		break;
	case 232:
		if (NEW_X_MG_TRIGGER_VALUES[1] < 3000)
			NEW_X_MG_TRIGGER_VALUES[1] += 10;
		break;
	case 233:
		if (NEW_Y_MG_TRIGGER_VALUES[0] < 3000)
			NEW_Y_MG_TRIGGER_VALUES[0] += 10;
		break;
	case 234:
		if (NEW_Y_MG_TRIGGER_VALUES[1] < 3000)
			NEW_Y_MG_TRIGGER_VALUES[1] += 10;
		break;
	case 235:
		if (NEW_Z_MG_TRIGGER_VALUES[0] < 3000)
			NEW_Z_MG_TRIGGER_VALUES[0] += 10;
		break;
	case 236:
		if (NEW_Z_MG_TRIGGER_VALUES[1] < 3000)
			NEW_Z_MG_TRIGGER_VALUES[1] += 10;
		break;
	case 240:
		menu_position = 230;
		break;
	case 241:
		if (new_alarm_delay < 240)
			new_alarm_delay += 5;
		break;
	case 300:
		menu_position = 200;
		break;
	case 320:
		menu_position = 310;
		break;
	case 400:
		menu_position = 300;
		break;
	case 411:
		NEW_PHONE_NUMBER[0]++;
		NEW_PHONE_NUMBER[0] %= 10;
		break;
	case 412:
		NEW_PHONE_NUMBER[1]++;
		NEW_PHONE_NUMBER[1] %= 10;
		break;
	case 413:
		NEW_PHONE_NUMBER[2]++;
		NEW_PHONE_NUMBER[2] %= 10;
		break;
	case 414:
		NEW_PHONE_NUMBER[3]++;
		NEW_PHONE_NUMBER[3] %= 10;
		break;
	case 415:
		NEW_PHONE_NUMBER[4]++;
		NEW_PHONE_NUMBER[4] %= 10;
		break;
	case 416:
		NEW_PHONE_NUMBER[5]++;
		NEW_PHONE_NUMBER[5] %= 10;
		break;
	case 417:
		NEW_PHONE_NUMBER[6]++;
		NEW_PHONE_NUMBER[6] %= 10;
		break;
	case 418:
		NEW_PHONE_NUMBER[7]++;
		NEW_PHONE_NUMBER[7] %= 10;
		break;
	case 419:
		NEW_PHONE_NUMBER[8]++;
		NEW_PHONE_NUMBER[8] %= 10;
		break;
	case 420:
		menu_position = 410;
		break;
	}
}

void Key2Pressed(void)
{
	switch (menu_position)
	{
	case 100:
		menu_position = 200;
		break;
	case 200:
		menu_position = 300;
		break;
	case 210:
		menu_position = 220;
		break;
	case 220:
		menu_position = 230;
		break;
	case 230:
		menu_position = 240;
		break;
	case 231:
		if (NEW_X_MG_TRIGGER_VALUES[0] > -3000)
			NEW_X_MG_TRIGGER_VALUES[0] -= 10;
		break;
	case 232:
		if (NEW_X_MG_TRIGGER_VALUES[1] > -3000)
			NEW_X_MG_TRIGGER_VALUES[1] -= 10;
		break;
	case 233:
		if (NEW_Y_MG_TRIGGER_VALUES[0] > -3000)
			NEW_Y_MG_TRIGGER_VALUES[0] -= 10;
		break;
	case 234:
		if (NEW_Y_MG_TRIGGER_VALUES[1] > -3000)
			NEW_Y_MG_TRIGGER_VALUES[1] -= 10;
		break;
	case 235:
		if (NEW_Z_MG_TRIGGER_VALUES[0] > -3000)
			NEW_Z_MG_TRIGGER_VALUES[0] -= 10;
		break;
	case 236:
		if (NEW_Z_MG_TRIGGER_VALUES[1] > -3000)
			NEW_Z_MG_TRIGGER_VALUES[1] -= 10;
		break;
	case 241:
		if (new_alarm_delay >= 5)
			new_alarm_delay -= 5;
		break;
	case 300:
		menu_position = 400;
		break;
	case 310:
		menu_position = 320;
		break;
	case 410:
		menu_position = 420;
		break;
	case 411:
		if (NEW_PHONE_NUMBER[0] == 0)
			NEW_PHONE_NUMBER[0] = 9;
		else
			NEW_PHONE_NUMBER[0]--;
		break;
	case 412:
		if (NEW_PHONE_NUMBER[1] == 0)
			NEW_PHONE_NUMBER[1] = 9;
		else
			NEW_PHONE_NUMBER[1]--;
		break;
	case 413:
		if (NEW_PHONE_NUMBER[2] == 0)
			NEW_PHONE_NUMBER[2] = 9;
		else
			NEW_PHONE_NUMBER[2]--;
		break;
	case 414:
		if (NEW_PHONE_NUMBER[3] == 0)
			NEW_PHONE_NUMBER[3] = 9;
		else
			NEW_PHONE_NUMBER[3]--;
		break;
	case 415:
		if (NEW_PHONE_NUMBER[4] == 0)
			NEW_PHONE_NUMBER[4] = 9;
		else
			NEW_PHONE_NUMBER[4]--;
		break;
	case 416:
		if (NEW_PHONE_NUMBER[5] == 0)
			NEW_PHONE_NUMBER[5] = 9;
		else
			NEW_PHONE_NUMBER[5]--;
		break;
	case 417:
		if (NEW_PHONE_NUMBER[6] == 0)
			NEW_PHONE_NUMBER[6] = 9;
		else
			NEW_PHONE_NUMBER[6]--;
		break;
	case 418:
		if (NEW_PHONE_NUMBER[7] == 0)
			NEW_PHONE_NUMBER[7] = 9;
		else
			NEW_PHONE_NUMBER[7]--;
		break;
	case 419:
		if (NEW_PHONE_NUMBER[8] == 0)
			NEW_PHONE_NUMBER[8] = 9;
		else
			NEW_PHONE_NUMBER[8]--;
		break;
	}
}
void Key2Released(void)
{
}
void Key3Pressed(void)
{
	if (menu_position == 100 || menu_position == 200 || menu_position == 300 || menu_position == 400)
		menu_position += 10;
	else if (menu_position == 210 || menu_position == 220 || (menu_position >= 230 && menu_position <= 235) || menu_position == 240 || menu_position == 310 || menu_position == 320 || (menu_position >= 410 && menu_position <= 418))
		menu_position++;
	else if (menu_position == 211)
		SetOrigin();
	else if (menu_position == 221)
	{
		menu_position--;
		X_MG_TRIGGER_VALUES[0] = NEW_X_MG_TRIGGER_VALUES[0];
		X_MG_TRIGGER_VALUES[1] = NEW_X_MG_TRIGGER_VALUES[1];
		Y_MG_TRIGGER_VALUES[0] = NEW_Y_MG_TRIGGER_VALUES[0];
		Y_MG_TRIGGER_VALUES[1] = NEW_Y_MG_TRIGGER_VALUES[1];
		Z_MG_TRIGGER_VALUES[0] = NEW_Z_MG_TRIGGER_VALUES[0];
		Z_MG_TRIGGER_VALUES[1] = NEW_Z_MG_TRIGGER_VALUES[1];
	}
	else if (menu_position == 236)
	{
		menu_position = 230;
		X_MG_TRIGGER_VALUES[0] = NEW_X_MG_TRIGGER_VALUES[0];
		X_MG_TRIGGER_VALUES[1] = NEW_X_MG_TRIGGER_VALUES[1];
		Y_MG_TRIGGER_VALUES[0] = NEW_Y_MG_TRIGGER_VALUES[0];
		Y_MG_TRIGGER_VALUES[1] = NEW_Y_MG_TRIGGER_VALUES[1];
		Z_MG_TRIGGER_VALUES[0] = NEW_Z_MG_TRIGGER_VALUES[0];
		Z_MG_TRIGGER_VALUES[1] = NEW_Z_MG_TRIGGER_VALUES[1];
	}
	else if (menu_position == 241)
	{
		menu_position = 240;
		alarm_delay = new_alarm_delay;
	}
	else if (menu_position == 419)
	{
		menu_position = 410;
		for (uint8_t i = 0; i < 9; i++)
		{
			PHONE_NUMBER[i] = NEW_PHONE_NUMBER[i];
		}
	}
	else if (menu_position == 420)
	{
		SendSMS(test);
		menu_position = 421;
	}
}
void Key3Released(void)
{
}
void KeyCancellPressed(void)
{
	if (menu_position >= 231 && menu_position <= 236)
	{
		NEW_X_MG_TRIGGER_VALUES[0] = -100;
		NEW_X_MG_TRIGGER_VALUES[1] = 100;
		NEW_Y_MG_TRIGGER_VALUES[0] = -100;
		NEW_Y_MG_TRIGGER_VALUES[1] = 100;
		NEW_Z_MG_TRIGGER_VALUES[0] = -100;
		NEW_Z_MG_TRIGGER_VALUES[1] = 100;
	}
	alarm_1st_grade = 0;
	alarm_2nd_grade = 0;
	alarm_sent = 0;
	new_alarm_delay = alarm_delay;
}
void ReadKeys(void)
{
	keys = KEYPORT;
}
void KeysHandle(void)
{
	// KEY 0
	if (bit_is_clear(keys, 0) && bit_is_set(prevkeys, 0))
	{
		prevkeys &= ~_BV(0);
		Key0Pressed();
	}
	if (bit_is_set(keys, 0) && bit_is_clear(prevkeys, 0))
	{
		prevkeys |= _BV(0);
		Key0Released();
	}
	// KEY 1
	if (bit_is_clear(keys, 1) && bit_is_set(prevkeys, 1))
	{
		prevkeys &= ~_BV(1);
		Key1Pressed();
	}
	if (bit_is_set(keys, 1) && bit_is_clear(prevkeys, 1))
	{
		prevkeys |= _BV(1);
		Key1Released();
	}
	// KEY 2
	if (bit_is_clear(keys, 2) && bit_is_set(prevkeys, 2))
	{
		prevkeys &= ~_BV(2);
		Key2Pressed();
	}
	if (bit_is_set(keys, 2) && bit_is_clear(prevkeys, 2))
	{
		prevkeys |= _BV(2);
		Key2Released();
	}
	// KEY 3
	if (bit_is_clear(keys, 3) && bit_is_set(prevkeys, 3))
	{
		prevkeys &= ~_BV(3);
		Key3Pressed();
	}
	if (bit_is_set(keys, 3) && bit_is_clear(prevkeys, 3))
	{
		prevkeys |= _BV(3);
		Key3Released();
	}
	// KEY CANCELL
	if (bit_is_clear(PIND, 5) && (prev_key_cancell == 0))
	{
		prev_key_cancell = 1;
		KeyCancellPressed();
	}
	if (bit_is_set(PIND, 5) && (prev_key_cancell == 1))
	{
		prev_key_cancell = 0;
	}
}

int main(void)
{
	Key1Released();
	DDRA = 0x00;
	PORTA |= 0xF0;
	DDRD &= ~_BV(5);
	PORTD |= _BV(5);
	DDRD |= _BV(6);
	sei();
	USART0Init(0x0047);
	USART1Init(0x0005);
	AdcInit();
	LcdInit();
	LcdPutTextP(txt_hello_line0, 3, 0);
	LcdPutTextP(txt_hello_line1, 1, 1);
	_delay_ms(1000);
	while (adc_finished == 0)
	{
		// wait for first full measurement
	}
	CURRENT_MV_VALUES[0] = (uint32_t)ADC_CURRENT_MV_VALUES[0] * VREF / 1024;
	CURRENT_MV_VALUES[1] = (uint32_t)ADC_CURRENT_MV_VALUES[1] * VREF / 1024;
	CURRENT_MV_VALUES[2] = (uint32_t)ADC_CURRENT_MV_VALUES[2] * VREF / 1024;
	NORM_MV_VALUES[0] = CURRENT_MV_VALUES[0];
	NORM_MV_VALUES[1] = CURRENT_MV_VALUES[1];
	NORM_MV_VALUES[2] = CURRENT_MV_VALUES[2];
	adc_finished = 0;
	LcdTimerInit();
	while (1)
	{
		KeysHandle();
		if (adc_finished == 1)
		{
			cli();
			CURRENT_MV_VALUES[0] = (uint32_t)ADC_CURRENT_MV_VALUES[0] * VREF / 1024;
			CURRENT_MV_VALUES[1] = (uint32_t)ADC_CURRENT_MV_VALUES[1] * VREF / 1024;
			CURRENT_MV_VALUES[2] = (uint32_t)ADC_CURRENT_MV_VALUES[2] * VREF / 1024;
			TEMP_MG_VALUES[0] = ((int16_t)CURRENT_MV_VALUES[0] - (int16_t)NORM_MV_VALUES[0]) * 10 / 8;
			TEMP_MG_VALUES[1] = ((int16_t)CURRENT_MV_VALUES[1] - (int16_t)NORM_MV_VALUES[1]) * 10 / 8;
			TEMP_MG_VALUES[2] = ((int16_t)CURRENT_MV_VALUES[2] - (int16_t)NORM_MV_VALUES[2]) * 10 / 8;
			sei();
			adc_finished = 0;
		}
		if (((TEMP_MG_VALUES[0] - CURRENT_MG_VALUES[0]) < 100) || ((CURRENT_MG_VALUES[0] - TEMP_MG_VALUES[0]) < 100))
		{
			CURRENT_MG_VALUES[0] = TEMP_MG_VALUES[0];
		}
		if (((TEMP_MG_VALUES[1] - CURRENT_MG_VALUES[1]) < 100) || ((CURRENT_MG_VALUES[1] - TEMP_MG_VALUES[1]) < 100))
		{
			CURRENT_MG_VALUES[1] = TEMP_MG_VALUES[1];
		}
		if (((TEMP_MG_VALUES[2] - CURRENT_MG_VALUES[2]) < 100) || ((CURRENT_MG_VALUES[2] - TEMP_MG_VALUES[2]) < 100))
		{
			CURRENT_MG_VALUES[2] = TEMP_MG_VALUES[2];
		}
		if (gps_overwrite_allowed == 0)
		{
			if (GPS_RECEIVED_DATA[18] == 'A')
			{
				gps_rmc_valid = 1;
				for (uint8_t i = 7; i < 13; i++)
					GPS_RMC_TIME[i - 7] = GPS_RECEIVED_DATA[i];
				for (uint8_t i = 20; i < 29; i++)
					GPS_RMC_LATITUDE[i - 20] = GPS_RECEIVED_DATA[i];
				GPS_RMC_LATITUDE[9] = GPS_RECEIVED_DATA[30];
				for (uint8_t i = 32; i < 42; i++)
					GPS_RMC_LONGITUDE[i - 32] = GPS_RECEIVED_DATA[i];
				GPS_RMC_LONGITUDE[10] = GPS_RECEIVED_DATA[43];
				if (GPS_RECEIVED_DATA[55] == ',')
				{
					for (uint8_t i = 56; i < 62; i++)
						GPS_RMC_DATE[i - 56] = GPS_RECEIVED_DATA[i];
				}
				else if (GPS_RECEIVED_DATA[56] == ',')
				{
					for (uint8_t i = 57; i < 63; i++)
						GPS_RMC_DATE[i - 57] = GPS_RECEIVED_DATA[i];
				}
			}
			else
				gps_rmc_valid = 0;
			gps_overwrite_allowed = 1;
		}
		if ((alarm_2nd_grade == 1) && (alarm_sent == 0))
		{
			SendSMS(alarm);
			alarm_sent = 1;
		}
		if (menu_position == 110)
		{
			if (((CURRENT_MG_VALUES[0] < NEW_X_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[0] > -3000)) || ((CURRENT_MG_VALUES[0] > NEW_X_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[0] < 3000)) || ((CURRENT_MG_VALUES[1] < NEW_Y_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[1] > -3000)) || ((CURRENT_MG_VALUES[1] > NEW_Y_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[1] < 3000)) || ((CURRENT_MG_VALUES[2] < NEW_Z_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[2] > -3000)) || ((CURRENT_MG_VALUES[2] > NEW_Z_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[2] < 3000)))
				alarm_1st_grade = 1;
			if ((alarm_1st_grade == 1) && (new_alarm_delay == 0))
			{
				alarm_2nd_grade = 1;
				alarm_1st_grade = 0;
			}
		}
		else
		{
			alarm_1st_grade = 0;
			alarm_2nd_grade = 0;
			alarm_sent = 0;
		}
		if (menu_position == 221)
		{
			if ((CURRENT_MG_VALUES[0] < NEW_X_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[0] > -3000))
			{
				NEW_X_MG_TRIGGER_VALUES[0] = CURRENT_MG_VALUES[0];
			}
			if ((CURRENT_MG_VALUES[0] > NEW_X_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[0] < 3000))
			{
				NEW_X_MG_TRIGGER_VALUES[1] = CURRENT_MG_VALUES[0];
			}
			if ((CURRENT_MG_VALUES[1] < NEW_Y_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[1] > -3000))
			{
				NEW_Y_MG_TRIGGER_VALUES[0] = CURRENT_MG_VALUES[1];
			}
			if ((CURRENT_MG_VALUES[1] > NEW_Y_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[1] < 3000))
			{
				NEW_Y_MG_TRIGGER_VALUES[1] = CURRENT_MG_VALUES[1];
			}
			if ((CURRENT_MG_VALUES[2] < NEW_Z_MG_TRIGGER_VALUES[0]) && (CURRENT_MG_VALUES[2] > -3000))
			{
				NEW_Z_MG_TRIGGER_VALUES[0] = CURRENT_MG_VALUES[2];
			}
			if ((CURRENT_MG_VALUES[2] > NEW_Z_MG_TRIGGER_VALUES[1]) && (CURRENT_MG_VALUES[2] < 3000))
			{
				NEW_Z_MG_TRIGGER_VALUES[1] = CURRENT_MG_VALUES[2];
			}
		}
	}
}

ISR(ADC_vect)
{
	// Interrupt occurs 11059200/128 = 86400 times per second
	// Interrupt counter reduce measurement by 25 so it goes 3456 times per second (1152 for each channel)
	uint16_t adc_reading = ADC;
	adc_interrupt_counter++;
	adc_interrupt_counter %= 25;
	if (adc_interrupt_counter == 0)
	{
		if (adc_finished == 1)
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

ISR(TIMER2_OVF_vect)
{
	// Timer2 overflows 42,19 times per second
	ReadKeys();
	timer2_ovf_counter++;
	timer2_ovf_counter %= 42;
	if (timer2_ovf_counter % 10 == 3)
	{
		LcdShowMenu(menu_position);
		if (alarm_1st_grade == 1)
			PORTD ^= _BV(6);
		else if (alarm_2nd_grade == 1)
			PORTD |= _BV(6);
		else
			PORTD &= ~_BV(6);
	}
	if (timer2_ovf_counter == 0)
	{
		if ((alarm_1st_grade == 1) && (new_alarm_delay != 0))
			new_alarm_delay--;
	}
}

ISR(USART0_RX_vect)
{
	usart0_received_data = UDR0;
	if (gps_overwrite_allowed == 1)
	{
		if (usart0_received_data == '$')
			gps_array_pointer = 0;
		else
			gps_array_pointer++;
		GPS_RECEIVED_DATA[gps_array_pointer] = usart0_received_data;
		if (usart0_received_data == '*' && GPS_RECEIVED_DATA[3] == 'R' && GPS_RECEIVED_DATA[4] == 'M' && GPS_RECEIVED_DATA[5] == 'C')
			gps_overwrite_allowed = 0;
	}
}
