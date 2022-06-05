#include <avr\io.h>
#include <avr\interrupt.h>
#include <defines.h>

volatile static unsigned char usart0_received_byte;
volatile static uint8_t gps_array_pointer = 0;
volatile static uint8_t gps_overwrite_allowed = TRUE;
volatile static unsigned char GPS_RECEIVED_DATA[GPS_DATA_LEN];

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

void USART0WriteChar(unsigned char data)
{
   //Wait until the transmitter is ready
   while(!(UCSR0A & (1<<UDRE0)))
   {
	  //Do nothing
   }
   //Now write the data to USART buffer
   UDR0 = data;
}
void GpsInit(void)
{
    USART0Init(0x0047);
}

uint8_t CheckGps(void)
{
	if (gps_overwrite_allowed == FALSE)
	{
		if (GPS_RECEIVED_DATA[18] == 'A')
		{
			return TRUE;
		}
		else
		{
			gps_overwrite_allowed = TRUE;
			return FALSE;
		}
	}
}

void GetGpsTime(unsigned char ARRAY[])
{
	for (uint8_t i = 0; i < RMC_TIME_LEN; i++)
	{
		ARRAY[i] = GPS_RECEIVED_DATA[i+7];
	}
}

void GetGpsDate(unsigned char ARRAY[])
{
	if (GPS_RECEIVED_DATA[55] == ',')
	{
		for (uint8_t i = 0; i < RMC_DATE_LEN; i++)
			ARRAY[i] = GPS_RECEIVED_DATA[i+56];
	}
	else if (GPS_RECEIVED_DATA[56] == ',')
	{
		for (uint8_t i = 0; i < RMC_DATE_LEN; i++)
			ARRAY[i] = GPS_RECEIVED_DATA[i+57];
	}
}

void GetGpsLatitude(unsigned char ARRAY[])
{
	for (uint8_t i = 0; i < (RMC_LATITUDE_LEN-1); i++)
	{
		ARRAY[i] = GPS_RECEIVED_DATA[i+20];
		ARRAY[RMC_LATITUDE_LEN-1] = GPS_RECEIVED_DATA[30];
	}
}

void GetGpsLongitude(unsigned char ARRAY[])
{
	for (uint8_t i = 0; i < (RMC_LONGITUDE_LEN-1); i++)
	{
		ARRAY[i] = GPS_RECEIVED_DATA[i+32];
		ARRAY[RMC_LONGITUDE_LEN-1] = GPS_RECEIVED_DATA[43];
	}
}

// Interrupt handler when data is received at USART0
ISR(USART0_RX_vect)
{
	usart0_received_byte = UDR0;
	if (gps_overwrite_allowed == TRUE)
	{
		if (usart0_received_byte == '$')
			gps_array_pointer = 0;
		else
			gps_array_pointer++;
			GPS_RECEIVED_DATA[gps_array_pointer] = usart0_received_byte;

		if (usart0_received_byte == '*' && GPS_RECEIVED_DATA[3] == 'R' && GPS_RECEIVED_DATA[4] == 'M' && GPS_RECEIVED_DATA[5] == 'C')
			gps_overwrite_allowed = FALSE;
	}
}