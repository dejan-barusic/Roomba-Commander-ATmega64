#include <avr/io.h>
#include <avr/interrupt.h>
#include "ATmega64UART.h"

// Define baud rate
#define F_CPU 11059200UL
#define BAUD_UART1 9600UL
#define BAUD_UART0 115200UL
#define UBRR_UART0 (uint16_t)(F_CPU/(16*BAUD_UART0)-1)
#define UBRR_UART1 (uint16_t)(F_CPU/(16*BAUD_UART1)-1)

void USART_Init(void) {
	/* Set baud rate UART0 */
	UBRR0H = (uint8_t)(UBRR_UART0>>8);
	UBRR0L = (uint8_t)UBRR_UART0;
	/* Set baud rate UART1 */
	UBRR1H = (uint8_t)(UBRR_UART1>>8);
	UBRR1L = (uint8_t)UBRR_UART1;

	/* Enable receiver and transmitter UART0 */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Enable receiver and transmitter UART1 */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);

	/* Set frame format: 8data, 2stop bit */
	//UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


void USART_SendByte(uint8_t data) {

	/* Wait for empty transmit buffer */
	//while ( !( UCSR0A & (1<<UDRE0)) );
	while(!UDRE0);

	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void USART0_SendChar(unsigned char c) {

	/* Wait for empty transmit buffer */
	while(!UDRE0);

	/* Put data into buffer, sends the data */
	UDR0 = c;
}

// Max 256 characters
void USART0_SendString(unsigned char *str) {

	for(uint8_t i = 0; str[i] != '\0'; ++i) {
		while(!( UCSR0A & (1<<UDRE0)));		// Wait for empty transmit buffer
		UDR0 = str[i];						// Put character into buffer and send
	}
}

// Wait until a byte has been received and return received data
uint8_t USART_ReceiveByte() {
	while((UCSR0A &(1<<RXC)) == 0);
	return UDR0;
}