#include <avr/io.h>
#include <avr/interrupt.h>
#include "ATmega64UART.h"

// Define baud rate
#define F_CPU 11059200UL
#define BAUD 9600UL
#define MYUBRR (uint16_t)(F_CPU/(16*BAUD)-1)

volatile unsigned char value;
/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */

//Interrupt Service Routine for Receive Complete
//ISR(USART0_RX_vect){
	//value = UDR0;             //read UART register into value
	//xQueueSendToBackFromISR( QueueHandle_t xQueue, void *pvItemToQueue BaseType_t *pxHigherPriorityTaskWoken);
//}

void USART_Init(void) {
	/* Set baud rate */
	UBRR0H = (uint8_t)(MYUBRR>>8);
	UBRR0L = (uint8_t)MYUBRR;

	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
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