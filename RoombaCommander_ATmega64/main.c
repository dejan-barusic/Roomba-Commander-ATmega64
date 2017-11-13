#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>


#ifdef GCC_MEGA_AVR
	/* EEPROM routines used only with the WinAVR compiler. */
	#include <avr/eeprom.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ATmega64UART.h"
#include "message.h"
#include "msgHeaders.h"
#include "RoombaSCI.h"


/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainLED_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainUART_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
//#define mainCOM_BAUD_RATE			( ( unsigned long ) 9600 )
#define CPU_Z 11059200UL

/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters );
static void vGreenLED( void *pvParameters );
static void vUART0TransmitService( void *pvParameters );
static void vUART1TransmitService( void *pvParameters );
static void vUART0ReceiveService( void *pvParameters );
static void vUART1ReceiveService( void *pvParameters );
void vApplicationIdleHook( void );
static void sendMessage( unsigned char *message, QueueHandle_t queue);

unsigned char message1[] = "Hello! ";
unsigned char message2[32];

/* -- UART0: SERIAL; UART1: BLUETOOTH -- */

QueueHandle_t bufferSendUART0Serial, bufferReceiveUART0Serial, bufferSendUART1Bluetooth, bufferReceiveUART1Bluetooth;
Message msg;

int main( void ) {

	bufferSendUART0Serial = xQueueCreate(16, sizeof(unsigned char));
	bufferReceiveUART0Serial = xQueueCreate(16, sizeof(unsigned char));
	bufferSendUART1Bluetooth = xQueueCreate(16, sizeof(unsigned char));
	bufferReceiveUART1Bluetooth = xQueueCreate(16, sizeof(unsigned char));

	DDRB |= (1 << PB0);
	DDRB |= (1 << PB1);


	USART_Init();

	/* Enable interrupts */
	UCSR0B |= (1 << RXCIE0);
	UCSR1B |= (1 << RXCIE1);
	sei();

	makeMessage(&msg, TEXT, '1', message1);
	getMessage(&msg, message2);

	/* Create the tasks defined within this file. */
	xTaskCreate( vRedLED, "RedLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	//xTaskCreate( vGreenLED, "GreenLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	xTaskCreate( vUART0TransmitService, "UART0Tx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vUART1TransmitService, "UART1Tx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vUART0ReceiveService, "UART0Rx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vUART1ReceiveService, "UART1Rx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );

	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (1 << PB0);
		sendMessage(message2, bufferSendUART0Serial);
		PORTB ^= (1 << PB0);
		vTaskDelay( 400 );
	}
}

static void vGreenLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		//PORTB ^= (1 << PB1);
		vTaskDelay( 50 );
	}
}

static void vUART0TransmitService( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART0 to fill, then pops first byte of data and
		   copies it to UART0 data register UDR0 effectively transmitting it. */ 
		xQueueReceive( bufferSendUART0Serial, &UDR0, portMAX_DELAY );
		while(!( UCSR0A & (1<<UDRE0)));		// Wait for transmit to finish
	}
}

static void vUART1TransmitService( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART1 to fill, then pops first byte of data and
		   copies it to UART1 data register UDR1 effectively transmitting it. */ 
		xQueueReceive( bufferSendUART1Bluetooth, &UDR1, portMAX_DELAY );
		while(!( UCSR1A & (1<<UDRE1)));		// Wait for transmit to finish
	}
}

static void vUART0ReceiveService( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferReceiveUART0 to fill, then pops first byte of data and
		   copies it to UART0 data register UDR0 effectively transmitting it. 
		   Test code - writes back to terminal. */ 
		xQueueReceive( bufferReceiveUART0Serial, &UDR0, portMAX_DELAY );
		while(!( UCSR0A & (1<<UDRE0)));		// Wait for transmit to finish
	}
}

static void vUART1ReceiveService( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferReceiveUART1 to fill, then pops first byte of data and
		   copies it to UART1 data register UDR1 effectively transmitting it. 
		   Test code - writes back to terminal. */ 
		xQueueReceive( bufferReceiveUART1Bluetooth, &UDR1, portMAX_DELAY );
		while(!( UCSR1A & (1<<UDRE1)));		// Wait for transmit to finish
	}
}

static void sendMessage( unsigned char *message, QueueHandle_t queue) {
	/* Receives a message and pushes it on UART send queue character by character.
	   Limited to 255 characters for efficiency (counter i takes one byte). 
	   Message must be null terminated. */
	for(uint8_t i = 0; message[i] != '\0'; ++i) {
		xQueueSendToBack( queue, message+i, pdMS_TO_TICKS(50));
	}
}

void vApplicationIdleHook( void ) {
	PORTB ^= (1 << PB1);
}

ISR(USART0_RX_vect) {
	xQueueSendToBackFromISR( bufferReceiveUART0Serial, &UDR0, NULL);
}

ISR(USART1_RX_vect) {
	xQueueSendToBackFromISR( bufferReceiveUART1Bluetooth, &UDR1, NULL);
}

/*-----------------------------------------------------------*/

