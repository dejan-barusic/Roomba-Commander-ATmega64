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
static void vCyclicalTask( void *pvParameters );
void vApplicationIdleHook( void );
static void sendMessage( unsigned char *message, QueueHandle_t queue);
static void sendCharacter( unsigned char character, QueueHandle_t queue);
static void flushQueue( QueueHandle_t queue);


/* -- UART0: SERIAL; UART1: BLUETOOTH -- */

/* Buffers for UART communication. */
QueueHandle_t bufferSendUART0Serial, bufferReceiveUART0Serial, bufferSendUART1Bluetooth, bufferReceiveUART1Bluetooth;

/* Storage of values for Roomba sensors and sensors on MCU pins. */
volatile uint8_t sensorData[27];


int main( void ) {

	bufferSendUART0Serial = xQueueCreate(32, sizeof(unsigned char));
	bufferReceiveUART0Serial = xQueueCreate(32, sizeof(unsigned char));
	bufferSendUART1Bluetooth = xQueueCreate(32, sizeof(unsigned char));
	bufferReceiveUART1Bluetooth = xQueueCreate(32, sizeof(unsigned char));

	DDRB |= (1 << PB0);
	DDRB |= (1 << PB1);

	USART_Init();

	/* Enable interrupts */
	UCSR0B |= (1 << RXCIE0);
	UCSR1B |= (1 << RXCIE1);
	sei();

	/* Create the tasks defined within this file. */
	xTaskCreate( vRedLED, "RedLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	//xTaskCreate( vGreenLED, "GreenLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	xTaskCreate( vUART0TransmitService, "UART0Tx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vUART1TransmitService, "UART1Tx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	//xTaskCreate( vUART0ReceiveService, "UART0Rx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vUART1ReceiveService, "UART1Rx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );
	xTaskCreate( vCyclicalTask, "Cyclical", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );

	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (1 << PB0);
		/* Periodically print sensor values for testing. */
		sendMessage( "Sensors: ", bufferSendUART1Bluetooth );
		sendMessage( sensorData, bufferSendUART1Bluetooth );
		sendCharacter( '\r', bufferSendUART1Bluetooth );
		sendCharacter( '\n', bufferSendUART1Bluetooth );
		vTaskDelay( pdMS_TO_TICKS(3000) );
	}
}

static void vGreenLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (1 << PB1);
		vTaskDelay( pdMS_TO_TICKS(1000) );
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

static void vCyclicalTask( void *pvParameters ) {
	/* Service for cyclical operations such as:
	    - Readout of Roomba sensors via serial
		- Polling of sensors on MCU pins
		- ...
	*/

	/* General purpose variables. */
	uint8_t buffer;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
		/*** READOUT OF ROOMBA SENSORS ***/

		/* Clear Roomba receiving buffer. */
		flushQueue(bufferReceiveUART0Serial);

		/* Send request to Roomba for sensor data (2 bytes). */
		buffer = SCI_SENSORS;
		xQueueSendToBack( bufferSendUART0Serial, &buffer, pdMS_TO_TICKS(10) );
		buffer = SCI_SENSORS_PACKET0;
		xQueueSendToBack( bufferSendUART0Serial, &buffer, pdMS_TO_TICKS(10) );

		/* Receive 26 bytes of sensor data from Roomba. */
		for( uint8_t i = 0; i < 26; ++i ) {
			if( errQUEUE_EMPTY == xQueueReceive( bufferReceiveUART0Serial, (sensorData + i), pdMS_TO_TICKS(50) )) {
				// Roomba not ready. Exit loop.
				break;
			}
		}

		/*** POLLING SENSORS ON MCU PINS ***/



		/* Cyclical delay. */
		vTaskDelay( pdMS_TO_TICKS(3000) );
	}
}

static void vSendCommandToRoomba( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
	}
}


static void sendMessage( unsigned char *message, QueueHandle_t queue) {
	/* Receives a message and pushes it on UART send queue character by character.
	   Limited to 255 characters for efficiency (counter i takes one byte). 
	   Message must be null terminated. */
	for(uint8_t i = 0; message[i] != '\0'; ++i) {
		xQueueSendToBack( queue, message+i, pdMS_TO_TICKS(10));
	}
}

static void sendCharacter( unsigned char character, QueueHandle_t queue) {
	/* Receives a character and pushes it on UART send queue. */
	xQueueSendToBack( queue, &character, pdMS_TO_TICKS(10));
}

static void flushQueue( QueueHandle_t queue) {
	/* Flushes out any data left on queue. */
	for(uint8_t i = uxQueueMessagesWaiting(queue); i > 0; ++i) {
		xQueueReceive( queue, NULL, 0 );
	}
}

void vApplicationIdleHook( void ) {
	//PORTB ^= (1 << PB1);
}

ISR(USART0_RX_vect) {
	/* UART0 receive complete ISR. 
	   Pushes received byte from UDR0 register on bufferReceiveUART0Serial queue. */
	xQueueSendToBackFromISR( bufferReceiveUART0Serial, &UDR0, NULL);
}

ISR(USART1_RX_vect) {
/* UART1 receive complete ISR. 
	   Pushes received byte from UDR1 register on bufferReceiveUART1Bluetooth queue. */
	xQueueSendToBackFromISR( bufferReceiveUART1Bluetooth, &UDR1, NULL);
}

/*-----------------------------------------------------------*/

