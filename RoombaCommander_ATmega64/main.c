#include <stdlib.h>
#include <string.h>

#ifdef GCC_MEGA_AVR
	/* EEPROM routines used only with the WinAVR compiler. */
	#include <avr/eeprom.h>
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ATmega64UART.h"


/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainLED_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainUART_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_BAUD_RATE			( ( unsigned long ) 9600 )
#define CPU_Z 11059200UL

/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters );
static void vGreenLED( void *pvParameters );
static void vUART0TransmitService( void *pvParameters );
void vApplicationIdleHook( void );
void sendMessage( unsigned char *message, QueueHandle_t queue);

unsigned char message[] = "Hello! ";
QueueHandle_t bufferSendUART0;
QueueHandle_t bufferReceiveUART0;

int main( void ) {

	bufferSendUART0 = xQueueCreate(16, sizeof(unsigned char));
	bufferReceiveUART0 = xQueueCreate(16, sizeof(unsigned char));

	DDRB |= (1 << PB0);
	DDRB |= (1 << PB1);

	USART_Init();

	/* Create the tasks defined within this file. */
	xTaskCreate( vRedLED, "RedLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	//xTaskCreate( vGreenLED, "GreenLED", configMINIMAL_STACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
	xTaskCreate( vUART0TransmitService, "UART0Tx", configMINIMAL_STACK_SIZE, NULL, mainUART_PRIORITY, NULL );

	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (1 << PB0);
		sendMessage(message, bufferSendUART0);
		PORTB ^= (1 << PB0);
		vTaskDelay( 200 );
	}
}

static void vGreenLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		//PORTB ^= (1 << PB1);
		vTaskDelay( 50 );
		//USART0_SendString(message);
	}
}

static void vUART0TransmitService( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART0 to fill, then pops first byte of data and
		   copies it to UART0 data register UDR0 effectively transmitting it. */ 
		xQueueReceive( bufferSendUART0, &UDR0, portMAX_DELAY );
		while(!( UCSR0A & (1<<UDRE0)));		// Wait for transmit to finish
	}
}

void sendMessage( unsigned char *message, QueueHandle_t queue) {
	/* Receives a message and pushes it on UART send queue character by character.
	   Limited to 255 characters for efficiency (counter i is single byte). */
	for(uint8_t i = 0; message[i] != '\0'; ++i) {
		xQueueSendToBack( queue, message+i, pdMS_TO_TICKS(50));
	}
}

void vApplicationIdleHook( void ) {
	PORTB ^= (1 << PB1);
}

/*-----------------------------------------------------------*/

