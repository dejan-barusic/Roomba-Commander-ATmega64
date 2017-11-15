
/*************************************************************/
/*                        INCLUDES                           */
/*************************************************************/

#include <stdlib.h>
#include <avr/interrupt.h>

#ifdef GCC_MEGA_AVR
	/* EEPROM routines used only with the WinAVR compiler. */
	#include <avr/eeprom.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "RoombaSCI.h"



/*************************************************************/
/*                       DEFINITIONS                         */
/*************************************************************/

/* CPU frequency. */
#define F_CPU 11059200UL

/* Baud rate. */
#define BAUD_UART1 9600UL
#define BAUD_UART0 115200UL
#define UBRR_UART0 (uint16_t)(F_CPU/(16*BAUD_UART0)-1)
#define UBRR_UART1 (uint16_t)(F_CPU/(16*BAUD_UART1)-1)

/* Tasks priority. */
#define mainLOW_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainNORMAL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainHIGH_PRIORITY			( tskIDLE_PRIORITY + 2 )

/* Errors. */
#define ERROR_OK					(uint8_t) 0		/* No error */
#define ERROR_BT_NORESPONSE			(uint8_t) 1		/* Bluetooth no response error */
#define ERROR_ROOMBA_NORESPONSE		(uint8_t) 2		/* Roomba no response error */
#define ERROR_ROOMBA_SEND			(uint8_t) 3		/* Function sendByteToRoomba() failed */
#define ERROR_BT_SEND				(uint8_t) 4		/* Function sendByteToBT() failed */
#define ERROR_ROOMBA_RECEIVE		(uint8_t) 5		/* Function getByteFromRoomba() failed */
#define ERROR_BT_RECEIVE			(uint8_t) 6		/* Function getByteFromBT() failed */



/*************************************************************/
/*                    FUNCTION SIGNATURES                    */
/*************************************************************/

static    void vRoombaTransmitServiceUART0		( void *pvParameters );
static    void vBluetoothTransmitServiceUART1	( void *pvParameters );
static    void vBluetoothReceiveServiceUART1	( void *pvParameters );
static    void vCyclicalTask					( void *pvParameters );
static    void vErrorHandler					( void *pvParameters );
          void vApplicationIdleHook				( void );

static    void sendMessage						( unsigned char *message, QueueHandle_t queue);

static    void sendByteToRoomba					( uint8_t byte );
static uint8_t getByteFromRoomba				( void );

static    void sendByteToBT						( uint8_t byte );
static uint8_t getByteFromBT					( void );

static    void error							( uint8_t byte );
static    void flushQueue						( QueueHandle_t queue);



/*************************************************************/
/*                     GLOBAL VARIABLES                      */
/*************************************************************/

/* Buffers for UART communication. */
QueueHandle_t	bufferRoombaSendUART0, 
				bufferRoombaReceiveUART0, 
				bufferBluetoothSendUART1, 
				bufferBluetoothReceiveUART1;

/* Global error. */
QueueHandle_t	globalError;

/* Storage of values for Roomba sensors and sensors on MCU pins. */
volatile uint8_t sensorData[27];



/*************************************************************/
/*                           MAIN                            */
/*************************************************************/


int main( void ) {

	/* MCU pins direction setup. */
	DDRB |= (1 << PB0);
	DDRB |= (1 << PB1);

	/* Set baud rate UART0. */
	UBRR0H = (uint8_t)(UBRR_UART0>>8);
	UBRR0L = (uint8_t)UBRR_UART0;

	/* Set baud rate UART1. */
	UBRR1H = (uint8_t)(UBRR_UART1>>8);
	UBRR1L = (uint8_t)UBRR_UART1;

	/* Enable receiver and transmitter UART0. */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);

	/* Enable receiver and transmitter UART1. */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);

	/* Enable interrupts. */
	UCSR0B |= (1 << RXCIE0);
	UCSR1B |= (1 << RXCIE1);
	sei();

	/* Create queues. */
	bufferRoombaSendUART0		= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferRoombaReceiveUART0	= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferBluetoothSendUART1	= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferBluetoothReceiveUART1 = xQueueCreate( 32, sizeof( uint8_t ) );
	globalError					= xQueueCreate(  1, sizeof( uint8_t ) );

	/* Create the tasks. */
	//xTaskCreate( vCyclicalTask,                  "Cyclic",  configMINIMAL_STACK_SIZE, NULL, mainLOW_PRIORITY,    NULL );
	xTaskCreate( vRoombaTransmitServiceUART0,    "UART0Tx", configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY, NULL );
	xTaskCreate( vBluetoothTransmitServiceUART1, "UART1Tx", configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY, NULL );
	xTaskCreate( vBluetoothReceiveServiceUART1,	 "UART1Rx", configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY, NULL );
	//xTaskCreate( vErrorHandler,                  "Error",   configMINIMAL_STACK_SIZE, NULL, mainHIGH_PRIORITY,   NULL );

	vTaskStartScheduler();

	return 0;
}





/*************************************************************/
/*                          TASKS                            */
/*************************************************************/


static void vCyclicalTask( void *pvParameters ) {
	/* Service for cyclical operations such as:
	    - Readout of Roomba sensors via serial
		- Polling of sensors on MCU pins
		- ...
	*/

	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
		/*** READOUT OF ROOMBA SENSORS ***/

		/* Clear Roomba receiving buffer. */
		flushQueue(bufferRoombaReceiveUART0);

		/* Send request to Roomba for sensor data (2 bytes). */
		sendByteToRoomba( SCI_SENSORS_OPCODE );
		sendByteToRoomba( SCI_SENSORS_PACKET0 );

		/* Receive and store sensor data bytes from Roomba into sensorData[] (as first 26 bytes). */
		for( uint8_t i = 0; i < SCI_SENSORS_PACKET0_BYTES; ++i ) {
			sensorData[i] = getByteFromRoomba();
		}


		/*** POLLING SENSORS ON MCU PINS ***/
		
		/* TODO */


		/* Cyclical delay. */
		vTaskDelay( pdMS_TO_TICKS(500) );
	}
}


static void vRoombaTransmitServiceUART0( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART0 to fill, then pops first byte of data and
		   copies it to UART0 data register UDR0 effectively transmitting it. */ 
		xQueueReceive( bufferRoombaSendUART0, &UDR0, portMAX_DELAY );
		while(!( UCSR0A & (1<<UDRE0)));		// Wait for transmit to finish
	}
}


static void vBluetoothTransmitServiceUART1( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART1 to fill, then pops first byte of data and
		   copies it to UART1 data register UDR1 effectively transmitting it. */ 
		xQueueReceive( bufferBluetoothSendUART1, &UDR1, portMAX_DELAY );
		while(!( UCSR1A & (1<<UDRE1)));		// Wait for transmit to finish
	}
}


static void vBluetoothReceiveServiceUART1( void *pvParameters ) {
	
	/* General purpose variables. */
	uint8_t opcode, data, numberOfDataBytes;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
		/* Wait until opcode is received from Bluetooth. */
		xQueueReceive( bufferBluetoothReceiveUART1, &opcode, portMAX_DELAY );

		/* If opcode is meant for Roomba pass it to serial along with any accompanying data bytes. */
		if( opcode >= SCI_START_OPCODE && opcode <= SCI_FORCE_SEEKING_DOCK_OPCODE ) {
			
			/* Set number of accompanying data bytes according to Roomba SCI specification. */
			switch( opcode ) {
				case SCI_START_OPCODE:
				case SCI_CONTROL_OPCODE:
				case SCI_SAFE_OPCODE:
				case SCI_FULL_OPCODE:
				case SCI_POWER_OPCODE:
				case SCI_SPOT_OPCODE:
				case SCI_CLEAN_OPCODE:
				case SCI_MAX_OPCODE:
				case SCI_FORCE_SEEKING_DOCK_OPCODE:
					numberOfDataBytes = 0;
					break;
				case SCI_BAUD_OPCODE:
				case SCI_MOTORS_OPCODE:
				case SCI_PLAY_OPCODE:
				case SCI_SENSORS_OPCODE: // Special case - returns data
					numberOfDataBytes = 1;
					break;
				case SCI_LEDS_OPCODE:
				case SCI_SONG_OPCODE: // Special case - variable length, 3 byte header
					numberOfDataBytes = 3;
					break;
				case SCI_DRIVE_OPCODE:
					numberOfDataBytes = 4;
					break;
				default:
					numberOfDataBytes = 0;
					break;
			}

			/* Send opcode byte to Roomba. */
			sendByteToRoomba( opcode );

			/* Handle standard Roomba opcodes. */
			if( opcode != SCI_SENSORS_OPCODE && opcode != SCI_SONG_OPCODE ) {

				/* Send data bytes from BT to Roomba. */
				while( numberOfDataBytes-- ) {
					sendByteToRoomba( getByteFromBT() );
				}
			}

			/* Handle special case: request sensor data from Roomba. */
			else if( opcode == SCI_SENSORS_OPCODE ) {

				/* Receive SCI sensors data byte from Bluetooth and send it to Roomba. */
				data = getByteFromBT();
				sendByteToRoomba( data );

				/* Set number of expected data bytes to receive from Roomba according to SCI sensors data byte. */
				switch (data) {
					case SCI_SENSORS_PACKET0:
						numberOfDataBytes = SCI_SENSORS_PACKET0_BYTES;
						break;
					case SCI_SENSORS_PACKET1:
						numberOfDataBytes = SCI_SENSORS_PACKET1_BYTES;
						break;
					case SCI_SENSORS_PACKET2:
						numberOfDataBytes = SCI_SENSORS_PACKET2_BYTES;
						break;
					case SCI_SENSORS_PACKET3:
						numberOfDataBytes = SCI_SENSORS_PACKET3_BYTES;
						break;
					default:
						numberOfDataBytes = 0;
						break;
				}

				/* Receive and store sensor data bytes from Roomba into sensorData[] (as first n bytes). */
				for( uint8_t i = 0; i < numberOfDataBytes; ++i ) {
					sensorData[i] = getByteFromRoomba();
				}
			}

			/* Handle special case: send new song to Roomba. */
			else if( opcode == SCI_SONG_OPCODE ) {
				
				/* Receive SCI song number byte from Bluetooth and send it to Roomba. */
				sendByteToRoomba( getByteFromBT() );

				/* Receive SCI song length byte from Bluetooth and send it to Roomba. */
				data = getByteFromBT();
				sendByteToRoomba( data );

				/* Song length is specified in notes. Each note consists of 2 bytes (note number and note duration). */
				numberOfDataBytes = 2 * data;

				/* Send notes to Roomba. */
				for( uint8_t i = 0; i < numberOfDataBytes; ++i ) {
					sendByteToRoomba( getByteFromBT() );
				}
			}

		}
	}
}


static void vErrorHandler( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
	}
}


void vApplicationIdleHook( void ) {
	//PORTB ^= (1 << PB1);
}





/*************************************************************/
/*                INTERRUPT SERVICE ROUTINES                 */
/*************************************************************/


ISR(USART0_RX_vect) {
	/* UART0 receive complete ISR. 
	   Pushes received byte from UDR0 register on bufferRoombaReceiveUART0 queue. */
	xQueueSendToBackFromISR( bufferRoombaReceiveUART0, &UDR0, NULL);
}

ISR(USART1_RX_vect) {
/* UART1 receive complete ISR. 
	   Pushes received byte from UDR1 register on bufferBluetoothReceiveUART1 queue. */
	xQueueSendToBackFromISR( bufferBluetoothReceiveUART1, &UDR1, NULL);
}





/*************************************************************/
/*                    UTILITY FUNCTIONS                      */
/*************************************************************/


static void sendMessage( unsigned char *message, QueueHandle_t queue) {
	/* Receives a message and pushes it on UART send queue character by character.
	   Limited to 255 characters for efficiency (counter i takes one byte). 
	   Message must be null terminated. */
	for(uint8_t i = 0; message[i] != '\0'; ++i) {
		xQueueSendToBack( queue, message+i, pdMS_TO_TICKS(30) );
	}
}

static void sendByteToRoomba( uint8_t byte ) {
	/* Pushes byte on UART0 send queue. */
	if( errQUEUE_FULL == xQueueSendToBack( bufferRoombaSendUART0, &byte, pdMS_TO_TICKS(30) ) ) {
		error(ERROR_ROOMBA_SEND);
	}
}

static uint8_t getByteFromRoomba( void ) {
	/* Gets byte from UART0 receive queue. */
	uint8_t byte;
	if( errQUEUE_EMPTY == xQueueReceive( bufferRoombaReceiveUART0, &byte, pdMS_TO_TICKS(30) ) ) {
		error(ERROR_BT_RECEIVE);
	}
	return byte;
}

static void sendByteToBT( uint8_t byte ) {
	/* Pushes byte on UART1 send queue. */
	if( errQUEUE_FULL == xQueueSendToBack( bufferBluetoothSendUART1, &byte, pdMS_TO_TICKS(30) ) ) {
		error(ERROR_BT_SEND);
	}
}

static uint8_t getByteFromBT( void ) {
	/* Gets byte from UART1 receive queue. */
	uint8_t byte;
	if( errQUEUE_EMPTY == xQueueReceive( bufferBluetoothReceiveUART1, &byte, pdMS_TO_TICKS(30) ) ) {
		error(ERROR_BT_RECEIVE);
	}
	return byte;
}

static void error( uint8_t byte ) {
	/* Receives an error and pushes it on globalError queue. */
	xQueueSendToBack( globalError, &byte, 0 );
}

static void flushQueue( QueueHandle_t queue) {
	/* Flushes out any data left on queue. */
	for(uint8_t i = uxQueueMessagesWaiting(queue); i > 0; ++i) {
		xQueueReceive( queue, NULL, 0 );
	}
}