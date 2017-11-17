
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
#define F_CPU						11059200UL

/* Baud rate. */
#define BAUD_UART1					9600UL
#define BAUD_UART0					115200UL
#define UBRR_UART0					( uint16_t ) ( F_CPU / ( 16 * BAUD_UART0 ) - 1 )
#define UBRR_UART1					( uint16_t ) ( F_CPU / ( 16 * BAUD_UART1 ) - 1 )

/* Tasks priority. */
#define mainLOW_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainNORMAL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainHIGH_PRIORITY			( tskIDLE_PRIORITY + 3 )
#define mainERROR_PRIORITY			( tskIDLE_PRIORITY + 4 )

/* Errors. */
#define ERROR_OK					( uint8_t ) 0		/* No error */
#define ERROR_BT_NORESPONSE			( uint8_t ) 1		/* Bluetooth no response error */
#define ERROR_ROOMBA_NORESPONSE		( uint8_t ) 2		/* Roomba no response error */
#define ERROR_ROOMBA_SEND			( uint8_t ) 3		/* Function sendByteToRoomba() failed */
#define ERROR_BT_SEND				( uint8_t ) 4		/* Function sendByteToBT() failed */
#define ERROR_ROOMBA_RECEIVE		( uint8_t ) 5		/* Function getByteFromRoomba() failed */
#define ERROR_BT_RECEIVE			( uint8_t ) 6		/* Function getByteFromBT() failed */
#define ERROR_DATA					( uint8_t ) 7		/* Unexpected data */

#define ADC_CHANNELS_USED			( uint8_t ) 1



/*************************************************************/
/*                    FUNCTION SIGNATURES                    */
/*************************************************************/

static    void vPeriodicTask					( void *pvParameters );
static    void vRoombaTransmitServiceUART0		( void *pvParameters );
static    void vBluetoothTransmitServiceUART1	( void *pvParameters );
static    void vBluetoothReceiveCommandUART1	( void *pvParameters );
static    void vErrorHandler					( void *pvParameters );
          void vApplicationIdleHook				( void );

static    void sendMessage						( unsigned char *message, QueueHandle_t queue );

static    void sendByteToRoomba					( uint8_t byte );
static uint8_t getByteFromRoomba				( void );

static    void sendByteToBT						( uint8_t byte );
static uint8_t getByteFromBT					( void );

static    void error							( uint8_t byte );
static    void flushQueue						( QueueHandle_t queue );

static    void handlerRoombaSCIStandard			( uint8_t opcode, uint8_t numberOfDataBytes );
static    void handlerRoombaSCISensors			( uint8_t opcode );
static    void handlerRoombaSCISong				( uint8_t opcode );



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
volatile uint8_t sensorDataRoomba[ 26 ], sensorDataMCU[ 8 ];



/*************************************************************/
/*                           MAIN                            */
/*************************************************************/


int main( void ) {

	/* MCU pins direction setup. */
	DDRB |= ( 1 << PB0 );
	DDRB |= ( 1 << PB1 );

	/* Set baud rate UART0. */
	UBRR0H = ( uint8_t ) ( UBRR_UART0 >> 8 );
	UBRR0L = ( uint8_t )   UBRR_UART0;

	/* Set baud rate UART1. */
	UBRR1H = ( uint8_t ) ( UBRR_UART1 >> 8 );
	UBRR1L = ( uint8_t )   UBRR_UART1;

	/* Enable receiver and transmitter UART0. */
	UCSR0B = ( 1 << RXEN0 ) | ( 1 << TXEN0 );

	/* Enable receiver and transmitter UART1. */
	UCSR1B = ( 1 << RXEN1 ) | ( 1 << TXEN1 );

	/* Enable interrupts. */
	UCSR0B |= ( 1 << RXCIE0 );
	UCSR1B |= ( 1 << RXCIE1 );
	sei( );

	/* Create queues. */
	bufferRoombaSendUART0		= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferRoombaReceiveUART0	= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferBluetoothSendUART1	= xQueueCreate( 32, sizeof( uint8_t ) );
	bufferBluetoothReceiveUART1 = xQueueCreate( 32, sizeof( uint8_t ) );
	globalError					= xQueueCreate(  1, sizeof( uint8_t ) );

	/* Create the tasks. */
	xTaskCreate( vPeriodicTask,                  "Periodic", configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY,    NULL );
	xTaskCreate( vBluetoothReceiveCommandUART1,	 "BTcmd",    configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY, NULL );
	xTaskCreate( vRoombaTransmitServiceUART0,    "UART0Tx",  configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY,   NULL );
	xTaskCreate( vBluetoothTransmitServiceUART1, "UART1Tx",  configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY,   NULL );
	xTaskCreate( vErrorHandler,                  "Error",    configMINIMAL_STACK_SIZE, NULL, mainNORMAL_PRIORITY,  NULL );

	vTaskStartScheduler( );

	return 0;
}





/*************************************************************/
/*                          TASKS                            */
/*************************************************************/


static void vPeriodicTask( void *pvParameters ) {
	/* Service for cyclical operations such as:
		- Polling of sensors on MCU pins
		- ...
	*/

	/* The parameters are not used. */
	( void ) pvParameters;

	/* Setup ADC registers. */

	/* ADCSRA – ADC	Control and Status Register A
	    Bit 7    – ADEN: ADC Enable
		          **ADC Enabled**
		Bit 6    – ADSC: ADC Start Conversion
		Bit 5    – ADATE: ADC Auto Trigger Enable
		Bit 4    – ADIF: ADC Interrupt Flag
		Bit 3    – ADIE: ADC Interrupt Enable
		Bits 2:0 – ADPS2:0: ADC Prescaler Select Bits
	              **Selected division factor 16 (ADPS2:0 - 100), that is sufficient for 8 bit resolution**
	*/
	ADCSRA = ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 0 << ADPS1 ) | ( 0 << ADPS0 );

	/* ADMUX – ADC Multiplexer Selection Register
	     Bit 7:6  – REFS1:0: Reference Selection Bits
		           **Selected AVCC with external capacitor at AREF pin. (REFS1:0 - 01)**
	     Bit 5    – ADLAR: ADC Left Adjust Result
		           **Enabled: result is formated so that high byte contains 8 bits of data and low remaining 2 bits.
				    Only high byte is used (8 bit resolution), low byte is discarded.
		 Bits 4:0 – MUX4:0: Analog Channel and Gain Selection Bits
	*/
	ADMUX = ( 1 << REFS0 ) | ( 1 << ADLAR );

	for( ;; ) {

		/* Poll ADC channels and store data in sensorDataMCU[ ]. */
		for( uint8_t activeSensor = 0; activeSensor < ADC_CHANNELS_USED; ++activeSensor ) {

			/* Clear MUX4:0 (ADC channel selection bits). */
			ADMUX &= 0xF0;

			/* Set MUX4:0 bits with new sensor. */
			ADMUX |= activeSensor;
		
			/* Start ADC in single conversion mode. */
			ADCSRA |= ( 1 << ADSC );

			/* Wait for AD conversion to complete.
			   TODO: Implement interrupt driven ADC. */
			while( ( ADCSRA & ( 1 << ADSC ) ) != 0 );
		
			/* Write sensor data in 8 bit resolution. */
			sensorDataMCU[ activeSensor ] = ADCH;

		}		

		/* Periodic delay. */
		vTaskDelay( pdMS_TO_TICKS( 500 ) );
	}
}

static void vRoombaTransmitServiceUART0( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART0 to fill, then pops first byte of data and
		   copies it to UART0 data register UDR0 effectively transmitting it. */ 
		xQueueReceive( bufferRoombaSendUART0, &UDR0, portMAX_DELAY );
		while( ! ( UCSR0A & ( 1 << UDRE0 ) ) );		// Wait for transmit to finish
	}
}


static void vBluetoothTransmitServiceUART1( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		/* Waits for bufferSendUART1 to fill, then pops first byte of data and
		   copies it to UART1 data register UDR1 effectively transmitting it. */ 
		xQueueReceive( bufferBluetoothSendUART1, &UDR1, portMAX_DELAY );
		while( ! ( UCSR1A & ( 1 << UDRE1 ) ) );		// Wait for transmit to finish
	}
}


static void vBluetoothReceiveCommandUART1( void *pvParameters ) {
	
	/* General purpose variables. */
	uint8_t opcode;

	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {

		/* Wait until opcode is received from Bluetooth. */
		xQueueReceive( bufferBluetoothReceiveUART1, &opcode, portMAX_DELAY );

		/* Set number of accompanying data bytes according to Roomba SCI specification. */
		switch( opcode ) {

			/* Opcode followed by 0 data bytes. */
			case SCI_START_OPCODE:
			case SCI_CONTROL_OPCODE:
			case SCI_SAFE_OPCODE:
			case SCI_FULL_OPCODE:
			case SCI_POWER_OPCODE:
			case SCI_SPOT_OPCODE:
			case SCI_CLEAN_OPCODE:
			case SCI_MAX_OPCODE:
			case SCI_FORCE_SEEKING_DOCK_OPCODE:
				handlerRoombaSCIStandard( opcode, 0 );
				break;

			/* Opcode followed by 1 data bytes. */
			case SCI_BAUD_OPCODE:
			case SCI_MOTORS_OPCODE:
			case SCI_PLAY_OPCODE:
				handlerRoombaSCIStandard( opcode, 1 );
				break;

			/* Opcode followed by 3 data bytes. */
			case SCI_LEDS_OPCODE:
				handlerRoombaSCIStandard( opcode, 3 );
				break;
			
			/* Opcode followed by 4 data bytes. */
			case SCI_DRIVE_OPCODE:
				handlerRoombaSCIStandard( opcode, 4 );
				break;

			/* Opcode followed by 1 data bytes. Roomba returns data. */
			case SCI_SENSORS_OPCODE:
				handlerRoombaSCISensors( opcode );
				break;

			/* Opcode followed by n data bytes. Serial sequence: 
			  [Opcode] [Song Number] [Song Length] [Note Number 1] [Note Duration 1] [Note Number 2] [Note Duration 2] etc. */
			case SCI_SONG_OPCODE:
				handlerRoombaSCISong( opcode );
				break;

			default:
				/* Received byte is not a valid opcode. */
				break;
		}
	}
}


static void vErrorHandler( void *pvParameters ) {
	
	/* The parameters are not used. */
	( void ) pvParameters;
	
	for( ;; ) {
		
		/* TODO */

	}
}


void vApplicationIdleHook( void ) {
	//PORTB ^= ( 1 << PB1 );
}





/*************************************************************/
/*                INTERRUPT SERVICE ROUTINES                 */
/*************************************************************/


ISR( USART0_RX_vect ) {
	/* UART0 receive complete ISR. 
	   Pushes received byte from UDR0 register on bufferRoombaReceiveUART0 queue. */
	xQueueSendToBackFromISR( bufferRoombaReceiveUART0, &UDR0, NULL );
}

ISR( USART1_RX_vect ) {
/* UART1 receive complete ISR. 
	   Pushes received byte from UDR1 register on bufferBluetoothReceiveUART1 queue. */
	xQueueSendToBackFromISR( bufferBluetoothReceiveUART1, &UDR1, NULL );
}





/*************************************************************/
/*                    UTILITY FUNCTIONS                      */
/*************************************************************/


static void sendMessage( unsigned char *message, QueueHandle_t queue) {
	/* Receives a message and pushes it on UART send queue character by character.
	   Limited to 255 characters for efficiency (counter i takes one byte). 
	   Message must be null terminated. */
	for( uint8_t i = 0; message[ i ] != '\0'; ++i ) {
		xQueueSendToBack( queue, message+i, pdMS_TO_TICKS( 30 ) );
	}
}

static void sendByteToRoomba( uint8_t byte ) {
	/* Pushes byte on UART0 send queue. */
	if( errQUEUE_FULL == xQueueSendToBack( bufferRoombaSendUART0, &byte, pdMS_TO_TICKS( 30 ) ) ) {
		error( ERROR_ROOMBA_SEND );
	}
}

static uint8_t getByteFromRoomba( void ) {
	/* Gets byte from UART0 receive queue. */
	uint8_t byte;
	if( errQUEUE_EMPTY == xQueueReceive( bufferRoombaReceiveUART0, &byte, pdMS_TO_TICKS( 30 ) ) ) {
		error( ERROR_BT_RECEIVE );
	}
	return byte;
}

static void sendByteToBT( uint8_t byte ) {
	/* Pushes byte on UART1 send queue. */
	if( errQUEUE_FULL == xQueueSendToBack( bufferBluetoothSendUART1, &byte, pdMS_TO_TICKS( 30 ) ) ) {
		error( ERROR_BT_SEND );
	}
}

static uint8_t getByteFromBT( void ) {
	/* Gets byte from UART1 receive queue. */
	uint8_t byte;
	if( errQUEUE_EMPTY == xQueueReceive( bufferBluetoothReceiveUART1, &byte, pdMS_TO_TICKS( 30 ) ) ) {
		error( ERROR_BT_RECEIVE );
	}
	return byte;
}

static void error( uint8_t byte ) {
	/* Receives an error and pushes it on globalError queue. */
	xQueueSendToBack( globalError, &byte, 0 );
}

static void flushQueue( QueueHandle_t queue ) {
	/* Flushes out any data left on queue. */
	for( uint8_t i = uxQueueMessagesWaiting( queue ); i > 0; ++i ) {
		xQueueReceive( queue, NULL, 0 );
	}
}

static void handlerRoombaSCIStandard( uint8_t opcode, uint8_t numberOfDataBytes ) {

	/* Send opcode byte to Roomba. */
	sendByteToRoomba( opcode );

	/* Send data bytes from BT to Roomba. */
	while( numberOfDataBytes-- ) {
		sendByteToRoomba( getByteFromBT( ) );
	}
}

static void handlerRoombaSCISensors( uint8_t opcode ) {

	uint8_t packet, lengthOfSensorData;

	/* Send opcode byte to Roomba. */
	sendByteToRoomba( opcode );

	/* Receive SCI sensors data packet byte from Bluetooth and send it to Roomba. */
	packet = getByteFromBT( );
	sendByteToRoomba( packet );

	/* Set number of expected data bytes to receive from Roomba according to SCI sensors data byte. */
	switch (packet) {
		case SCI_SENSORS_PACKET0:
		lengthOfSensorData = SCI_SENSORS_PACKET0_BYTES;
		break;
		case SCI_SENSORS_PACKET1:
		lengthOfSensorData = SCI_SENSORS_PACKET1_BYTES;
		break;
		case SCI_SENSORS_PACKET2:
		lengthOfSensorData = SCI_SENSORS_PACKET2_BYTES;
		break;
		case SCI_SENSORS_PACKET3:
		lengthOfSensorData = SCI_SENSORS_PACKET3_BYTES;
		break;
		default:
		error( ERROR_DATA );
		return;
	}

	/* Receive and store sensor data bytes from Roomba into 
	   sensorData[] (as first n bytes) and send it to Bluetooth. */
	for( uint8_t i = 0; i < lengthOfSensorData; ++i ) {
		sensorDataRoomba[ i ] = getByteFromRoomba( );
		sendByteToBT( sensorDataRoomba[ i ] );
	}
}

static void handlerRoombaSCISong( uint8_t opcode ) {

	uint8_t notes, length;

	/* Send opcode byte to Roomba. */
	sendByteToRoomba( opcode );

	/* Receive SCI song number byte from Bluetooth and send it to Roomba. */
	sendByteToRoomba( getByteFromBT( ) );

	/* Receive SCI song number of notes byte from Bluetooth and send it to Roomba. */
	notes = getByteFromBT( );
	sendByteToRoomba( notes );

	/* Song length is specified in notes. Each note consists of 2 bytes (note number and note duration). */
	length = 2 * notes;

	/* Send notes to Roomba. */
	for( uint8_t i = 0; i < length; ++i ) {
		sendByteToRoomba( getByteFromBT( ) );
	}
}