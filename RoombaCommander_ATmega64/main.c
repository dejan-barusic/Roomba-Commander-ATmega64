#include <stdlib.h>
#include <string.h>

#ifdef GCC_MEGA_AVR
	/* EEPROM routines used only with the WinAVR compiler. */
	#include <avr/eeprom.h>
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "ATmega64UART.h"


/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainRED_LED_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainGREEN_LED_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_BAUD_RATE			( ( unsigned long ) 9600 )
#define CPU_Z 11059200UL

/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters );
static void vGreenLED( void *pvParameters );
//static void vPutChar( void *pvParameters );

//xComPortHandle serialPort;

unsigned char message[] = "Hello! ";

int main( void ) {

	DDRB |= (1 << PB0);
	DDRB |= (1 << PB1);

	//serialPort = xSerialPortInitMinimal(115200, 8);
	USART_Init();

	/* Create the tasks defined within this file. */
	xTaskCreate( vRedLED, "Red LED", configMINIMAL_STACK_SIZE, NULL, mainRED_LED_TASK_PRIORITY, NULL );
	xTaskCreate( vGreenLED, "Green LED", configMINIMAL_STACK_SIZE, NULL, mainGREEN_LED_TASK_PRIORITY, NULL );
	//xTaskCreate( vPutChar, "Put Char", configMINIMAL_STACK_SIZE, NULL, 3, NULL );

	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vRedLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (0 << PB0);
		/* pdMS_TO_TICKS is not working as expected. 
		   Use formula: (time_ms*configTICK_RATE_HZ)/1000 */
		vTaskDelay( 50 );
	}
}

static void vGreenLED( void *pvParameters ) {

	/* The parameters are not used. */
	( void ) pvParameters;

	for( ;; ) {
		PORTB ^= (1 << PB1);
		vTaskDelay( 50 );
		USART0_SendString(message);
	}
}

//static void vPutChar( void *pvParameters ) {
//
	///* The parameters are not used. */
	//( void ) pvParameters;
//
	//for( ;; ) {
		////xSerialPutChar(serialPort, 'X', pdMS_TO_TICKS(100));
		//vTaskDelay( pdMS_TO_TICKS(1000) );
	//}
//}
/*-----------------------------------------------------------*/

