#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* le fichier .h du TP  */
#include "myTasks.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 7 )

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLed1Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed2Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed3Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed4Task, pvParameters );
/*-----------------------------------------------------------*/
unsigned char argument =3;
void vInit_myTasks( UBaseType_t uxPriority )
{
//BaseType_t xLEDTask;
		xTaskCreate( vLed1Task, "LED1", ledSTACK_SIZE, &argument, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed2Task, "LED2", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed3Task, "LED3", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed4Task, "LED4", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/
void allume_num_tache(unsigned char num)
{   if(num&0x04) {LPC_GPIO3->FIOCLR = 1<<26;} else {LPC_GPIO3->FIOCLR = 1<<26;}
	  if(num&0x02) {LPC_GPIO3->FIOCLR = 1<<25;} else {LPC_GPIO3->FIOCLR = 1<<25;}
	  if(num&0x01) {LPC_GPIO0->FIOCLR = 1<<22;} else {LPC_GPIO0->FIOCLR = 1<<22;}
}
//static portTASK_FUNCTION( vLed1Task, pvParameters )
void vLed1Task( void *pvParameters )
{ 
  LPC_GPIO2->FIODIR |=0x10;
	for(;;)
	{ LPC_GPIO2->FIOCLR = 0xFF;
		LPC_GPIO2->FIOSET = 0x10;
		allume_num_tache(1);
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */


static portTASK_FUNCTION( vLed2Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
  LPC_GPIO2->FIODIR |=0x20;
	for(;;)
	{ LPC_GPIO2->FIOCLR = 0xFF;
		LPC_GPIO2->FIOSET = 0x20;
		allume_num_tache(2);
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */
static portTASK_FUNCTION( vLed3Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
  LPC_GPIO2->FIODIR |=0x40;
	for(;;)
	{ LPC_GPIO2->FIOCLR = 0xFF;
		LPC_GPIO2->FIOSET = 0x40;
		allume_num_tache(3);
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

static portTASK_FUNCTION( vLed4Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
  LPC_GPIO2->FIODIR |=0x80;
	for(;;)
	{ LPC_GPIO2->FIOCLR = 0xFF;
		LPC_GPIO2->FIOSET = 0x80;
		allume_num_tache(4);
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

