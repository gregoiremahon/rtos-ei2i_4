#include <stdlib.h>
int reload_B= 900;
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* le fichier .h du TP  */
#include "myTasks.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 7 )

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLed_R_Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed_V_Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed_B_Task, pvParameters );
static portTASK_FUNCTION_PROTO( vChange_Rythme_B_Task, pvParameters );
/*-----------------------------------------------------------*/

void vInit_myTasks( UBaseType_t uxPriority )
{
//BaseType_t xLEDTask;
		xTaskCreate( vLed_R_Task, "LEDR", ledSTACK_SIZE, NULL, uxPriority, \
	                                               ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed_V_Task, "LEDV", ledSTACK_SIZE, NULL, uxPriority,  \
	                                               ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed_B_Task, "LEDB", ledSTACK_SIZE, NULL, uxPriority,  \
	                                               ( TaskHandle_t * ) NULL );
	  xTaskCreate( vChange_Rythme_B_Task, "CHGB", ledSTACK_SIZE, NULL, uxPriority,\
	                                               ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/
void actualise_ledV(int consV) 
{  if (consV) {LPC_GPIO3->FIOSET = 1<<25;}
   else {LPC_GPIO3->FIOCLR = 1<<25;} 
}
void  actualise_ledR(int consR )  
{  if (consR) {LPC_GPIO0->FIOSET = 1<<22;} 
  else {LPC_GPIO0->FIOCLR = 1<<22;}
}	
void actualise_ledB(int consB)
{ if (consB) {LPC_GPIO3->FIOSET = 1<<26;} 
  else {LPC_GPIO3->FIOCLR = 1<<26;}
}	
static portTASK_FUNCTION( vLed_R_Task, pvParameters )
{	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
	for(;;)
	{actualise_ledR(1);
		vTaskDelay(300);
	 actualise_ledR(0);
		vTaskDelay(700);		
	}
} 


static portTASK_FUNCTION( vLed_V_Task, pvParameters )
{/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
	for(;;)
	{actualise_ledV(1);
		vTaskDelay(100);
	 actualise_ledV(0);
		vTaskDelay(300);		
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */
static portTASK_FUNCTION( vLed_B_Task, pvParameters )
{	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
	for(;;)
	{actualise_ledV(1);
		vTaskDelay(100);
	 actualise_ledV(0);
		vTaskDelay(reload_B);		
	}
} 
static portTASK_FUNCTION( vChange_Rythme_B_Task, pvParameters )
{	unsigned char old_bouton=0;
	unsigned char bouton;
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
	for(;;)
	{	bouton = ((LPC_GPIO0->FIOPIN)&(1<<10))?1:0;
  if(old_bouton && !bouton) 
	 {switch (reload_B)
		{case 900: //1hZ   période 900+100ms
		 reload_B = 400; 		 break;
     case 400: //2Hz   période 400+100ms
     reload_B = 233; 		 break;
     case 233: //3 Hz  période 233 +100ms
     reload_B = 150; 		 break;			 
		 case 150: //4Hz   période 150+100ms
		 reload_B = 900;		 break;	 
	 }	}
  old_bouton=bouton;
	vTaskDelay(100);
} } 
