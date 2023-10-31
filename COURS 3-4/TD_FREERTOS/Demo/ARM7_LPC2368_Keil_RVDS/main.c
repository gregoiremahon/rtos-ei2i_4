
/* Standard includes. */
#include <stdlib.h>
// pour le lpc2378 Scheduler includes. 
#include "FreeRTOS.h"
#include "task.h"

#include "myTasks.h"

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )


 // Configuration minimal du processeur pour avoir les 8 Leds du Port2 disponible
 
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
void gestion_abort(void)
{
while(1) ;

}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
 // Configuration minimal du processeur au niveau p�riph�riques
 //pour le projet
	prvSetupHardware();
	/* initialisation des taches */
	vInit_myTasks( mainLED_TASK_PRIORITY );
	// toutes les taches ont �t� cr��es - Demarrer le scheduler.
  // Les taches tournent en USER/SYSTEM mode  :  USER UNPRIVILEGED
	//le Scheduler tourne en Superviseur mode   :  HANDLER 
	// Le processeur doit basculer  quand vTaskStartScheduler est appel�
  // cela est possible car le processeur d�marre en user privileged 
	vTaskStartScheduler();
  gestion_abort(); 	/* on ne doit jamais atteindre cet appel */

}

static void prvSetupHardware( void )
{
//   SCS|=1;  // port 0 et Port 1 en FastIO
 LPC_SC->PCONP     |= (1 << 15);            /* power on sur GPIO & IOCON , deja sous tension par d�faut*/ 

  LPC_GPIO3->FIODIR |= 0x03<<25;   //P3.25 et P3.26 en sortie
	LPC_GPIO0->FIODIR |= 0x01<<22;   //P0.22 en sortie	 

	
	}
