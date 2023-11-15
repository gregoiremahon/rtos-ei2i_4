/*----------------------------------------------------------------------------
 * Name:    IRQ_OS_TP1.c
 * Purpose: IRQ Handler
  *----------------------------------------------------------------------------*/

#include <LPC17xx.h>                         /* LPC17xx definitions           */
// redefinir les variables de OS_TP1.c précédé de extern
extern volatile uint8_t  clock_1s;   
// definir ici des variables globales pour 
unsigned long ticks = 10;
/*----------------------------------------------------------------------------
  Systick Interrupt Handler , s'inspirer du cours ou de port.c ligne 411
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void SysTickHandler (void) {
	//où doit être déclaré le nom de cette fonction pour être déclenchée
	// par le bon évènement ?
	

}
/*----------------------------------------------------------------------------
  Pending SV Handler , s'inspirer du cours ou de port.c ligne 374
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void PendSVHandler (void) {
	//où doit être déclaré le nom de cette fonction pour être déclenchée
	// par le bon évènement ?
	

}
/*----------------------------------------------------------------------------
  SVC Handler , s'inspirer du cours ou de port.c ligne 374
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void SVCHandler (void) {
	//où doit être déclaré le nom de cette fonction pour être déclenchée
	// par le bon évènement ?
	

}
