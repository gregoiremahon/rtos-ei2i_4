/*----------------------------------------------------------------------------
 * Name:    IRQ_OS_TP1.c
 * Purpose: IRQ Handler
  *----------------------------------------------------------------------------*/

#include <LPC17xx.h>                         /* LPC17xx definitions           */
// redefinir les variables de OS_TP1.c pr�c�d� de extern
extern volatile uint8_t  clock_1s;   
// definir ici des variables globales pour 
unsigned long ticks = 10;
/*----------------------------------------------------------------------------
  Systick Interrupt Handler , s'inspirer du cours ou de port.c ligne 411
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void SysTickHandler (void) {
	//o� doit �tre d�clar� le nom de cette fonction pour �tre d�clench�e
	// par le bon �v�nement ?
	

}
/*----------------------------------------------------------------------------
  Pending SV Handler , s'inspirer du cours ou de port.c ligne 374
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void PendSVHandler (void) {
	//o� doit �tre d�clar� le nom de cette fonction pour �tre d�clench�e
	// par le bon �v�nement ?
	

}
/*----------------------------------------------------------------------------
  SVC Handler , s'inspirer du cours ou de port.c ligne 374
  ECRIRE ICI LE ROLE DE CETTE IT
 *----------------------------------------------------------------------------*/
void SVCHandler (void) {
	//o� doit �tre d�clar� le nom de cette fonction pour �tre d�clench�e
	// par le bon �v�nement ?
	

}
