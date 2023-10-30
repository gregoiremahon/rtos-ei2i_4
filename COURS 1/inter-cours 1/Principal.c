#include <stdio.h>
#include "LPC17xx.h"                 


//************************************ IT TIMER0 ***************************************
void TIMER0_IRQHandler(void)
{ LPC_GPIO3->FIOSET = 1<<26;LPC_GPIO3->FIOCLR = 1<<26;
	LPC_GPIO3->FIOSET = 1<<26;LPC_GPIO3->FIOCLR = 1<<26;
	LPC_GPIO3->FIOSET = 1<<26;LPC_TIM0->IR=1; //acquittement
	LPC_GPIO3->FIOCLR = 1<<26;
}	
void SysTick_Handler (void) {
LPC_GPIO3->FIOSET = 1<<25; LPC_GPIO3->FIOCLR = 1<<25;	
LPC_GPIO3->FIOSET = 1<<25; LPC_GPIO3->FIOCLR = 1<<25;	
LPC_GPIO3->FIOSET = 1<<25; LPC_GPIO3->FIOCLR = 1<<25;	
LPC_GPIO3->FIOSET = 1<<25; LPC_GPIO3->FIOCLR = 1<<25;	
}
void init_gpio(void)
{ LPC_SC->PCONP     |= (1 << 15);  /* power ON GPIO & IOCON (déjà par défaut)*/ 
  LPC_GPIO3->FIODIR |= 0x03<<25;   //P3.25 et P3.26 en sortie
	LPC_GPIO0->FIODIR |= 0x01<<22;   //P0.22 en sortie
}	
//------------------------------------------------
void init_timer0()
{ LPC_SC->PCONP  |= (1 << 1); /*power on sur timer0 , déjà sous tension par défaut */
	// vérifier dans system_LPC17xx.C (configuration Wizard) que l'horloge périphérique
	//  est bien  sélectionnée sur CCLK/1 (menu clock configuration, registre PCLKSEL0)
	LPC_TIM0->TCR =0x03;
  LPC_TIM0->CTCR =0x00;
  LPC_TIM0->MR0 =120; //120 cycles avant IT;
	LPC_TIM0->MCR = 0x03;
  NVIC_EnableIRQ(TIMER0_IRQn)  ;
  NVIC_SetPriority(TIMER0_IRQn, 31);// 31 priorité la plus faible , 0 la plus élevée
	LPC_TIM0->TCR =0x01;
}
//-----------------------------------------------
void init_proc()
{ init_gpio();
	//init_timer0();             // Génère une interruption tous les 90 cycles  (0.9us) 
  //SysTick_Config(200);       /* Genere une interruption tous les 200 cycles (2us) */
}
//--------------- Main Program -----------------------------------
int main (void) {
  init_proc();
  while (1) {/* Loop forever                  */
    LPC_GPIO0->FIOSET = 1<<22;
		LPC_GPIO0->FIOCLR = 1<<22;
	 }
}
