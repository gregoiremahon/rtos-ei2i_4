/*--------------------------------------------
 * clignotements
 *--------------------------------------------*/

//#include <stdio.h>
#include "LPC17xx.H"   /* LPC17xx definitions   */
uint16_t reload_B=900;
unsigned char mae_R=0;	
unsigned char i=0;
unsigned long tempo=0;
/* Import variables externes du fichier IRQ_TD1.c                                  */

unsigned char flag_bp =0;
uint32_t tick=10;
unsigned long cpt1 =300;
unsigned long cpt2 = 100;
unsigned long cpt3=100;

    

// une fonction
void actualise_ledV(int consV) 
{  if (consV) {LPC_GPIO3->FIOSET = 1<<25;}
   else {LPC_GPIO3->FIOCLR = 1<<25;} 
}
// une fonction inline
__inline void actualise_ledB(int consB)
{ if (consB) {LPC_GPIO3->FIOSET = 1<<26;} 
  else {LPC_GPIO3->FIOCLR = 1<<26;}
}
// une macro
#define actualise_ledR( consR )  \
  if (consR) {LPC_GPIO0->FIOSET = 1<<22;} \
  else {LPC_GPIO0->FIOCLR = 1<<22;}\
// prototypes de fonctions
void tache_cligneR(void);
void tache_cligneV(void);	
void tache_cligneB(void);
void tache_scrutation_bouton(void);
/*----------------------------------------------------------------------------
  Main Program
 *----------------------------------------------------------------------------*/
//**************************************************************
void tache_cligneR(void)
{switch(mae_R)
	{case 0:
		if(cpt1==0) {mae_R=1;cpt1=700;actualise_ledR(0);}
	 break;
	 case 1:
		if(cpt1==0) {mae_R=0;cpt1=300;actualise_ledR(1);}
     break;		
	}
}

void tache_cligneV(void)
{static unsigned char mae_V=0;
  switch(mae_V)
	{case 0:
		if(cpt2==0) {mae_V=1;cpt2=300;actualise_ledV(0);}
	 break;
	 case 1:
		if(cpt2==0) {mae_V=0;cpt2=100;actualise_ledV(1);}
   break;		
	}	
}
void tache_cligneB(void)
{static unsigned char mae_B=0;//déclaration ?????
  switch(mae_B)
	{case 0:
		if(cpt3==0) {mae_B=1;cpt3=reload_B;
		             actualise_ledB(0);
		            }
	 break;
	 case 1:
		if(cpt3==0) {mae_B=0;cpt3=100;
			           actualise_ledB(1);
		            }
   break;		
			
	}		
}
void tache_scrutation_bouton(void)
{static unsigned char old_bouton;//non initialisée pour la premiere fois
 unsigned char bouton;
	bouton = ((LPC_GPIO0->FIOPIN)&(1<<10))?1:0;
  if(old_bouton && !bouton) 
	 {switch (reload_B)
      {	 case 900:reload_B = 400;break; //1hZ   période 900+100ms		  		 
         case 400:reload_B = 233;break; //2Hz   période 400+100ms
         case 233:reload_B = 150;break; //3 Hz  période 233+100ms     		 
		     case 150:reload_B = 900;break; //4Hz   période 150+100ms		 	 
	   }
	}
  old_bouton=bouton;	 
}	


//************************************ IT TIMER0 ***************************************
void TIMER0_IRQHandler(void)
{LPC_TIM0->IR =0x01;
	if(cpt1) cpt1--;
 if(cpt2) cpt2--;
 if(cpt3) cpt3--;
 if (!(--tick)) {flag_bp=1;tick=100;}	
}	
void SysTick_Handler (void) {

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
  LPC_TIM0->MR0 =100; //120 cycles avant IT;
	LPC_TIM0->MCR = 0x03;
  NVIC_EnableIRQ(TIMER0_IRQn)  ;
  NVIC_SetPriority(TIMER0_IRQn, 31);// 31 priorité la plus faible , 0 la plus élevée
	LPC_TIM0->TCR =0x01;
}
//-----------------------------------------------
void init_proc()
{ init_gpio();
	init_timer0();             // Génère une interruption tous les 90 cycles  (0.9us) 
  //SysTick_Config(200);       /* Genere une interruption tous les 200 cycles (2us) */
}
//--------------- Main Program -----------------------------------
int main (void) {
  init_proc();
  while (1) { 
		tache_cligneR();
		tache_cligneV();
		tache_cligneB();
		if(flag_bp) {flag_bp=0;tache_scrutation_bouton();}
  }
}
