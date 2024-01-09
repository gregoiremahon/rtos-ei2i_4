
//************************************ IT TIMER0 ***************************************
void TIMER0_IRQHandler(void) //nom à vérifier dans le fichier .s
{ // appeler la fonction de mise a jour du DAC

  LPC_TIM0->IR=1; //acquittement
}	

void init_timer0() //recopier pour un autre timer ....
{ LPC_SC->PCONP     |= (1 << 1); /*power on sur timer0 , déjà sous tension par défaut */
	LPC_TIM0->TCR =0x03;
  LPC_TIM0->CTCR =0x00;
  LPC_TIM0->MR0 = 52; // valeur pour etre à 48 KHz environ : 100 000 000 / 48 000 = 208 à diviser par 4 (CCLK/4 par defaut)
	LPC_TIM0->MCR = 0x03;
	
//choisir un niveau de priorité en regardant dans RTOSconfig.h, le niveau à partir duquel
//l interruption devient prioritaire sur l'OS mais devient incompatible OS
  NVIC_SetPriority (TIMER0_IRQn, xxxxxxxx);
  NVIC_EnableIRQ(TIMER0_IRQn)  ;
	LPC_TIM0->TCR =0x01;
}

void bloque_timer0()
{ LPC_TIM0->TCR =0x03;}

void debloque_timer0()
{ LPC_TIM0->TCR =0x03;}


