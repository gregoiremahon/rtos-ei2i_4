
uint32_t val_ADC0;
uint32_t val_ADC1;
uint32_t val_ADC2;

void ADC_IRQHandler(void) 
{LPC_ADC->ADCR &= ~(1<<16); //arret de la conversion
	//remarque on revient ici avec ADC0 en overrun
	//car le convertisseur a relancé la conversion sur ADC0 avant qu'on le stoppe
	val_ADC0= (LPC_ADC->ADDR0>>6)&0x000003FF; //recuperation et effacement des bits DONE et OVERRUN (par la lecture du registre)
	val_ADC1= (LPC_ADC->ADDR1>>6)&0x000003FF;
	val_ADC2= (LPC_ADC->ADDR2>>6)&0x000003FF;
	
 // LPC_ADC->ADSTAT |= 1<<16 ; // acquittement de l'IT inutile car la lecture du troisieme efface le bit d'IT 
// ici on pourrait notifier (from_isr) la tache gere_adc   qui va exploiter les conversions 

// on mettra aussi  ici directement à jour la variable  qui gère la vitesse de défilement du chenillard en exploitant val_ADC2.

}


void init_adc(void)//lire le chapitre 27 de la doc
{LPC_SC->PCONP|=1<<12; //mettre l'adc sous tension...
 LPC_ADC->ADCR |= 1<<21 ;  //sortir du Power Down
	//  on ne touche pas à l'horloge de l'ADC qui est de 25MHz 
	// (sinon voir le LPC2300.S, configuration wizard,CLOCK SETUP, PCLKSEL0  PLCK_ADC bit 25 24)
	// mettre les pattes souhaitées en AD0.0 AD0.1 AD0.2   par exemple...
	//           P0.23       P0.24      P0.25  
  //Vref sur la carte est fixé à 3V3 par le hardware 	
	LPC_PINCON->PINSEL1 |= (0x01<<14)|(0x01<<16)|(0x01<<18); //3 entrées analogiques 
	LPC_PINCON->PINMODE1 |= (0x02<<14)|(0x02<<16)|(0x02<<18); //3 entrées sans pull up ni pull down 
	LPC_ADC->ADCR|=1<<8; // 25MHz divisé par 1+1 = 2 pour avoir une horloge 12.5 < 13 MHz (doc)
	                     // la conversion prend 64 cycles classiquement soit 5.12us
	LPC_ADC->ADINTEN = 0X04; // AD0.2 source de l'IT de fin de conversion...(dernière convertie parmi les 3)
	LPC_ADC->ADCR|=0x7; // on lancera 3 conversions AD0.0 AD0.1 AD0.2
	//on on veut une IT en fin de conversion
	// sinon encore plus simple,ne pas mettre d'IT pour arreter les conversions 
	// qui se font en boucle infinie par le convertisseur
	// et on peut lire les dernières valeurs converties dans les registres de résultat de conversion
	 // De toute manière quand on est prévenu, une conversion sur AD0.0 se relance avant qu'on ait le temps d'arreter...
	 
  NVIC_SetPriority (ADC_IRQn, xxxxxxxx);//choisir une priorité compatible OS mais plus prioritaire que les ITs OS
  NVIC_EnableIRQ(ADC_IRQn)  ;	
	//le lancement de conversion se fait quelque part  par :  LPC_ADC->ADCR|=1<<16;// burst de conversion

}