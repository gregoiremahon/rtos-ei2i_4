
void init_PWM(void) // a modifier 
{ LPC_SC->PCONP     |= (1 << 6);//déjà par défaut...
  LPC_PINCON->PINSEL4 |= 0x00000005;//P2.0 à P2.1 deviennent PWM1.1 PWM1.2
	LPC_PWM1->TCR = 0x03;
	LPC_PWM1->PCR |=(0x03 << 9); //deux pwm
	LPC_PWM1->MR0 = 4094 ; 
	LPC_PWM1->MR1 = 2048;
	LPC_PWM1->MR2 = 2048;
    LPC_PWM1->LER = 0x07; 	
	LPC_PWM1->MCR = 0x02;	
	LPC_PWM1->TCR = 0x09;
	// pas d'IT activée
} 

