#include <stdio.h>
#include "LPC17xx.H"                         /* LPC17xx definitions           */

int i=0;
int cpt_it=0; 
unsigned long tempo=0;
 volatile uint8_t  clock_1s=0;
void actualise_ledV(int consV) 
{  if (consV&0x01) {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = 1<<25;} 
    else {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = 1<<25;} 
}
__inline void actualise_ledB(int consB)
{ if (consB&0x02) {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = 1<<26;} 
  else {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = 1<<26;}
}
// le #define a disparu  !!!!!!!!


void init_timer0()
{ ((LPC_SC_TypeDef *) ((0x40080000UL) + 0x7C000) )->PCONP     |= (1 << 1);  
	((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->TCR =0x03;
  ((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->CTCR =0x00;
  ((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->MR0 =128; 
	((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->MCR = 0x03;
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0]=2;
	((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->TCR =0x01;
}
void TIMER0_IRQHandler(void)
{ cpt_it++;
  if (cpt_it&0x04) {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = 1<<22;} else {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = 1<<22;}; 
  ((LPC_TIM_TypeDef *) ((0x40000000UL) + 0x04000) )->IR=1; 
}	
void init_gpio(void)
{ ((LPC_SC_TypeDef *) ((0x40080000UL) + 0x7C000) )->PCONP     |= (1 << 15);    
  ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR |= 0x03<<25;   
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR |= 0x01<<22;   
}	
void init_proc()
{ init_gpio();
	init_timer0();
  SysTick_Config(100);        
}
int main (void) {
  init_proc();
clock_1s=0;
  while (1) {                                 
if(clock_1s){
	   clock_1s=0;
		 i++;
    actualise_ledV(i);
	  actualise_ledB(i);
	  if (i&0x04) {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = 1<<22;} else {((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = 1<<22;};    
	 }
  }
}
