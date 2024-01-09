

void init_gpio(void) // a modifier
{ LPC_SC->PCONP     |= (1 << 15);  /* power on sur GPIO & IOCON , deja sous tension par défaut*/ 
// LED RGB   qui permet de disposer d'espions
  LPC_GPIO3->FIODIR |= 0x03<<25;   //P3.25 et P3.26 en sortie
  LPC_GPIO0->FIODIR |= 0x01<<22;   //P0.22 en sortie

  // sorties necessaires pour le chenillard	
  LPC_GPIO2->FIODIR |= 0x0F8; //P2.3 à P2.7 en sortie 
  	//sorties témoins de l'IT ADC et de la tache gere_conversion
	LPC_GPIO0->FIODIR |= 0x03 <<10; //P0.10 et P0.11 en sortie (led)
	//rajouter des sorties pour faire des témoins de passage dans les diverses taches et mesurer les durées d'execution
	// en particulier pour la tache qui recalcule le buffer audio...
}



#define BP_OFF 0
#define BP_ON 1

 unsigned char button_P0_0(void)
	{if((LPC_GPIO0->FIOPIN0)&1) 
	    {return BP_OFF;} 
	 else  
      {return BP_ON;} 
	}	
 unsigned char button_P0_1(void)
	{if((LPC_GPIO0->FIOPIN0)&2) 
	    {return BP_OFF;} 
	 else  
	    {return BP_ON;} 
	}	
 unsigned char button_P0_2(void)
	{if((LPC_GPIO0->FIOPIN0)&4) 
	   {return BP_OFF;} 
	 else  
	   {return BP_ON;} 
	}	
 unsigned char button_P0_3(void)
	{if((LPC_GPIO0->FIOPIN0)&8) 
	   {return BP_OFF;} 
	 else  
	   {return BP_ON;} 
	}	
	
	
unsigned char lecture_etat_codeur(void)
	{unsigned char prov = (LPC_GPIO2->FIOPIN) >> 11;
		prov =prov ^ 0x03;//pour avoir un bit à un quand les inters sont fermés
		return prov&0x03;// on ne veut que les deux bits du codeur
	}	

}	
//****************************************************************
void 	gere_codeur_periodiquement(void)
{static unsigned char codeur =0;
 unsigned char old_codeur=codeur; //pour faire un détecteur de front
		 codeur = lecture_etat_codeur();
		 if (codeur ==3)//on arrive dans la position entre deux crans
		 {if (old_codeur == 1) {pos_cur++; pos_cur&=3;flag_actualise=1;}//dans le sens CW
		  if (old_codeur == 2) {pos_cur--; pos_cur&=3;flag_actualise=1;}//dans le sens CCW
		 }

}
	
	

