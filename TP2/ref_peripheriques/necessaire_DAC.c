
uint32_t  tableau_son1[256] ;
uint32_t  tableau_son2[256] ;
uint32_t  * tableau_son = tableau_son1;
uint8_t   index_son=0;
volatile uint8_t  alterne=0;


void maj_dacr()
{ LPC_DAC->DACR=val_son; // pour être le plus précis possible sur l'instant de mise à jour
                         // on met à jour puis on calcule la prochaine valeur...
	val_son=tableau_son[index_son];
	if(!(++index_son)) // test de changement de tableau (256 cases)
	{// prevenir le main ou autre qu'on change de tableau pour faire recalculer le tableau	
	// changer de tableau
	 alterne ^=1;
	 tableau_son = (alterne) ? tableau_son2 :tableau_son1;
	}
} 


// fonction à appeler dans une tache notifiée d'un besoin de recalcul d'un tableau d échantillons
// penser à initialiser le son
calcul_tableau(uint32_t * tab)
{static uint32_t index_a_virgule=0; //on doit réutiliser la valeur pour assurer la continuité
 uint32_t ech_avant, ech_apres;	
 uint8_t pos=0;// la boucle do while va etre executée 256 fois
    do
    {//mise à jour de inc_freq
	  // tester ici si une rampe son est disponible, alors utiliser un élément de la rampe pour inc_freq
     // sinon on utilise la dernière valeur du tableau de la rampe (vitesse constante)
     // penser à remettre l indicateur de disponibilité de rampe à 0 en fin de boucle
     
     // gestion d'une lecture à vitesse variable
    index_a_virgule+=inc_freq;// index_a_virgule contient 8bits de poids fort de vrai index et 8 bits après la virgule
	if(index_a_virgule>=36*256) {index_a_virgule-=36*256;}
	// partie après la virgule et index du tableau de son d origine
	virgule=index_a_virgule&255; index=index_a_virgule>>8;
	// recupération des echantillons avant et apres
	ech_avant=son1K[index];ech_apres=son1K[index+1];//astuce, le tableau contient 37 cases
	// moyenne pondérée par la valeur de l index à virgule
	val_son_prov = ((ech_avant * (256-virgule) + ech_apres*virgule))>>8  ;
	// maintenant , gerer ici le volume...en tenant compte d'une rampe de volume éventuelle
	//attention on doit amplifier ou atténuer l'écart par rapport à la valeur 512 et toujours etre centré à 512
	val_son_prov = ....;
	// mise à jour du bloc son...
	tab[pos] = 	(val_son_prov<<2)& 0xFFC0; // pour etre compatible avec le registre DACR
    } while(++pos) ;

void init_dac()
{
	 // attribution de la patte P0.26 à la sortie DAC
		LPC_PINCON->PINSEL1|=0x02<<20;
		LPC_PINCON->PINMODE1=0x02<<20;

LPC_DAC->DACR=0x00007FC0; // BIAS 0, VALUE= DACR initialise a 1.65Volt 
		
}
	
}
