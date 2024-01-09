
uint32_t  tableau_son1[256] ;
uint32_t  tableau_son2[256] ;
uint32_t  * tableau_son = tableau_son1;
uint8_t   index_son=0;
volatile uint8_t  alterne=0;


void maj_dacr()
{ LPC_DAC->DACR=val_son; // pour �tre le plus pr�cis possible sur l'instant de mise � jour
                         // on met � jour puis on calcule la prochaine valeur...
	val_son=tableau_son[index_son];
	if(!(++index_son)) // test de changement de tableau (256 cases)
	{// prevenir le main ou autre qu'on change de tableau pour faire recalculer le tableau	
	// changer de tableau
	 alterne ^=1;
	 tableau_son = (alterne) ? tableau_son2 :tableau_son1;
	}
} 


// fonction � appeler dans une tache notifi�e d'un besoin de recalcul d'un tableau d �chantillons
// penser � initialiser le son
calcul_tableau(uint32_t * tab)
{static uint32_t index_a_virgule=0; //on doit r�utiliser la valeur pour assurer la continuit�
 uint32_t ech_avant, ech_apres;	
 uint8_t pos=0;// la boucle do while va etre execut�e 256 fois
    do
    {//mise � jour de inc_freq
	  // tester ici si une rampe son est disponible, alors utiliser un �l�ment de la rampe pour inc_freq
     // sinon on utilise la derni�re valeur du tableau de la rampe (vitesse constante)
     // penser � remettre l indicateur de disponibilit� de rampe � 0 en fin de boucle
     
     // gestion d'une lecture � vitesse variable
    index_a_virgule+=inc_freq;// index_a_virgule contient 8bits de poids fort de vrai index et 8 bits apr�s la virgule
	if(index_a_virgule>=36*256) {index_a_virgule-=36*256;}
	// partie apr�s la virgule et index du tableau de son d origine
	virgule=index_a_virgule&255; index=index_a_virgule>>8;
	// recup�ration des echantillons avant et apres
	ech_avant=son1K[index];ech_apres=son1K[index+1];//astuce, le tableau contient 37 cases
	// moyenne pond�r�e par la valeur de l index � virgule
	val_son_prov = ((ech_avant * (256-virgule) + ech_apres*virgule))>>8  ;
	// maintenant , gerer ici le volume...en tenant compte d'une rampe de volume �ventuelle
	//attention on doit amplifier ou att�nuer l'�cart par rapport � la valeur 512 et toujours etre centr� � 512
	val_son_prov = ....;
	// mise � jour du bloc son...
	tab[pos] = 	(val_son_prov<<2)& 0xFFC0; // pour etre compatible avec le registre DACR
    } while(++pos) ;

void init_dac()
{
	 // attribution de la patte P0.26 � la sortie DAC
		LPC_PINCON->PINSEL1|=0x02<<20;
		LPC_PINCON->PINMODE1=0x02<<20;

LPC_DAC->DACR=0x00007FC0; // BIAS 0, VALUE= DACR initialise a 1.65Volt 
		
}
	
}
