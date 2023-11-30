
#include <stdio.h>
#include <LPC17xx.h>                         /* LPC17xx definitions           */
//les fichiers et lignes indiqu�es sont dans le TP1_FREERTOS
// ****** structure et variables indispensable � l'OS **********
typedef struct tskTaskControlBlock 		
{//remplir avec les consignes de TP (ordre des variables � respecter), et le cours
	// si besoin s'inspirer du tCB de task.c ligne 252 (pas de listes pour nous...)
	// uint32_t est la meme chose qu'un unsigned long, unsigned int
  // v�rifier dans le map !!!!	
	uint32_t * pxTopOfStack;
	uint32_t ulPriority;
	int32_t lState;
	uint32_t ulTickRDV;
} tskTCB;

typedef tskTCB TCB_t;
// variables indispensables pour faire tourner notre OS
// penser � les recopier dans les variable externes
TCB_t * pxCurrentTCB;
uint32_t TickCount=0; // incrémentée à chaque interruption

// d�finir le nombre fixe de t�che utilis�es sans compter idle...
#define NBTASK 2
//d�clarer tableau de TCB
TCB_t liste_TCB[NBTASK+1];

// d�finir la taille des piles minimales
#define STACKSIZE 64
// pr�d�clarer toutes les piles sous forme de tableaux

uint32_t stack_tache1[STACKSIZE];
uint32_t stack_tache2[STACKSIZE];
uint32_t stack_idle[STACKSIZE];




// ***********  variables propres au projet exemple, � d�truire...


int i=0;
int cpt_it=0; 
unsigned long tempo=0;
volatile uint8_t  clock_1s=0;
 
 //si vous gal�rez sur partager des variables d'un fichier � l'autre,
 // des �tiquettes de fonctions d'un fichier � l'autre et que le temps
 // d�file sans que vous arriviez � vous en sortir, ALORS
 // d�commenter la ligne ci dessous, sortez le fichier Irq_OS_TP1.c 
 // du project, et ne red�finissez aucune variable avec le mot extern
 // c'est tr�s sale, mais �a peut gagner du temps.... 
 // #include "Irq_OS_TP1.c"
 
//*********************************************************** 
//P3.25 led verte //P3.26 led bleu // P0.22 led rouge
void init_gpio()
{
  LPC_SC->PCONP |= (1 << 15);/* power on sur GPIO & IOCON */
//P3.25 led verte
//P3.26 led bleu
// P0.22 led rouge
  LPC_GPIO3->FIODIR |= 0x03<<25;   
	//P3.25 et P3.26 en sortie
	LPC_GPIO0->FIODIR |= 0x01<<22;   
	//P0.22 en sortie

}
// une fonction
void actualise_ledV(int consV) 
{  if (consV) {LPC_GPIO3->FIOSET = 1<<25;}
   else {LPC_GPIO3->FIOCLR = 1<<25;} 
}

// une fonction 
void actualise_ledB(int consB)
{ if (consB) {LPC_GPIO3->FIOSET = 1<<26;} 
  else {LPC_GPIO3->FIOCLR = 1<<26;}
}
// une fonction
void actualise_ledR(int consR)
{ if (consR) {LPC_GPIO0->FIOSET = 1<<22;} 
  else {LPC_GPIO0->FIOCLR = 1<<22;}
}

void init_timer0()
{ LPC_SC->PCONP     |= (1 << 1); /*power on sur timer0 , d�j� sous tension par d�faut */
	LPC_TIM0->TCR =0x03;
  LPC_TIM0->CTCR =0x00;
  LPC_TIM0->MR0 =128; // 25000000/5;
	LPC_TIM0->MCR = 0x03;
//  NVIC_EnableIRQ(TIMER0_IRQn)  ;
	LPC_TIM0->TCR =0x01;
}
//*****IT TIMER0 � utiliser en fin de tp***************************************
void TIMER0_IRQHandler(void)
{ cpt_it++;
  actualise_ledR(cpt_it); 
  LPC_TIM0->IR=1; //acquittement
}	
	
//-----------------------------------------------
void init_proc()
{ init_gpio();
	//init_timer0();//juste en fin de tp
  SysTick_Config(100000);
	/* baremetal interrupt each 100000 cycles (1ms) */

}

void Task_Yield()
{// il faudra armer la commutation de tache
	// on s'inspirera de portYIELD() dans portmacro.h ligne 83
// en �vitant les cascades de #define ....	
// mais pas avant le point 4 de l'ordre de r�alisation
	// dans un premier temps, on �crit des lignes 
	//juste pour v�rifier qu'on passera par l�
	actualise_ledR(1);
	actualise_ledR(0);
	// est ce qu'on va bien aller au bon endroit
}
void Task_idle(/*a modifier*/void ) {// cette tache provoquera une commutation en boucle infinie
        // pour tenter de redonner la main � une vraie tache

        // il suffit d'appeler Task_Yield en boucle
        // mais dans un premier temps, on va tester qu'on atteint bien
        // Task_kill si on a oubli�  de faire la boucle infinie...	


          actualise_ledV(1);
          actualise_ledV(0);
          // est ce qu'on va bien aller au bon endroit 
          //(le LR r�cup�r� sur notre pile est il bien?)	
        }	
/*void Task_idle(void *pvParameters) {
    // Cette tâche provoquera une commutation en boucle infinie
    // pour tenter de redonner la main à une vraie tâche

    while(1) {
        // Appel à Task_Yield pour provoquer une commutation de tâche
        Task_Yield();

        // Pour tester qu'on atteint bien Task_kill si on oublie de faire la boucle infinie,
        // on utilise les fonctions d'actualisation des LEDs.
        // Ces lignes peuvent être commentées ou supprimées une fois le test effectué.
        actualise_ledV(1);
        actualise_ledV(0);

        // Ici, vous pouvez ajouter un petit délai si nécessaire pour ralentir la commutation.
    }

    // En théorie, on ne devrait jamais sortir de la boucle while(1).
    // Si cela se produit, c'est une indication qu'il y a une erreur.
    // Dans ce cas, la tâche doit être "tuée" en appelant Task_kill.
    Task_kill();
}*/

void Task_kill()
{ // on devra arriver ici si une tache quitte
	// on lui donnera alors un etat tache d�truite et on provoquera une commutation de tache 
  // ainsi, elle ne reprendra jamais la main (la tache idle ne devra pas etre d�truite)
	actualise_ledB(1);
	actualise_ledB(0);
}	
//--------------- Main Program -----------------------------------
/*void Task_create(void * pxnomFonction, TCB_t * pxnomTCB, uint32_t * plStack, uint32_t stackSize, uint32_t priority, void * pxParam)
{// s'inspirer du cours et de la fonction xcreateTaskStatic(....)
 // pour les parametres � passer � notre fonction
 // puisqu'on ne fait pas d'allocation dynamique de m�moire	
 // on fera tout dans cette fonction, m�me la fausse pile....
	// on se d�brouillera pour que	chaque registre contienne son num�ro :
	// R1 contient 0x01010101, R12.... 0x12121212 ...
	// sauf ceux qui ont un role important � jouer bien sur
	// Faire attention on fait du decrement before
	//uint32_t pwrite_stack = *plStack + stackSize; //prend la fin de pile
	//ou
	uint32_t pwrite_stack = plStack[stackSize]; // prend la fin de pile
	// mettre � jour les quatre variables du TCB
	
	
}*/
__asm void vPortSVCHandler( void )
{
	PRESERVE8
	extern pxCurrentTCB
	ldr	r3, = pxCurrentTCB	/* Restore the context. */
	ldr r1, [r3]			/* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
	ldr r0, [r1]			/* The first item in pxCurrentTCB is the task top of stack. */
	ldmia r0!, {r4-r11}		/* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
	msr psp, r0				/* Restore the task stack pointer. */
	isb
	mov r0, #0
	msr	basepri, r0
	orr r14, #0xd
	bx r14
}
__asm void prvStartFirstTask( void )
{
	PRESERVE8

	/* Use the NVIC offset register to locate the stack. */
	ldr r0, =0xE000ED08
	ldr r0, [r0]
	ldr r0, [r0]

	/* Set the msp back to the start of the stack. */
	msr msp, r0
	/* Globally enable interrupts. */
	cpsie i
	cpsie f
	dsb
	isb
	/* Call SVC to start the first task. */
	svc 0
	nop
	nop
}


void Task_create(void * pxFunctionName, TCB_t *pxTCB, uint32_t *plStack, uint32_t stackSize, uint32_t priority, void *pxParam) {
    // Pointe vers le haut de la pile (en tenant compte de la d�cr�mentation pr�alable)
    uint32_t *pwrite_stack = &plStack[stackSize]; // pl car pointeur sur un long

    // Initialiser la pile pour simuler un contexte d'interruption
    // MODE DECREMENT BEFORE
		    
	  // Initialisation de xPSR - �tat du registre avec le bit T d�fini pour le mode Thumb
    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x01000000;   // xPSR (0x01000000)
	
    pwrite_stack--;
    *pwrite_stack = (uint32_t)pxFunctionName & 0xfffffffeUL; // PC

    pwrite_stack--;
    *pwrite_stack = (uint32_t)Task_kill;    // LR -> Renvoi vers la fonction Task_kill

    // Initialisation des registres R12, R3, R2, R1, R0
    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x12121212;   // R12

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x03030303;   // R3

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x02020202;   // R2

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x01010101;   // R1

    pwrite_stack--;
    *pwrite_stack = (uint32_t)pxParam;      // R0

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x11111111;   // R11

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x10101010;   // R10

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x09090909;   // R9

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x08080808;   // R8

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x07070707;   // R7

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x06060606;   // R6

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x05050505;   // R5

    pwrite_stack--;
    *pwrite_stack = (uint32_t)0x04040404;   // R4


    // Mise à jour de pxTopOfStack dans la structure TCB
    pxTCB->pxTopOfStack = pwrite_stack;
    pxTCB->ulPriority = priority;
    pxTCB->lState = priority; // état 1,2,3,4 car priority est un uint32_t et on est au démarrage: 
    /* priorité de la plus basse (celui de la tâche idle qui est donc plus prioritaire que les
    tâche en attente) à la plus élevée ( on peut ne pas se limiter à 4 niveaux si on souhaite
    plus de niveaux de priorité)*/
    pxTCB->ulTickRDV = TickCount; // Initialiser le rendez-vous de tick
}

void tache1(){

    printf("On est dans la tache1\n");

}

void tache2(){

    printf("On est dans la tache2\n");

}

/*void creation_des_taches(void)
{	
Task_create();// � modifier
//Task_create(......);
//Task_create(.....);
	
}*/

void creation_des_taches(void)
{    
    // Création de la tâche 1
    Task_create(
        tache1,     									// Fonction exécutée par la tâche pxFunctionName = 
        &liste_TCB[0],                // TCB de la tâche pxTCB = 
        stack_tache1,                 // Pile de la tâche plStack = 
        STACKSIZE,                    // Taille de la pile stackSize = 
        1,                            // Priorité de la tâche priority = 
        NULL                          // Argument passé à la tâche (NULL si aucun) pxParam = 
    );

    // Création de la tâche 2
    Task_create(
        tache2,     									// Fonction exécutée par la tâche
        &liste_TCB[1],                // TCB de la tâche
        stack_tache2,                 // Pile de la tâche
        STACKSIZE,                    // Taille de la pile
        2,                            // Priorité de la tâche
        NULL                          // Argument passé à la tâche (NULL si aucun)
    );

    // Création de la tâche idle
    Task_create(
        (void (*)(void *))Task_idle,  // Fonction exécutée par la tâche idle
        &liste_TCB[NBTASK],           // TCB de la tâche idle
        stack_idle,                   // Pile de la tâche idle
        STACKSIZE,                    // Taille de la pile
        0,                            // Priorité la plus basse
        NULL                          // Argument passé à la tâche (NULL si aucun)
    );
}



void lancement_OS(void)
{//s'inspirer de tasks.c ligne 1967, on simplifiera et on regroupera tout
 //	dans cette seule fonction (sauf pour la partie assembleur) 
 // au lieu d'avoir une cascade d'appel
  
	//creer la tache idle par un Task_create
	//initialiser les variables de l'os
	
	// configurer le timer TICK � l'aide 
	// de la fonction SysTick_Config appel� dans init_proc ( a d�placer)
	
	// d�finir les priorit�s des IT � leur plus faible valeur et et les autoriser
	//on utilisera NVIC_SetPriority (NOM_IRQn, Niveau de priorit� choisi);
	//la liste des IRQn est dans LPC17XX.h ligne 41
	// la priorit� la plus basse est la priorit� 31, la plus haute 0
	// code dans core_cm3.h ligne 1638

	
	// lancer la premi�re t�che
	prvStartFirstTask(); // Fonction récupérée dans le code de FreeRTOS 'Port.c' Ligne 225.
	
	// si on quitte, on est plant�...
}
void gestion_plantage_OS(void)
{
	while(1) ; // on est plant� si on arrive ici
}	
int main (void) {
  init_proc();
	//SysTick_Config(SystemCoreClock/20);      
 	/* Generate interrupt each 1 ms */
	creation_des_taches();
  lancement_OS();
	gestion_plantage_OS();
}
