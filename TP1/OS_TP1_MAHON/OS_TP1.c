/*
    Nom du fichier: OS_TP1.c
    Auteur: Grégoire MAHON EI2I4 II (Groupe B)
    RTOS TP1
    Description: 
        Ce fichier contient le code source pour le premier TP de RTOS.
        Le code implémente les bases d'un système d'exploitation minimaliste
        sur une plateforme LPC17xx. Il inclut la création et la gestion de tâches, 
        la gestion des interruptions et une simple commutation de contexte.
*/


#include <stdio.h>
#include <LPC17xx.h>
#include "OS_TP1.h" // Fichier d'entête créé

// Déclaration du type TCB_t comme alias de tskTCB.
typedef tskTCB TCB_t;

// Déclaration de variables globales.
TCB_t *pxCurrentTCB; // Pointeur vers le Bloc de Contrôle de Tâche (TCB) courant.
uint32_t TickCount = 0; // Compteur de ticks système.
#define NBTASK 2 // Nombre de tâches.
TCB_t liste_TCB[NBTASK+1]; // Tableau des TCBs pour les tâches.
#define STACKSIZE 64 // Taille de la pile pour chaque tâche.
uint32_t stack_tache1[STACKSIZE]; // Pile pour la tâche 1.
uint32_t stack_tache2[STACKSIZE]; // Pile pour la tâche 2.
uint32_t stack_idle[STACKSIZE]; // Pile pour la tâche "Idle".
// Déclarations de fonctions
void Task_Yield(void);
void Task_kill(void);

// Initialisation des GPIOs pour les LED.
void init_gpio() {
    LPC_SC->PCONP |= (1 << 15);// Active l'alimentation des GPIOs.
    LPC_GPIO3->FIODIR |= 0x03 << 25; // Configure les ports pour les LED.
    LPC_GPIO0->FIODIR |= 0x01 << 22;
}

// Fonctions pour activer ou désactiver les LED.
void actualise_ledV(int consV) {
    if (consV) LPC_GPIO3->FIOSET = 1 << 25;
    else LPC_GPIO3->FIOCLR = 1 << 25;
}

void actualise_ledB(int consB) {
    if (consB) LPC_GPIO3->FIOSET = 1 << 26;
    else LPC_GPIO3->FIOCLR = 1 << 26;
}

void actualise_ledR(int consR) {
    if (consR) LPC_GPIO0->FIOSET = 1 << 22;
    else LPC_GPIO0->FIOCLR = 1 << 22;
}
// Initialisation du processeur.
void init_proc() {
    init_gpio();
    SysTick_Config(100000); // Configuration du timer SysTick.
}

// Fonction pour céder volontairement le CPU à une autre tâche.
void Task_Yield() {
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Fonction de la tâche "Idle".
void Task_idle() {
    while(1) {
        Task_Yield(); // Boucle infinie, cède le CPU.
    }
}

// Fonction appelée lorsqu'une tâche se termine.
void Task_kill() {
    while(1);
}

// Création d'une tâche
void Task_create(void (*pxFunctionName)(), TCB_t *pxTCB, uint32_t *plStack, uint32_t stackSize, uint32_t priority, void *pxParam) {
    int i; // etrange mais erreur de compilation si on declare i dans la boucle for
    uint32_t *pwrite_stack = &plStack[stackSize - 1];

    *(--pwrite_stack) = (uint32_t)0x01000000;  // xPSR
    *(--pwrite_stack) = (uint32_t)pxFunctionName;  // PC
    *(--pwrite_stack) = (uint32_t)Task_kill;  // LR

    for (i = 12; i > 0; i--) {
        *(--pwrite_stack) = (uint32_t)(i * 0x01010101);  // Valeurs fausses pour R4-R11
    }

    pxTCB->pxTopOfStack = pwrite_stack;
    pxTCB->ulPriority = priority;
    pxTCB->lState = 1;  // Task Ready state
    pxTCB->ulTickRDV = TickCount;
}

// Définition des tâches
void tache1() {
    printf("On est dans la tache1\n");
}

void tache2() {
    printf("On est dans la tache2\n");
}

void creation_des_taches() {
    Task_create(tache1, &liste_TCB[0], stack_tache1, STACKSIZE, 1, NULL);
    Task_create(tache2, &liste_TCB[1], stack_tache2, STACKSIZE, 2, NULL);
    Task_create(Task_idle, &liste_TCB[NBTASK], stack_idle, STACKSIZE, 0, NULL);
}

// Fonction pour démarrer la première tâche.
__asm void prvStartFirstTask(void) {
	// Code assembleur pour initialiser le contexte de la première tâche.
    PRESERVE8
    LDR R0, =0xE000ED08
    LDR R0, [R0]
    LDR R0, [R0]
    MSR MSP, R0
    CPSIE I
    CPSIE F
    DSB
    ISB
    SVC 0
    NOP
    NOP
}

void lancement_OS() {
    creation_des_taches(); // Crée les tâches.
    pxCurrentTCB = &liste_TCB[0]; // Définit la première tâche à exécuter.
    prvStartFirstTask();
}

int main(void) {
    init_proc();
    lancement_OS();
    while(1); // Boucle infinie (ne devrait jamais être atteinte).
}
