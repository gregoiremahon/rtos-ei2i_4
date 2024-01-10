/*
    Nom du fichier: Irq_OS_TP1.c
    Auteur: Grégoire MAHON EI2I4 II (Groupe B)
    RTOS TP1
*/

#ifndef OS_TP1_H
#define OS_TP1_H

#include <stdint.h>

// Définition de la structure tskTaskControlBlock
typedef struct tskTaskControlBlock {
    uint32_t *pxTopOfStack; // Pointeur vers le sommet de la pile
    uint32_t ulPriority;    // Priorité de la tâche
    int32_t lState;         // État de la tâche
    uint32_t ulTickRDV;     // Rendez-vous basé sur le Tick pour le réveil de la tâche
} tskTCB;

// Déclaration des variables globales
extern tskTCB *pxCurrentTCB;
extern uint32_t TickCount;
extern tskTCB liste_TCB[];
extern uint32_t stack_tache1[];
extern uint32_t stack_tache2[];
extern uint32_t stack_idle[];

// Déclaration des constantes
#define NBTASK 2
#define STACKSIZE 64

// Déclaration des fonctions
void Task_create(void (*pxFunctionName)(), tskTCB *pxTCB, uint32_t *plStack, uint32_t stackSize, uint32_t priority, void *pxParam);
void Task_Yield(void);
void Task_idle(void);
void Task_kill(void);
void init_gpio(void);
void actualise_ledV(int consV);
void actualise_ledB(int consB);
void actualise_ledR(int consR);
void init_proc(void);
void tache1(void);
void tache2(void);
void creation_des_taches(void);
void lancement_OS(void);
void prvStartFirstTask(void);

#endif // OS_TP1_H
