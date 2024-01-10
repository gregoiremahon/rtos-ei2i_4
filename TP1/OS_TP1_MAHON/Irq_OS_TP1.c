/*
    Nom du fichier: Irq_OS_TP1.c
    Auteur: Gr�goire MAHON EI2I4 II (Groupe B)
    RTOS TP1
				Le fichier g�re les interruptions essentielles pour
        un syst�me d'exploitation minimaliste, y compris le SysTick pour le
        comptage du temps et la commutation de contexte, ainsi que les
        interruptions PendSV et SVC pour la gestion avanc�e des t�ches et des
        appels syst�me.

        - SysTick_Handler: Gestionnaire d'interruption pour le timer SysTick.
          Il incr�mente un compteur de tick et d�clenche une commutation de
          contexte si n�cessaire.
        - PendSV_Handler: Gestionnaire d'interruption pour PendSV. Utilis�
          principalement pour la commutation de contexte entre les t�ches.
        - SVC_Handler: Gestionnaire d'interruption pour les appels syst�me
          (Supervisor Call). Utilis� pour les op�rations de niveau privil�gi�,
          comme d�marrer la premi�re t�che.
*/

#include <LPC17xx.h>
#include "OS_TP1.h"

#ifndef configMAX_SYSCALL_INTERRUPT_PRIORITY
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY 255
#endif

#if configMAX_SYSCALL_INTERRUPT_PRIORITY == 0
    #error configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0. See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
#endif



// Gestionnaire d'interruption pour le timer SysTick.
// appel�e � chaque tick du timer SysTick.
void SysTick_Handler(void) {
    int i;
    TickCount++; // Incr�mente le compteur de ticks syst�me.

    // Boucle pour v�rifier l'�tat de chaque t�che et d�terminer si une commutation est n�cessaire.
    for (i = 0; i <= NBTASK; i++) {
        // Si une t�che est en attente et que son rendez-vous est atteint, elle passe � l'�tat pr�t.
        if (liste_TCB[i].lState == 2 && liste_TCB[i].ulTickRDV <= TickCount) {
            liste_TCB[i].lState = 1;
        }
    }
		// D�clenche une interruption PendSV pour effectuer une commutation de contexte si n�cessaire.
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Gestionnaire d'interruption pour PendSV.
// Utilis� pour la commutation de contexte entre les t�ches.
__asm void PendSV_Handler(void) {
    extern pxCurrentTCB; // Pointeur vers le TCB courant.
		extern vTaskSwitchContext; // Fonction pour changer de t�che.
    PRESERVE8
		// Sauvegarde l'�tat actuel de la t�che et pr�pare la pile pour la prochaine t�che.
    mrs r0, psp
    ldr r3, =pxCurrentTCB
    ldr r2, [r3]

    stmdb r0!, {r4-r11}
    str r0, [r2]

    stmdb sp!, {r3, r14}
    mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
    msr basepri, r0
    bl vTaskSwitchContext // Appelle la fonction de commutation de t�che.
    mov r0, #0
    msr basepri, r0
    ldmia sp!, {r3, r14}

    ldr r1, [r3]
    ldr r0, [r1]
    ldmia r0!, {r4-r11}
    msr psp, r0
    bx r14
    nop
}

// Gestionnaire d'interruption pour les appels syst�me (Supervisor Call).
__asm void SVC_Handler(void) {
    PRESERVE8 // Pr�serve l'alignement de la pile.

		// Extrait les informations de l'appel syst�me de la pile et les traite.
    mrs r0, msp
    ldm r0, {r0-r3}
    ldr r2, =pxCurrentTCB
    ldr r2, [r2]
    str r0, [r2]
    bx lr // Retour de l'interruption.
}

void vTaskSwitchContext(void) {
    int i;
    static int currentTaskIndex = 0;
    int foundTask = 0;
    int nextTaskIndex;

    // Recherche de la prochaine t�che pr�te � �tre ex�cut�e
    for (i = 1; i <= NBTASK; i++) {
        nextTaskIndex = (currentTaskIndex + i) % NBTASK;
        if (liste_TCB[nextTaskIndex].lState == 1) {
            foundTask = 1;
            break;
        }
    }

    // Mise � jour de pxCurrentTCB si une t�che pr�te est trouv�e
    if (foundTask) {
        pxCurrentTCB = &liste_TCB[nextTaskIndex];
        currentTaskIndex = nextTaskIndex;
    } else {
        pxCurrentTCB = &liste_TCB[NBTASK]; // T�che Idle
    }
}
