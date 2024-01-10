/*
    Nom du fichier: Irq_OS_TP1.c
    Auteur: Grégoire MAHON EI2I4 II (Groupe B)
    RTOS TP1
				Le fichier gère les interruptions essentielles pour
        un système d'exploitation minimaliste, y compris le SysTick pour le
        comptage du temps et la commutation de contexte, ainsi que les
        interruptions PendSV et SVC pour la gestion avancée des tâches et des
        appels système.

        - SysTick_Handler: Gestionnaire d'interruption pour le timer SysTick.
          Il incrémente un compteur de tick et déclenche une commutation de
          contexte si nécessaire.
        - PendSV_Handler: Gestionnaire d'interruption pour PendSV. Utilisé
          principalement pour la commutation de contexte entre les tâches.
        - SVC_Handler: Gestionnaire d'interruption pour les appels système
          (Supervisor Call). Utilisé pour les opérations de niveau privilégié,
          comme démarrer la première tâche.
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
// appelée à chaque tick du timer SysTick.
void SysTick_Handler(void) {
    int i;
    TickCount++; // Incrémente le compteur de ticks système.

    // Boucle pour vérifier l'état de chaque tâche et déterminer si une commutation est nécessaire.
    for (i = 0; i <= NBTASK; i++) {
        // Si une tâche est en attente et que son rendez-vous est atteint, elle passe à l'état prêt.
        if (liste_TCB[i].lState == 2 && liste_TCB[i].ulTickRDV <= TickCount) {
            liste_TCB[i].lState = 1;
        }
    }
		// Déclenche une interruption PendSV pour effectuer une commutation de contexte si nécessaire.
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Gestionnaire d'interruption pour PendSV.
// Utilisé pour la commutation de contexte entre les tâches.
__asm void PendSV_Handler(void) {
    extern pxCurrentTCB; // Pointeur vers le TCB courant.
		extern vTaskSwitchContext; // Fonction pour changer de tâche.
    PRESERVE8
		// Sauvegarde l'état actuel de la tâche et prépare la pile pour la prochaine tâche.
    mrs r0, psp
    ldr r3, =pxCurrentTCB
    ldr r2, [r3]

    stmdb r0!, {r4-r11}
    str r0, [r2]

    stmdb sp!, {r3, r14}
    mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
    msr basepri, r0
    bl vTaskSwitchContext // Appelle la fonction de commutation de tâche.
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

// Gestionnaire d'interruption pour les appels système (Supervisor Call).
__asm void SVC_Handler(void) {
    PRESERVE8 // Préserve l'alignement de la pile.

		// Extrait les informations de l'appel système de la pile et les traite.
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

    // Recherche de la prochaine tâche prête à être exécutée
    for (i = 1; i <= NBTASK; i++) {
        nextTaskIndex = (currentTaskIndex + i) % NBTASK;
        if (liste_TCB[nextTaskIndex].lState == 1) {
            foundTask = 1;
            break;
        }
    }

    // Mise à jour de pxCurrentTCB si une tâche prête est trouvée
    if (foundTask) {
        pxCurrentTCB = &liste_TCB[nextTaskIndex];
        currentTaskIndex = nextTaskIndex;
    } else {
        pxCurrentTCB = &liste_TCB[NBTASK]; // Tâche Idle
    }
}
