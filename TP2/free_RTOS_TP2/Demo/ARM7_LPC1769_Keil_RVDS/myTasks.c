/*
 * Auteur : Grégoire MAHON EI2I4
 * Groupe : II GROUPE B
 * Fichier : myTasks.c
 * Projet : Chenillard et Contrôle de Véhicule - TP2 FreeRTOS
 */

#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* le fichier .h du TP  */
#include "myTasks.h"

#include "LPC17xx.h"

// #define ledSTACK_SIZE		128

void adjustChenillardSpeed(uint32_t speedValue);

/* déclaration des diverses taches avec la fonction prototype */
//static portTASK_FUNCTION_PROTO( vLed1Task, pvParameters );//exemple de déclaration

static void vChenillardTask(void *pvParameters);
static void vMotorControlTask(void *pvParameters);
static void vSensorTask(void *pvParameters);
static void vSoundTask(void *pvParameters);

/*-------------------variables globales propres aux taches------------*/
unsigned char argument_tache1 = 3;

uint32_t tableau_son1[256];
uint32_t tableau_son2[256];
uint32_t *tableau_son = tableau_son1;
uint8_t index_son = 0;
volatile uint8_t alterne = 0;
uint32_t val_son;

// ADC
uint32_t val_ADC0;
uint32_t val_ADC1;
uint32_t val_ADC2;

// Valeurs des boutons poussoirs
#define BP_ON 1
#define BP_OFF 0
unsigned char readButton1(void);
unsigned char readButton2(void);

#define VITESSE_MAX 50 // Exemple de valeur maximale pour la vitesse
volatile int differentiel_vitesse = 0;

// CHENILLARD
volatile int val_attente = 500; // Exemple de valeur initiale pour la vitesse du chenillard
volatile int vitesse_moyenne = 0; // A CHANGER POUR CONNAITRE LETAT DU VEHICULE
#define ANIMATION_LENGTH_MOVING 8
#define ANIMATION_LENGTH_FREEZED 14

const uint32_t sequence_moving[ANIMATION_LENGTH_MOVING] = {
    // Séquence pour le véhicule en mouvement
    1 << 7, 1 << 6, 1 << 5, 1 << 4, 
    1 << 3, 1 << 4, 1 << 5, 1 << 6
};

const uint32_t sequence_freezed[ANIMATION_LENGTH_FREEZED] = {
    // Séquence pour le véhicule à l'arrêt
    1 << 7, 3 << 6, 7 << 5, 15 << 4, 31 << 3, 
    30 << 4, 28 << 5, 24 << 6, 16 << 7, 8 << 6,
    7 << 5, 3 << 6, 1 << 7, 1 << 6
};


/*--------------------- TOOLS --------------------- */
uint32_t min(uint32_t a, uint32_t b) {
    return (a < b) ? a : b;
}

/*---------------------INITIALISATION DES PERIPHERIQUES---------------------*/

void initChenillard() {
    LPC_GPIO2->FIODIR |= 0xF8; // Configure P2.3 à P2.7 en sortie
}

void initCodeur() {
    LPC_GPIO2->FIODIR &= ~((1 << 11) | (1 << 12) | (1 << 10)); // Configurer P2.11, P2.12 et P2.10 comme entrées
}

void init_dac() {
    LPC_PINCON->PINSEL1 |= 0x02 << 20;
    LPC_PINCON->PINMODE1 = 0x02 << 20;

    LPC_DAC->DACR = 0x00007FC0; // DAC initialise à 1.65 Volt
}


void init_adc() {
    LPC_SC->PCONP |= 1 << 12; // Mettre l'ADC sous tension
    LPC_ADC->ADCR |= 1 << 21; // Sortir du Power Down
    LPC_PINCON->PINSEL1 |= (0x01 << 14) | (0x01 << 16) | (0x01 << 18); // 3 entrées analogiques
    LPC_PINCON->PINMODE1 |= (0x02 << 14) | (0x02 << 16) | (0x02 << 18); // 3 entrées sans pull-up ni pull-down
    LPC_ADC->ADCR |= 1 << 8; // Horloge de l'ADC à 12.5 MHz
    LPC_ADC->ADINTEN = 0x04; // AD0.2 source de l'IT de fin de conversion
    LPC_ADC->ADCR |= 0x7; // Lancer les conversions AD0.0, AD0.1, AD0.2
    NVIC_SetPriority(ADC_IRQn, 5); // Choisir une priorité compatible avec l'OS
    NVIC_EnableIRQ(ADC_IRQn); // Activer l'interruption de l'ADC
    // Lancement des conversions continu par burst
    LPC_ADC->ADCR |= 1 << 16; 
}


void init_PWM() {
    LPC_SC->PCONP |= (1 << 6); // Power on pour PWM
    LPC_PINCON->PINSEL4 |= 0x00000005; // P2.0 à P2.1 deviennent PWM1.1, PWM1.2
    LPC_PWM1->TCR = 0x03;
    LPC_PWM1->PCR |= (0x03 << 9); // Deux PWM activés
    LPC_PWM1->MR0 = 4094; 
    LPC_PWM1->MR1 = 2048;
    LPC_PWM1->MR2 = 2048;
    LPC_PWM1->LER = 0x07;   
    LPC_PWM1->MCR = 0x02;   
    LPC_PWM1->TCR = 0x09;
    // Pas d'IT activée
}

void ADC_IRQHandler(void) {
    LPC_ADC->ADCR &= ~(1 << 16); // Arrêt de la conversion
    val_ADC0 = (LPC_ADC->ADDR0 >> 6) & 0x3FF; // Récupération des valeurs converties
    val_ADC1 = (LPC_ADC->ADDR1 >> 6) & 0x3FF;
    val_ADC2 = (LPC_ADC->ADDR2 >> 6) & 0x3FF;
    // Ici, on pourrait notifier une tâche qui va exploiter les conversions
    // ...
    // Relancer les conversions
    LPC_ADC->ADCR |= 1 << 16; 
}

void adjustMotorSpeed(uint32_t speedMotor1, uint32_t speedMotor2) {
    LPC_PWM1->MR1 = speedMotor1;
    LPC_PWM1->MR2 = speedMotor2;
    LPC_PWM1->LER |= 0x03; // Mise à jour des registres Match
}

void toggleLEDsInSequence() {
	  // Fonction pour le chenillard, allume les LEDs en séquence.
    static uint32_t ledState = 0;

    LPC_GPIO2->FIOCLR = 0x0F8; // Éteindre toutes les LEDs
    LPC_GPIO2->FIOSET = (1 << (3 + ledState)); // Allumer une LED

    ledState = (ledState + 1) % 5; // Passer à la LED suivante
}


void calcul_tableau(uint32_t *tab) {
    static uint32_t index_a_virgule = 0;
    uint32_t ech_avant, ech_apres;
    uint8_t pos = 0;
    uint8_t inc_freq; // Assurez-vous que cette variable est définie correctement
    uint8_t virgule, index;
    uint32_t val_son_prov;
    uint32_t son1K[37]; // Assurez-vous que ce tableau est défini et rempli correctement

    do {
        index_a_virgule += inc_freq;
        if (index_a_virgule >= 36 * 256) {
            index_a_virgule -= 36 * 256;
        }

        virgule = index_a_virgule & 255;
        index = index_a_virgule >> 8;

        ech_avant = son1K[index];
        ech_apres = son1K[index + 1];

        val_son_prov = ((ech_avant * (256 - virgule) + ech_apres * virgule)) >> 8;
        // Ici, ajoutez la logique pour gérer le volume
        // val_son_prov = ...;

        tab[pos] = (val_son_prov << 2) & 0xFFC0;
    } while (++pos);
}

void maj_dacr() {
    LPC_DAC->DACR = val_son; // Mise à jour du DAC
    val_son = tableau_son[index_son];
    if (!(++index_son)) {
        alterne ^= 1;
        tableau_son = (alterne) ? tableau_son2 : tableau_son1;
    }
}

unsigned char readButton1() {
    // Exemple de lecture de l'état du bouton sur P0.0
    return (LPC_GPIO0->FIOPIN0 & 1) ? BP_OFF : BP_ON;
}

unsigned char readButton2() {
    // Exemple de lecture de l'état du bouton sur P0.1
    return (LPC_GPIO0->FIOPIN0 & 2) ? BP_OFF : BP_ON;
}

unsigned char lectureEtatCodeur() {
    return (LPC_GPIO2->FIOPIN >> 11) & 0x03; // Lecture des états des pins du codeur
}

/*--------------------- TACHES --------------------- */

static void vSoundTask(void *pvParameters) {
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Recalcul toutes les 1000 ms
    // Initialisation du DAC
    init_dac();
    xLastWakeTime = xTaskGetTickCount(); // Initialiser la dernière heure de réveil

    while (1) {
        maj_dacr();

        // Vérifier si le temps écoulé depuis le dernier calcul est suffisant
        if (xTaskGetTickCount() - xLastWakeTime > xFrequency) {
            calcul_tableau(tableau_son);
            xLastWakeTime = xTaskGetTickCount(); // Réinitialiser la dernière heure de réveil
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // DELAI A REVOIR
    }
}

static void vChenillardTask(void *pvParameters) {
    int indexMoving = 0, indexFreezed = 0;
    initChenillard();

    while (1) {
        LPC_GPIO2->FIOCLR = 0xF8; // Éteindre toutes les LEDs avant de changer l'état

        if (vitesse_moyenne > 0) {
            LPC_GPIO2->FIOSET = sequence_moving[indexMoving];
            indexMoving = (indexMoving + 1) % ANIMATION_LENGTH_MOVING;
        } else {
            LPC_GPIO2->FIOSET = sequence_freezed[indexFreezed];
            indexFreezed = (indexFreezed + 1) % ANIMATION_LENGTH_FREEZED;
        }

        vTaskDelay(pdMS_TO_TICKS(val_attente));
    }
}

static void vMotorControlTask(void *pvParameters) {
		uint32_t speedMotor1 = 2048; // Vitesse initiale pour le moteur 1
    uint32_t speedMotor2 = 2048; // Vitesse initiale pour le moteur 2
	
    // Initialisation du PWM pour le contrôle des moteurs
    init_PWM();

    while (1) {
        // Logique pour ajuster la vitesse des moteurs
        if (readButton1() == BP_ON) {
            // Augmenter la vitesse du moteur 1
            speedMotor1 = min(speedMotor1 + 100, 4095);
        }
        if (readButton2() == BP_ON) {
            // Augmenter la vitesse du moteur 2
            speedMotor2 = min(speedMotor2 + 100, 4095);
        }

        // Appliquer les vitesses ajustées aux moteurs
        adjustMotorSpeed(speedMotor1, speedMotor2);

        vTaskDelay(pdMS_TO_TICKS(100)); // Ajustez ce délai selon vos besoins
    }
}

static void vSensorTask(void *pvParameters) {
		uint8_t old_codeur = 0;
    uint8_t codeur = 0;
    initCodeur(); // Initialiser le codeur

    while (1) {
        codeur = lectureEtatCodeur(); // Lire l'état actuel du codeur

        switch (old_codeur) {
            case 0: if (codeur == 1) differentiel_vitesse++; if (codeur == 2) differentiel_vitesse--; break;
            case 1: if (codeur == 3) differentiel_vitesse++; if (codeur == 0) differentiel_vitesse--; break;
            case 2: if (codeur == 0) differentiel_vitesse++; if (codeur == 3) differentiel_vitesse--; break;
            case 3: if (codeur == 2) differentiel_vitesse++; if (codeur == 1) differentiel_vitesse--; break;
        }

        // Limiter la vitesse différentielle à VITESSE_MAX
        if (differentiel_vitesse > VITESSE_MAX) differentiel_vitesse = VITESSE_MAX;
        if (differentiel_vitesse < -VITESSE_MAX) differentiel_vitesse = -VITESSE_MAX;

        old_codeur = codeur;

        // Notifier la tâche de mise à jour des PWM si nécessaire
        if (old_codeur != codeur) {
            // Code pour notifier la tâche PWM (à implémenter)
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Fréquence de 200 Hz
    }
}

void vInit_myTasks( UBaseType_t uxPriority )
{	
		//ici on cree toutes les taches et tous les sémaphores, mutex ....
		//xTaskCreate( vLed1Task, "LED1", ledSTACK_SIZE, &argument_tache1, uxPriority, ( TaskHandle_t * ) NULL );

    // Chenillard
    xTaskCreate(vChenillardTask, "Chenillard", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);

    // Contrôle des moteurs
    xTaskCreate(vMotorControlTask, "Motor Control", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);

    // Capteurs
    xTaskCreate(vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);

    // Gestion du son
    xTaskCreate(vSoundTask, "Sound", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);
}
