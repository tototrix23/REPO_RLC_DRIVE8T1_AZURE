/*
 * synchro.h
 *
 *  Created on: 18 nov. 2024
 *      Author: Christophe
 */

#ifndef SYNCHRO_SYNCHRO_H_
#define SYNCHRO_SYNCHRO_H_

#include <_core/c_common.h>
#include <rtc/rtc.h>

typedef enum e_synchro_state
{
    synchro_state_idle                    = 0,
    synchro_state_init                    = 1,
    synchro_state_wait_for_time_and_pulse = 2,
    synchro_state_wait_for_start_cycle    = 3,
    synchro_state_wait_down               = 4,
    synchro_state_wait_up                 = 5,
    synchro_state_last_cycle_resync       = 6,

}e_synchro_state;


typedef struct sSyncStimuli
{
    uint8_t   pps_flag; // Flag indiquant si un front montant est détécté sur le signal PPS
    st_rtc_t  rtc_info;
}sSyncStimuli;

// Structure contenant les informations relatives à la configuration de l'USB
typedef struct sSyncConfig
{
    uint8_t   panelCount;     // Nombre d'affiches
    uint16_t  shortTimeMs;  // Temps de défilement
    uint16_t  longTimeMs;       // Temps d'exposition
}sSyncConfig;

typedef struct sSyncCalcValues
{
    uint16_t  tShort;             // Durée d'un temps 'court' (durée de défilement + durée d'exposition)
    uint16_t  tLong;              // Durée d'un temps 'long' (tShort + durée d'exposition car double exposition)
    uint8_t   longPulsesByCycle;  // Nombre de pulses long dans un cycle
    uint8_t   shortPulsesByCycle; // Nombre de pulses courts dans un cycle
    uint16_t  longTimeByCycle;    // Nombre de temps longs dans un cycle
    uint16_t  shortTimeByCycle;   // Nombre de temps courts dans un cycle
    uint16_t  cyclesByHour;       // Nombre de cycles entiers dans une heure
    double dTShort;
    double dTLong;
    double dTCycle;
}sSyncCalcValues;

// Structure contenant les données relatives au fonction de la machine à état
typedef struct sSyncProcess
{
    e_synchro_state   smState;      // Etat en cours de la machine
    uint16_t  currentCycle;         // Compteur de cycle (valeur maximale = sSyncCalcValues.cyclesByHour)
    uint8_t   currentIndex;         // Index qui permet de savoir sur quelle affiche l'algorithme se trouve
    uint16_t  ledTime;              // Compteur de temps qui permet de gérer l'allumage de la LED
    uint32_t  uTimerValue;          // Compteur de temps
    uint32_t  uTimerValue2;         // Compteur de temps
    double dTriggerMs;              //
    uint32_t  uTriggerMs;           //
    bool_t  longPulseFlag;         // Indique que l'émission d'un pulse long est requise
    bool_t  shortPulseFlag;        // Indique que l'émission d'un pulse court est requise
}sSyncProcess;


// Structure globale qui contient des instances des structures précédentes
typedef struct s_synchro
{
    struct sSyncStimuli structStimuli;
    struct sSyncConfig structConfig;
    struct sSyncCalcValues structCalcValues;
    struct sSyncProcess structProcess;
    bool_t active;
}s_synchro;


return_t synchro_init(void);
return_t synchro_set_params(uint8_t panels,uint16_t shortTimeMs,uint16_t longTimeMs);
void synchro_clear_signals(void);
bool_t synchro_is_active(void);
bool_t synchro_is_long_signal(void);
bool_t synchro_is_short_signal(void);


void synchro_process(void);

#endif /* SYNCHRO_SYNCHRO_H_ */
