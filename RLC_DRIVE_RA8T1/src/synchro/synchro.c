/*
 * synchro.c
 *
 *  Created on: 18 nov. 2024
 *      Author: Christophe
 */

#include <hal_data.h>
#include "synchro.h"
#include <time.h>
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>


#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "GPS"

static s_synchro sync_inst;
static c_timespan_t synchro_ts;


void sync_irq_callback (external_irq_callback_args_t * p_args)
{
    PARAMETER_NOT_USED(p_args);
    sync_inst.structStimuli.pps_flag = 1;
    sync_inst.structStimuli.rtc_info = rtc_get();
    h_time_update(&synchro_ts);
    tx_event_flags_set(&g_event_synchro_flag, 0b0001, TX_OR);
}


return_t synchro_init(void)
{
    return_t ret = X_RET_OK;
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    memset(&sync_inst,0x00,sizeof(s_synchro));

    h_time_update(&synchro_ts);

    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_sync_irq_ctrl, &g_sync_irq_cfg);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    err = R_ICU_ExternalIrqEnable(&g_sync_irq_ctrl);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    h_time_update(&synchro_ts);

    end:
    tx_mutex_put(&g_mutex_synchro);
    return ret;
}

return_t synchro_set_params(uint8_t panels,uint16_t shortTimeMs,uint16_t longTimeMs)
{
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);

    if(panels < 2)
    {
        sync_inst.structProcess.smState = synchro_state_idle;
        sync_inst.active = FALSE;
        goto end;
    }

    if((sync_inst.structConfig.panelCount != panels) ||
       (sync_inst.structConfig.shortTimeMs != shortTimeMs) ||
       (sync_inst.structConfig.longTimeMs != longTimeMs)
    )

    sync_inst.structConfig.panelCount = panels;
    sync_inst.structConfig.shortTimeMs = shortTimeMs;
    sync_inst.structConfig.longTimeMs = longTimeMs;



    sync_inst.structCalcValues.tShort = shortTimeMs;
    sync_inst.structCalcValues.tLong = longTimeMs;
    sync_inst.structCalcValues.longPulsesByCycle = 1;
    sync_inst.structCalcValues.shortPulsesByCycle = (uint8_t)(((sync_inst.structConfig.panelCount-1)*2)-1);

    sync_inst.structCalcValues.longTimeByCycle = (sync_inst.structConfig.panelCount == 2) ? 0 : 2;
    sync_inst.structCalcValues.shortTimeByCycle = (uint16_t)((sync_inst.structConfig.panelCount == 2) ? 2 : ((sync_inst.structConfig.panelCount-2)*2));
    volatile double theoricalCycleTime = (sync_inst.structCalcValues.longTimeByCycle * sync_inst.structCalcValues.tLong) + (sync_inst.structCalcValues.shortTimeByCycle * sync_inst.structCalcValues.tShort);
    volatile double theoricalCyclesByHour = (3600000.0) / (theoricalCycleTime);
    sync_inst.structCalcValues.cyclesByHour =  (uint16_t)(theoricalCyclesByHour);

    volatile double recalCycleTime = 3600000.0/sync_inst.structCalcValues.cyclesByHour;
    volatile double recalcTshort = (recalCycleTime - (sync_inst.structCalcValues.longTimeByCycle * sync_inst.structCalcValues.tLong)) / sync_inst.structCalcValues.shortTimeByCycle;
    volatile double recalcTlong = sync_inst.structCalcValues.tLong * 1.0;
    volatile double deltaTshort = recalcTshort- sync_inst.structCalcValues.tShort;
    volatile double dtemp1 = deltaTshort * (sync_inst.structCalcValues.shortTimeByCycle*1.0);
    volatile double dtemp2 = dtemp1 / (sync_inst.structCalcValues.longPulsesByCycle + sync_inst.structCalcValues.shortPulsesByCycle);

    sync_inst.structCalcValues.dTShort = (sync_inst.structCalcValues.tShort + dtemp2);
    sync_inst.structCalcValues.dTLong = (sync_inst.structConfig.panelCount == 2) ? 2 : (recalcTlong+ dtemp2);
    sync_inst.structCalcValues.dTCycle = (sync_inst.structCalcValues.shortTimeByCycle * sync_inst.structCalcValues.dTShort ) + (sync_inst.structCalcValues.longTimeByCycle * sync_inst.structCalcValues.dTLong);

    sync_inst.structStimuli.pps_flag = 0;
    tx_event_flags_set(&g_event_synchro_flag, 0b0000, TX_AND);
    sync_inst.structProcess.smState = synchro_state_init;

    end:
    tx_mutex_put(&g_mutex_synchro);
    return X_RET_OK;
}

void synchro_clear_signals(void)
{
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    sync_inst.structProcess.longPulseFlag = 0;
    sync_inst.structProcess.shortPulseFlag = 0;
    tx_mutex_put(&g_mutex_synchro);
}

bool_t synchro_is_active(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    ret = sync_inst.active;
    tx_mutex_put(&g_mutex_synchro);
    return ret;
}
bool_t synchro_is_long_signal(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    ret = sync_inst.structProcess.longPulseFlag;
    if(ret == TRUE)
        sync_inst.structProcess.longPulseFlag = FALSE;
    tx_mutex_put(&g_mutex_synchro);
    return ret;
}

bool_t synchro_is_short_signal(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    ret = sync_inst.structProcess.shortPulseFlag;
    if(ret == TRUE)
        sync_inst.structProcess.shortPulseFlag = FALSE;
    tx_mutex_put(&g_mutex_synchro);
    return ret;
}

void synchro_process(void)
{
    tx_mutex_get(&g_mutex_synchro,TX_WAIT_FOREVER);
    fsp_err_t err;

    if(sync_inst.structProcess.smState >= synchro_state_wait_for_start_cycle)
    {
        c_timespan_t ts = synchro_ts;
        bool_t elasped = FALSE;
        h_time_is_elapsed_ms(&ts, 10000, &elasped);
        if(elasped  == TRUE)
        {
            sync_inst.active = FALSE;
            sync_inst.structProcess.smState = synchro_state_init;
            LOG_W(LOG_STD,"Synchro by GPS lost");
        }
        else
        {
            if(sync_inst.active == FALSE)
            {
                LOG_I(LOG_STD,"Synchro by GPS OK");
                sync_inst.active = TRUE;
            }
        }
    }



    switch(sync_inst.structProcess.smState)
    {
        case synchro_state_idle:
            break;

        case synchro_state_init:
            tx_event_flags_set(&g_event_synchro_flag, 0b0000, TX_AND);
            sync_inst.structProcess.smState = synchro_state_wait_for_time_and_pulse;
            break;

        case synchro_state_wait_for_time_and_pulse:
        {
            ULONG actual_events;
            err = tx_event_flags_get(&g_event_synchro_flag, 0b0001, TX_AND_CLEAR, &actual_events, TX_NO_WAIT);
            if(err != FSP_SUCCESS)
                goto end;

            volatile st_rtc_t r = rtc_get();
            if(r.configured == FALSE)
                goto end;



            if(r.minute == 0 && r.second <= 1)
                goto end;

            sync_inst.structProcess.uTimerValue = (uint32_t)(r.time_ms_min_sec_ms);
            double dPos = (double)((double)sync_inst.structProcess.uTimerValue / sync_inst.structCalcValues.dTCycle);
            uint16_t uPos = (uint16_t)dPos;
            sync_inst.structProcess.currentCycle = (uint16_t)(uPos+1);
            if(sync_inst.structProcess.currentCycle > sync_inst.structCalcValues.cyclesByHour)
                goto end;

            sync_inst.structProcess.dTriggerMs = (sync_inst.structProcess.currentCycle * sync_inst.structCalcValues.dTCycle);
            sync_inst.structProcess.uTriggerMs = (uint32_t)( sync_inst.structProcess.dTriggerMs);
            sync_inst.structProcess.currentIndex = 0x00;
            sync_inst.structProcess.smState = synchro_state_wait_for_start_cycle;
        }
            break;

        case synchro_state_wait_for_start_cycle:
        {
            st_rtc_t r = rtc_get();
            sync_inst.structProcess.uTimerValue = (uint32_t)(r.time_ms_min_sec_ms);

            if( (sync_inst.structProcess.currentCycle >= sync_inst.structCalcValues.cyclesByHour &&
                 r.minute==0x00)
                ||
                (sync_inst.structProcess.uTimerValue >= sync_inst.structProcess.uTriggerMs)
              )
            {
                sync_inst.structProcess.longPulseFlag = TRUE; // Indique que l'emission d'un pulse long est requise.
                sync_inst.structProcess.shortPulseFlag = FALSE;// Indique que l'emission d'un pulse court n'est pas requise
                LOG_D(LOG_STD,"Sync long pulse");

                if(sync_inst.structProcess.currentCycle >= sync_inst.structCalcValues.cyclesByHour) // Si il s'agit de la première transition de l'heure
                {
                    // Compteur de cycles reinitialisé
                    sync_inst.structProcess.currentCycle = 0;
                    // Compteur de ms réinitialisé.
                    sync_inst.structProcess.uTimerValue = 0;

                    // Le prochain déclenchement est initialisé sur un temp court
                    sync_inst.structProcess.dTriggerMs = sync_inst.structCalcValues.dTShort;
                }
                else // Sinon
                {
                    // Le prochain déclanchement est incrémenté avec la durée d'un temps court.
                    sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTShort;
                }

                // Convertion de la valeur à virgule vers en entier.
                sync_inst.structProcess.uTriggerMs = (uint32_t)sync_inst.structProcess.dTriggerMs;
                // On indique que nous sommes sur la première affiche
                sync_inst.structProcess.currentIndex = 1;

                // Si seulement deux affiches sont présentes dans le train
                if(sync_inst.structConfig.panelCount == 2)
                {
                    // Alors on va directement à l'état 'SYNC_WAIT_DOWN'
                    sync_inst.structProcess.smState = synchro_state_wait_down;
                }
                else // Sinon
                {
                    // On va à l'état 'SYNC_WAIT_UP'
                    sync_inst.structProcess.smState = synchro_state_wait_up;
                }
            }
        }
            break;

            case synchro_state_wait_up:
            {
                st_rtc_t r = rtc_get();
                sync_inst.structProcess.uTimerValue = (uint32_t)(r.time_ms_min_sec_ms);
                // Si la condition de déclenchement est atteinte
                if(sync_inst.structProcess.uTimerValue >= sync_inst.structProcess.uTriggerMs)
                {
                    // Flag de demande de pulse long désactivé
                    sync_inst.structProcess.longPulseFlag = FALSE;
                    // Flag de demande de pulse court activé
                    sync_inst.structProcess.shortPulseFlag = TRUE;

                    LOG_D(LOG_STD,"Sync short pulse up");

                    // Si nous étions sur la dernière affiche du train avant la bande mère basse
                    if(sync_inst.structProcess.currentIndex == (sync_inst.structConfig.panelCount-1))
                    {
                        // Alors on décrémente le compteur d'affiche car le train part dans le sens opposé
                        sync_inst.structProcess.currentIndex--;
                        // So il y a seulement deux affiches
                        if(sync_inst.structConfig.panelCount == 2)
                        {
                            // Un cycle complet est éffectué --> passage à l'état 'SYNC_LAST_CYCLE_RESYNC'
                            sync_inst.structProcess.smState = synchro_state_last_cycle_resync;
                        }
                        else // Sinon
                        {
                            // Passage à l'état 'SYNC_WAIT_DOWN'
                            sync_inst.structProcess.smState = synchro_state_wait_down;
                        }
                        // La prochaine condition de déclanchement est calculée en ajoutant un temps court.
                        sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTShort;
                        sync_inst.structProcess.uTriggerMs = (uint32_t)sync_inst.structProcess.dTriggerMs;
                    }
                    else // Sinon
                    {
                        // On incrémente le compteur d'affiche
                        sync_inst.structProcess.currentIndex++;

                        // Si la prochaine affiche est la dernière affiche du train
                        if(sync_inst.structProcess.currentIndex == (sync_inst.structConfig.panelCount-1))
                        {
                            // Alors la prochaine condition de déclenchement est calculée en ajoutant un temps long
                            // (double exposition)
                            sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTLong;
                        }
                        else // Sinon
                        {
                            // Alors la prochaine condition de déclenchement est calculée en ajoutant un temps court
                            sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTShort;
                        }
                        // Cast vers un entier 32 bits.
                        sync_inst.structProcess.uTriggerMs = (uint32_t)sync_inst.structProcess.dTriggerMs;
                    }
                }
            }
                break;

                case synchro_state_wait_down:
                {
                    st_rtc_t r = rtc_get();
                    sync_inst.structProcess.uTimerValue = (uint32_t)(r.time_ms_min_sec_ms);
                    // Si la condition de déclenchement est atteinte
                    if(sync_inst.structProcess.uTimerValue >= sync_inst.structProcess.uTriggerMs)
                    {

                        // Flag de demande de pulse long désactivé
                        sync_inst.structProcess.longPulseFlag = FALSE;
                        // Flag de demande de pulse court activé
                        sync_inst.structProcess.shortPulseFlag = TRUE;

                        LOG_D(LOG_STD,"Sync short pulse down");

                        // Le compteur d'affiche est décrémenté
                        sync_inst.structProcess.currentIndex--;
                        // Si la prochaine affiche est la première du train
                        if(sync_inst.structProcess.currentIndex == 0x00)
                        {
                            // Si le train ne contient que 2 affiches
                            if(sync_inst.structConfig.panelCount == 2)
                            {
                                // Alors on ajoute un temps court à la condition de déclenchement car il n'y a pas de
                                // double exposition
                                sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTShort;
                            }
                            else // Sinon
                            {
                                // Alors la prochaine condition de déclenchement est calculée en ajoutant un temps long
                                // (double exposition)
                                sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTLong;
                            }

                            sync_inst.structProcess.uTriggerMs = (uint32_t)sync_inst.structProcess.dTriggerMs;

                            // Un cycle complet est éffectué --> passage à l'état 'SYNC_LAST_CYCLE_RESYNC'
                            sync_inst.structProcess.smState = synchro_state_last_cycle_resync;
                        }
                        else // Sinon
                        {
                            // Alors on ajoute un temps court à la condition de déclenchement
                            sync_inst.structProcess.dTriggerMs += sync_inst.structCalcValues.dTShort;
                            sync_inst.structProcess.uTriggerMs = (uint32_t)sync_inst.structProcess.dTriggerMs;
                        }
                    }
                }
                break;

                case synchro_state_last_cycle_resync:
                    sync_inst.structProcess.currentCycle++;
                    sync_inst.structProcess.smState = synchro_state_wait_for_start_cycle;
                break;
    }

    end:
    tx_mutex_put(&g_mutex_synchro);
}






