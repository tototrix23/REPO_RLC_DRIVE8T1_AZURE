/*
 * config.h
 *
 *  Created on: Aug 28, 2023
 *      Author: Christophe
 */

#ifndef CONFIG_COMPILER_CONFIG_H_
#define CONFIG_COMPILER_CONFIG_H_

#include <_target/t_targets.h>
#include <_target/t_operating_systems.h>
#include <__config/config.h>

#include <tx_user.h>


#undef DEBUG_MODE



//========================================================================
//
// RLIB
//
//========================================================================
#define R_LIB_UNIT_TEST_MODE
#define TARGET                        TARGET_RENESAS_RA
#define OPERATING_SYSTEM              T_OS_THREADX

#define LOG_ENABLE                    1
#define R_LIB_CHECK_PARAM_ENABLE      1
#define R_LIB_ASSERT_MODE             R_LIB_ASSERT_RETURN_CODE
#define R_LIB_LOG_LEVEL               LOG_LVL_DEBUG

#define USE_SALLOC                    1
#define SALLOC_SIZE_BYTES             4000U//4000U


//========================================================================
//
// FIRMWARE
//
//========================================================================
extern const char drive_firmware[10];

#define FW_CHECK_PARAM_ENABLE     1

#define THREADX_TICKS             TX_TIMER_TICKS_PER_SECOND
#define GLOBAL_TIMER_TICKS_FIRST  1
#define GLOBAL_TIMER_TICKS        1
//------------------------------------------------------------------------
// ADC
//------------------------------------------------------------------------
// Definition des canaux à utiliser
#define ADC_CHANNEL_VIN         14
#define ADC_CHANNEL_VBATT       15
#define ADC_CHANNEL_IIN         5
#define ADC_CHANNEL_VHALL1      26
#define ADC_CHANNEL_VHALL2      27

// Définition de l'échantillonage
#define ADC_VIN_AVERAGE         32.0f
#define ADC_VBATT_AVERAGE       32.0f
#define ADC_IIN_AVERAGE         1000.0f//128.0f
#define ADC_VHALL1_AVERAGE      32.0f
#define ADC_VHALL2_AVERAGE      32.0f
#define ADC_IMOT_AVERAGE        1024.0f

// Adaptation des niveau en fonction des ponts diviseurs
#define ADC_VM_ADAPT(f)            (uint32_t)((float)f*8.137207031f) //8,137207031
#define ADC_VIN_ADAPT(f)           (uint32_t)((float)f*8.137207031f)
#define ADC_VBATT_ADAPT(f)         (uint32_t)((float)f*8.137207031f)
#define ADC_IIN_ADAPT(f)           (uint32_t)((float)f*2.685546875f)
#define ADC_VHALL1_ADAPT(f)        (uint32_t)((float)f*3.482288855f)
#define ADC_VHALL2_ADAPT(f)        (uint32_t)((float)f*3.482288855f)
#define ADC_IMOT_ADAPT_GAIN_X5(f)  (uint32_t)((float)f*3.22265625f)
#define ADC_IMOT_ADAPT_GAIN_X10(f) (uint32_t)((float)f*1.611328125f)
#define ADC_IMOT_ADAPT_GAIN_X20(f) (uint32_t)((float)f*0.805664063f)
#define ADC_IMOT_ADAPT_GAIN_X40(f) (uint32_t)((float)f*0.402832031f)

//------------------------------------------------------------------------
// REMOTE CONTROL
//------------------------------------------------------------------------
#define REMOTECTRL_ACTIVE_LEVEL 1


#endif /* CONFIG_COMPILER_CONFIG_H_ */
