/*
 * motor_error_souces.h
 *
 *  Created on: 2 sept. 2024
 *      Author: Christophe
 */

#ifndef MOTOR_ERRORS_MOTOR_ERROR_SOURCES_H_
#define MOTOR_ERRORS_MOTOR_ERROR_SOURCES_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_hal/h_motors/h_drv8323s/h_drv8323s.h>


typedef union{
   struct
   {
       unsigned overcurrent      : 1;
       unsigned motorH_fault     : 1;
       unsigned motorL_fault     : 1;
       unsigned firmware         : 1;
       unsigned                  : 12;
   }bits;
   uint16_t value;
}motor_error_sources_flags_t;

typedef struct st_motor_error_sources
{
    motor_error_sources_flags_t flags;
    struct
    {
        DRV8323S_FAULT_STATUS1_REG status1;
        DRV8323S_VGS_STATUS2_REG status2;
    }motorH_bits;

    struct
    {
        DRV8323S_FAULT_STATUS1_REG status1;
        DRV8323S_VGS_STATUS2_REG status2;
    }motorL_bits;

    int16_t firmware_code;
}motor_error_sources_t;


void motor_error_sources_init(void);
void motor_error_sources_set_overcurrent(void);
void motor_error_sources_set_driversH(void);
void motor_error_sources_set_driversL(void);
void motor_error_sources_set_firmware(int16_t code);
bool_t motor_error_sources_is_error(void);
motor_error_sources_t motor_error_sources_get_snapshot(void);

#endif /* MOTOR_ERRORS_MOTOR_ERROR_SOURCES_H_ */
