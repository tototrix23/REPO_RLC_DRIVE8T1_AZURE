/*
 * emergency_stop.h
 *
 *  Created on: 24 juil. 2024
 *      Author: Christophe
 */

#ifndef MOTOR_EMERGENCY_EMERGENCY_H_
#define MOTOR_EMERGENCY_EMERGENCY_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <return_codes.h>


typedef union{
   struct
   {
       unsigned overcurrent      : 1;
       unsigned motor1_fault     : 1;
       unsigned motor2_fault     : 1;
       unsigned                  : 13;
   }bits;
   uint16_t value;
}emergency_src_t;




void motor_emergency_init(void);
void motor_emergency_set_overcurrent(void);
void motor_emergency_set_motor1_fault(void);
void motor_emergency_set_motor2_fault(void);
bool_t motor_emergency_is_error(void);
emergency_src_t motor_emergency_get_data(void);



#endif /* MOTOR_EMERGENCY_EMERGENCY_H_ */
