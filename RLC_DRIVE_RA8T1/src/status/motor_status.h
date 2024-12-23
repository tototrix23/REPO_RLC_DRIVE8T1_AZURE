/*
 * system.h
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#ifndef STATUS_MOTOR_STATUS_H_
#define STATUS_MOTOR_STATUS_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_hal/h_motors/h_drv8323s/h_drv8323s.h>
typedef struct st_system_motor_status_t
{
   union
   {
       uint16_t value :16;
       struct{
           bool_t overcurrent_vm :1;
           bool_t vcc_12v    :1;
           bool_t vcc_hall_h :1;
           bool_t vcc_hall_l :1;
           bool_t config_driver_h :1;
           bool_t config_driver_l :1;
           bool_t fsp_h :1;
           bool_t fsp_l :1;
           bool_t fault_driver_h : 1;
           bool_t fault_driver_l : 1;
       }bits;
   }error_lvl1;

   struct
   {
       DRV8323S_FAULT_STATUS1_REG status1;
       DRV8323S_VGS_STATUS2_REG status2;
   }error_lvl1_motorH;

   struct
   {
       DRV8323S_FAULT_STATUS1_REG status1;
       DRV8323S_VGS_STATUS2_REG status2;
   }error_lvl1_motorL;

   union
   {
       uint16_t value:16;
       struct
       {
           bool_t error_pattern_h :1;
           bool_t error_pattern_l :1;
           bool_t timeout_pulses_h :1;
           bool_t timeout_pulses_l :1;
           bool_t unknown :1;
       }bits;
   }error_lvl2;


   union
   {
      uint16_t value:16;
      struct
      {
          bool_t damaged_panels :1;
          bool_t motor_driving_h :1;
          bool_t motor_driving_l :1;
          bool_t unknown :1;
      }bits;
   }error_lvl3;

}st_system_motor_status_t;



void motor_status_init(void);
void motor_status_set(st_system_motor_status_t new_status);
st_system_motor_status_t motor_status_get(void);
void motor_status_clear(void);
bool_t motor_status_check_error(void);

#endif /* STATUS_MOTOR_STATUS_H_ */

