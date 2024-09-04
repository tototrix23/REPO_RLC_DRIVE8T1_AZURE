/*
 * motor_mode.h
 *
 *  Created on: 17 oct. 2023
 *      Author: Christophe
 */

#ifndef APPLICATION_MOTOR_DRIVING_MODE_H_
#define APPLICATION_MOTOR_DRIVING_MODE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <return_codes.h>
#include <system_status/system_status.h>
#include <motor/errors/motor_error_sources.h>

#define CHECK_STOP_REQUEST()              {\
                                          if(drive_stop_request())\
                                             {\
                                                 LOG_D(LOG_STD,"stop request");\
                                                 return F_RET_MOTOR_DRIVE_CANCELLED;\
                                             }\
                                          }\

#define CHECK_STOP_REQUEST_NESTED()       {\
                                             motor_check_fault_pins();\
                                             if(drive_control.stop_order == TRUE || motor_error_sources_is_error() == TRUE) return F_RET_MOTOR_DRIVE_CANCELLED;\
                                          }\

#define CHECK_STOP_REQUEST_NESTED_CPLX()  {\
                                          motor_check_fault_pins();\
                                          if(drive_control.stop_order == TRUE || motor_error_sources_is_error() == TRUE) \
                                             {\
                                              return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVE_CANCELLED);\
                                              return ret;\
                                             }\
                                          }\


#define MOTOR_SET_ERROR_EVENT_AND_RETURN(mode,event) {\
                                                        drive_control.running = FALSE;\
                                                        motor_error_sources_set_firmware(event);\
                                                        set_drive_mode(MOTOR_ERROR_MODE);\
                                                        return event;\
                                                     }\



typedef enum  e_drive_mode
{
    MOTOR_MANUAL_MODE  = 1,
    MOTOR_INIT_MODE    = 2,
    MOTOR_AUTO_MODE    = 3,
    MOTOR_ERROR_MODE   = 4,
} drive_mode_t;


typedef struct st_drive_control_t
{
    bool_t stop_order;
    bool_t running;
    bool_t changing;
}drive_control_t;

extern drive_control_t drive_control;

bool_t drive_stop_request(void);
return_t set_drive_mode(drive_mode_t mode);
void motor_check_fault_pins(void);

#endif /* APPLICATION_MOTOR_DRIVING_MODE_H_ */
