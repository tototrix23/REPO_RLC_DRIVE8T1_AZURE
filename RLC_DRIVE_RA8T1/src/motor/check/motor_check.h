/*
 * motor_check.h
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#ifndef MOTOR_CHECK_MOTOR_CHECK_H_
#define MOTOR_CHECK_MOTOR_CHECK_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <system_status/system_status.h>

return_t motor_check(st_system_motor_status_t *sys_mot);
return_t motor_check_process_error_sources(st_system_motor_status_t *sys_mot);


#endif /* MOTOR_CHECK_MOTOR_CHECK_H_ */
