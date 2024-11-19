/*
 * system_status.h
 *
 *  Created on: 13 nov. 2024
 *      Author: Christophe
 */

#ifndef STATUS_SYSTEM_STATUS_H_
#define STATUS_SYSTEM_STATUS_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


typedef struct st_system_status_t
{
   union
   {
       uint16_t value :16;
       struct{
           unsigned error_flash     :1;
           unsigned error_eeprom    :1;
           unsigned error_relay     :1;
           unsigned dummy           :13;
       }bits;
   };
}st_system_status_t;




#endif /* STATUS_SYSTEM_STATUS_H_ */
