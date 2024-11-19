/*
 * eeprom.h
 *
 *  Created on: 6 nov. 2024
 *      Author: Christophe
 */


#include <_core/c_common.h>
#include <hal_data.h>



typedef enum
{
    EEPROM_PATTERN,
    EEPROM_MOTOR_STATUS,
    EEPROM_VAR_COUNT
}nv_var_id;

return_t eeprom_init(void);


return_t eeprom_update_from_eeprom(uint16_t id);
return_t eeprom_update_to_eeprom(uint16_t id);
