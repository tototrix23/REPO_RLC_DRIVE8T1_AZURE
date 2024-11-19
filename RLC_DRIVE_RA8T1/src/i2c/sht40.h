/*
 * sht40.h
 *
 *  Created on: 11 sept. 2024
 *      Author: Christophe
 */

#ifndef SHT40_SENSOR_SHT40_H_
#define SHT40_SENSOR_SHT40_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


typedef struct st_sensor_t
{
   bool_t valid;
   float temperature;
   float humidity;
}st_sensor_t;


return_t sht40_init(void);

return_t sht40_read(st_sensor_t *data);

#endif /* SHT40_SENSOR_SHT40_H_ */
