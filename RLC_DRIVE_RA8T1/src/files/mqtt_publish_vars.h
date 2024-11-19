/*
 * mqtt_publish_vars.h
 *
 *  Created on: 28 oct. 2024
 *      Author: Christophe
 */

#ifndef FILES_MQTT_PUBLISH_VARS_H_
#define FILES_MQTT_PUBLISH_VARS_H_

#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>

#define MQTT_PUBLISH_ON_CHANGE     0
#define MQTT_PUBLISH_ON_DELTA      1
#define MQTT_PUBLISH_ON_PERIOD_S   2

#define MQTT_TYPE_FLOAT                   0
#define MQTT_TYPE_UINT8                   1
#define MQTT_TYPE_UINT16                  3
#define MQTT_TYPE_UINT32                  5
#define MQTT_TYPE_UINT64                  7
#define MQTT_TYPE_BOOL                    9
#define MQTT_TYPE_MOTOR_TYPE              10
#define MQTT_TYPE_MOTOR_STATUS            11
#define MQTT_TYPE_DRIVE_MODE              12
#define MQTT_TYPE_SENSOR                  13
#define MQTT_TYPE_VOLTAGES                14
#define MQTT_TYPE_CHAR                    15
#define MQTT_TYPE_SETTINGS                16
#define MQTT_TYPE_SYSTEM_STATUS           17


typedef struct st_mqtt_vars_t
{
   uint8_t publish_mode;
   void *var_pointer;
   void *save_pointer;
   int type;
   size_t variable_size;
   size_t override_size;
   float delta;
   uint32_t period_ms;
   c_timespan_t timespan;
   void(*func_get_value)(void*);
   return_t(*func_publish)(void*);
   void(*func_override_change_check)(void*,bool_t*);
   bool_t first_time_publish;
}st_mqtt_vars_t;


return_t mqtt_vars_init(void);
return_t mqtt_vars_process(void *var);
return_t mqtt_vars_process_all(void);

#endif /* FILES_MQTT_PUBLISH_VARS_H_ */
