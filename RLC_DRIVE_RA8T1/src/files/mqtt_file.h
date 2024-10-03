/*
 * file.h
 *
 *  Created on: 7 juin 2024
 *      Author: Christophe
 */

#ifndef FILES_MQTT_FILE_H_
#define FILES_MQTT_FILE_H_

#include <_core/c_common.h>
#include <rtc/rtc.h>


#define FILE_TYPE_PAYLOAD  0
#define FILE_TYPE_EVENT    1




#define FILE_TYPE_MQTT_PUBLISH_SENSOR      2




typedef struct st_json_file_t
{
    st_rtc_t rtc;
    char topic[32];
    char *ptr_data;
}
__attribute__((packed))      // Remove interfield padding.
__attribute__((aligned(4)))  // Set alignment and add tail padding.
json_file_t;


return_t json_file_add_to_queue(char *topic,char *ptr);
return_t json_create_full_mqtt_publish(char *ptr_full,json_file_t *json_descriptor);



return_t mqtt_publish_temperature_humidity(void);


#endif /* FILES_MQTT_FILE_H_ */
