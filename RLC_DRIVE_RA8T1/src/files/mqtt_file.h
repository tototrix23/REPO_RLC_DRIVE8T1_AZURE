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
#include <i2c/sht40.h>
#include <status/motor_status.h>
#include <exchanged_data/exchanged_data.h>
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


return_t mqtt_publish_event(uint16_t code);

return_t mqtt_publish_sensor(st_sensor_t *src);
return_t mqtt_publish_poster_count(uint8_t *src);
return_t mqtt_publish_battery_detected(bool_t *src);
return_t mqtt_publish_scrolling_enabled(bool_t *src);
return_t mqtt_publish_lighting_enabled(bool_t *src);
return_t mqtt_publish_motor_status(st_system_motor_status_t *src);
return_t mqtt_publish_voltages(st_voltages_t *src);
return_t mqtt_publish_motor_type(motor_type_t *src);
return_t mqtt_publish_board_version(uint8_t *src);
return_t mqtt_publish_firmware(char *src);
return_t mqtt_publish_drive_mode(drive_mode_t *src);
return_t mqtt_publish_scrolling_id(char *src);
return_t mqtt_publish_lighting_id(char *src);
return_t mqtt_publish_system_status(st_system_status_t *src);
#endif /* FILES_MQTT_FILE_H_ */
