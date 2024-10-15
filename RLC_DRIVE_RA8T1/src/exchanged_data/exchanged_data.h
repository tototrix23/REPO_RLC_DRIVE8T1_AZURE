/*
 * data.h
 *
 *  Created on: 5 mars 2024
 *      Author: Christophe
 */

#ifndef EXCHANGED_DATA_EXCHANGED_DATA_H_
#define EXCHANGED_DATA_EXCHANGED_DATA_H_

#include <stdint.h>
#include <_core/c_common.h>
#include <system_status/system_status.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <sht40_sensor/sht40.h>

typedef struct st_data_t
{
   st_sensor_t sensor_data;
   st_system_status_t system_status;
   motor_type_t motor_type;
   uint8_t poster_count;
   bool_t gps_sync;
   drive_mode_t drive_mode;
   bool_t relay_activated;
   bool_t scrolling_enabled;
   bool_t lighting_enabled;
   uint64_t settings_scrolling_id;
   uint64_t settings_lighting_id;
   float main_voltage;
   float battery_voltage;
   bool_t battery_detected;
   uint8_t board_version;
}st_data_t;

extern st_data_t exchanged_data;

void exchdat_set_sensor(st_sensor_t sensor_data);
st_sensor_t exchdat_get_sensor(void);


void exchdat_set_system_status(st_system_status_t status);
st_system_status_t exchdat_get_system_status(void);

void exchdat_set_motor_type(motor_type_t type);
motor_type_t exchdat_get_motor_type(void);

void exchdat_set_poster_count(uint8_t panels);
uint8_t exchdat_get_poster_count(void);

void exchdat_set_gps_sync(bool_t gps_sync);
bool_t exchdat_get_gps_sync(void);

void exchdat_set_drive_mode(drive_mode_t mode);
drive_mode_t exchdat_get_drive_mode(void);

void exchdat_set_relay_activated(bool_t activated);
bool_t exchdat_get_relay_activated(void);

void exchdat_set_scrolling_enabled(bool_t enabled);
bool_t exchdat_get_scrolling_enabled(void);

void exchdat_set_lighting_enabled(bool_t enabled);
bool_t exchdat_get_lighting_enabled(void);

void exchdat_set_scrolling_id(uint64_t id);
uint64_t exchdat_get_scrolling_id(void);

void exchdat_set_lighting_id(uint64_t id);
uint64_t exchdat_get_lighting_id(void);

void exchdat_set_main_voltage(float voltage);
float exchdat_get_main_voltage(void);

void exchdat_set_battery_voltage(float voltage);
float exchdat_get_battery_voltage(void);

void exchdat_set_battery_detected(bool_t detected);
bool_t exchdat_get_battery_detected(void);

void exchdat_set_board_version(uint8_t version);
uint8_t exchdat_get_board_version(void);

void exchanged_data_init(void);

#endif /* EXCHANGED_DATA_EXCHANGED_DATA_H_ */
