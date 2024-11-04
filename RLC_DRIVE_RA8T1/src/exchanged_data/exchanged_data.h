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
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <sht40_sensor/sht40.h>
#include <status/motor_status.h>


typedef struct st_voltages_t
{
    float main_voltage;
    float battery_voltage;
}st_voltages_t;


typedef struct st_data_t
{
   st_sensor_t sensor_data;
   st_system_motor_status_t motor_status;
   motor_type_t motor_type;
   uint8_t poster_count;
   bool_t gps_sync;
   drive_mode_t drive_mode;
   bool_t scrolling_enabled;
   bool_t lighting_enabled;
   uint64_t settings_scrolling_id;
   uint64_t settings_lighting_id;
   st_voltages_t voltages;
   bool_t battery_detected;
   uint8_t board_version;
   char firmware[32];
}st_data_t;

extern st_data_t exchanged_data;





void exchdat_set_sensor(st_sensor_t sensor_data);
st_sensor_t exchdat_get_sensor(void);
void exchdat_get_sensor2(st_sensor_t *dest);


void exchdat_set_motor_status(st_system_motor_status_t status);
st_system_motor_status_t exchdat_get_motor_status(void);
void exchdat_get_motor_status2(st_system_motor_status_t *dest);

void exchdat_set_motor_type(motor_type_t type);
motor_type_t exchdat_get_motor_type(void);
void exchdat_get_motor_type2(motor_type_t *dest);

void exchdat_set_poster_count(uint8_t panels);
uint8_t exchdat_get_poster_count(void);
void exchdat_get_poster_count2(uint8_t *dest);

void exchdat_set_gps_sync(bool_t gps_sync);
bool_t exchdat_get_gps_sync(void);
void exchdat_get_gps_sync2(bool_t *dest);

void exchdat_set_drive_mode(drive_mode_t mode);
drive_mode_t exchdat_get_drive_mode(void);
void exchdat_get_drive_mode2(drive_mode_t *dest);



void exchdat_set_scrolling_enabled(bool_t enabled);
bool_t exchdat_get_scrolling_enabled(void);
void exchdat_get_scrolling_enabled2(bool_t *dest);

void exchdat_set_lighting_enabled(bool_t enabled);
bool_t exchdat_get_lighting_enabled(void);
void exchdat_get_lighting_enabled2(bool_t *dest);

void exchdat_set_scrolling_id(uint64_t id);
uint64_t exchdat_get_scrolling_id(void);
void exchdat_get_scrolling_id2(uint64_t *dest);

void exchdat_set_lighting_id(uint64_t id);
uint64_t exchdat_get_lighting_id(void);
void exchdat_get_lighting_id2(uint64_t *dest);

/*void exchdat_set_main_voltage(float voltage);
float exchdat_get_main_voltage(void);
void exchdat_get_main_voltage2(float *dest);

void exchdat_set_battery_voltage(float voltage);
float exchdat_get_battery_voltage(void);
void exchdat_get_battery_voltage2(float *dest);*/


void exchdat_set_voltages(st_voltages_t v);
void exchdat_get_voltages2(st_voltages_t *dest);
//st_voltages_t

void exchdat_set_battery_detected(bool_t detected);
bool_t exchdat_get_battery_detected(void);
void exchdat_get_battery_detected2(bool_t *dest);

void exchdat_set_board_version(uint8_t version);
uint8_t exchdat_get_board_version(void);
void exchdat_get_board_version2(uint8_t *dest);

void exchdat_set_firmware(char *firmware);
char* exchdat_get_firmware(void);
void exchdat_get_firmware2(char *dest);

void exchanged_data_init(void);

#endif /* EXCHANGED_DATA_EXCHANGED_DATA_H_ */
