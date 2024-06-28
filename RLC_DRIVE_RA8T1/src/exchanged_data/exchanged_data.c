/*
 * data.c
 *
 *  Created on: 5 mars 2024
 *      Author: Christophe
 */


#include "exchanged_data.h"

st_data_t exchanged_data;

void exchanged_data_init(void)
{
	memset(&exchanged_data,0x00,sizeof(st_data_t));
}


void exchdat_set_temperature_humidity(float temp,float hum)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.temperature = temp;
	exchanged_data.humidity = hum;
	tx_mutex_put(&g_exchanged_data_mutex);
}
float exchdat_get_temperature(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.temperature;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
float exchdat_get_humidity(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.humidity;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_system_status(st_system_status_t status)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.system_status = status;
	tx_mutex_put(&g_exchanged_data_mutex);
}
st_system_status_t exchdat_get_system_status(void)
{
    st_system_status_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.system_status;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_motor_type(motor_type_t type)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.motor_type = type;
	tx_mutex_put(&g_exchanged_data_mutex);
}
motor_type_t exchdat_get_motor_type(void)
{
	motor_type_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.motor_type;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_panels(uint8_t panels)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.panels = panels;
	tx_mutex_put(&g_exchanged_data_mutex);
}
uint8_t exchdat_get_panels(void)
{
	uint8_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.panels;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_gps_sync(bool_t gps_sync)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.gps_sync = gps_sync;
	tx_mutex_put(&g_exchanged_data_mutex);
}
bool_t exchdat_get_gps_sync(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.gps_sync;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_drive_mode(drive_mode_t mode)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.drive_mode = mode;
	tx_mutex_put(&g_exchanged_data_mutex);
}
drive_mode_t exchdat_get_drive_mode(void)
{
	drive_mode_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.drive_mode;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_relay_activated(bool_t activated)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.relay_activated = activated;
	tx_mutex_put(&g_exchanged_data_mutex);
}
bool_t exchdat_get_relay_activated(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.relay_activated;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_scrolling_timestamped(bool_t timestamped)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.scrolling_timestamped = timestamped;
	tx_mutex_put(&g_exchanged_data_mutex);
}
bool_t exchdat_get_scrolling_timestamped(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.scrolling_timestamped;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_lighting_timestamped(bool_t timestamped)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.lighting_timestamped = timestamped;
	tx_mutex_put(&g_exchanged_data_mutex);
}

bool_t exchdat_get_lighting_timestamped(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.lighting_timestamped;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_set_scrolling_id(uint64_t id)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.settings_scrolling_id = id;
	tx_mutex_put(&g_exchanged_data_mutex);
}
uint64_t exchdat_get_scrolling_id(void)
{
	uint64_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.settings_scrolling_id;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_lighting_id(uint64_t id)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.settings_lighting_id = id;
	tx_mutex_put(&g_exchanged_data_mutex);
}
uint64_t exchdat_get_lighting_id(void)
{
	uint64_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.settings_lighting_id;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_main_voltage(float voltage)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.main_voltage = voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
}
float exchdat_get_main_voltage(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.main_voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_battery_voltage(float voltage)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.battery_voltage = voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
}
float exchdat_get_battery_voltage(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.battery_voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
void exchdat_set_battery_detected(bool_t detected)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.battery_detected = detected;
	tx_mutex_put(&g_exchanged_data_mutex);
}
bool_t exchdat_get_battery_detected(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.battery_detected;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_set_board_version(uint8_t version)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.board_version = version;
	tx_mutex_put(&g_exchanged_data_mutex);
}

uint8_t exchdat_get_board_version(void)
{
	uint8_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.board_version;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}
