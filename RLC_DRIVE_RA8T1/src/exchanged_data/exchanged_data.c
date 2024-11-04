/*
 * data.c
 *
 *  Created on: 5 mars 2024
 *      Author: Christophe
 */

#include <string.h>
#include "exchanged_data.h"
#include <_config_compiler/config.h>
#include <files/mqtt_publish_vars.h>
st_data_t exchanged_data;

void exchanged_data_init(void)
{
	memset(&exchanged_data,0x00,sizeof(st_data_t));


}




void exchdat_set_motor_status(st_system_motor_status_t status)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(&exchanged_data.motor_status,&status,sizeof(st_system_motor_status_t));
    tx_mutex_put(&g_exchanged_data_mutex);
    mqtt_vars_process(&exchanged_data.motor_status);
}

st_system_motor_status_t exchdat_get_motor_status(void)
{
    st_system_motor_status_t st;
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(&st,&exchanged_data.motor_status,sizeof(st_system_motor_status_t));
    tx_mutex_put(&g_exchanged_data_mutex);
    return st;
}

void exchdat_get_motor_status2(st_system_motor_status_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(dest,&exchanged_data.motor_status,sizeof(st_system_motor_status_t));
    tx_mutex_put(&g_exchanged_data_mutex);
}



void exchdat_set_sensor(st_sensor_t sensor_data)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	memcpy(&exchanged_data.sensor_data,&sensor_data,sizeof(st_sensor_t));
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.sensor_data);
}


st_sensor_t exchdat_get_sensor(void)
{
    st_sensor_t data;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	memcpy(&data,&exchanged_data.sensor_data,sizeof(st_sensor_t));
	tx_mutex_put(&g_exchanged_data_mutex);
	return data;
}

void exchdat_get_sensor2(st_sensor_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(dest,&exchanged_data.sensor_data,sizeof(st_sensor_t));
    tx_mutex_put(&g_exchanged_data_mutex);

}


void exchdat_set_motor_type(motor_type_t type)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.motor_type = type;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.motor_type);
}
motor_type_t exchdat_get_motor_type(void)
{
	motor_type_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.motor_type;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_motor_type2(motor_type_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.motor_type;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_poster_count(uint8_t poster_count)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.poster_count = poster_count;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.poster_count);
}
uint8_t exchdat_get_poster_count(void)
{
	uint8_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.poster_count;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_poster_count2(uint8_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.poster_count;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_gps_sync(bool_t gps_sync)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.gps_sync = gps_sync;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.gps_sync);
}
bool_t exchdat_get_gps_sync(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.gps_sync;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_gps_sync2(bool_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.gps_sync;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_drive_mode(drive_mode_t mode)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.drive_mode = mode;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.drive_mode);
}
drive_mode_t exchdat_get_drive_mode(void)
{
	drive_mode_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.drive_mode;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_drive_mode2(drive_mode_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.drive_mode;
    tx_mutex_put(&g_exchanged_data_mutex);
}



void exchdat_set_scrolling_enabled(bool_t enabled)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.scrolling_enabled = enabled;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.scrolling_enabled);
}
bool_t exchdat_get_scrolling_enabled(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.scrolling_enabled;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_scrolling_enabled2(bool_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.scrolling_enabled;
    tx_mutex_put(&g_exchanged_data_mutex);

}

void exchdat_set_lighting_enabled(bool_t enabled)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.lighting_enabled = enabled;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.lighting_enabled);
}

bool_t exchdat_get_lighting_enabled(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.lighting_enabled;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_lighting_enabled2(bool_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.lighting_enabled;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_scrolling_id(uint64_t id)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.settings_scrolling_id = id;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.settings_scrolling_id);
}
uint64_t exchdat_get_scrolling_id(void)
{
	uint64_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.settings_scrolling_id;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_scrolling_id2(uint64_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.settings_scrolling_id;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_lighting_id(uint64_t id)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.settings_lighting_id = id;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.settings_lighting_id);
}
uint64_t exchdat_get_lighting_id(void)
{
	uint64_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.settings_lighting_id;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_lighting_id2(uint64_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.settings_lighting_id;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_voltages(st_voltages_t v)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(&exchanged_data.voltages,&v,sizeof(st_voltages_t));
    tx_mutex_put(&g_exchanged_data_mutex);
    mqtt_vars_process(&exchanged_data.voltages);
}

void exchdat_get_voltages2(st_voltages_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    memcpy(dest,&exchanged_data.voltages,sizeof(st_voltages_t));
    tx_mutex_put(&g_exchanged_data_mutex);

}

/*void exchdat_set_main_voltage(float voltage)
{
    static float v_save=0.0f;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.main_voltage = voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	if(fabs(v_save-voltage) > 0.2f)
    {
        v_save = voltage;
        mqtt_publish_voltages();
    }
}

float exchdat_get_main_voltage(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.main_voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_main_voltage2(float *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = 22.3f;//exchanged_data.main_voltage;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_battery_voltage(float voltage)
{
    static float v_save=0;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.battery_voltage = voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	if(fabs(v_save-voltage) > 0.2f)
    {
        v_save = voltage;
        mqtt_publish_voltages();
    }
}

float exchdat_get_battery_voltage(void)
{
	float value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.battery_voltage;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_battery_voltage2(float *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.battery_voltage;
    tx_mutex_put(&g_exchanged_data_mutex);
}*/


void exchdat_set_battery_detected(bool_t detected)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.battery_detected = detected;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.battery_detected);
}
bool_t exchdat_get_battery_detected(void)
{
	bool_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.battery_detected;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_battery_detected2(bool_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.battery_detected;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_board_version(uint8_t version)
{
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	exchanged_data.board_version = version;
	tx_mutex_put(&g_exchanged_data_mutex);
	mqtt_vars_process(&exchanged_data.board_version);
}

uint8_t exchdat_get_board_version(void)
{
	uint8_t value;
	tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
	value = exchanged_data.board_version;
	tx_mutex_put(&g_exchanged_data_mutex);
	return value;
}

void exchdat_get_board_version2(uint8_t *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    *dest = exchanged_data.board_version;
    tx_mutex_put(&g_exchanged_data_mutex);
}

void exchdat_set_firmware(char *firmware)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    strcpy(exchanged_data.firmware,firmware);
    tx_mutex_put(&g_exchanged_data_mutex);
    mqtt_vars_process(&exchanged_data.firmware);
}

char* exchdat_get_firmware(void)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    char *ptr = exchanged_data.firmware;
    tx_mutex_put(&g_exchanged_data_mutex);
    return ptr;
}

void exchdat_get_firmware2(char *dest)
{
    tx_mutex_get(&g_exchanged_data_mutex,TX_WAIT_FOREVER);
    strcpy(dest,exchanged_data.firmware);
    tx_mutex_put(&g_exchanged_data_mutex);
}
