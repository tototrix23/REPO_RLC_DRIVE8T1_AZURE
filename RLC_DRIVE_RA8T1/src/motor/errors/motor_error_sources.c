/*
 * motor_error_souces.c
 *
 *  Created on: 2 sept. 2024
 *      Author: Christophe
 */


#include <motor/errors/motor_error_sources.h>


static motor_error_sources_t error_sources;


void motor_error_sources_init(void)
{
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    memset(&error_sources,0x00,sizeof(motor_error_sources_t));
    tx_mutex_put(&g_mutex_motor_error_sources);
}

void motor_error_sources_set_overcurrent(void)
{
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    error_sources.flags.bits.overcurrent = 1;
    tx_mutex_put(&g_mutex_motor_error_sources);
}

void motor_error_sources_set_driversH(void)
{
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    error_sources.flags.bits.motorH_fault = 1;
    tx_mutex_put(&g_mutex_motor_error_sources);
}

void motor_error_sources_set_driversL(void)
{
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    error_sources.flags.bits.motorL_fault = 1;
    tx_mutex_put(&g_mutex_motor_error_sources);
}

void motor_error_sources_set_firmware(int16_t code)
{
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    error_sources.flags.bits.firmware = 1;
    error_sources.firmware_code = code;
    tx_mutex_put(&g_mutex_motor_error_sources);
}

bool_t motor_error_sources_is_error(void)
{
    bool_t res = FALSE;
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    if(error_sources.flags.value != 0) res = TRUE;
    tx_mutex_put(&g_mutex_motor_error_sources);
    return res;
}

motor_error_sources_t motor_error_sources_get_snapshot(void)
{
    motor_error_sources_t obj;
    tx_mutex_get(&g_mutex_motor_error_sources,TX_WAIT_FOREVER);
    memcpy(&obj,&error_sources,sizeof(motor_error_sources_t));
    tx_mutex_put(&g_mutex_motor_error_sources);
    return obj;
}
