/*
 * system.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include <vee/vee.h>
#include <exchanged_data/exchanged_data.h>
#include <system_status/system_status.h>
bool_t flag_overcurrent_vm;
//st_system_status_t system_inst;

void system_status_init(void)
{
    tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
    flag_overcurrent_vm = FALSE;
    st_system_status_t sys_status;
    memset(&sys_status,0x00,sizeof(st_system_status_t));
    exchdat_set_system_status(sys_status);
    tx_mutex_put(&g_mutex_system_status);
}


void system_status_set_motor(st_system_motor_status_t *ptr_value)
{
    st_system_status_t current = system_status_get();
    if(memcmp(ptr_value,&current.motor,sizeof(st_system_motor_status_t)) != 0)
    {
        tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
        st_system_status_t sys_status = exchdat_get_system_status();
        memcpy(&sys_status.motor,ptr_value,sizeof(st_system_motor_status_t));
        exchdat_set_system_status(sys_status);
        vee_write_by_id(EEPROM_SYSTEM_STATUS);
        tx_mutex_put(&g_mutex_system_status);
    }
}

st_system_status_t system_status_get(void)
{
    tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
    st_system_status_t ret = exchdat_get_system_status();
    tx_mutex_put(&g_mutex_system_status);
    return ret;
}

void system_status_clear_all(void)
{
    system_status_init();
    tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
    vee_write_by_id(EEPROM_SYSTEM_STATUS);
    //exchdat_set_system_status(system_inst);
    tx_mutex_put(&g_mutex_system_status);
}

void system_status_clear_motor(void)
{
    if(system_status_check_error() == TRUE)
    {
        tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
        LOG_I(LOG_STD,"Reset des erreurs moteur");
        flag_overcurrent_vm = FALSE;
        st_system_status_t sys_status = exchdat_get_system_status();
        memset(&sys_status.motor,0x00,sizeof(st_system_motor_status_t));
        exchdat_set_system_status(sys_status);
        vee_write_by_id(EEPROM_SYSTEM_STATUS);
        tx_mutex_put(&g_mutex_system_status);
    }
}

bool_t system_status_check_error(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_system_status,TX_WAIT_FOREVER);
    st_system_status_t sys = exchdat_get_system_status();
    if(sys.motor.error_lvl1.value != 0x00 ||
       sys.motor.error_lvl2.value != 0x00 ||
       sys.motor.error_lvl3.value != 0x00)
    {
        ret = TRUE;
    }
    tx_mutex_put(&g_mutex_system_status);
    return ret;
}
