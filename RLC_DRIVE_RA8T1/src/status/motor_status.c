/*
 * system.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */


#include <exchanged_data/exchanged_data.h>
#include <status/motor_status.h>



void motor_status_init(void)
{
    tx_mutex_get(&g_mutex_motor_status,TX_WAIT_FOREVER);
    st_system_motor_status_t motor_status;
    memset(&motor_status,0x00,sizeof(st_system_motor_status_t));
    exchdat_set_motor_status(motor_status);
    tx_mutex_put(&g_mutex_motor_status);
}


void motor_status_set(st_system_motor_status_t new_status)
{

    tx_mutex_get(&g_mutex_motor_status,TX_WAIT_FOREVER);
    exchdat_set_motor_status(new_status);
    tx_mutex_put(&g_mutex_motor_status);

}

st_system_motor_status_t motor_status_get(void)
{
    tx_mutex_get(&g_mutex_motor_status,TX_WAIT_FOREVER);
    st_system_motor_status_t ret = exchdat_get_motor_status();
    tx_mutex_put(&g_mutex_motor_status);
    return ret;
}

void motor_status_clear(void)
{
    if(motor_status_check_error() == TRUE)
    {
        motor_status_init();
        tx_mutex_get(&g_mutex_motor_status,TX_WAIT_FOREVER);
        //vee_write_by_id(EEPROM_MOTOR_STATUS);
        tx_mutex_put(&g_mutex_motor_status);
    }
}

bool_t motor_status_check_error(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_motor_status,TX_WAIT_FOREVER);
    st_system_motor_status_t sys = exchdat_get_motor_status();
    if(sys.error_lvl1.value != 0x00 ||
       sys.error_lvl2.value != 0x00 ||
       sys.error_lvl3.value != 0x00
       )
    {
        ret = TRUE;
    }
    tx_mutex_put(&g_mutex_motor_status);
    return ret;
}
