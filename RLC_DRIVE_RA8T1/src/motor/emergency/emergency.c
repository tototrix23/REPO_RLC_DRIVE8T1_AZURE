/*
 * emergency_stop.c
 *
 *  Created on: 24 juil. 2024
 *      Author: Christophe
 */

#include "emergency.h"

emergency_src_t emergency_src;
void motor_emergency_common_operations(void);

void motor_emergency_init(void)
{
    tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
    memset(&emergency_src,0x00,sizeof(emergency_src_t));
    tx_mutex_put(&g_emergency_mutex);
}

void motor_emergency_set_overcurrent(void)
{
    tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
    //motor_emergency_common_operations();
    emergency_src.bits.overcurrent = 1;
    tx_mutex_put(&g_emergency_mutex);
}

void motor_emergency_set_motor1_fault(void)
{
    if(emergency_src.bits.motor1_fault == 0)
    {
        tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
        //motor_emergency_common_operations();
        emergency_src.bits.motor1_fault = 1;
        tx_mutex_put(&g_emergency_mutex);
    }
}

void motor_emergency_set_motor2_fault(void)
{
    if(emergency_src.bits.motor2_fault == 0)
    {
        tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
        //motor_emergency_common_operations();
        emergency_src.bits.motor2_fault = 1;
        tx_mutex_put(&g_emergency_mutex);
    }
}

bool_t motor_emergency_is_error(void)
{
    bool_t res = FALSE;
    tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
    if(emergency_src.value != 0) res = TRUE;
    tx_mutex_put(&g_emergency_mutex);
    return res;
}

emergency_src_t motor_emergency_get_data(void)
{
    emergency_src_t obj;
    tx_mutex_get(&g_emergency_mutex,TX_WAIT_FOREVER);
    obj = emergency_src;
    tx_mutex_put(&g_emergency_mutex);
    return obj;
}

void motor_emergency_common_operations(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_SWITCH_CMD,BSP_IO_LEVEL_LOW );
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_LOW );
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_LOW );
}
