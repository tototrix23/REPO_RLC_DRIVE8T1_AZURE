/*
 * i2c_callback.c
 *
 *  Created on: 6 nov. 2024
 *      Author: Christophe
 */


#include "i2c_callback.h"

i2c_master_event_t iic_event;


void i2c_sensor_callback (i2c_master_callback_args_t * p_args)
{
    iic_event = p_args->event;

    switch(iic_event)
    {
        case I2C_MASTER_EVENT_ABORTED:
        {
            break;
        }
        case I2C_MASTER_EVENT_RX_COMPLETE:
        {
            break;
        }
        case I2C_MASTER_EVENT_TX_COMPLETE:
        {
            break;
        }
    }
}
