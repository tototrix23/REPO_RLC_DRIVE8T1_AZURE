/*
 * sht40.c
 *
 *  Created on: 11 sept. 2024
 *      Author: Christophe
 */


#include "sht40.h"


i2c_master_event_t iic_event;


return_t sht40_init(void)
{
    return_t ret = X_RET_OK;
    return ret;
}


return_t sht40_read(float *temp,float *rh)
{
    return_t ret = X_RET_OK;
    fsp_err_t err;
    volatile uint32_t  timeout_ms;
    uint8_t g_i2c_tx_buffer[2];
    uint8_t g_i2c_rx_buffer[6];
    err = R_SCI_B_I2C_Open(&g_i2c_sensor_ctrl, &g_i2c_sensor_cfg);
    if(err != FSP_SUCCESS)
        return -1;

    iic_event = I2C_MASTER_EVENT_ABORTED;
    g_i2c_tx_buffer[0] = 0xFD;
    err = R_SCI_B_I2C_Write(&g_i2c_sensor_ctrl, &g_i2c_tx_buffer[0], 1, false);
    if(err != FSP_SUCCESS)
    {
        R_SCI_B_I2C_Close(&g_i2c_sensor_ctrl);
        return -1;
    }

    timeout_ms = 10;
    while ((I2C_MASTER_EVENT_TX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        R_SCI_B_I2C_Close(&g_i2c_sensor_ctrl);
        return -1;
    }

    iic_event = I2C_MASTER_EVENT_ABORTED;
    err = R_SCI_B_I2C_Read(&g_i2c_sensor_ctrl, g_i2c_rx_buffer, 6, false);
    if(err != FSP_SUCCESS)
    {
        R_SCI_B_I2C_Close(&g_i2c_sensor_ctrl);
        return -1;
    }

    timeout_ms = 10;
    while ((I2C_MASTER_EVENT_RX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        R_SCI_B_I2C_Close(&g_i2c_sensor_ctrl);
        return -1;
    }
    uint16_t t_ticks = (uint16_t)(g_i2c_rx_buffer[0] * 256 + g_i2c_rx_buffer[1]);
    float t = (float)(-45.0 + 175.0 * (float)t_ticks/65535.0);
    uint16_t rh_ticks = (uint16_t)(g_i2c_rx_buffer[3] * 256 + g_i2c_rx_buffer[4]);
    float r = (float)(-6.0 + 125.0 * (float)rh_ticks/65535.0);
    *temp = t;
    *rh = r;

    R_SCI_B_I2C_Close(&g_i2c_sensor_ctrl);
    return ret;
}


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
