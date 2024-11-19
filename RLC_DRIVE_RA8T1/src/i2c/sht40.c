/*
 * sht40.c
 *
 *  Created on: 11 sept. 2024
 *      Author: Christophe
 */


#include "sht40.h"
#include "i2c_callback.h"


return_t sht40_init(void)
{
    return_t ret = X_RET_OK;
    return ret;
}


return_t sht40_read(st_sensor_t *data)
{
    return_t ret = X_RET_OK;
    fsp_err_t err;
    volatile uint32_t  timeout_ms;
    uint8_t g_i2c_tx_buffer[2];
    uint8_t g_i2c_rx_buffer[6];

    tx_mutex_get(&g_i2c_mutex,TX_WAIT_FOREVER);
    memset(data,0x00,sizeof(st_sensor_t));

    err = R_SCI_B_I2C_Open(&g_i2c0_ctrl, &g_i2c0_cfg);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    err = R_SCI_B_I2C_SlaveAddressSet(&g_i2c0_ctrl,0x44,I2C_MASTER_ADDR_MODE_7BIT);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    iic_event = I2C_MASTER_EVENT_ABORTED;
    g_i2c_tx_buffer[0] = 0xFD;
    err = R_SCI_B_I2C_Write(&g_i2c0_ctrl, &g_i2c_tx_buffer[0], 1, false);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    timeout_ms = 10;
    while ((I2C_MASTER_EVENT_TX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        ret = -1;
        goto end;
    }

    iic_event = I2C_MASTER_EVENT_ABORTED;
    err = R_SCI_B_I2C_Read(&g_i2c0_ctrl, g_i2c_rx_buffer, 6, false);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }

    timeout_ms = 10;
    while ((I2C_MASTER_EVENT_RX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        ret = -1;
        goto end;
    }
    uint16_t t_ticks = (uint16_t)(g_i2c_rx_buffer[0] * 256 + g_i2c_rx_buffer[1]);
    float t = (float)(-45.0 + 175.0 * (float)t_ticks/65535.0);
    uint16_t rh_ticks = (uint16_t)(g_i2c_rx_buffer[3] * 256 + g_i2c_rx_buffer[4]);
    float r = (float)(-6.0 + 125.0 * (float)rh_ticks/65535.0);
    data->temperature = t;
    data->humidity = r;
    data->valid = TRUE;

    end:
    R_SCI_B_I2C_Close(&g_i2c0_ctrl);
    tx_mutex_put(&g_i2c_mutex);
    return ret;
}



