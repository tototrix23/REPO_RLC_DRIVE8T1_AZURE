/*
 * impl_spi_motors.c
 *
 *  Created on: 25 sept. 2023
 *      Author: Ch.Leclercq
 */

#include "hal_data.h"
#include "impl_spi_motors.h"

static volatile bool_t spi_motor_transfer_complete = FALSE;
static volatile bool_t spi_opened = FALSE;

void sci_b_spi_callback_drivers(spi_callback_args_t *p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        spi_motor_transfer_complete = TRUE;
    }
    else
    {
        volatile uint8_t err=1;
    }
}

return_t spi_motor_open(void)
{
    fsp_err_t err = FSP_SUCCESS;
    if (spi_opened == TRUE)
        return X_RET_OK;
    err = R_SCI_B_SPI_Open (&g_sci_spi_drivers_ctrl, &g_sci_spi_drivers_cfg);
    if (err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_OPEN;
    else
    {
        spi_opened = TRUE;
        return X_RET_OK;
    }
}

return_t spi_motor_close(void)
{
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_Close (&g_sci_spi_drivers_ctrl);
    spi_opened = FALSE;
    if (err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_CLOSE;
    else
        return X_RET_OK;
}

return_t spi_motor_read(char *buffer_tx, char *buffer_rx, uint16_t count)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(buffer_tx != NULL);
    ASSERT(buffer_rx != NULL);
    ASSERT(count > 0);
#endif
    spi_motor_transfer_complete = FALSE;
    fsp_err_t err = FSP_SUCCESS;

    /*uint16_t tx16 = buffer_tx[0];
    tx16 <<= 8;
    tx16 = (uint16_t)(tx16+buffer_tx[1]);

    volatile uint16_t rx_buffer;*/

    err = R_SCI_B_SPI_WriteRead (&g_sci_spi_drivers_ctrl, buffer_tx, buffer_rx, 2, SPI_BIT_WIDTH_8_BITS);
    if (err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_READ;

    while (spi_motor_transfer_complete == FALSE)
    {
        __NOP();
    }
    volatile uint8_t i=0;

    return X_RET_OK;
}

return_t spi_motor_write(char *buffer_tx, char *buffer_rx, uint16_t count)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(buffer_tx != NULL);
    ASSERT(count > 0);
#endif
    spi_motor_transfer_complete = FALSE;
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_WriteRead (&g_sci_spi_drivers_ctrl, buffer_tx, buffer_rx, count, SPI_BIT_WIDTH_8_BITS);
    if (err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_WRITE;
    while (spi_motor_transfer_complete == FALSE)
    {
        __NOP();
    }

    return X_RET_OK;
}

return_t spi_motor_mot1_cs_active(void)
{
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT1_CS, BSP_IO_LEVEL_LOW);
    return X_RET_OK;
}
return_t spi_motor_mot1_cs_inactive(void)
{
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT1_CS, BSP_IO_LEVEL_HIGH);
    return X_RET_OK;
}
return_t spi_motor_mot2_cs_active(void)
{
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT2_CS, BSP_IO_LEVEL_LOW);
    return X_RET_OK;
}
return_t spi_motor_mot2_cs_inactive(void)
{
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT2_CS, BSP_IO_LEVEL_HIGH);
    return X_RET_OK;
}
