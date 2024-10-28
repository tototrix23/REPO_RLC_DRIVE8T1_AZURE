/*
 * flash_impl.c
 *
 *  Created on: 21 oct. 2024
 *      Author: Christophe
 */

#include "hal_data.h"
#include "flash_impl.h"

int flash_read_status(uint8_t *st);
int flash_write_and_read(uint8_t *tx_b,uint8_t *rx_b,uint32_t count);
int flash_write_enable(void);
int flash_write_disable(void);
int flash_write_align(ULONG flash_address, ULONG *source, ULONG words);





volatile bool_t g_transfer_complete = false;

void sci_b_spi_lfs_callback (spi_callback_args_t * p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_transfer_complete = TRUE;
    }
    else
    {
        volatile int crash = p_args->event;
        crash = p_args->event;
        crash = p_args->event;
    }
}


int flash_read_status(uint8_t *st)
{
    int ret = 0;
    uint8_t cmd_buffer[2];
    uint8_t in_buffer[2];
    cmd_buffer[0] = 0x05;
    cmd_buffer[1] = 0xFF;

    ret = flash_write_and_read(cmd_buffer,in_buffer,2);
    if(ret != 0) return ret;
    *st = in_buffer[1];
    return 0;
}

int flash_write_and_read(uint8_t *tx_b,uint8_t *rx_b,uint32_t count)
{
    fsp_err_t err = FSP_SUCCESS;
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;
    __NOP();
    __NOP();
    err = R_SCI_B_SPI_WriteRead(&g_sci_spi_lfs_ctrl, tx_b,rx_b, count, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    __NOP();
    __NOP();
    while (false == g_transfer_complete);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();
    return -1;
}

int flash_write_enable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x06;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}

int flash_write_disable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x04;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}

int flash_read(uint32_t flash_address, uint8_t *destination, uint32_t bytes)
{
    fsp_err_t err = FSP_SUCCESS;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    __NOP();
    __NOP();
    g_transfer_complete = false;

    ULONG *dest = destination;
    ULONG addr = (ULONG)flash_address;

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = 0x03;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);


    //SPI_EVENT_ERR_READ_OVERFLOW

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    __NOP();
    __NOP();

    while (false == g_transfer_complete);// tx_thread_sleep(1);
    g_transfer_complete = FALSE;
    __NOP();
    __NOP();
    err= R_SCI_B_SPI_Read(&g_sci_spi_lfs_ctrl, dest, bytes, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    __NOP();
    __NOP();
    while (false == g_transfer_complete);// tx_thread_sleep(1);

    g_transfer_complete = FALSE;

    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();
    R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();
    R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return -1;
}


/*int flash_write(ULONG *flash_address, ULONG *source, ULONG words)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);

    fsp_err_t err = FSP_SUCCESS;

    volatile uint8_t tab[4];
    ULONG i = 0;

    volatile ULONG addr = (ULONG)flash_address;
    ULONG *src = source;

    volatile ULONG words_operation = 0;
    volatile ULONG words_count=0;
    bool_t end = FALSE;
    do
    {
        volatile ULONG sector_start = (addr + (words_count * 4))/4096;
        volatile ULONG words_left = (words - words_count);

        if(words_left > 64) words_operation = 64;
        else words_operation = words_left;


        volatile ULONG sector_end = (addr + (words_operation * 4))/4096;

        if(sector_start != sector_end)
        {
             volatile ULONG stop = 0;
             stop = 0;
             stop = 1;
             stop = 0;
              stop = 1;

        }
        else
        {
            err = flash_write_align(addr,source+words_count,words_operation);
            if(err != 0)
            {
                goto error;
            }
        }

        addr += (words_operation*4);
        words_count += words_operation;
        if(words_count == words) end = TRUE;

    }while(!end);

    return 0;


    error:

    tx_mutex_put(&g_mutex_spi);
    return -1;
}*/


int flash_write(uint32_t flash_address, uint8_t *source, uint32_t bytes)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    ret = flash_write_enable();
    if(ret != 0)
    {
        return ret;
    }

    fsp_err_t err = FSP_SUCCESS;

    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    __NOP();
    __NOP();
    g_transfer_complete = false;

    volatile uint8_t tab[4];
    ULONG i = 0;

    volatile ULONG addr = (ULONG)flash_address;
    ULONG *src = source;

    uint8_t cmd_buffer[4];

    cmd_buffer[0] = 0x02;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    __NOP();
    __NOP();
    while (false == g_transfer_complete);
    g_transfer_complete = false;

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, source, bytes, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    __NOP();
    __NOP();
    while (false == g_transfer_complete);
    g_transfer_complete = false;





    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();

    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
            R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
            tx_mutex_put(&g_mutex_spi);
            return ret;
        }
        if((st & 0x02) == 0x00)
        {
            R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
            tx_mutex_put(&g_mutex_spi);
            return 0;
        }
        __NOP();
        __NOP();
    }

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return -1;
}



int flash_erase(ULONG block)
{

    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    ret = flash_write_enable();
    if(ret != 0)
    {
        R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    ULONG addr = (ULONG)((block * 4096));
    uint8_t cmd_buffer[4];
    uint8_t in_buffer[4];

    cmd_buffer[0] = 0x20;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    //ret = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);

    ret = flash_write_and_read(cmd_buffer,in_buffer,4);
    if(ret != 0)
    {
        R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    __NOP();
    __NOP();

    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
            R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
            tx_mutex_put(&g_mutex_spi);
            return ret;
        }
        if((st & 0x02) == 0x00)
        {
            R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
            tx_mutex_put(&g_mutex_spi);
            return 0;
        }
        __NOP();
        __NOP();
    }
    return 0;
}


int flash_erase_chip(void)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    ret = flash_write_enable();
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0xC7;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    __NOP();
    __NOP();


    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
           tx_mutex_put(&g_mutex_spi);
           return ret;
        }
        if((st & 0x02) == 0x00)
        {
           tx_mutex_put(&g_mutex_spi);
           return 0;
        }
        __NOP();
        __NOP();
    }
    return 0;
}

int flash_open(void)
{
    return R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
}
int flash_close(void)
{
    return R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
}

