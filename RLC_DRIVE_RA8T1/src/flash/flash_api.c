/*
 * flash_api.c
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */

#include "hal_data.h"
#include "bsp_api.h"
#include "flash_api.h"

#define SPI_B_PRV_OPEN                                  (0x78535049U)



const spi_flash_api_t g_spi_b_on_spi_flash =
{
    .open           = R_SPI_B_Open,
    .directWrite    = R_SPI_B_DirectWrite,
    .directRead     = R_SPI_B_DirectRead,
    .directTransfer = R_SPI_B_DirectTransfer,
    .spiProtocolSet = R_SPI_B_SpiProtocolSet,
    .write          = R_SPI_B_Write,
    .erase          = R_SPI_B_Erase,
    .statusGet      = R_SPI_B_StatusGet,
    .xipEnter       = R_SPI_B_XipEnter,
    .xipExit        = R_SPI_B_XipExit,
    .bankSet        = R_SPI_B_BankSet,
    .close          = R_SPI_B_Close,
    .autoCalibrate  = R_SPI_B_AutoCalibrate,
};

static volatile bool g_transfer_complete = false;




fsp_err_t R_SPI_B_Open(spi_flash_ctrl_t * const p_ctrl, spi_flash_cfg_t const * const p_cfg)
{
    fsp_err_t ret = FSP_SUCCESS;
    ospi_b_instance_ctrl_t * p_instance_ctrl = (ospi_b_instance_ctrl_t *) p_ctrl;
    ret = R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    if(ret == FSP_SUCCESS)
    {
        p_instance_ctrl->open  = SPI_B_PRV_OPEN;
    }

    return ret;
}

fsp_err_t R_SPI_B_Close(spi_flash_ctrl_t * const p_ctrl)
{
    fsp_err_t ret = FSP_SUCCESS;
    ospi_b_instance_ctrl_t * p_instance_ctrl = (ospi_b_instance_ctrl_t *) p_ctrl;
    R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    p_instance_ctrl->open = 0x0;
    return ret;
}


fsp_err_t R_SPI_B_DirectWrite(spi_flash_ctrl_t * const p_ctrl,
                               uint8_t const * const    p_src,
                               uint32_t const           bytes,
                               bool const               read_after_write)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_DirectRead(spi_flash_ctrl_t * const p_ctrl, uint8_t * const p_dest, uint32_t const bytes)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_DirectTransfer(spi_flash_ctrl_t * const            p_ctrl,
                                  spi_flash_direct_transfer_t * const p_transfer,
                                  spi_flash_direct_transfer_dir_t     direction)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_SpiProtocolSet(spi_flash_ctrl_t * const p_ctrl, spi_flash_protocol_t spi_protocol)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_XipEnter(spi_flash_ctrl_t * const p_ctrl)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_XipExit(spi_flash_ctrl_t * const p_ctrl)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_Write(spi_flash_ctrl_t * const p_ctrl,
                         uint8_t const * const    p_src,
                         uint8_t * const          p_dest,
                         uint32_t                 byte_count)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_Erase(spi_flash_ctrl_t * const p_ctrl, uint8_t * const p_device_address, uint32_t byte_count)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_StatusGet(spi_flash_ctrl_t * const p_ctrl, spi_flash_status_t * const p_status)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_BankSet(spi_flash_ctrl_t * const _ctrl, uint32_t bank)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}

fsp_err_t R_SPI_B_AutoCalibrate(spi_flash_ctrl_t * const p_ctrl)
{
    fsp_err_t ret = FSP_SUCCESS;
    return ret;
}
