/*
 * flash_api.h
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_API_H_
#define FLASH_FLASH_API_H_

#include "common_data.h"

extern const spi_flash_api_t g_spi_b_on_spi_flash;


fsp_err_t R_SPI_B_Open(spi_flash_ctrl_t * const p_ctrl, spi_flash_cfg_t const * const p_cfg);
fsp_err_t R_SPI_B_Close(spi_flash_ctrl_t * const p_ctrl);
fsp_err_t R_SPI_B_DirectWrite(spi_flash_ctrl_t * const p_ctrl,
                               uint8_t const * const    p_src,
                               uint32_t const           bytes,
                               bool const               read_after_write);
fsp_err_t R_SPI_B_DirectRead(spi_flash_ctrl_t * const p_ctrl, uint8_t * const p_dest, uint32_t const bytes);
fsp_err_t R_SPI_B_DirectTransfer(spi_flash_ctrl_t * const            p_ctrl,
                                  spi_flash_direct_transfer_t * const p_transfer,
                                  spi_flash_direct_transfer_dir_t     direction);
fsp_err_t R_SPI_B_SpiProtocolSet(spi_flash_ctrl_t * const p_ctrl, spi_flash_protocol_t spi_protocol);
fsp_err_t R_SPI_B_XipEnter(spi_flash_ctrl_t * const p_ctrl);
fsp_err_t R_SPI_B_XipExit(spi_flash_ctrl_t * const p_ctrl);
fsp_err_t R_SPI_B_Write(spi_flash_ctrl_t * const p_ctrl,
                         uint8_t const * const    p_src,
                         uint8_t * const          p_dest,
                         uint32_t                 byte_count);
fsp_err_t R_SPI_B_Erase(spi_flash_ctrl_t * const p_ctrl, uint8_t * const p_device_address, uint32_t byte_count);
fsp_err_t R_SPI_B_StatusGet(spi_flash_ctrl_t * const p_ctrl, spi_flash_status_t * const p_status);
fsp_err_t R_SPI_B_BankSet(spi_flash_ctrl_t * const _ctrl, uint32_t bank);
fsp_err_t R_SPI_B_AutoCalibrate(spi_flash_ctrl_t * const p_ctrl);

#endif /* FLASH_FLASH_API_H_ */
