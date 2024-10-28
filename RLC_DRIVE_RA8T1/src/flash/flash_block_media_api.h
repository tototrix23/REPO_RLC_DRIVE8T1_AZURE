/*
 * flash_block_media_api.h
 *
 *  Created on: 21 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_BLOCK_MEDIA_API_H_
#define FLASH_FLASH_BLOCK_MEDIA_API_H_

#include "bsp_api.h"
#include "r_spi_flash_api.h"
#include "rm_block_media_api.h"

extern const rm_block_media_api_t g_rm_block_media_on_spi_custom;



fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_Open(rm_block_media_ctrl_t * const p_ctrl, rm_block_media_cfg_t const * const p_cfg);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_MediaInit(rm_block_media_ctrl_t * const p_ctrl);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_Read(rm_block_media_ctrl_t * const p_ctrl,
                                  uint8_t * const               p_dest,
                                  uint32_t const                start_block,
                                  uint32_t const                num_blocks);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_Write(rm_block_media_ctrl_t * const p_ctrl,
                                   uint8_t const * const         p_src,
                                   uint32_t const                start_block,
                                   uint32_t const                num_blocks);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_Erase(rm_block_media_ctrl_t * const p_ctrl,
                                   uint32_t const                start_block,
                                   uint32_t const                num_blocks);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_CallbackSet(rm_block_media_ctrl_t * const p_ctrl,
                                         void (                      * p_callback)(
                                             rm_block_media_callback_args_t *),
                                         void const * const                     p_context,
                                         rm_block_media_callback_args_t * const p_callback_memory);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_StatusGet(rm_block_media_ctrl_t * const p_ctrl, rm_block_media_status_t * const p_status);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_InfoGet(rm_block_media_ctrl_t * const p_ctrl, rm_block_media_info_t * const p_info);
fsp_err_t RM_BLOCK_MEDIA_SPI_CUSTOM_Close(rm_block_media_ctrl_t * const p_ctrl);


#endif /* FLASH_FLASH_BLOCK_MEDIA_API_H_ */
