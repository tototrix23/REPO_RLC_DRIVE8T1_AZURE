/*
 * flash.h
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_STACK_H_
#define FLASH_FLASH_STACK_H_

#include "common_data.h"
#include "flash_api.h"



extern const spi_flash_instance_t g_flash;
extern ospi_b_instance_ctrl_t g_flash_ctrl;
extern const spi_flash_cfg_t g_flash_cfg;

extern rm_block_media_instance_t g_rm_block_media1;
extern rm_block_media_spi_instance_ctrl_t g_rm_block_media1_ctrl;
extern const rm_block_media_cfg_t g_rm_block_media1_cfg;

void g_rm_filex_block_media_1_callback(rm_filex_block_media_callback_args_t *p_args);

extern rm_filex_block_media_instance_ctrl_t g_rm_filex_block_media_1_ctrl;
extern const rm_filex_block_media_cfg_t g_rm_filex_block_media_1_cfg;
extern const rm_filex_block_media_instance_t g_rm_filex_block_media_1_instance;



#endif /* FLASH_FLASH_STACK_H_ */
