/*
 * flash.h
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

#include "common_data.h"
#include "lx_api.h"
#include "flash_api.h"

extern LX_NOR_FLASH g_lx_nor1;
extern const rm_filex_levelx_nor_instance_t g_rm_filex_levelx_nor1_instance;

/** Access the FileX LevelX NOR instance using these structures when calling API functions directly (::p_api is not used). */
extern rm_filex_levelx_nor_instance_ctrl_t g_rm_filex_levelx_nor1_ctrl;
extern const rm_filex_levelx_nor_cfg_t g_rm_filex_levelx_nor1_cfg;

UINT g_rm_levelx_nor_spi1_initialize(LX_NOR_FLASH *p_nor_flash);
fsp_err_t g_rm_levelx_nor_spi1_close();
void g_rm_filex_levelx_nor_1_callback(rm_filex_levelx_nor_callback_args_t *p_args);

#endif /* FLASH_FLASH_H_ */
