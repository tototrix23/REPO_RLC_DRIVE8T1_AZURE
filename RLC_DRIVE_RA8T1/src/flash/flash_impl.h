/*
 * flash_impl.h
 *
 *  Created on: 21 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_IMPL_H_
#define FLASH_FLASH_IMPL_H_

#include <_core/c_common.h>
#include "common_data.h"


int flash_read(uint32_t flash_address, uint8_t *destination, uint32_t bytes);
int flash_write(uint32_t flash_address, uint8_t *source, uint32_t bytes);
int flash_erase(ULONG block);
int flash_erase_chip(void);
int flash_open(void);
int flash_close(void);



#endif /* FLASH_FLASH_IMPL_H_ */
