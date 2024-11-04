/*
 * firmware.c
 *
 *  Created on: 31 oct. 2024
 *      Author: Christophe
 */
#include "pending_firmware.h"


static st_pending_firmware_t pending_firmware;

return_t pending_firmware_set(st_pending_firmware_t value)
{
   memcpy(&pending_firmware,&value,sizeof(st_pending_firmware_t));
   return X_RET_OK;
}

void pending_firmware_get(st_pending_firmware_t *dest)
{
   memcpy(dest,&pending_firmware,sizeof(st_pending_firmware_t));
}
