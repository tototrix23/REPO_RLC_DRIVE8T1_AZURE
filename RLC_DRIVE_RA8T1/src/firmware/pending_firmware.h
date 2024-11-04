/*
 * firmware.h
 *
 *  Created on: 31 oct. 2024
 *      Author: Christophe
 */

#ifndef FIRMWARE_PENDING_FIRMWARE_H_
#define FIRMWARE_PENDING_FIRMWARE_H_

#include <stdint.h>
#include <_core/c_common.h>

typedef struct st_pending_firmware_t
{
    bool_t pending;
    char firmware[32];
}st_pending_firmware_t;


return_t pending_firmware_set(st_pending_firmware_t value);
void pending_firmware_get(st_pending_firmware_t *dest);


#endif /* FIRMWARE_PENDING_FIRMWARE_H_ */
