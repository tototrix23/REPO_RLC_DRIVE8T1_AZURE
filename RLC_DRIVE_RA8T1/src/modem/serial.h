/*
 * serial.h
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

#include <stdint.h>
#include <_core/c_common.h>

typedef struct st_serials_t
{
    char serial[16];
}st_serials_t;


void serials_init(void);
void serials_set(st_serials_t *ptr);
st_serials_t serials_get(void);


#endif /* SERIAL_SERIAL_H_ */
