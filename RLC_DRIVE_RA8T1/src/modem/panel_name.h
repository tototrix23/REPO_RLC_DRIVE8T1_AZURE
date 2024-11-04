/*
 * panel_name.h
 *
 *  Created on: 30 oct. 2024
 *      Author: Christophe
 */

#ifndef MODEM_PANEL_NAME_H_
#define MODEM_PANEL_NAME_H_

#include <stdint.h>
#include <_core/c_common.h>

typedef struct st_panel_name_t
{
    bool_t valid;
    char serial[32];
}st_panel_name_t;


void panel_name_init(void);
void panel_name_set(st_panel_name_t *ptr);
st_panel_name_t panel_name_get(void);

#endif /* MODEM_PANEL_NAME_H_ */
