/*
 * panel_name.c
 *
 *  Created on: 30 oct. 2024
 *      Author: Christophe
 */

#include <hal_data.h>
#include "panel_name.h"


st_panel_name_t system_panel_name;

void panel_name_init(void)
{
    tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
    memset(&system_panel_name,0x00,sizeof(st_panel_name_t));
    tx_mutex_put(&g_modem_vars_mutex);
}

void panel_name_set(st_panel_name_t *ptr)
{
    tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
    memcpy(&system_panel_name,ptr,sizeof(st_panel_name_t));
    tx_mutex_put(&g_modem_vars_mutex);
}

st_panel_name_t panel_name_get(void)
{
    tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
    st_panel_name_t s;
    memcpy(&s,&system_panel_name,sizeof(st_panel_name_t));
    tx_mutex_put(&g_modem_vars_mutex);
    return s;
}

