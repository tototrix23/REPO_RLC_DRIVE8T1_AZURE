/*
 * serial.c
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */
#include <hal_data.h>
#include "serial.h"

st_serials_t system_serials;

void serials_init(void)
{
	tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
    memset(&system_serials,0x00,sizeof(st_serials_t));
    tx_mutex_put(&g_modem_vars_mutex);
}

void serials_set(st_serials_t *ptr)
{
	tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
	memcpy(&system_serials,ptr,sizeof(st_serials_t));
	tx_mutex_put(&g_modem_vars_mutex);
}

st_serials_t serials_get(void)
{
	tx_mutex_get(&g_modem_vars_mutex,TX_WAIT_FOREVER);
	st_serials_t s;
	memcpy(&s,&system_serials,sizeof(st_serials_t));
	tx_mutex_put(&g_modem_vars_mutex);
	return s;
}
