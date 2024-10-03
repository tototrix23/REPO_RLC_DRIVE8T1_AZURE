/*
 * comms_modem_data.h
 *
 *  Created on: 6 mars 2024
 *      Author: Christophe
 */

#ifndef COMMS_MODEM_COMMS_MODEM_DATA_H_
#define COMMS_MODEM_COMMS_MODEM_DATA_H_

#include <stdint.h>
#include <_core/c_common.h>

#define MODEM_DATA_SIZE  1024


typedef struct st_modem_data_t
{
	char data[MODEM_DATA_SIZE];
	bool_t valid;
}st_modem_data_t;

extern st_modem_data_t modem_data;

#endif /* COMMS_MODEM_COMMS_MODEM_DATA_H_ */
