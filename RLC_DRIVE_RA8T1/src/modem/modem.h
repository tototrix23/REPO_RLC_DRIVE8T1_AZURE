/*
 * modem.h
 *
 *  Created on: 18 sept. 2024
 *      Author: Christophe
 */

#ifndef MODEM_MODEM_H_
#define MODEM_MODEM_H_

#include <_core/c_common.h>


return_t modem_init(void);
return_t modem_get_datetime(void);
return_t modem_get_serials(void);
return_t modem_send_mqtt_publish(char *type,char *data);
#endif /* MODEM_MODEM_H_ */
