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
return_t modem_process_send(TX_QUEUE *queue,char *type,char *data_tx,char **data_rx,uint8_t retry,uint32_t timeout_ms);
return_t modem_process_send_only(char *data_tx);

return_t modem_send_mqtt_publish(char *type,char *data);
#endif /* MODEM_MODEM_H_ */
