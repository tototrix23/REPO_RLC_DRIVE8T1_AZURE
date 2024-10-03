/*
 * JSON_process.h
 *
 *  Created on: 27 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef CJSON_JSON_PROCESS_H_
#define CJSON_JSON_PROCESS_H_

#include <stdint.h>
#include <_core/c_common.h>
#include <exchanged_data/exchanged_data.h>
return_t json_process_get_datetime(char *ptr);
return_t json_process_get_serials(char *ptr);
return_t json_process_lte_connect(char *ptr);
return_t json_process_get_data(char *ptr);
return_t json_process_send_data_and_event(char *ptr);
return_t json_process_mqtt_publish(char *ptr,int *status_code);

#endif /* CJSON_JSON_PROCESS_H_ */