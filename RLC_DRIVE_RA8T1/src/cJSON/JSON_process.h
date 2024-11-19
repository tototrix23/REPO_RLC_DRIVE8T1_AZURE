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
#include <cJSON/cJSON.h>
#include <exchanged_data/exchanged_data.h>
return_t json_process_get_datetime(char *ptr,int *status_code);
return_t json_process_get_serials(char *ptr,int *status_code);
return_t json_process_lte_connect(char *ptr);
return_t json_process_get_data(char *ptr);
return_t json_process_send_data_and_event(char *ptr);
return_t json_process_mqtt_publish(char *ptr,int *status_code);
return_t json_process_mqtt_subscribe(char *ptr,int *status_code);
return_t json_process_verify_received_type(char *type,char *data);
return_t json_process_get_panel_name(char *ptr,int *status_code);

return_t json_process_subsbribe_firmware(cJSON *ptr_cjson);
return_t json_process_subsbribe_scrolling_settings(e_settings_type type,cJSON *ptr_cjson);
#endif /* CJSON_JSON_PROCESS_H_ */
