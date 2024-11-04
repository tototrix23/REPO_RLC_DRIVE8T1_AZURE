#include "mqtt_subscribe_thread.h"
#include "mqtt_publish_thread.h"

#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <modem/serial.h>
#include <modem/panel_name.h>
#include <modem/modem.h>
#include <cJSON/cJSON.h>
#include <cJSON/JSON_process.h>
#include <return_codes.h>
#include <my_malloc.h>
#include <firmware/pending_firmware.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "mqtt subscribe thread"

extern TX_THREAD mqtt_publish_thread;

bool_t mqtt_subscribe_done = FALSE;

return_t modem_order_get_panel_name(void);
return_t mqtt_subscribe_process(void);
return_t mqtt_subscribe_process_received_data(char *ptr);
return_t mqtt_subscribe_process_received_payload(cJSON *json_ptr,char *topic);
return_t mqtt_subscribe_list(void);

char* mqtt_subscribe_add_topic_to_list(char* buffer,char*prefix,char *entry_point,char *topic);


return_t modem_order_get_panel_name(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,"{\"type\": \"get_panel_name\",\"data\":null }");

    char *msg_rx = 0x00;

    //tx_queue_flush(msg_queue);
    ret = modem_process_send(&g_queue_modem_msg_mqtt_subscribe,"get_panel_name",tx_array,&msg_rx,1,5000);
    if(ret != X_RET_OK)
    {
        if(ret == F_RET_COMMS_OUT_TIMEOUT)
        {
            LOG_E(LOG_STD,"error timeout");
        }
        else
        {
            LOG_E(LOG_STD,"error %d",ret);
        }
        goto end;
    }

    // Traitement de la réponse
    int status_code;
    ret = json_process_get_panel_name(msg_rx,&status_code);
    if(ret == X_RET_OK)
    {
    }
    else if(ret == F_RET_JSON_RESPONSE_ERROR )
    {
        LOG_E(LOG_STD,"error response %d",status_code);
    }
    else
    {
        LOG_E(LOG_STD,"error  %d",ret);
    }

    end:
    if(msg_rx != 0x00)
        FREE((void**)&msg_rx);

    return ret;
}


char* mqtt_subscribe_add_topic_to_list(char* buffer,char*prefix,char *entry_point,char *topic)
{
    char t[256];
    char *ptr_buffer = buffer;
    char *ptr = t;
    ptr_buffer = strcat(ptr_buffer,"{\"topic\":");

    ptr = strcpy(ptr,"\"Mqtt/");
    ptr = strcat(ptr,prefix);
    ptr = strcat(ptr,"/");
    ptr = strcat(ptr,entry_point);
    ptr = strcat(ptr,"/");
    ptr = strcat(ptr,topic);
    ptr = strcat(ptr,"\",");
    ptr = strcat(ptr,"\"quality_of_service\":1}");
    ptr_buffer = strcat(ptr_buffer,t);
    return ptr_buffer;
}


return_t mqtt_subscribe_list(void)
{
    return_t ret = X_RET_OK;
    char tx_array[512];
    char *ptr = tx_array;
    memset(tx_array,0x00,sizeof(tx_array));


    st_panel_name_t p = panel_name_get();
    if(p.valid == FALSE)
        return F_RET_COMMS_MQTT_NO_PANELNAME;


    st_serials_t s = serials_get();

    strcpy(tx_array,"{\"type\": \"mqtt_subscribe\",\"data\":{\"mqtt_topic\":[");

    ptr = mqtt_subscribe_add_topic_to_list(ptr,"Firmwares",s.serial,"AuthorizedFirmwareVersion/Drive");
    ptr = strcat(ptr,"]}}");


    char *msg_rx = 0x00;
    ret = modem_process_send(&g_queue_modem_msg_mqtt_subscribe,"mqtt_subscribe",tx_array,&msg_rx,1,5000);
    if(ret != X_RET_OK)
    {
        if(ret == F_RET_COMMS_OUT_TIMEOUT)
        {
            LOG_E(LOG_STD,"error timeout");
        }
        else
        {
            LOG_E(LOG_STD,"error %d",ret);
        }
        goto end;
    }

    // Traitement de la réponse
    int status_code;
    ret = json_process_mqtt_subscribe(msg_rx,&status_code);
    if(ret == X_RET_OK)
    {
        /*st_serials_t s = serials_get();
        LOG_D(LOG_STD,"serial %s",s.serial);*/
    }
    else if(ret == F_RET_JSON_RESPONSE_ERROR )
    {
        LOG_E(LOG_STD,"error response %d",status_code);
    }
    else
    {
        LOG_E(LOG_STD,"error  %d",ret);
    }
    end:
    if(msg_rx != 0x00)
        FREE((void**)&msg_rx);
    return ret;
}

return_t mqtt_subscribe_process_received_payload(cJSON *json_ptr,char *topic)
{
    return_t ret = X_RET_OK;

    cJSON *ptr_json = cJSON_Parse(json_ptr->valuestring);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

    cJSON *json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }


    char *ptr_topic = 0x00;
    ptr_topic = strstr(topic,"AuthorizedFirmwareVersion/Drive");
    if(ptr_topic != 0x00)
    {


        cJSON *json_firmware = cJSON_GetObjectItemCaseSensitive(json_data, "firmware_version");
        if(json_firmware == NULL)
        {
            ret = F_RET_JSON_FIND_OBJECT;
            goto end;
        }



        if(cJSON_IsString(json_firmware))
        {
            LOG_D(LOG_STD,"AuthorizedFirmwareVersion/Drive [%s]",json_firmware->valuestring);

            st_pending_firmware_t p;
            char *current_f = exchdat_get_firmware();
            if(strcmp(current_f,json_firmware->valuestring) != 0x00)
            {
                p.pending = TRUE;
                strcpy(p.firmware,json_firmware->valuestring);
            }
            else
            {
                p.pending = FALSE;
            }
            pending_firmware_set(p);
        }
        else
        {
            ret = F_RET_JSON_BAD_TYPE;
            goto end;
        }

        goto end;
    }
    else
    {
        ret = F_RET_JSON_NOT_FOUND;
    }
    end:
    cJSON_Delete(ptr_json);
    return ret;
}

return_t mqtt_subscribe_process_received_data(char *ptr)
{
    return_t ret = X_RET_OK;
    uint32_t msg_id=0;
    char topic[64];
    char payload[256];
    memset(&topic,0x00,sizeof(topic));
    memset(&payload,0x00,sizeof(payload));

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    // Traitement du message ID
    cJSON *json_msg_topic = cJSON_GetObjectItemCaseSensitive(json_data, "topic");
    if(json_msg_topic == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsString(json_msg_topic))
    {
        strcpy(topic,json_msg_topic->valuestring);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    // Traitement du message ID
    cJSON *json_msg_id = cJSON_GetObjectItemCaseSensitive(json_data, "message_id");
    if(json_msg_id == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_msg_id))
    {
        msg_id = json_msg_id->valueint;
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    // Traitement de la payload
    cJSON *json_msg_payload = cJSON_GetObjectItemCaseSensitive(json_data, "payload");
    if(json_msg_payload == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsString(json_msg_payload))
    {




        strcpy(payload,json_msg_topic->valuestring);

        ret = mqtt_subscribe_process_received_payload(json_msg_payload,topic);
        if(ret != X_RET_OK)
        {
            LOG_E(LOG_STD,"Error %d",ret);
        }
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }


    // Préparation de la réponse
    char tx_array[256];
    memset(tx_array,0x00,sizeof(tx_array));
    char *ptr_tx = tx_array;
    ptr_tx = strcpy(ptr_tx,"{\"type\": \"mqtt_received_data\",\"data\":{\"status_code\":");
    if(ret == X_RET_OK)
        ptr_tx = strcat(ptr_tx,"0");
    else
        ptr_tx = strcat(ptr_tx,"-1");


    ptr_tx = strcat(ptr_tx,",\"message_id\":");
    char buffer[16];
    sprintf(buffer,"%lu",msg_id);
    ptr_tx = strcat(ptr_tx,buffer);
    ptr_tx = strcat(ptr_tx,"}}");


    // Envoi de la réponse
    modem_process_send_only(tx_array);

    end:
    cJSON_Delete(ptr_json);
    return ret;
}

return_t mqtt_subscribe_process(void)
{
    return_t ret = X_RET_OK;
    char *msg;
    uint32_t status = tx_queue_receive(&g_queue_modem_msg_mqtt_subscribe, &msg, TX_NO_WAIT);
    if(status == TX_SUCCESS)
    {
        if(json_process_verify_received_type("mqtt_get_list_subscribe_topic",msg) == X_RET_OK)
        {
           mqtt_subscribe_done = FALSE;
        }
        else
        {
            if(json_process_verify_received_type("mqtt_received_data",msg) == X_RET_OK)
            {
                ret = mqtt_subscribe_process_received_data(msg);
            }
        }
        FREE((void**)&msg);
    }
    return ret;
}


/* MQTT subscribe Thread entry function */
void mqtt_subscribe_thread_entry(void)
{
    return_t ret=X_RET_OK;
    LOG_I(LOG_STD,"Thread start");


    while(mqtt_subscribe_done == FALSE)
    {
        ret = modem_order_get_panel_name();
        if(ret == X_RET_OK)
        {
            ret = mqtt_subscribe_list();
            if(ret == X_RET_OK)
                mqtt_subscribe_done = TRUE;
        }
        delay_ms(3000);
    }

    tx_thread_resume(&mqtt_publish_thread);

    while (1)
    {

        while(mqtt_subscribe_done == FALSE)
        {
            ret = modem_order_get_panel_name();
            if(ret == X_RET_OK)
            {
                ret = mqtt_subscribe_list();
                if(ret == X_RET_OK)
                    mqtt_subscribe_done = TRUE;
            }
            delay_ms(3000);
        }

        mqtt_subscribe_process();
        tx_thread_sleep (10);
    }
}
