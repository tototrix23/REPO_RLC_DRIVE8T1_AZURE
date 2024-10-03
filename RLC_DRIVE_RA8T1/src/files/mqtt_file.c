/*
 * json_file.c
 *
 *  Created on: 7 juin 2024
 *      Author: Christophe
 */
#include <stdio.h>
#include <files/mqtt_file.h>
#include <exchanged_data/exchanged_data.h>
#include <modem/serial.h>
#include <hal_data.h>
#include "common_data.h"
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "mqtt file"


return_t json_file_add_to_queue(char *topic,char *ptr)
{
    ASSERT(ptr != 0x00);

    json_file_t *ptr_st = (json_file_t*)malloc(sizeof(json_file_t));
    if(ptr != 0x00)
    {
        memset(ptr_st, 0, sizeof(json_file_t));
        ptr_st->ptr_data = ptr;
        st_rtc_t r = rtc_get();
        ptr_st->rtc = r;
        memcpy(ptr_st->topic,topic,strlen(topic));

        uint32_t status = tx_queue_send(&g_queue_json, &ptr_st, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            LOG_E(LOG_STD,"Error adding 'json_file_t' to queue");
            free(ptr);
            free(ptr_st);
            return X_RET_CONTAINER_FULL;
        }
    }
    else
    {
        free(ptr);
        return X_RET_MEMORY_ALLOCATION;
    }
    return X_RET_OK;
}


return_t json_create_full_mqtt_publish(char *ptr_full,json_file_t *json_descriptor)
{
    return_t ret = X_RET_OK;
    sprintf(ptr_full,"{\"type\": \"mqtt_publish\",");
    strcat(ptr_full,"\"data\":{");
    strcat(ptr_full,"\"topic\":");

    st_serials_t s = serials_get();
    char buffer[64];
    sprintf(buffer,"\"data/%s/%s\",",s.serial,json_descriptor->topic);
    strcat(ptr_full,buffer);
    strcat(ptr_full,"\"quality_of_service\":2,");
    strcat(ptr_full,"\"payload\":\"{");
    strcat(ptr_full,"\\\"timestamp_unix\\\":");
    sprintf(buffer,"%llu,",json_descriptor->rtc.time_ms);
    strcat(ptr_full,buffer);
    strcat(ptr_full,json_descriptor->ptr_data);
    strcat(ptr_full,"}\",");
    strcat(ptr_full,"\"dup_flag\":0,");
    strcat(ptr_full,"\"retain_flag\":0");
    strcat(ptr_full,"}");
    strcat(ptr_full,"}");

    return ret;
}


return_t mqtt_publish_temperature_humidity(void)
{
    return_t ret = X_RET_OK;

    char *payload = (char *)malloc(64);


    if(payload != 0x00)
    {
         memset(payload,0x00,64);
         float temp = exchdat_get_temperature();
         float hum = exchdat_get_humidity();
         char buffer[32];
         sprintf(payload,"\\\"data\\\": {");
         strcat(payload,"\\\"temperature\\\":");
         sprintf(buffer,"%.02f",temp);
         strcat(payload,buffer);
         strcat(payload,",");

         strcat(payload,"\\\"humidity\\\":");
         sprintf(buffer,"%.02f",hum);
         strcat(payload,buffer);
         strcat(payload,"}");

         ret = json_file_add_to_queue("sensor", payload);
         return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

