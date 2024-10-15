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
    sprintf(buffer,"\"Mqtt/Datas/%s/%s\",",s.serial,json_descriptor->topic);
    strcat(ptr_full,buffer);
    strcat(ptr_full,"\"quality_of_service\":1,");
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


return_t mqtt_publish_event(uint16_t code)
{
    return_t ret = X_RET_OK;
    char *payload = (char *)malloc(64);
    if(payload != 0x00)
    {
         memset(payload,0x00,64);
         char buffer[8];
         sprintf(payload,"\\\"data\\\": {");
         strcat(payload,"\\\"code\\\":");
         sprintf(buffer,"%d",code);
         strcat(payload,buffer);
         strcat(payload,"}");

         ret = json_file_add_to_queue("Events", payload);
         return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_temperature_humidity(void)
{
    return_t ret = X_RET_OK;

    char *payload = (char *)malloc(128);


    if(payload != 0x00)
    {
         memset(payload,0x00,128);
         st_sensor_t sensor_data = exchdat_get_sensor();
         char buffer[32];
         sprintf(payload,"\\\"data\\\": {");
         strcat(payload,"\\\"temperature\\\":");
         if(sensor_data.valid == TRUE){
             sprintf(buffer,"%.02f",sensor_data.temperature);
             strcat(payload,buffer);
         }
         else
         {
             strcat(payload,"null");
         }
         strcat(payload,",");
         strcat(payload,"\\\"humidity\\\":");

         if(sensor_data.valid == TRUE){
             sprintf(buffer,"%.02f",sensor_data.humidity);
             strcat(payload,buffer);
         }
         else
         {
             strcat(payload,"null");
         }
         strcat(payload,"}");

         ret = json_file_add_to_queue("Sensor", payload);
         return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}


return_t mqtt_publish_poster_count(void)
{
    return_t ret = X_RET_OK;
    char *payload = (char *)malloc(64);
    if(payload != 0x00)
    {
        uint8_t count = exchdat_get_poster_count();
        memset(payload,0x00,64);
        char buffer[8];
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"poster_count\\\":");
        sprintf(buffer,"%d",count);
        strcat(payload,buffer);
        strcat(payload,"}");
        ret = json_file_add_to_queue("PostersCount", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_battery_detected(void)
{
    return_t ret = X_RET_OK;
    char *payload = (char *)malloc(64);
    if(payload != 0x00)
    {
        bool_t detected = exchdat_get_battery_detected();
        memset(payload,0x00,64);
        char buffer[8];
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"battery_detected\\\":");
        if(detected)
            strcat(payload,"true");
        else
            strcat(payload,"false");
        strcat(payload,"}");
        ret = json_file_add_to_queue("BatteryDetected", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_scrolling_enabled(void)
{
    return_t ret = X_RET_OK;
    char *payload = (char *)malloc(64);
    if(payload != 0x00)
    {
        bool_t enabled = exchdat_get_scrolling_enabled();
        memset(payload,0x00,64);
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"scrolling_enabled\\\":");
        if(enabled)
            strcat(payload,"true");
        else
            strcat(payload,"false");
        strcat(payload,"}");
        ret = json_file_add_to_queue("ScrollingEnabled", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_lighting_enabled(void)
{
    return_t ret = X_RET_OK;
    char *payload = (char *)malloc(64);
    if(payload != 0x00)
    {
        bool_t enabled = exchdat_get_lighting_enabled();
        memset(payload,0x00,64);
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"lighting_enabled\\\":");
        if(enabled)
            strcat(payload,"true");
        else
            strcat(payload,"false");
        strcat(payload,"}");
        ret = json_file_add_to_queue("LightingEnabled", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_motor_status(void)
{
    return_t ret = X_RET_OK;
    return ret;
}
