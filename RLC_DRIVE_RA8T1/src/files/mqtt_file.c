/*
 * json_file.c
 *
 *  Created on: 7 juin 2024
 *      Author: Christophe
 */
#include <stdio.h>
#include <hal_data.h>
#include "common_data.h"
#include <adc/adc.h>
#include <files/mqtt_file.h>
#include <exchanged_data/exchanged_data.h>
#include <modem/serial.h>
#include <my_malloc.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "mqtt file"


return_t json_file_add_to_queue(char *topic,char *ptr)
{
    ASSERT(ptr != 0x00);

    json_file_t *ptr_st = (json_file_t*)MALLOC(sizeof(json_file_t));
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
            FREE((void**)&ptr);
            FREE((void**)&ptr_st);
            return X_RET_CONTAINER_FULL;
        }
    }
    else
    {
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
    const uint16_t malloc_size = 128;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
         memset(payload,0x00,malloc_size);
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

return_t mqtt_publish_sensor(st_sensor_t *src)
{
    const uint16_t malloc_size = 128;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);


    if(payload != 0x00)
    {
         memset(payload,0x00,malloc_size);
         st_sensor_t sensor_data;
         memcpy(&sensor_data,src,sizeof(st_sensor_t));
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


return_t mqtt_publish_poster_count(uint8_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        uint8_t count = *src;
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

return_t mqtt_publish_battery_detected(bool_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        bool_t detected = *src;
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

return_t mqtt_publish_scrolling_enabled(bool_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        bool_t enabled = *src;
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

return_t mqtt_publish_lighting_enabled(bool_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        bool_t enabled = *src;
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

return_t mqtt_publish_motor_status(st_system_motor_status_t *src)
{
    const uint16_t malloc_size = 300;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        char buffer[8];
        st_system_motor_status_t status;
        memcpy(&status,src,sizeof(st_system_motor_status_t));
        sprintf(payload,"\\\"data\\\": {");

        strcat(payload,"\\\"error_lvl1\\\":");
        sprintf(buffer,"%d",status.error_lvl1.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl1_motor_high_status1\\\":");
        sprintf(buffer,"%d",status.error_lvl1_motorH.status1.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl1_motor_high_status2\\\":");
        sprintf(buffer,"%d",status.error_lvl1_motorH.status2.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl1_motor_low_status1\\\":");
        sprintf(buffer,"%d",status.error_lvl1_motorL.status1.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl1_motor_low_status2\\\":");
        sprintf(buffer,"%d",status.error_lvl1_motorL.status2.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl2\\\":");
        sprintf(buffer,"%d",status.error_lvl2.value);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"error_lvl3\\\":");
        sprintf(buffer,"%d",status.error_lvl3.value);
        strcat(payload,buffer);

        strcat(payload,"}");

        //volatile uint16_t size_t = strlen(payload);

        ret = json_file_add_to_queue("MotorStatus", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_voltages(st_voltages_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        char buffer[16];
        /*st_adc_t adc_snapshot;
        adc_get_snapshot(&adc_snapshot);

        float f_vin = (float)(adc_snapshot.vin / 1000.0f);
        float f_vbatt = (float)(adc_snapshot.vbatt / 1000.0f);*/

        memset(payload,0x00,malloc_size);
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"main_voltage\\\":");
        sprintf(buffer,"%.02f",src->main_voltage);
        strcat(payload,buffer);
        strcat(payload,",");

        strcat(payload,"\\\"battery_voltage\\\":");
        sprintf(buffer,"%.02f",src->battery_voltage);
        strcat(payload,buffer);

        strcat(payload,"}");
        ret = json_file_add_to_queue("Voltages", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_motor_type(motor_type_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        motor_type_t type = *src;
        char buffer[8];
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"motor_type\\\":");
        sprintf(buffer,"%d",type);
        strcat(payload,buffer);
        strcat(payload,"}");
        ret = json_file_add_to_queue("MotorType", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_board_version(uint8_t *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        uint8_t version = *src;
        char buffer[8];
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"board_version\\\":");
        sprintf(buffer,"%d",version);
        strcat(payload,buffer);
        strcat(payload,"}");
        ret = json_file_add_to_queue("DriveBoardVersion", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_firmware(char *src)
{
    const uint16_t malloc_size = 64;
    return_t ret = X_RET_OK;
    char *payload = (char *)MALLOC(malloc_size);
    if(payload != 0x00)
    {
        memset(payload,0x00,malloc_size);
        char* firmware = src;
        char buffer[32];
        sprintf(payload,"\\\"data\\\": {");
        strcat(payload,"\\\"firmware_version\\\":\\\"");
        sprintf(buffer,"%s",firmware);
        strcat(payload,buffer);
        strcat(payload,"\\\"}");
        ret = json_file_add_to_queue("FirmwareDriveVersion", payload);
        return ret;
    }
    else
    {
        return X_RET_MEMORY_ALLOCATION;
    }
    return ret;
}

return_t mqtt_publish_drive_mode(drive_mode_t *src)
{

    return_t ret = X_RET_OK;
    return ret;
}

