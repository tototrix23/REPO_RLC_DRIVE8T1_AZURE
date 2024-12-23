#include "mqtt_publish_thread.h"
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <cJSON/JSON_process.h>
#include <flash/flash_routines.h>
#include <time.h>
#include <return_codes.h>
#include <my_malloc.h>
#include <modem/modem.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "mqtt publish thread"

static TX_QUEUE *msg_queue = &g_queue_modem_msg_mqtt_publish;

return_t mqtt_pusblish_process_json(void);
return_t mqtt_publish_send(char *buffer);



return_t mqtt_pusblish_process_json(void)
{
    return_t ret = 0x00;
    tx_mutex_get(&g_flash_memory_mutex,TX_WAIT_FOREVER);

    ret = fs_open();
    if(ret != X_RET_OK)
       goto end;

    ret = fs_set_directory(dir_data);
    if(ret != X_RET_OK)
       goto end;


   bool_t file_found = FALSE;
   uint64_t time_ref = 0xFFFFFFFFFFFFFFFF;
   char oldest_filename[64];

   bool_t end = FALSE;
   char filename[64];
   ULONG file_size;
   UINT year;
   UINT month;
   UINT day;
   UINT hour;
   UINT minut;
   UINT second;



   bool_t first_file_processed = FALSE;
   do{

       if(first_file_processed == FALSE)
       {
           ret = fs_first_file_find(filename,&file_size,&year,&month,&day,&hour,&minut,&second);
           first_file_processed = TRUE;
       }
       else
       {
           ret = fs_next_file_find(filename,&file_size,&year,&month,&day,&hour,&minut,&second);
       }
       if(ret == X_RET_OK)
       {
           //LOG_D(LOG_STD,"parse %s",filename);
           if(file_size == 0)
           {
               LOG_E(LOG_STD,"Size null %",filename);
               fs_file_delete(filename);

           }
           else
           {

               if(second>59 ||minut>59 ||
                  hour>23 || year<2024 || year>2050 ||  month>12 ||
                   day >31)
               {
                   LOG_E(LOG_STD,"Bad timestamp %",filename);
                   fs_file_delete(filename);
               }
               else
               {
                   struct tm myDate;
                   myDate.tm_mday = day;
                   myDate.tm_mon = month-1;
                   myDate.tm_year = year-1900;   // Date == April 6, 2014
                   myDate.tm_hour = hour;
                   myDate.tm_min = minut;
                   myDate.tm_sec = second;
                   volatile uint64_t timestamp = mktime(&myDate);
                   timestamp = timestamp*1000;
                   st_rtc_t r = rtc_get();
                   volatile uint64_t time_diff = 0x00;
                   if(timestamp < r.time_ms)
                       time_diff = r.time_ms - timestamp;

                   if(time_diff > 259200000)//259200000)
                   {
                       LOG_E(LOG_STD,"time_diff %s   %llu",filename,time_diff);
                       ret = fs_file_delete(filename);
                   }
                   else
                   {
                       char *ptr = strstr(filename,".json");
                       if(ptr != NULL)
                       {
                           uint32_t s = ptr - filename;
                           char extract[32];
                           memset(extract,0x00,sizeof(extract));
                           memcpy(extract,filename,s);
                           char *ptr_end=0x0;
                           uint64_t file_name_ts = strtoull(filename,&ptr_end,10);
                           if(file_name_ts == 0)
                           {
                               LOG_E(LOG_STD,"file_name_ts %s",filename);
                               fs_file_delete(filename);
                           }
                           else
                           {
                               if(file_name_ts < time_ref)
                               {
                                   file_found = TRUE;
                                   time_ref = file_name_ts;
                                   strcpy(oldest_filename,filename);
                               }
                           }

                       }
                       else
                       {
                           LOG_E(LOG_STD,"ext %s",filename);
                           fs_file_delete(filename);
                       }
                   }
               }
           }
       }
       else
       {
           end = TRUE;
       }

       tx_thread_sleep(1);
   }while(!end);






   if(file_found == TRUE)
   {
       LOG_D(LOG_STD,"found %s",oldest_filename);
       FX_FILE file;
       ret = fs_file_open(&file,oldest_filename,FX_OPEN_FOR_READ);
       if(ret != X_RET_OK)
       {
           fs_file_delete(oldest_filename);
       }
       else
       {
           uint32_t size = (uint32_t)file.fx_file_current_file_size;
           char *ptr_read = MALLOC(size);
           if(ptr_read != 0x00)
           {
               memset(ptr_read,0x00,size);
               uint64_t actual_read = 0;
               ret = fs_file_read(&file, ptr_read, size, &actual_read);
               if(ret == X_RET_OK)
               {
                   fs_file_close(&file);

                   bool_t json_valid = FALSE;
                   cJSON *ptr_json = cJSON_Parse(ptr_read);
                   if(ptr_json == NULL)
                   {
                       json_valid = FALSE;
                   }
                   else
                       json_valid = TRUE;

                   cJSON_Delete(ptr_json);

                   if(json_valid == TRUE)
                   {
                       ret = mqtt_publish_send(ptr_read);
                       if(ret == X_RET_OK || ret == F_RET_COMMS_MQTT_TIMEOUT || ret == F_RET_COMMS_MQTT_GENERIC)
                       {
                           if(ret == X_RET_OK)
                           {
                              LOG_D(LOG_STD,"MQTT publish success");
                           }
                           else
                           {
                               LOG_E(LOG_STD,"MQTT publish error %d",ret);
                           }

                           if(ret == X_RET_OK || ret == F_RET_COMMS_MQTT_GENERIC)
                           {
                               fs_file_delete(oldest_filename);
                           }
                       }
                       else
                       {
                           if(ret == F_RET_COMMS_MQTT_BROK_NOT_CONNECTED)
                           {
                               LOG_W(LOG_STD,"Broker not connected");
                           }
                           else
                           {
                               LOG_E(LOG_STD,"Error %d",ret);
                           }
                       }
                   }
                   else
                   {
                       LOG_E(LOG_STD,"Error parsing, delete %s",oldest_filename);
                       fs_file_delete(oldest_filename);
                   }
               }
               FREE((void**)&ptr_read);
           }
       }

      // fs_file_delete(oldest_filename);
   }

   fs_flush ();
   fs_close();


   end:
   tx_mutex_put(&g_flash_memory_mutex);
   return ret;
}


return_t mqtt_publish_send(char *buffer)
{
    return_t ret = 0x00;

    char *msg_rx = 0x00;
    //
    //tx_queue_flush(msg_queue);

    volatile char *ptr_test = strstr(buffer,"ScrollingSettingId");
    if(ptr_test != 0x00)
    {
        volatile uint8_t ffff=0;
        ffff = 1;
    }

    ret = modem_process_send(msg_queue,"mqtt_publish",buffer,&msg_rx,1,40000);
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
    ret = json_process_mqtt_publish(msg_rx,&status_code);
    if(ret == X_RET_OK)
    {
        //LOG_D(LOG_STD,"MQTT publish success %d",status_code);
    }
    else if(ret == F_RET_JSON_RESPONSE_ERROR )
    {
        LOG_E(LOG_STD,"error response %d",status_code);
        if(status_code == -16)
            ret = F_RET_COMMS_MQTT_BUSY;
        else if(status_code == -70)
            ret = F_RET_COMMS_MQTT_TIMEOUT;
        else if(status_code == -114)
            ret = F_RET_COMMS_MQTT_LTE_NOT_CONNECTED;
        else if(status_code == -116)
            ret = F_RET_COMMS_MQTT_BROK_NOT_CONNECTED;
        else
            ret = F_RET_COMMS_MQTT_GENERIC;
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


/* MQTT publish Thread entry function */
void mqtt_publish_thread_entry(void)
{
    return_t ret = X_RET_OK;
    LOG_I(LOG_STD,"Thread start");




    delay_ms(1000);
    /* TODO: add your own code here */
    while (1)
    {
        ret = mqtt_pusblish_process_json();
        if(ret == X_RET_OK || ret == F_RET_FS_NO_MORE_ENTRIES)
        {
            delay_ms(2000);
        }
        else
        {
            delay_ms(10000);
        }
        tx_thread_sleep (1);
    }
}
