#include "mqtt_publish_thread.h"
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <cJSON/JSON_process.h>
#include <flash/flash_routines.h>
#include <time.h>
#include <return_codes.h>
#include <my_malloc.h>
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
    int err = 0x00;

    ret = fs_open();
    if(ret != X_RET_OK)
       goto end;

    ret = fs_set_directory(dir_data);
    if(ret != X_RET_OK)
       goto end;


   volatile bool_t file_found = FALSE;
   volatile uint64_t time_ref = 0xFFFFFFFFFFFFFFFF;
   volatile char oldest_filename[64];
   int err_remove = 0;

   bool_t end = FALSE;
   volatile char filename[64];
   volatile ULONG file_size;
   volatile UINT year;
   volatile UINT month;
   volatile UINT day;
   volatile UINT hour;
   volatile UINT minut;
   volatile UINT second;

   fsp_err_t fsp_err=0;

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
           if(file_size == 0)
           {
               LOG_E(LOG_STD,"Size null %",filename);
               fs_file_delete(filename);

           }
           else
           {

               if(second <0 || second>59 || minut<0 || minut>59 || hour<0 ||
                  hour>23 || year<2024 || year>2050 || month<0 || month>12 ||
                  day<0 || day >31)
               {
                   LOG_E(LOG_STD,"Bad timestamp %",filename);
                   fs_file_delete(filename);
               }
               else
               {
                   struct tm myDate;
                   myDate.tm_mday = day;
                   myDate.tm_mon = month;
                   myDate.tm_year = year-1900;   // Date == April 6, 2014
                   myDate.tm_hour = hour;
                   myDate.tm_min = minut;
                   myDate.tm_sec = second;
                   uint64_t timestamp = mktime(&myDate);
                   timestamp = timestamp*1000;
                   st_rtc_t r = rtc_get();
                   volatile uint64_t time_diff = r.time_ms - timestamp;

                   if(time_diff > 259200000)//259200000)
                   {
                       ret = fs_file_delete(filename);
                   }
                   else
                   {
                       char *ptr = strstr(filename,".json");
                       if(ptr != NULL)
                       {
                           volatile uint32_t s = ptr - filename;
                           volatile char extract[32];
                           memset(extract,0x00,sizeof(extract));
                           memcpy(extract,filename,s);
                           char *ptr_end;
                           volatile uint64_t file_name_ts = strtoull(filename,ptr_end,10);
                           if(file_name_ts == 0)
                           {
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

       //tx_thread_sleep(1);
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
               FREE((void**)&ptr_read);
           }
       }

      // fs_file_delete(oldest_filename);
   }


   ULONG b;
   //fs_bytes_available(&b);
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
    ret = modem_process_send(msg_queue,"mqtt_publish",buffer,&msg_rx,3,60000);
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
            LOG_E(LOG_STD,"%d",ret);
            delay_ms(60000);
        }
        /*delay_ms(2000);
        LOG_D(LOG_STD,"TEST");*/

        tx_thread_sleep (1);
    }
}
