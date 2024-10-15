#include "mqtt_publish_thread.h"
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <files/lfs_impl.h>
#include <cJSON/JSON_process.h>
#include <return_codes.h>

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
    tx_mutex_get(&g_flash_mutex,TX_WAIT_FOREVER);

    lfs_dir_t lfs_dir;
    int err = 0x00;


    // Ouverture du dossier JSON
    err = lfs_dir_open(&lfs,&lfs_dir,dir_json);
    if(err != 0x00)
    {
        goto end;
    }

   bool_t file_found = FALSE;
   uint64_t time_ref = 0xFFFFFFFFFFFFFFFF;
   char oldest_filename[64];
   int err_remove = 0;

   struct lfs_info dir_info;
   do{
      err = lfs_dir_read(&lfs,&lfs_dir,&dir_info);
      if(err == 1)
      {
          if(dir_info.type == LFS_TYPE_REG)
          {
              char full_path_name[64];
              strcpy(full_path_name,dir_json);
              strcat(full_path_name,"/");
              strcat(full_path_name,dir_info.name);

              uint64_t timestamp;
              lfs_ssize_t size = lfs_getattr(&lfs, full_path_name,
                    0, &timestamp, sizeof(uint64_t));

              if (size == LFS_ERR_NOATTR || size != sizeof(uint64_t) || size < 0) {
                // maybe assume arbitrary timestamp if missing?
                timestamp = 0;

                LOG_E(LOG_STD,"--- '%s' (%lu bytes) %llu",dir_info.name,dir_info.size,timestamp);

                err_remove = lfs_remove(&lfs, full_path_name);
                if(err_remove != 0)
                {
                    LOG_E(LOG_STD,"--- '%s' error deleting",dir_info.name);
                }
                else
                {
                    LOG_I(LOG_STD,"--- '%s' success deleting",dir_info.name);
                }

              }
              else
              {
                  //LOG_I(LOG_STD,"--- '%s' (%lu bytes) %llu",dir_info.name,dir_info.size,timestamp);

                  if(timestamp < time_ref)
                  {

                      st_rtc_t r = rtc_get();
                      uint64_t time_diff = r.time_ms - timestamp;

                      if(time_diff > 259200000)
                      {
                          err_remove = lfs_remove(&lfs, full_path_name);
                          LOG_W(LOG_STD,"old file %s",full_path_name);
                          if(err_remove != 0)
                          {
                              LOG_E(LOG_STD,"--- '%s' error deleting",dir_info.name);
                          }
                          else
                          {
                              LOG_I(LOG_STD,"--- '%s' success deleting",dir_info.name);
                          }
                      }
                      else
                      {
                          file_found = TRUE;
                          time_ref = timestamp;
                          strcpy(oldest_filename,full_path_name);
                      }
                  }
              }
          }
          else if(dir_info.type == LFS_TYPE_DIR)
          {

          }
      }

      tx_thread_sleep(10);
   }while(err > 0);



   if(file_found == TRUE)
   {
       LOG_D(LOG_STD,"oldest filename %s",oldest_filename);

       lfs_file_t file;
       err = lfs_file_open(&lfs, &file, oldest_filename, LFS_O_RDONLY);
       if(err == 0)
       {
           char buffer[1000];
           lfs_soff_t file_size = lfs_file_size(&lfs, &file);
           lfs_ssize_t count = lfs_file_read(&lfs, &file,buffer,file_size);
           if(count  != 0x00)
           {
              ret = mqtt_publish_send(buffer);
              lfs_file_close(&lfs, &file);
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
                      err_remove = lfs_remove(&lfs, oldest_filename);
                      if(err_remove != 0)
                      {
                          LOG_E(LOG_STD,"--- '%s' error deleting",oldest_filename);
                      }
                      else
                      {
                          LOG_D(LOG_STD,"--- '%s' success deleting",oldest_filename);
                      }
                  }
              }
              else
              {

              }
           }
           else
           {
               lfs_file_close(&lfs, &file);
               LOG_E(LOG_STD,"file null size %s",oldest_filename);
               err_remove = lfs_remove(&lfs, oldest_filename);
               if(err_remove != 0)
               {
                   LOG_E(LOG_STD,"--- '%s' error deleting",oldest_filename);
               }
               else
               {
                   LOG_D(LOG_STD,"--- '%s' success deleting",oldest_filename);
               }
           }
       }
       else
       {
          lfs_file_close(&lfs, &file);
       }
   }
   else
   {

   }
   lfs_dir_close(&lfs,&lfs_dir);

   end:
   tx_mutex_put(&g_flash_mutex);

   return ret;
}


return_t mqtt_publish_send(char *buffer)
{
    return_t ret = 0x00;

    char *msg_rx = 0x00;
    //
    tx_queue_flush(msg_queue);
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
        if(status_code == -70)
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
        free(msg_rx);

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
        if(ret != X_RET_OK)
        {
            delay_ms(60000);
        }
        else
        {
            delay_ms(2000);
        }
        tx_thread_sleep (1);
    }
}
