#include <stdio.h>
#include "log_thread.h"
#include "_lib_impl_cust/impl_log/impl_log.h"
#include <flash/flash_routines.h>
#include <files/mqtt_file.h>
#include <rtc/rtc.h>
#include <modem/serial.h>
#include <my_malloc.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "log thread"


volatile uint32_t log_line=0;

/* Log Thread entry function */
void log_thread_entry(void)
{
    /* TODO: add your own code here */
    log_t *ptr = 0x00;
    while (1)
    {
        log_line = __LINE__;
        volatile uint32_t status = tx_queue_receive(&g_queue_log, &ptr, TX_NO_WAIT);
        log_line = __LINE__;
        if(status == TX_SUCCESS)
        {
            log_line = __LINE__;
            impl_log_write(ptr->mode,ptr->color,ptr->module, 0x00, ptr->func, ptr->line, ptr->text);
            log_line = __LINE__;
            FREE((void**)&ptr);
            log_line = __LINE__;
        }
        log_line = __LINE__;


        // Récupération de la RTC pour vérifier si un timestamp correcte est disponible dans le système.
        st_rtc_t r = rtc_get();
        log_line = __LINE__;
        // Récupération du numéro de série pour vérifier si ce dernier a bien été récupéré dans le modem.
        st_serials_t ser = serials_get();
        log_line = __LINE__;

        if(r.configured == TRUE && strlen(ser.serial)>0 )
        {
            log_line = __LINE__;
            json_file_t *ptr_json = 0x00;
            status = tx_queue_receive(&g_queue_json, &ptr_json, TX_NO_WAIT);
            log_line = __LINE__;
            if(status == TX_SUCCESS)
            {
                log_line = __LINE__;
                tx_mutex_get(&g_flash_memory_mutex,TX_WAIT_FOREVER);
                log_line = __LINE__;
                return_t ret = fs_open();
                log_line = __LINE__;
                if(ret == X_RET_OK)
                {
                    log_line = __LINE__;
                    ret = fs_set_directory(dir_data);
                    log_line = __LINE__;
                    if(ret == X_RET_OK)
                    {
                        log_line = __LINE__;
                        r = rtc_get();
                        log_line = __LINE__;
                        if(ptr_json->rtc.configured == FALSE)
                        {
                            ptr_json->rtc.time_ms = r.time_ms-ptr_json->rtc.time_ms;
                        }
                        log_line = __LINE__;
                        char *ptr_write = MALLOC(1024);
                        log_line = __LINE__;
                        if(ptr_write != 0x00)
                        {
                            log_line = __LINE__;
                            // RAZ du buffer
                            memset(ptr_write,0x00,1024);
                            log_line = __LINE__;
                            // Création du flux MQTT.
                            json_create_full_mqtt_publish(ptr_write,ptr_json);
                            log_line = __LINE__;
                            // Création du nom de fichier.
                            char filename[32];
                            memset(filename,0x00,sizeof(filename));
                            log_line = __LINE__;
                            sprintf(filename,"%llu",r.time_ms);
                            log_line = __LINE__;
                            strcat(filename,".json");
                            log_line = __LINE__;
                            // Création du fichier.
                            uint64_t size = strlen(ptr_write);
                            log_line = __LINE__;
                            ret = fs_file_create_and_write(filename,ptr_write,size);
                            log_line = __LINE__;
                            if(ret == X_RET_OK)
                            {
                                LOG_D(LOG_STD,"Success creating %s file [%s]",filename,ptr_json->topic);
                            }
                            else
                            {
                                LOG_E(LOG_STD,"Error creating %s file [%s]",filename,ptr_json->topic);
                            }
                            log_line = __LINE__;
                            // Libération de la mémoire allouée pour le contenu MQTT.
                            FREE((void**)&ptr_write);
                            log_line = __LINE__;
                        }
                    }
                }
                log_line = __LINE__;
                fs_close();
                log_line = __LINE__;
                FREE((void**)&ptr_json->ptr_data);
                log_line = __LINE__;
                // Liberation de la zone mémoire allouée à la structure.
                FREE((void**)&ptr_json);
                log_line = __LINE__;
                tx_mutex_put(&g_flash_memory_mutex);
                log_line = __LINE__;
            }
        }


        tx_thread_sleep (10);
    }
}
