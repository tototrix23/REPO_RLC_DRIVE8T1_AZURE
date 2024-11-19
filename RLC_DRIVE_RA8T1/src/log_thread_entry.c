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
        tx_thread_sleep (1);
        volatile uint32_t status = tx_queue_receive(&g_queue_log, &ptr, TX_NO_WAIT);
        if(status == TX_SUCCESS)
        {
            impl_log_write(ptr->mode,ptr->color,ptr->module, 0x00, ptr->func, ptr->line, ptr->text);
            FREE((void**)&ptr);
        }
        else
        {

            // Récupération de la RTC pour vérifier si un timestamp correcte est disponible dans le système.
            st_rtc_t r = rtc_get();
            // Récupération du numéro de série pour vérifier si ce dernier a bien été récupéré dans le modem.
            st_serials_t ser = serials_get();

            if(r.configured == TRUE && strlen(ser.serial)>0 )
            {
                json_file_t *ptr_json = 0x00;
                status = tx_queue_receive(&g_queue_json, &ptr_json, TX_NO_WAIT);
                if(status == TX_SUCCESS)
                {
                    tx_mutex_get(&g_flash_memory_mutex,TX_WAIT_FOREVER);
                    return_t ret = fs_open();
                    if(ret == X_RET_OK)
                    {
                        ret = fs_set_directory(dir_data);
                        if(ret == X_RET_OK)
                        {
                            r = rtc_get();
                            if(ptr_json->rtc.configured == FALSE)
                            {
                                ptr_json->rtc.time_ms = r.time_ms-ptr_json->rtc.time_ms;
                            }
                            volatile size_t malloc_size = strlen(ptr_json->ptr_data)+256;
                            char *ptr_write = MALLOC(malloc_size);
                            if(ptr_write != 0x00)
                            {
                                // RAZ du buffer
                                memset(ptr_write,0x00,malloc_size);
                                // Création du flux MQTT.
                                json_create_full_mqtt_publish(ptr_write,ptr_json);
                                // Création du nom de fichier.
                                char filename[32];
                                memset(filename,0x00,sizeof(filename));
                                sprintf(filename,"%llu",r.time_ms);
                                strcat(filename,".json");
                                // Création du fichier.
                                uint64_t size = strlen(ptr_write);
                                ret = fs_file_create_and_write(filename,ptr_write,size);
                                if(ret == X_RET_OK)
                                {
                                    LOG_D(LOG_STD,"Success creating %s file [%s]",filename,ptr_json->topic);
                                }
                                else
                                {
                                    LOG_E(LOG_STD,"Error creating %s file [%s]",filename,ptr_json->topic);
                                }
                                // Libération de la mémoire allouée pour le contenu MQTT.
                                FREE((void**)&ptr_write);

                                fs_flush ();
                            }
                        }
                    }
                    fs_close();
                    FREE((void**)&ptr_json->ptr_data);
                    // Liberation de la zone mémoire allouée à la structure.
                    FREE((void**)&ptr_json);
                    tx_mutex_put(&g_flash_memory_mutex);

                    tx_thread_sleep (1);
                }
            }
        }
    }
}
