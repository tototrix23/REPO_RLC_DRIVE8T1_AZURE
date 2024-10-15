#include "log_thread.h"
#include "_lib_impl_cust/impl_log/impl_log.h"
#include <files/lfs_impl.h>
#include <files/mqtt_file.h>
#include <rtc/rtc.h>
#include <modem/serial.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "log thread"



/* Log Thread entry function */
void log_thread_entry(void)
{
    /* TODO: add your own code here */
    log_t *ptr = 0x00;
    while (1)
    {
        volatile uint32_t status = tx_queue_receive(&g_queue_log, &ptr, TX_NO_WAIT);
        if(status == TX_SUCCESS)
        {

            impl_log_write(ptr->mode,ptr->color,ptr->module, 0x00, ptr->func, ptr->line, ptr->text);
            free(ptr);
        }
        tx_thread_sleep (1);



        // Récupération de la RTC pour vérifier si un timestamp correcte est disponible dans le système.
        st_rtc_t r = rtc_get();
        // Récupération du numéro de série pour vérifier si ce dernier a bien été récupéré dans le modem.
        st_serials_t ser = serials_get();



        // Si tout est OK.
        if(r.configured == TRUE && strlen(ser.serial)>0)
        {
            bool_t error_flash = FALSE;

            json_file_t *ptr_json = 0x00;
            // Tentative de lecture d'une entrée dans la FIFO en RAM.
            status = tx_queue_receive(&g_queue_json, &ptr_json, TX_NO_WAIT);

            // Si une entrée est disponible.
            if(status == TX_SUCCESS)
            {
                // Récupération du mutex donnant accès à la mémoire FLASH.
                tx_mutex_get(&g_flash_mutex,TX_WAIT_FOREVER);

                lfs_dir_t dir;
                bool_t dir_opened = FALSE;
                int err = 0x00;
                char *path;

                path = (char*)dir_json;

                // Ouverture du dossier JSON
                err = lfs_dir_open(&lfs,&dir,dir_json);
                if(err == 0x00)
                {
                    dir_opened = TRUE;
                }

                // Si le dossier est bien présent et s'il est ouvert.
                if(dir_opened == TRUE)
                {
                    lfs_file_t file;

                    // Lecture de la RTC.
                    st_rtc_t rtc_ts = rtc_get();

                    // Correction du timestamp de l'entrée si nécessaire (si crée avant qu'un timestamp soit connnu).
                    if(ptr_json->rtc.configured == FALSE)
                    {
                        ptr_json->rtc.time_ms = rtc_ts.time_ms-ptr_json->rtc.time_ms;
                    }


                    // Allocation d'un buffer de travail.
                    char *ptr_full = malloc(1024);
                    if(ptr_full != 0x00)
                    {
                        // RAZ du buffer
                        memset(ptr_full,0x00,1024);
                        // Création du flux MQTT.
                        json_create_full_mqtt_publish(ptr_full,ptr_json);
                        // Création du nom de fichier.
                        char filename[32];
                        memset(filename,0x00,sizeof(filename));
                        char temp_timestamp[16];
                        sprintf(filename,"%s/",path);
                        sprintf(temp_timestamp,"%llu",rtc_ts.time_ms);
                        strcat(filename,temp_timestamp);
                        strcat(filename,".json");

                        // Création du fichier.
                        rtc_ts = rtc_get();
                        err = lfs_file_open(&lfs, &file, filename, LFS_O_RDWR | LFS_O_CREAT);
                        // Si création OK.
                        if(err == 0x00)
                        {
                            // Calcul de nombre de caractère dans le flux à copier.
                            volatile lfs_size_t length = strlen(ptr_full);
                            // Copie du flux.
                            lfs_ssize_t s = lfs_file_write(&lfs, &file,ptr_full,length);
                            if(s<0)
                            {
                                LOG_E(LOG_STD,"Error writing file %s  -> code %d",filename,s);
                            }
                            else
                            {
                                lfs_file_sync(&lfs, &file);
                            }
                            // Fermeture du fichier.
                            lfs_file_close(&lfs, &file);
                            LOG_I(LOG_STD,"Success creating file %s",filename);

                            // Enregistrement du timestamp dans les attributs du fichier.
                            err = lfs_setattr(&lfs, filename,
                                    0, &rtc_ts.time_ms, sizeof(uint64_t));

                            if(err == 0x00)
                            {

                            }
                            else
                            {
                                LOG_E(LOG_STD,"Error writing attributes");
                            }
                        }
                        else
                        {
                            LOG_E(LOG_STD,"Error creating file %s with code %d",filename,err);
                            if(err == LFS_ERR_CORRUPT || err == LFS_ERR_NAMETOOLONG)
                            {
                                error_flash = TRUE;
                            }
                        }
                        // Fermeture du dossier.
                        lfs_dir_close(&lfs,&dir);
                        // Libération de la mémoire allouée pour le contenu MQTT.
                        free(ptr_full);

                        // Traitement spécifique pour formatter la mémoire si une erreur indique une corruption
                        if(error_flash)
                        {
                            LOG_E(LOG_STD,"Format FLASH memory");
                            LFS_Init(TRUE);
                        }
                    }
                }
                else
                {
                    LOG_E(LOG_STD,"Error opening directory");
                }

                // Liberation de la zone mémoire allouée au texte.
                free(ptr_json->ptr_data);
                // Liberation de la zone mémoire allouée à la structure.
                free(ptr_json);
                // Libération du mutex de la FLASH externe.
                tx_mutex_put(&g_flash_mutex);
            }
        }
        tx_thread_sleep (1);
    }
}
