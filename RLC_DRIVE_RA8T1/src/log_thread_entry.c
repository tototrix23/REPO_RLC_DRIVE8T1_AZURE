#include "log_thread.h"
#include "_lib_impl_cust/impl_log/impl_log.h"
#include <files/lfs_impl.h>
#include <files/json_file.h>
#include <rtc/rtc.h>

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
        volatile uint32_t status = tx_queue_receive(&g_queue_log, &ptr, TX_NO_WAIT);//TX_WAIT_FOREVER
        if(status == TX_SUCCESS)
        {

            impl_log_write(ptr->mode,ptr->color,ptr->module, 0x00, ptr->func, ptr->line, ptr->text);
            free(ptr);
        }
        //tx_thread_sleep (1);


        json_file_t *ptr_json = 0x00;
        status = tx_queue_receive(&g_queue_json, &ptr_json, TX_NO_WAIT);
        if(status == TX_SUCCESS)
        {
            lfs_dir_t dir;
            bool_t dir_opened = FALSE;
            int err_open_dir = 0x00;
            char *path;

            switch(ptr_json->type)
            {
                case FILE_TYPE_PAYLOAD:
                    path = (char*)dir_payloads;
                    err_open_dir = lfs_dir_open(&lfs,&dir,dir_payloads);
                    if(err_open_dir == 0x00)
                    {
                        dir_opened = TRUE;
                    }
                    break;

                case FILE_TYPE_EVENT:
                    path = (char*)dir_events;
                    err_open_dir = lfs_dir_open(&lfs,&dir,dir_events);
                    if(err_open_dir == 0x00)
                    {
                        dir_opened = TRUE;
                    }
                    break;

                default:
                    break;
            }

            if(dir_opened == TRUE)
            {
                lfs_file_t file;
                st_rtc_t rtc_ts = rtc_get();

                char filename[32];
                char temp_timestamp[16];
                sprintf(filename,"%s/pay_",path);
                sprintf(temp_timestamp,"%lu",rtc_ts.time_ms);
                strcat(filename,temp_timestamp);
                //strcat(filename,"pay_%lu",rtc_ts.time_ms);
                strcat(filename,".json");

                volatile count = 0;


                int err_open = lfs_file_open(&lfs, &file, filename, LFS_O_RDWR | LFS_O_CREAT | LFS_O_EXCL);
                if(err_open == 0x00)
                {
                    lfs_size_t length = strlen(ptr_json->ptr_data);

                    lfs_ssize_t s = lfs_file_write(&lfs, &file,ptr_json->ptr_data,length);
                    if(s<0)
                    {
                        LOG_E(LOG_STD,"Error writing file %s  -> code %d",filename,s);
                    }
                    else
                    {
                        volatile int err = lfs_file_sync(&lfs, &file);
                        volatile int i=0;
                    }
                    lfs_file_close(&lfs, &file);
                    LOG_I(LOG_STD,"Success creating file %s",filename);
                }
                else
                {
                    LOG_E(LOG_STD,"Error creating file %s",filename);
                }

                lfs_dir_close(&lfs,&dir);
            }
            else
            {
                LOG_E(LOG_STD,"Error opening directory");
            }



            // Liberation de la zone mémoire allouée au texte
            free(ptr_json->ptr_data);
            // Liberation de la zone mémoire allouée à la structure
            free(ptr_json);
        }
        tx_thread_sleep (1);

    }
}
