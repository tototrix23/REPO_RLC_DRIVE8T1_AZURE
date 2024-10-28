#include "modem_thread.h"
#include "mqtt_subscribe_thread.h"


#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <modem/modem.h>
#include <modem/serial.h>
#include <cJSON/cJSON.h>
#include <cJSON/JSON_process.h>
#include <flash/flash_routines.h>
#include <my_malloc.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "modem thread"

extern TX_THREAD mqtt_subscribe_thread;

static TX_QUEUE *msg_queue = &g_queue_modem_msg_generic;
return_t modem_order_get_serial(void);
return_t modem_order_get_datetime(void);

return_t modem_order_get_serial(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,"{\"type\": \"get_serial\",\"data\":null }");

    char *msg_rx = 0x00;

    //tx_queue_flush(msg_queue);
    ret = modem_process_send(msg_queue,"get_serial",tx_array,&msg_rx,1,5000);
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
    ret = json_process_get_serials(msg_rx,&status_code);
    if(ret == X_RET_OK)
    {
        /*st_serials_t s = serials_get();
        LOG_D(LOG_STD,"serial %s",s.serial);*/
    }
    else if(ret == F_RET_JSON_RESPONSE_ERROR )
    {
        LOG_E(LOG_STD,"error response %d",status_code);
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

return_t modem_order_get_datetime(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,"{\"type\": \"get_datetime\",\"data\":null }");

    char *msg_rx = 0x00;

    //tx_queue_flush(msg_queue);
    ret = modem_process_send(msg_queue,"get_datetime",tx_array,&msg_rx,1,5000);//  modem_process_outgoing("get_datetime",tx_array,2,1000);
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
    ret = json_process_get_datetime(msg_rx,&status_code);
    if(ret == X_RET_OK)
    {
        //st_rtc_t r = rtc_get();
        //LOG_D(LOG_STD,"%llu",r.time_ms);
    }
    else if(ret == F_RET_JSON_RESPONSE_ERROR )
    {
        LOG_E(LOG_STD,"error response %d",status_code);
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

/* Modem Thread entry function */
void modem_thread_entry(void)
{

    return_t ret = X_RET_OK;
    modem_init();


    do
    {
        delay_ms(1000);
        ret = modem_order_get_serial();
    }while(ret != X_RET_OK);

    st_serials_t s = serials_get();
    LOG_I(LOG_STD,"serial %s",s.serial);

    tx_mutex_get(&g_flash_memory_mutex,TX_WAIT_FOREVER);
    do
    {
        delay_ms(1000);
        ret = modem_order_get_datetime();
    }while(ret != X_RET_OK);

    st_rtc_t r = rtc_get();
    LOG_I(LOG_STD,"%llu",r.time_ms);


    ret = fs_open();
    if(ret == X_RET_OK)
    {
        fs_set_timestamp();
        bool_t dir_exist;
        ret = fs_is_directory_exist(dir_data,&dir_exist);
        if(ret == X_RET_OK)
        {
            if(dir_exist == FALSE)
            {
                ret = fs_create_directory(dir_data);
                if(ret == X_RET_OK)
                {

                }
                else
                {
                    LOG_E(LOG_STD,"Error %d",ret);
                }
            }
        }
        else
        {
            LOG_E(LOG_STD,"Error %d",ret);
        }
    }
    else
    {
        LOG_E(LOG_STD,"Error %d",ret);
    }
    fs_close();
    tx_mutex_put(&g_flash_memory_mutex);


    delay_ms(1000);
    tx_thread_resume(&mqtt_subscribe_thread);


    uint32_t count = 0;

    /* TODO: add your own code here */
    while (1)
    {
        delay_ms(10000);
        ret = modem_order_get_datetime();

        if(ret == X_RET_OK)
        {
            r = rtc_get();
            LOG_D(LOG_STD,"%llu [%d]",r.time_ms,count);
            count++;
        }
        tx_thread_sleep(1);
    }
}
