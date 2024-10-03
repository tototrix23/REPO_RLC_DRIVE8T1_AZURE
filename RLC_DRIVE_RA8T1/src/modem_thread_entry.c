#include "modem_thread.h"
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>
#include <modem/modem.h>
#include <modem/serial.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "modem thread"


void modem_process_json_folder(void);




/* Modem Thread entry function */
void modem_thread_entry(void)
{

    return_t ret = X_RET_OK;
    bool_t end= FALSE;
    modem_init();


    do
    {
        delay_ms(1000);
        ret = modem_get_serials();
    }while(ret != X_RET_OK);

    end = FALSE;
    do
    {
        ret = modem_get_datetime();
        if(ret != X_RET_OK)
        {
            delay_ms(2000);
        }
        else
        {
            st_rtc_t r = rtc_get();
            LOG_D(LOG_STD,"Unix time %llu",r.time_ms);
            end = TRUE;
        }
    }while(end == FALSE);
    delay_ms(1000);



    /* TODO: add your own code here */
    while (1)
    {



    }
}
