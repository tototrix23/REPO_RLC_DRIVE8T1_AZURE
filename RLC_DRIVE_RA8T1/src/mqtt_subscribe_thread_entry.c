#include "mqtt_subscribe_thread.h"
#include "mqtt_publish_thread.h"

#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <rtc/rtc.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "mqtt subscribe thread"

extern TX_THREAD mqtt_publish_thread;


/* MQTT subscribe Thread entry function */
void mqtt_subscribe_thread_entry(void)
{

    LOG_I(LOG_STD,"Thread start");

    tx_thread_resume(&mqtt_publish_thread);

    /* TODO: add your own code here */
    while (1)
    {
        tx_thread_sleep (1);
    }
}
