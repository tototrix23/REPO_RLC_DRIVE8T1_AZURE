/*
 * settings.c
 *
 *  Created on: 5 nov. 2024
 *      Author: Christophe
 */
#include "return_codes.h"
#include "settings.h"
#include <rtc/rtc.h>


bool_t setting_is_only_hour(uint64_t ts);


bool_t setting_is_only_hour(uint64_t ts)
{
    if(ts<86400000)
        return true;
    else
        return FALSE;
}

bool_t settings_is_valid(st_settings_with_id_t *ptr)
{
    bool_t ret = FALSE;
    if(ptr->id[0] != 0x00)
    {
        ret = TRUE;
    }
    return ret;
}

return_t settings_is_in_range(st_settings_with_id_t *ptr,bool_t *res)
{
    return_t ret = X_RET_OK;
    *res = FALSE;
    ret = settings_is_valid(ptr);
    if(ret == FALSE)
       return F_RET_SETTINGS_NOT_VALID;


    st_rtc_t r = rtc_get();
    if(r.configured == FALSE)
        return F_RET_RTC_NOT_CONFIGURED;


    int i=0;
    for(i=0;i<SETTINGS_MAX_COUNT;i++)
    {
        st_time_settings_t set = ptr->array[i];

        if(set.start != 0 || set.stop != 0)
        {
            if(set.start != 0 && set.stop != 0)
            {
                if(setting_is_only_hour(set.start)==TRUE && setting_is_only_hour(set.stop)==TRUE)
                {
                    if(set.start <= set.stop)
                    {
                       if(r.time_ms_without_date>set.start && r.time_ms_without_date<set.stop)
                           *res = TRUE;
                    }
                    else
                    {
                        if(r.time_ms_without_date>set.stop || r.time_ms_without_date<set.start)
                           *res = TRUE;
                    }
                }
                else if(setting_is_only_hour(set.start)==FALSE && setting_is_only_hour(set.stop)==FALSE)
                {
                    if(r.time_ms>set.start && r.time_ms<set.stop)
                       *res = TRUE;
                }
            }
            else if(set.start != 0 && set.stop == 0)
            {
                if(setting_is_only_hour(set.start)==FALSE)
                {
                    if(r.time_ms>set.start)
                       *res = TRUE;
                }
            }
            else if(set.start == 0 && set.stop != 0)
            {
                if(setting_is_only_hour(set.stop)==FALSE)
                {
                    if(r.time_ms<set.stop)
                       *res = TRUE;
                }
            }
        }
    }
    return ret;
}

