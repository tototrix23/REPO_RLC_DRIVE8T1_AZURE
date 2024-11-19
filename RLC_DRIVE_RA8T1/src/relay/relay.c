/*
 * relay.c
 *
 *  Created on: 15 nov. 2024
 *      Author: Christophe
 */

#include "relay.h"
#include <exchanged_data/exchanged_data.h>
#include <settings/settings.h>
#include <rtc/rtc.h>

void relay_off(void);
void relay_on(void);

return_t relay_init(void)
{
    relay_off();
    return X_RET_OK;
}
return_t relay_process(void)
{

    st_settings_with_id_t light_settings = exchdat_get_lighting_settings();
    bool_t current_enabled = exchdat_get_lighting_enabled();
    if(light_settings.id[0] != 0x00)
    {
        if(light_settings.mode == setting_mode_force_off)
        {
           if(current_enabled == TRUE)
           {
              relay_off();
           }
           return X_RET_OK;
        }
        else if(light_settings.mode == setting_mode_force_on)
        {
            if(current_enabled == FALSE)
            {
               relay_on();
            }
            return X_RET_OK;
        }
    }
    else
    {
        if(current_enabled == TRUE)
        {
           relay_off();
        }
        return X_RET_OK;
    }


    st_rtc_t r = rtc_get();
    if(r.configured == FALSE)
    {
        return F_RET_RTC_NOT_CONFIGURED;
    }

    bool_t active;
    settings_is_in_range(&light_settings,&active);
    if(current_enabled == TRUE && active == FALSE)
    {
        relay_off();
    }
    else if(current_enabled == FALSE && active == TRUE)
    {
        relay_on();
    }

    return X_RET_OK;
}



void relay_off(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_RELAY_CMD,BSP_IO_LEVEL_LOW );
    exchdat_set_lighting_enabled(FALSE);

    delay_ms(50);
    bsp_io_level_t relay_verify;
    R_IOPORT_PinRead(&g_ioport_ctrl,IO_RELAY_VERIFY,&relay_verify);
    volatile bit value;
    if(relay_verify != BSP_IO_LEVEL_LOW)
    {
        value.b = 1;
    }
    else
    {
        value.b = 0;
    }
    exchdat_set_system_status_error_relay(value);

}

void relay_on(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_RELAY_CMD,BSP_IO_LEVEL_HIGH );
    exchdat_set_lighting_enabled(TRUE);
    delay_ms(50);
    bsp_io_level_t relay_verify;
    R_IOPORT_PinRead(&g_ioport_ctrl,IO_RELAY_VERIFY,&relay_verify);
    volatile bit value;
    if(relay_verify != BSP_IO_LEVEL_HIGH)
    {
        value.b = 1;
    }
    else
    {
        value.b = 0;
    }
    exchdat_set_system_status_error_relay(value);
}
