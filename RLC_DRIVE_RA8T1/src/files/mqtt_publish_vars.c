/*
 * mqtt_publish_vars.c
 *
 *  Created on: 28 oct. 2024
 *      Author: Christophe
 */
#include <_core/c_math/c_math.h>
#include <_hal/h_time/h_time.h>
#include "mqtt_publish_vars.h"
#include "mqtt_file.h"
#include <exchanged_data/exchanged_data.h>

return_t mqtt_vars_process_internal(st_mqtt_vars_t *ptr_var,bool_t forced);


void mqtt_sensor_override_check(st_mqtt_vars_t *ptr_var,bool_t *res);
void mqtt_voltages_override_check(st_mqtt_vars_t *ptr_var,bool_t *res);


st_mqtt_vars_t mqtt_vars_table[]=
{
   {   .publish_mode = MQTT_PUBLISH_ON_DELTA,
       .type = MQTT_TYPE_VOLTAGES,
       .var_pointer = &exchanged_data.voltages,
       .func_get_value = (void(*)(void*))exchdat_get_voltages2,
       .delta = 0.5f,
       .func_publish = (return_t(*)(void*))mqtt_publish_voltages,
       .func_override_change_check = (void(*)(void*,bool_t*)) mqtt_voltages_override_check
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_BOOL,
       .var_pointer = &exchanged_data.battery_detected,
       .func_get_value = (void*)(void*)exchdat_get_battery_detected2,
       .func_publish = (return_t(*)(void*))mqtt_publish_battery_detected,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_UINT8,
       .var_pointer = &exchanged_data.board_version,
       .func_get_value = (void*)(void*)exchdat_get_board_version2,
       .func_publish = (return_t(*)(void*))mqtt_publish_board_version,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_UINT8,
       .var_pointer = &exchanged_data.poster_count,
       .func_get_value = (void*)(void*)exchdat_get_poster_count2,
       .func_publish = (return_t(*)(void*))mqtt_publish_poster_count,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_BOOL,
       .var_pointer = &exchanged_data.scrolling_enabled,
       .func_get_value = (void*)(void*)exchdat_get_scrolling_enabled2,
       .func_publish = (return_t(*)(void*))mqtt_publish_scrolling_enabled,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_BOOL,
       .var_pointer = &exchanged_data.lighting_enabled,
       .func_get_value = (void*)(void*)exchdat_get_lighting_enabled2,
       .func_publish = (return_t(*)(void*))mqtt_publish_lighting_enabled,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_MOTOR_STATUS,
       .var_pointer = &exchanged_data.motor_status,
       .func_get_value = (void*)(void*)exchdat_get_motor_status2,
       .func_publish = (return_t(*)(void*))mqtt_publish_motor_status,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_MOTOR_TYPE,
       .var_pointer = &exchanged_data.motor_type,
       .func_get_value = (void*)(void*)exchdat_get_motor_type2,
       .func_publish = (return_t(*)(void*))mqtt_publish_motor_type,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_DRIVE_MODE,
       .var_pointer = &exchanged_data.drive_mode,
       .func_get_value = (void*)(void*)exchdat_get_drive_mode2,
       .func_publish = (return_t(*)(void*))mqtt_publish_drive_mode,
   },
   {   .publish_mode = MQTT_PUBLISH_ON_PERIOD_S,
       .type = MQTT_TYPE_SENSOR,
       .period_ms = 60000,
       .var_pointer = &exchanged_data.sensor_data,
       .func_get_value = (void*)(void*)exchdat_get_sensor2,
       .func_publish = (return_t(*)(void*))mqtt_publish_sensor,
       .func_override_change_check = (void(*)(void*,bool_t*)) mqtt_sensor_override_check
   },
   {   .publish_mode = MQTT_PUBLISH_ON_CHANGE,
       .type = MQTT_TYPE_CHAR,
       .var_pointer = &exchanged_data.firmware,
       .override_size = sizeof(exchanged_data.firmware),
       .func_get_value = (void*)(void*)exchdat_get_firmware2,
       .func_publish = (return_t(*)(void*))mqtt_publish_firmware,
   },


};



uint16_t mqtt_vars_count = sizeof(mqtt_vars_table)/sizeof(st_mqtt_vars_t);



return_t mqtt_vars_init(void)
{
    return_t ret = X_RET_OK;
    size_t allocated_bytes = 0;
    uint16_t i = 0;
    for(i=0;i<mqtt_vars_count;i++)
    {
        switch(mqtt_vars_table[i].type)
        {
            case MQTT_TYPE_FLOAT:
                mqtt_vars_table[i].variable_size = sizeof(float);
                break;

            case MQTT_TYPE_BOOL:
                mqtt_vars_table[i].variable_size = sizeof(bool_t);
                break;

            case MQTT_TYPE_UINT8:
                mqtt_vars_table[i].variable_size = sizeof(uint8_t);
                break;

            case MQTT_TYPE_UINT16:
                mqtt_vars_table[i].variable_size = sizeof(uint16_t);
                break;

            case MQTT_TYPE_UINT32:
                mqtt_vars_table[i].variable_size = sizeof(uint32_t);
                break;

            case MQTT_TYPE_UINT64:
                mqtt_vars_table[i].variable_size = sizeof(uint64_t);
                break;

            case MQTT_TYPE_MOTOR_TYPE:
                mqtt_vars_table[i].variable_size = sizeof(motor_type_t);
                break;

            case MQTT_TYPE_MOTOR_STATUS:
                mqtt_vars_table[i].variable_size = sizeof(st_system_motor_status_t);
                break;

            case MQTT_TYPE_DRIVE_MODE:
                mqtt_vars_table[i].variable_size = sizeof(drive_mode_t);
                break;

            case MQTT_TYPE_SENSOR:
                mqtt_vars_table[i].variable_size = sizeof(st_sensor_t);
                break;

            case MQTT_TYPE_CHAR:
                mqtt_vars_table[i].variable_size = sizeof(char);
                break;

        }

        if(mqtt_vars_table[i].override_size != 0)
            mqtt_vars_table[i].variable_size = mqtt_vars_table[i].override_size;

        mqtt_vars_table[i].first_time_publish = TRUE;
        mqtt_vars_table[i].save_pointer = 0x00;
        mqtt_vars_table[i].save_pointer = malloc(mqtt_vars_table[i].variable_size);
        if(mqtt_vars_table[i].save_pointer == 0)
            return X_RET_MEMORY_ALLOCATION;

        memset(mqtt_vars_table[i].save_pointer,0x00,mqtt_vars_table[i].variable_size);

        allocated_bytes +=mqtt_vars_table[i].variable_size;
        c_timespan_init(&mqtt_vars_table[i].timespan);
        h_time_update(&mqtt_vars_table[i].timespan);
    }
    LOG_D(LOG_STD,"MQTT variables %lu bytes allocated",allocated_bytes);





    return ret;
}


return_t mqtt_vars_process(void *var)
{
    return_t ret = X_RET_OK;
    uint8_t i=0;
    st_mqtt_vars_t *ptr = 0x00;
    for(i=0;i<mqtt_vars_count;i++)
    {
        ptr = &mqtt_vars_table[i];
        if(ptr->var_pointer == var)
        {
            bool_t forced = ptr->first_time_publish;
            ret = mqtt_vars_process_internal(ptr,forced);
            ptr->first_time_publish = FALSE;
            return ret;
        }
    }
    return ret;
}

return_t mqtt_vars_process_all(void)
{
    return_t ret = X_RET_OK;
    uint8_t i=0;
    st_mqtt_vars_t *ptr = 0x00;
    for(i=0;i<mqtt_vars_count;i++)
    {
        bool_t forced = ptr->first_time_publish;
        ptr = &mqtt_vars_table[i];
        mqtt_vars_process_internal(ptr,forced);
        ptr->first_time_publish = FALSE;
    }
    return ret;
}

return_t mqtt_vars_process_internal(st_mqtt_vars_t *ptr_var,bool_t forced)
{
    return_t ret = X_RET_OK;


    switch(ptr_var->publish_mode)
    {
        case MQTT_PUBLISH_ON_DELTA:
        {
            if(ptr_var->func_override_change_check != 0x00)
            {
                bool_t changed = FALSE;
                ptr_var->func_override_change_check(ptr_var,&changed);
                if(changed == TRUE || forced == TRUE)
                {
                    ptr_var->func_publish(ptr_var->save_pointer);
                }
            }
            else
            {
                switch(ptr_var->type)
                {
                    case MQTT_TYPE_FLOAT:
                    {
                        float value;
                        ptr_var->func_get_value(&value);
                        float *ptr_save = (float*)ptr_var->save_pointer;
                        float diff = (float)(fabs(value - *ptr_save));

                        if((diff > ptr_var->delta) || forced==TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;


                    default:
                        break;
                }
            }
        }
            break;

        case MQTT_PUBLISH_ON_CHANGE:
        {
            if(ptr_var->func_override_change_check != 0x00)
            {
                bool_t changed = FALSE;
                ptr_var->func_override_change_check(ptr_var,&changed);
                if(changed == TRUE || forced == TRUE)
                {
                    ptr_var->func_publish(ptr_var->save_pointer);
                }
            }
            else
            {
                switch(ptr_var->type)
                {
                    case MQTT_TYPE_FLOAT:
                    {
                        float value;
                        ptr_var->func_get_value(&value);
                        float *ptr_save = (float*)ptr_var->save_pointer;
                        bool_t equality = c_math_float_equality(value,*ptr_save);
                        if(equality == FALSE || forced==TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_BOOL:
                    {
                        bool_t value;
                        ptr_var->func_get_value(&value);
                        bool_t *ptr_save = (bool_t*)ptr_var->save_pointer;

                        if(*ptr_save != value|| forced==TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_MOTOR_STATUS:
                    {
                        st_system_motor_status_t value;
                        ptr_var->func_get_value(&value);
                        st_system_motor_status_t *ptr_save = (st_system_motor_status_t*)ptr_var->save_pointer;
                        int compare = memcmp(&value,ptr_save,sizeof(st_system_motor_status_t));
                        if(compare != 0x00 || forced == TRUE)
                        {
                            memcpy(ptr_save,&value,sizeof(st_system_motor_status_t));
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_MOTOR_TYPE:
                    {
                        motor_type_t value;
                        ptr_var->func_get_value(&value);
                        motor_type_t *ptr_save = (motor_type_t*)ptr_var->save_pointer;

                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_UINT8:
                    {
                        uint8_t value;
                        ptr_var->func_get_value(&value);
                        uint8_t *ptr_save = (uint8_t*)ptr_var->save_pointer;

                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_UINT16:
                    {
                        uint16_t value;
                        ptr_var->func_get_value(&value);
                        uint16_t *ptr_save = (uint16_t*)ptr_var->save_pointer;

                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_UINT32:
                    {
                        uint32_t value;
                        ptr_var->func_get_value(&value);
                        uint32_t *ptr_save = (uint32_t*)ptr_var->save_pointer;

                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;


                    case MQTT_TYPE_UINT64:
                    {
                        uint64_t value;
                        ptr_var->func_get_value(&value);
                        uint64_t *ptr_save = (uint64_t*)ptr_var->save_pointer;

                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_DRIVE_MODE:
                    {
                        drive_mode_t value;
                        ptr_var->func_get_value(&value);
                        drive_mode_t *ptr_save = (drive_mode_t*)ptr_var->save_pointer;
                        if(value != *ptr_save || forced == TRUE)
                        {
                            *ptr_save = value;
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    case MQTT_TYPE_SENSOR:
                    {
                        st_sensor_t value;
                        ptr_var->func_get_value(&value);
                        st_sensor_t *ptr_save = (st_sensor_t*)ptr_var->save_pointer;
                        int compare = memcmp(&value,ptr_save,sizeof(st_sensor_t));
                        if(compare != 0x00 || forced == TRUE)
                        {
                            memcpy(ptr_save,&value,sizeof(st_sensor_t));
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;


                    case MQTT_TYPE_CHAR:
                    {
                        char *value=0;
                        ptr_var->func_get_value(value);
                        char *ptr_save = (char*)ptr_var->save_pointer;

                        int compare = strcmp(value,ptr_save);
                        if(compare != 0x00 || forced == TRUE)
                        {
                            strcpy(ptr_save,value);
                            ptr_var->func_publish(ptr_var->save_pointer);
                        }
                    }
                        break;

                    default:
                        break;
                }
            }
        }
            break;

        case MQTT_PUBLISH_ON_PERIOD_S:
        {
            bool_t elapsed = FALSE;
            h_time_is_elapsed_ms(&ptr_var->timespan, ptr_var->period_ms, &elapsed);
            if(elapsed == TRUE || forced == TRUE)
            {
                h_time_update(&ptr_var->timespan);

                switch(ptr_var->type)
                {
                    case MQTT_TYPE_FLOAT:
                    {
                        float value;
                        ptr_var->func_get_value(&value);
                        float *ptr_save = (float*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_BOOL:
                    {
                        bool_t value;
                        ptr_var->func_get_value(&value);
                        bool_t *ptr_save = (bool_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_MOTOR_STATUS:
                    {
                        st_system_motor_status_t value;
                        ptr_var->func_get_value(&value);
                        st_system_motor_status_t *ptr_save = (st_system_motor_status_t*)ptr_var->save_pointer;
                        memcpy(ptr_save,&value,sizeof(st_system_motor_status_t));
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_MOTOR_TYPE:
                    {
                        motor_type_t value;
                        ptr_var->func_get_value(&value);
                        motor_type_t *ptr_save = (motor_type_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);

                    }
                        break;

                    case MQTT_TYPE_UINT8:
                    {
                        uint8_t value;
                        ptr_var->func_get_value(&value);
                        uint8_t *ptr_save = (uint8_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_UINT16:
                    {
                        uint16_t value;
                        ptr_var->func_get_value(&value);
                        uint16_t *ptr_save = (uint16_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_UINT32:
                    {
                        uint32_t value;
                        ptr_var->func_get_value(&value);
                        uint32_t *ptr_save = (uint32_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_UINT64:
                    {
                        uint64_t value;
                        ptr_var->func_get_value(&value);
                        uint64_t *ptr_save = (uint64_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_DRIVE_MODE:
                    {
                        drive_mode_t value;
                        ptr_var->func_get_value(&value);
                        drive_mode_t *ptr_save = (drive_mode_t*)ptr_var->save_pointer;
                        *ptr_save = value;
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                        break;

                    case MQTT_TYPE_SENSOR:
                    {
                        st_sensor_t value;
                        ptr_var->func_get_value(&value);
                        st_sensor_t *ptr_save = (st_sensor_t*)ptr_var->save_pointer;
                        memcpy(ptr_save,&value,sizeof(st_sensor_t));
                        ptr_var->func_publish(ptr_var->save_pointer);
                    }
                    break;

                    case MQTT_TYPE_CHAR:
                    {
                        char *value=0x0;
                        ptr_var->func_get_value(value);
                        char *ptr_save = (char*)ptr_var->save_pointer;
                        strcpy(ptr_save,value);
                        ptr_var->func_publish(ptr_var->save_pointer);

                    }
                        break;



                    default:
                        break;
                }
            }
        }
            break;
    }
    return ret;
}



void mqtt_sensor_override_check(st_mqtt_vars_t *ptr_var,bool_t *res)
{
    *res = FALSE;
    st_sensor_t value;
    ptr_var->func_get_value(&value);
    st_sensor_t *ptr_save = (st_sensor_t*)ptr_var->save_pointer;

    if(ptr_save->valid != value.valid)
    {
        *res = TRUE;
    }


    if(value.valid == TRUE)
    {
        switch(ptr_var->publish_mode)
        {
            case MQTT_PUBLISH_ON_DELTA:
            {
                if((fabs(value.temperature-ptr_save->temperature)>3.0f)||
                   (fabs(value.humidity-ptr_save->humidity)>5.0f))
                {
                    *res = TRUE;
                }
            }
                break;

            case MQTT_PUBLISH_ON_CHANGE:
            {
                bool_t equality_temp = c_math_float_equality(value.temperature,ptr_save->temperature);
                bool_t equality_hr = c_math_float_equality(value.humidity,ptr_save->humidity);
                if( equality_temp == FALSE || equality_hr == FALSE)
                {
                    *res = TRUE;
                }
            }
                break;

            default:

                break;
        }
    }

    if(*res == TRUE)
    {
        memcpy(ptr_save,&value,sizeof(st_sensor_t));
    }
}


void mqtt_voltages_override_check(st_mqtt_vars_t *ptr_var,bool_t *res)
{
    *res = FALSE;
    st_voltages_t value;
    ptr_var->func_get_value(&value);
    st_voltages_t *ptr_save = (st_voltages_t*)ptr_var->save_pointer;


    switch(ptr_var->publish_mode)
    {
        case MQTT_PUBLISH_ON_DELTA:
        {
            if((fabs(value.main_voltage-ptr_save->main_voltage)>ptr_var->delta)||
               (fabs(value.battery_voltage-ptr_save->battery_voltage)>ptr_var->delta))
            {
                *res = TRUE;
            }
        }
            break;

        case MQTT_PUBLISH_ON_CHANGE:
        {
            bool_t equ1 = c_math_float_equality(value.main_voltage,ptr_save->main_voltage);
            bool_t equ2 = c_math_float_equality(value.battery_voltage,ptr_save->battery_voltage);
            if( equ1 == FALSE || equ2 == FALSE)
            {
                *res = TRUE;
            }
        }
            break;

        default:

            break;
    }


    if(*res == TRUE)
    {
        memcpy(ptr_save,&value,sizeof(st_voltages_t));
    }
}




