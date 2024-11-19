/*
 * JSON_process.c
 *
 *  Created on: 27 févr. 2024
 *      Author: Ch.Leclercq
 */
#include <stdio.h>
#include <cJSON/cJSON.h>
#include "JSON_process.h"
#include <rtc/rtc.h>
#include <modem/serial.h>
#include <modem/panel_name.h>
#include <modem/modem_data.h>
#include <firmware/pending_firmware.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "json_process"

static return_t json_process_response_common(char *ptr);

return_t json_process_get_datetime(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
    	ret = F_RET_JSON_PARSE;
    	goto end;
    }
    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
    if(json_status == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_status))
    {
        *status_code = json_status->valueint;
        if(json_status->valueint != 0)
        {
            ret = F_RET_JSON_RESPONSE_ERROR;
            goto end;
        }
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    cJSON *json_unix = cJSON_GetObjectItemCaseSensitive(json_data, "timestamp_unix");
    if(json_unix == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }



    if(!cJSON_IsNumber(json_unix))
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    if((uint64_t)json_unix->valuedouble != 0)
    {
        rtc_set((uint64_t)(json_unix->valuedouble));
    }

    end:
    cJSON_Delete(ptr_json);
    return ret;
}

return_t json_process_get_panel_name(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }


    cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
    if(json_status == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_status))
    {
        *status_code = json_status->valueint;
        if(json_status->valueint != 0)
        {
            ret = F_RET_JSON_RESPONSE_ERROR;
            goto end;
        }
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    // Récupération du serial (IMEI)
    st_panel_name_t p;
    memset(&p,0x00,sizeof(st_panel_name_t));
    cJSON *json_var = cJSON_GetObjectItemCaseSensitive(json_data, "panel_name");
    if(json_var == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNull(json_var))
    {
        p.valid=FALSE;
    }
    else if(cJSON_IsString(json_var))
    {
        p.valid = TRUE;
        strcpy(p.serial,json_var->valuestring);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }



    panel_name_set(&p);


    end:
    cJSON_Delete(ptr_json);
    return ret;
}

return_t json_process_get_serials(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }


    cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
    if(json_status == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_status))
    {
        *status_code = json_status->valueint;
        if(json_status->valueint != 0)
        {
            ret = F_RET_JSON_RESPONSE_ERROR;
            goto end;
        }
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    // Récupération du serial (IMEI)
    st_serials_t s;
    memset(&s,0x00,sizeof(st_serials_t));
    cJSON *json_var = cJSON_GetObjectItemCaseSensitive(json_data, "serial");
    if(json_var == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNull(json_var) || cJSON_IsString(json_var))
    {
    	strcpy(s.serial,json_var->valuestring);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }



    serials_set(&s);


    end:
    cJSON_Delete(ptr_json);
    return ret;
}


return_t json_process_mqtt_publish(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }


    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
    if(json_status == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_status))
    {
        *status_code = json_status->valueint;
        if(json_status->valueint == 0)
            goto end;
        else
        {
            ret = F_RET_JSON_RESPONSE_ERROR;
            goto end;
        }

    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    end:
       cJSON_Delete(ptr_json);
       return ret;
}

return_t json_process_mqtt_subscribe(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }


    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
    if(json_status == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsNumber(json_status))
    {
        *status_code = json_status->valueint;
        if(json_status->valueint == 0)
            goto end;
        else
        {
            ret = F_RET_JSON_RESPONSE_ERROR;
            goto end;
        }

    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    end:
       cJSON_Delete(ptr_json);
       return ret;
}

return_t json_process_lte_connect(char *ptr)
{
	return_t ret = X_RET_OK;

	const cJSON *json_data = NULL;
	cJSON *ptr_json = cJSON_Parse(ptr);
	if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

	json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
	if(ptr_json == NULL)
	{
		ret = F_RET_JSON_FIND_OBJECT;
		goto end;
	}

	cJSON *json_status = cJSON_GetObjectItemCaseSensitive(json_data, "status_code");
	if(json_status == NULL)
	{
		ret = F_RET_JSON_FIND_OBJECT;
		goto end;
	}

	if(cJSON_IsNumber(json_status))
	{
		if(json_status->valueint == 0)
			goto end;
		else
		{
			ret = F_RET_JSON_RESPONSE_ERROR;
			goto end;
		}

	}
	else
	{
		ret = F_RET_JSON_BAD_TYPE;
		goto end;
	}

	end:
	cJSON_Delete(ptr_json);
	return ret;
}



return_t json_process_get_data(char *ptr)
{
	return_t ret = X_RET_OK;
	memset(&modem_data,0x00,sizeof(modem_data));
	ret = json_process_response_common(ptr);
	if(ret != X_RET_OK)
		return ret;


	const cJSON *json_data = NULL;
	cJSON *ptr_json = cJSON_Parse(ptr);
	if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

	json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
	if(json_data == NULL)
	{
		ret = F_RET_JSON_FIND_OBJECT;
		goto end;
	}

	char *ptrsub = strstr(ptr,"\"data\":");
	char *pos_start = strchr(ptrsub,'{');
	char *pos_stop = strchr(ptrsub,'}');
	volatile uint32_t length = pos_stop-pos_start;

	if(pos_start != NULL && pos_stop != NULL)
	{
		strncpy(modem_data.data,pos_start,length+1);
		modem_data.valid = TRUE;
	}
	else
	{
		ret = F_RET_JSON_PARSE;
		goto end;
	}







	end:
	cJSON_Delete(ptr_json);
	return ret;
}



static return_t json_process_response_common(char *ptr)
{
	return_t ret = X_RET_OK;
	cJSON *json_data = NULL;
	cJSON *ptr_json = cJSON_Parse(ptr);
	if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }

	json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
	if(json_data == NULL)
	{
		ret = F_RET_JSON_FIND_OBJECT;
		goto end;
	}

	end:
	cJSON_Delete(ptr_json);
	return ret;
}


return_t json_process_verify_received_type(char *type,char *data)
{
    return_t ret = X_RET_OK;
    const cJSON *json_type = NULL;
    cJSON *ptr_json = cJSON_Parse(data);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }
    json_type = cJSON_GetObjectItemCaseSensitive(ptr_json, "type");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(!cJSON_IsString(json_type))
    {
        ret = X_RET_ERR_GENERIC;
        goto end;
    }

    if(strcmp(json_type->valuestring,type) != 0x0)
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }


    end:
    cJSON_Delete(ptr_json);
    return ret;
}


return_t json_process_subsbribe_firmware(cJSON *ptr_cjson)
{
    return_t ret = X_RET_OK;
    cJSON *json_firmware = cJSON_GetObjectItemCaseSensitive(ptr_cjson, "firmware_version");
    if(json_firmware == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(cJSON_IsString(json_firmware))
    {
        LOG_D(LOG_STD,"AuthorizedFirmwareVersion/Drive [%s]",json_firmware->valuestring);

        st_pending_firmware_t p;
        char *current_f = exchdat_get_firmware();
        if(strcmp(current_f,json_firmware->valuestring) != 0x00)
        {
            p.pending = TRUE;
            strcpy(p.firmware,json_firmware->valuestring);
        }
        else
        {
            p.pending = FALSE;
        }
        pending_firmware_set(p);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    end:
    return ret;
}

return_t json_process_subsbribe_scrolling_settings(e_settings_type type,cJSON *ptr_cjson)
{
    return_t ret = X_RET_OK;

    st_settings_with_id_t new_settings;
    memset(&new_settings,0x00,sizeof(st_settings_with_id_t));

    // ID
    cJSON *json_id = cJSON_GetObjectItemCaseSensitive(ptr_cjson, "id");
    if(json_id == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end_error;
    }

    if(cJSON_IsString(json_id))
    {
        strcpy(new_settings.id,json_id->valuestring);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end_error;
    }

    // MODE
    cJSON *json_mode = cJSON_GetObjectItemCaseSensitive(ptr_cjson, "mode");
    if(json_id == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end_error;
    }

    if(cJSON_IsNumber(json_mode))
    {
        new_settings.mode = (uint8_t)json_mode->valueint;
        LOG_D(LOG_STD,"Mode: %d",new_settings.mode);
    }
    else
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end_error;
    }

    if(new_settings.mode == setting_mode_force_off || new_settings.mode == setting_mode_force_on)
        goto end;

    // PLAGES
    cJSON *json_conf = cJSON_GetObjectItemCaseSensitive(ptr_cjson, "configurations");
    if(json_conf == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end_error;
    }

    uint8_t index=0;
    cJSON *conf;
    cJSON_ArrayForEach(conf, json_conf)
    {
        cJSON *start = cJSON_GetObjectItemCaseSensitive(conf, "start_time_unix");
        if(start == NULL){
           ret = F_RET_JSON_FIND_OBJECT;
           goto end_error;
        }
        cJSON *end = cJSON_GetObjectItemCaseSensitive(conf, "end_time_unix");
        if(end == NULL){
           ret = F_RET_JSON_FIND_OBJECT;
           goto end_error;
        }

        if (!cJSON_IsNumber(start) || !cJSON_IsNumber(end))
        {
            ret = F_RET_JSON_BAD_TYPE;
            goto end_error;
        }


        new_settings.array[index].start = (uint64_t)start->valuedouble;
        new_settings.array[index].stop = (uint64_t)end->valuedouble;
        index++;
    }

    end:

    // Sauvegarde des nouveaux paramètres
    if(type == setting_scrolling)
       exchdat_set_scrolling_settings(new_settings);
    else if(type == setting_lighting)
       exchdat_set_lighting_settings(new_settings);

    end_error:
    return ret;
}
