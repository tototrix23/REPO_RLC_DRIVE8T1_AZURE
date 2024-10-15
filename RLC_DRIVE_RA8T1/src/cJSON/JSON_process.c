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
#include <modem/modem_data.h>
#include <return_codes.h>

static return_t json_process_response_common(char *ptr);

return_t json_process_get_datetime(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    	return F_RET_JSON_PARSE;

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

return_t json_process_get_serials(char *ptr,int *status_code)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    	return F_RET_JSON_PARSE;

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
        return F_RET_JSON_PARSE;


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
		return F_RET_JSON_PARSE;

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
		return F_RET_JSON_PARSE;

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
		return F_RET_JSON_PARSE;
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


