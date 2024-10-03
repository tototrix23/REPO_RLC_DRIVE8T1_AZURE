/*
 * modem.c
 *
 *  Created on: 18 sept. 2024
 *      Author: Christophe
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <hal_data.h>
#include "modem.h"
#include "modem_thread.h"
#include <_hal/h_time/h_time.h>
#include <cJSON/cJSON.h>
#include <cJSON/JSON_process.h>
#include <return_codes.h>
#include <rtc/rtc.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "modem"


typedef struct tx_info_t{
    bool_t in_progress;
    return_t code;
}tx_info_t;

typedef struct rx_info_t{
    bool_t in_progress;
    return_t code;
    bool_t waiting_for_process;
}rx_info_t;

#define TX_MAX_CHAR 2048
#define RX_MAX_CHAR 2048


char modem_rx_array[TX_MAX_CHAR];
uint16_t modem_rx_index = 0;
char modem_tx_array[RX_MAX_CHAR];
tx_info_t modem_tx_info;
rx_info_t modem_rx_info;
bool_t modem_tx_in_progress = FALSE;
bool_t modem_rx_in_progress = FALSE;


void modem_rx_clear(void);
void modem_send(char *data);
return_t modem_verify_received_json_type(char *type);
return_t modem_process_outgoing(char *type,char *data,uint8_t retry,uint32_t timeout_ms);
return_t process_unsolicited_rx(void);


return_t modem_init(void)
{
    return_t ret = X_RET_OK;

    fsp_err_t fsp_err = R_SCI_B_UART_Open (&g_uart_modem_ctrl, &g_uart_modem_cfg);
    if(fsp_err != FSP_SUCCESS)
        return -1;

    modem_rx_clear();
    modem_tx_in_progress = FALSE;
    modem_rx_in_progress = FALSE;
    return ret;
}

void modem_rx_clear(void)
{
    tx_mutex_get(&g_mutex_uartrx_modem,TX_WAIT_FOREVER);
    modem_rx_info.in_progress = FALSE;
    modem_rx_index = 0;
    memset(modem_rx_array,0x00,sizeof(modem_rx_array));
    modem_rx_info.waiting_for_process = FALSE;
    tx_mutex_put(&g_mutex_uartrx_modem);
}

void modem_send(char *data)
{
    modem_tx_info.in_progress=TRUE;
    R_SCI_B_UART_Write(&g_uart_modem_ctrl, (uint8_t*)data, strlen(data));
    while(modem_tx_info.in_progress == TRUE)
        delay_ms(1);
}

return_t modem_verify_received_json_type(char *type)
{
    return_t ret = X_RET_OK;
    const cJSON *json_type = NULL;
    cJSON *ptr_json = cJSON_Parse(modem_rx_array);
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

return_t modem_process_outgoing(char *type,char *data,uint8_t retry,uint32_t timeout_ms)
{
    return_t ret = X_RET_OK;
    modem_tx_in_progress = TRUE;
    c_timespan_t ts;
    h_time_update(&ts);

    uint8_t r=0;
    bool_t finished = FALSE;
    do
    {
        modem_rx_clear();
        modem_send(data);
        h_time_update(&ts);

        bool_t rx_finished = FALSE;
        bool_t rx_error = FALSE;
        do
        {
            bool_t elasped = FALSE;
            h_time_is_elapsed_ms(&ts, timeout_ms, &elasped);
            if(elasped)
            {
                h_time_update(&ts);
                r++;
                if(r>=retry)
                {
                    ret = F_RET_COMMS_OUT_TIMEOUT;
                    goto end;
                }
                else
                {
                    rx_finished = TRUE;
                    rx_error = TRUE;
                }
            }
            else if(modem_rx_info.waiting_for_process == TRUE)
            {
                if(modem_verify_received_json_type(type) != X_RET_OK)
                {
                    r++;
                    if(r>=retry)
                    {
                        ret = F_RET_COMMS_OUT_BAD_RESPONSE;
                        goto end;
                    }
                    else
                    {

                        rx_finished = TRUE;
                        rx_error = TRUE;
                    }
                }
                else
                {
                    rx_finished = TRUE;
                    rx_error = FALSE;
                }
            }
        }while(!rx_finished);

        if(rx_error == TRUE)
        {
            delay_ms(100);
        }
        else
        {
            finished = TRUE;
            ret = X_RET_OK;
        }
    }while(!finished);



    end:
    modem_tx_in_progress = FALSE;
    return ret;
}


return_t modem_get_datetime(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,"{\"type\": \"get_datetime\",\"data\":null }\r\n");

    ret =   modem_process_outgoing("get_datetime",tx_array,2,1000);
    if(ret != X_RET_OK)
    {
        LOG_E(LOG_STD,"get datetime %d",ret);
        goto end;
    }


    // Traitement de la réponse
    ret = json_process_get_datetime(modem_rx_array);
    if(ret == X_RET_OK)
    {
    }
    else
    {
        LOG_E(LOG_STD,"get datetime %d",ret);
    }
    end:
    modem_rx_clear();
    modem_tx_in_progress = FALSE;
    return ret;
}

return_t modem_get_serials(void)
{
    return_t ret = X_RET_OK;

    char tx_array[128];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,"{\"type\": \"get_identification\",\"data\":null }\r\n");

    ret =   modem_process_outgoing("get_identification",tx_array,2,1000);
    if(ret != X_RET_OK)
    {
        LOG_E(LOG_STD,"Get modem data %d",ret);
        goto end;
    }

    // Traitement de la réponse
    ret = json_process_get_serials(modem_rx_array);

    if(ret == X_RET_OK)
    {
        LOG_D(LOG_STD,"Get modem serials");
    }
    else
    {
        LOG_E(LOG_STD,"Get modem serials %d",ret);
    }

    end:
    modem_rx_clear();
    modem_tx_in_progress = FALSE;
    return ret;
}


return_t modem_send_mqtt_publish(char *type,char *data)
{
    return_t ret = X_RET_OK;

    char tx_array[512];
    memset(tx_array,0x00,sizeof(tx_array));
    strcpy(tx_array,data);
    ret =   modem_process_outgoing("type",tx_array,1,5000);
    if(ret != X_RET_OK)
    {
        LOG_E(LOG_STD,"%s %d",type,ret);
        goto end;
    }


    int status_code=0;
    ret = json_process_mqtt_publish(modem_rx_array,&status_code);
    if(ret == X_RET_OK)
    {

    }
    else
    {
        if(ret == F_RET_JSON_RESPONSE_ERROR)
        {
            LOG_E(LOG_STD,"F_RET_JSON_RESPONSE_ERROR %d",status_code);
        }
    }
    end:
    modem_rx_clear();
    modem_tx_in_progress = FALSE;
    return ret;
}

return_t process_unsolicited_rx(void)
{
    return_t ret = X_RET_OK;
    const cJSON *json_type = NULL;
    cJSON *ptr_json = cJSON_Parse(modem_rx_array);
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

    LOG_I(LOG_STD,"unsolicited type %s",json_type->valuestring);


    end:
    cJSON_Delete(ptr_json);
    return ret;
}


void modem_callback(uart_callback_args_t *p_args)
{
    static char end_pattern[2];

    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        modem_rx_info.in_progress = TRUE;
        if(modem_rx_index < RX_MAX_CHAR)
        {
            modem_rx_array[modem_rx_index++] = (char)p_args->data;
            modem_rx_info.code = X_RET_OK;

            end_pattern[0] = end_pattern[1];
            end_pattern[1] = (char)p_args->data;

            if(end_pattern[0] == '\r' && end_pattern[1] == '\n')
            {
                modem_rx_info.waiting_for_process = TRUE;

                if(modem_tx_in_progress == FALSE)
                {
                    process_unsolicited_rx();
                    modem_rx_clear();

                }
            }
        }
        else
        {
            modem_rx_info.code = X_RET_CONTAINER_FULL;
        }
    }
    else if(UART_EVENT_TX_DATA_EMPTY == p_args->event)
    {
        modem_tx_info.in_progress = FALSE;
        modem_tx_info.code = X_RET_OK;
    }
    else if((UART_EVENT_ERR_PARITY == p_args->event || UART_EVENT_ERR_FRAMING == p_args->event ||
            UART_EVENT_ERR_OVERFLOW == p_args->event || UART_EVENT_BREAK_DETECT == p_args->event)
            )
    {
        modem_tx_info.in_progress = FALSE;
        modem_tx_info.code = X_RET_ERR_GENERIC;
    }
    else
    {

    }
}