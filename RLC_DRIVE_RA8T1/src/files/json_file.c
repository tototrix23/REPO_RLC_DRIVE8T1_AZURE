/*
 * json_file.c
 *
 *  Created on: 7 juin 2024
 *      Author: Christophe
 */
#include <hal_data.h>
#include "common_data.h"
#include "json_file.h"
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "json file"


return_t json_file_add_to_queue(uint8_t type,char *ptr)
{
    ASSERT(ptr != 0x00);

    json_file_t *ptr_st = (json_file_t*)malloc(sizeof(json_file_t));
    if(ptr != 0x00)
    {
        memset(ptr_st, 0, sizeof(json_file_t));
        ptr_st->type = type;
        ptr_st->ptr_data = ptr;
        uint32_t status = tx_queue_send(&g_queue_json, &ptr_st, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            LOG_E(LOG_STD,"Error adding 'json_file_t' to queue");
            free(ptr);
            free(ptr_st);
            return X_RET_CONTAINER_FULL;
        }
    }
    else
    {
        LOG_E(LOG_STD,"Error creating 'json_file_t'");
        return X_RET_MEMORY_ALLOCATION;
    }
    return X_RET_OK;
}
/*
 log_t *ptr = (log_t*)malloc(sizeof(log_t));
    if(ptr != 0x00)
    {
        memset(ptr, 0, sizeof(log_t));
        vsnprintf(ptr->text,sizeof(ptr->text)-1, s, argp);
        strcpy(ptr->func,func);
        strcpy(ptr->module,module);
        ptr->line = line;
        ptr->mode = mode;
        ptr->color = LOG_COLOR_ERROR;

        uint32_t status = tx_queue_send(&g_queue_log, &ptr, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            free(ptr);
        }
    }
 */

