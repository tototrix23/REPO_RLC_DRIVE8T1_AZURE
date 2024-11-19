/*
 * synchro.c
 *
 *  Created on: 18 nov. 2024
 *      Author: Christophe
 */

#include <hal_data.h>
#include "synchro.h"

static s_synchro sync_inst;

static uint32_t sync_flag = 0;

void sync_irq_callback (external_irq_callback_args_t * p_args)
{
   sync_flag = 1;

   volatile uint8_t xxx=0;
   xxx = 1;
}

return_t synchro_init(void)
{
    return_t ret = X_RET_OK;
    memset(&sync_inst,0x00,sizeof(s_synchro));

    fsp_err_t err = R_ICU_ExternalIrqOpen(&g_sync_irq_ctrl, &g_sync_irq_cfg);
    if(err != FSP_SUCCESS)
    {
        return -1;
    }

    err = R_ICU_ExternalIrqEnable(&g_sync_irq_ctrl);
    if(err != FSP_SUCCESS)
    {
        return -1;
    }

    return X_RET_OK;
}





