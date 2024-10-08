/*
 * telecommande.c
 *
 *  Created on: 13 oct. 2023
 *      Author: Ch.Leclercq
 */
#include "remotectrl.h"
#include <motor/drive_mode.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "remotectrl"

bsp_io_level_t m12_auto_previous=0;
bsp_io_level_t m12_enrl_previous=0;
bsp_io_level_t m12_enrh_previous=0;
bsp_io_level_t m12_derh_previous=0;

bsp_io_level_t m12_auto=0;
bsp_io_level_t m12_enrl=0;
bsp_io_level_t m12_enrh=0;
bsp_io_level_t m12_derh=0;


return_t remotectrl_enter_manual(void);
return_t remotectrl_exit_manual(void);

static bool_t manual_mode = FALSE;

return_t remotectrl_process(void)
{
	R_IOPORT_PinRead(&g_ioport_ctrl, IO_M12_AUTO,&m12_auto );
	R_IOPORT_PinRead(&g_ioport_ctrl, IO_M12_ENRL,&m12_enrl );
	R_IOPORT_PinRead(&g_ioport_ctrl, IO_M12_ENRH,&m12_enrh );
	R_IOPORT_PinRead(&g_ioport_ctrl, IO_M12_DERH,&m12_derh );


	if(m12_auto != m12_auto_previous)
	{
	    //LOG_D(LOG_STD,"M12_AUTO %d",m12_auto);
	    m12_auto_previous = m12_auto;
	}
	if(m12_enrl != m12_enrl_previous)
    {
        //LOG_D(LOG_STD,"M12_ENRL %d",m12_enrl);
        m12_enrl_previous = m12_enrl;
    }
	if(m12_enrh != m12_enrh_previous)
    {
        //LOG_D(LOG_STD,"M12_ENRH %d",m12_enrh);
        m12_enrh_previous = m12_enrh;
    }
	if(m12_derh != m12_derh_previous)
    {
        //LOG_D(LOG_STD,"M12_DERH %d",m12_derh);
        m12_derh_previous = m12_derh;
    }


	if(m12_auto == REMOTECTRL_ACTIVE_LEVEL && manual_mode == FALSE)
	{
	    manual_mode = TRUE;
	    remotectrl_enter_manual();
	}
	else if(m12_auto != REMOTECTRL_ACTIVE_LEVEL && manual_mode == TRUE)
	{
	    manual_mode = FALSE;
	    remotectrl_exit_manual();
	}
	return X_RET_OK;
}


return_t remotectrl_enter_manual(void)
{
    LOG_D(LOG_STD,"Enter manual mode");
    set_drive_mode(MOTOR_MANUAL_MODE);
    return X_RET_OK;
}

return_t remotectrl_exit_manual(void)
{
    //motors_instance.error = MOTORS_ERROR_NONE;
    LOG_D(LOG_STD,"Exit manual mode");
    set_drive_mode(MOTOR_INIT_MODE);
    return X_RET_OK;
}
