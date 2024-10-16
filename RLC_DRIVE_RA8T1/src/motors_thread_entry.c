#include "motors_thread.h"
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_hal/h_drv8316/h_drv8316.h>
#include <_hal/h_time/h_time.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>
#include <motor/motor.h>
#include <motor/drive_process/drive_process.h>
#include <motor/drive_process/drive_sequence.h>
#include <motor/config_spi/config_spi.h>
#include <motor/check/motor_check.h>
#include <motor/errors/motor_error_sources.h>
#include <status/motor_status.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "motor thread"






void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);

        // Flag indiquant le défaut
        motor_error_sources_set_overcurrent();
    }
}


/* Motors Thread entry function */
void motors_thread_entry(void)
{
    volatile return_t ret = X_RET_OK;

    /*uint32_t cnt=0;
    while(1)
    {
        LOG_E(LOG_STD,"Test %d",cnt);
        cnt++;
        tx_thread_sleep(100);
    }*/
    LOG_I(LOG_STD,"Motor thread start");
    // Initialisation POEG
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);




    // Initialisation des drivers moteurs
    ret = motor_config_spi_init();
    ret = motor_config_spi(&drv_mot1);
    ret = motor_config_spi(&drv_mot2);





    // Analyse du status dans la VEE
    // Si le process moteur est déjà en défault alors on passe directement en mode MOTOR_ERROR_MODE
    if(motor_status_check_error() == TRUE)
    {
        // On ouvre le FSP à cette occasion car sinon la stack moteur n'est pas configurée.
        // Elle est configurée normalement dans la fonction "motor_check"
        motor_init_fsp();
        motors_instance.motorH->motor_ctrl_instance->p_api->configSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorH);
        motors_instance.motorL->motor_ctrl_instance->p_api->configSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorL);

        LOG_E(LOG_STD,"Starting on status error");
        set_drive_mode(MOTOR_ERROR_MODE);
    }
    else
    {
        LOG_I(LOG_STD,"Checking motors level1...");
        // Analyse des différents organes relatifs au pilotage des moteurs
        c_timespan_t ts1;
        h_time_update(&ts1);
        st_system_motor_status_t sys_mot;
        memset(&sys_mot,0x00,sizeof(st_system_motor_status_t));
        ret = motor_check(&sys_mot);
        if(ret != X_RET_OK)
        {
            LOG_E(LOG_STD,"Check motors level1 NOK");
        }
        else
        {
            LOG_I(LOG_STD,"Check motors level1 OK");
        }
        c_timespan_t ts2;
        h_time_get_elapsed(&ts1, &ts2);
    }



    // Demarrage de la boucle de traitement
    while (1)
    {
        drive_process();
        tx_thread_sleep (1);
    }
}
