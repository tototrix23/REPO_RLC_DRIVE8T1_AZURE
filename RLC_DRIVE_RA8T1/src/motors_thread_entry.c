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
#include <system_status/system_status.h>
#include <motor/config_spi/config_spi.h>
#include <motor/check/motor_check.h>
#include <motor/emergency/emergency.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "motor thread"






void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
        // Desactivation des drivers moteurs
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_SWITCH_CMD,BSP_IO_LEVEL_LOW );
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_LOW );
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_LOW );

        // Flag indiquant le défaut
        flag_overcurrent_vm = TRUE;

        motor_emergency_set_overcurrent();
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


    // Analyse du status dans la VEE
    // Si le process moteur est déjà en défault alors on passe directement en mode MOTOR_ERROR_MODE
    if(system_status_check_error() == TRUE)
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
        ret = motor_check(TRUE);
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


    /*
    // Activation de l'alimentation du moteur
    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
    // Temporisation pour la stabilisation
    delay_ms(1000);

    ret = h_drv8316_read_all_registers(&drv_mot1);
    drv_mot1.registers.ctrl2.bits.SLEW = 1;//3;
    drv_mot1.registers.ctrl2.bits.PWM_MODE = 0;
    drv_mot1.registers.ctrl4.bits.OCP_MODE = 0;
    drv_mot1.registers.ctrl4.bits.OCP_DEG = 0;
    drv_mot1.registers.ctrl4.bits.OCP_LVL = 0;
    drv_mot1.registers.ctrl5.bits.CSA_GAIN = 0;
    drv_mot1.registers.ctrl5.bits.EN_AAR = 0;
    drv_mot1.registers.ctrl5.bits.EN_ASR = 0;
    drv_mot1.registers.ctrl10.bits.DLY_TARGET = 0xB;//0x5;
    drv_mot1.registers.ctrl10.bits.DLYCMP_EN = 1;
    ret = h_drv8316_write_all_registers(&drv_mot1);
    ret = h_drv8316_read_all_registers(&drv_mot1);

    ret = h_drv8316_read_all_registers(&drv_mot2);
    drv_mot2.registers.ctrl2.bits.SLEW = 3;
    drv_mot2.registers.ctrl2.bits.PWM_MODE = 0;
    drv_mot2.registers.ctrl4.bits.OCP_MODE = 0;
    drv_mot2.registers.ctrl4.bits.OCP_DEG = 0;
    drv_mot2.registers.ctrl4.bits.OCP_LVL = 0;
    drv_mot2.registers.ctrl5.bits.CSA_GAIN = 0;
    drv_mot2.registers.ctrl5.bits.EN_AAR = 0;
    drv_mot2.registers.ctrl5.bits.EN_ASR = 0;
    drv_mot2.registers.ctrl10.bits.DLY_TARGET = 0x5;
    drv_mot2.registers.ctrl10.bits.DLYCMP_EN = 1;
    ret = h_drv8316_write_all_registers(&drv_mot2);
    ret = h_drv8316_read_all_registers(&drv_mot2);

    delay_ms(50);*/



    // Demarrage de la boucle de traitement
    while (1)
    {
        drive_process();
        tx_thread_sleep (1);
        //delay_ms(500);
        //LOG_D(LOG_STD,"while motor");

    }
}
