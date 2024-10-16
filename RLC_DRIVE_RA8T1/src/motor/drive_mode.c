/*
 * motor_mode.c
 *
 *  Created on: 17 oct. 2023
 *      Author: Christophe
 */
#include <remotectrl/remotectrl.h>
#include "motor.h"
#include "drive_mode.h"
#include <motor/modes/init_mode.h>
#include <motor/modes/manual_mode.h>
#include <motor/modes/auto_mode.h>
#include <motor/drive_process/drive_sequence.h>
#include <exchanged_data/exchanged_data.h>
#include <motor/errors/motor_error_sources.h>
#include <status/motor_status.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "DRIVE"

drive_control_t drive_control;


void motor_check_fault_pins(void)
{
    bsp_io_level_t mot1_fault;
    bsp_io_level_t mot2_fault;
    R_IOPORT_PinRead(&g_ioport_ctrl, IO_MOT1_FAULT,&mot1_fault );
    R_IOPORT_PinRead(&g_ioport_ctrl, IO_MOT2_FAULT,&mot2_fault );
    if(mot1_fault == 0)
    {
        LOG_E(LOG_STD,"mot1_fault");
        motor_error_sources_set_driversH();
    }

    if(mot2_fault == 0)
    {
        LOG_E(LOG_STD,"mot1_fault");
        motor_error_sources_set_driversL();
    }
}

bool_t drive_stop_request(void)
{

    motor_check_fault_pins();

   if(drive_control.stop_order == TRUE)
   {
       motor_profil_t *ptr = &motors_instance.profil;
       sequence_result_t sequence_result;
       motor_drive_sequence(&ptr->sequences.off,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
       drive_control.running = FALSE;
       return TRUE;
   }
   else if(motor_error_sources_is_error() == TRUE)
   {
       motor_error_sources_t src = motor_error_sources_get_snapshot();
       LOG_E(LOG_STD,"error sources %d",src.flags);
       motor_profil_t *ptr = &motors_instance.profil;
       sequence_result_t sequence_result;
       motor_drive_sequence(&ptr->sequences.off,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
       drive_control.running = FALSE;
       set_drive_mode(MOTOR_ERROR_MODE);
       return TRUE;
   }
   else
   {
       return FALSE;
   }
}

return_t set_drive_mode(drive_mode_t mode)
{
    drive_mode_t current_mode = exchdat_get_drive_mode();//motors_instance.mode;
    if (mode != current_mode)
    {
        drive_control.changing = TRUE;
        bool_t change_order = FALSE;

        // Gestion de l'entrée dans le mode manuel
        // Permet d'executer certaines actions
        if(mode == MOTOR_MANUAL_MODE)
        {
            motor_status_clear();
        }


        if(motor_status_check_error() == TRUE)
            mode = MOTOR_ERROR_MODE;


        switch(mode)
        {
            case MOTOR_ERROR_MODE:
                switch(current_mode)
                {
                    case MOTOR_MANUAL_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_INIT_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_AUTO_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_ERROR_MODE:
                        break;
                }
                break;


            case MOTOR_MANUAL_MODE:
                switch(current_mode)
                {
                    case MOTOR_MANUAL_MODE:
                        break;

                    case MOTOR_INIT_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_AUTO_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_ERROR_MODE:
                        change_order = TRUE;
                        break;
                }
                break;

            case MOTOR_INIT_MODE:
                switch(current_mode)
                {
                    case MOTOR_MANUAL_MODE:
                        change_order = TRUE;
                        break;

                    case MOTOR_INIT_MODE:
                        break;

                    case MOTOR_AUTO_MODE:
                        break;

                    case MOTOR_ERROR_MODE:
                        break;
                }
                break;

            case MOTOR_AUTO_MODE:
                switch(current_mode)
                {
                    case MOTOR_MANUAL_MODE:
                        break;

                    case MOTOR_INIT_MODE:
                        break;

                    case MOTOR_AUTO_MODE:
                        break;

                    case MOTOR_ERROR_MODE:
                        break;
                }
                break;
        }


        if(change_order == TRUE)
        {
            drive_control.stop_order = TRUE;
            while(drive_control.running == TRUE)
                tx_thread_sleep(10);
            drive_control.stop_order = FALSE;
        }


        exchdat_set_drive_mode(mode);
        drive_control.changing = FALSE;

        switch (mode)
        {


            case MOTOR_MANUAL_MODE:
                LOG_D(LOG_STD,"Set manual mode");

            break;

            case MOTOR_INIT_MODE:
                LOG_D(LOG_STD,"Set init mode");
            break;

            case MOTOR_AUTO_MODE:
                LOG_D(LOG_STD,"Set auto mode");
            break;

            case MOTOR_ERROR_MODE:
                LOG_E(LOG_STD,"Set error mode");
            break;
        }

    }
    return X_RET_OK;
}
