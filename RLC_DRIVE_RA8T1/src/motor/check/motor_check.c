/*
 * motor_check.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */
#include <_hal/h_motors/h_drv8323s/h_drv8323s.h>
#include "motor_check.h"
#include <motor/motor.h>
#include <motor/drive_process/drive_sequence.h>
#include <motor/config_spi/config_spi.h>
#include <adc/adc.h>
#include <return_codes.h>
#include <status/motor_status.h>


return_t motor_check_process_error_sources(st_system_motor_status_t *sys_mot)
{
    return_t ret = X_RET_OK;
    return_t ret_sub = X_RET_OK;
    if(motor_error_sources_is_error() == FALSE) return ret;

    motor_error_sources_t src = motor_error_sources_get_snapshot();

    if(src.flags.bits.overcurrent == 1)
    {
        sys_mot->error_lvl1.bits.overcurrent_vm = 1;
        ret = -1;
    }

    if(src.flags.bits.motorH_fault == 1)
    {
        ret = -1;
        sys_mot->error_lvl1.bits.fault_driver_h = 1;
        ret_sub = h_drv8323s_read_status_registers(&drv_mot1);
        if(ret_sub == X_RET_OK)
        {
            sys_mot->error_lvl1_motorH.status1.value = drv_mot1.registers.fault_status1.value;
            sys_mot->error_lvl1_motorH.status2.value = drv_mot1.registers.vgs_status2.value;
        }
        //h_drv8323s_clear_fault(&drv_mot1);
    }

    if(src.flags.bits.motorL_fault == 1)
    {
        ret = -1;
        sys_mot->error_lvl1.bits.fault_driver_l = 1;
        ret_sub = h_drv8323s_read_status_registers(&drv_mot2);
        if(ret_sub == X_RET_OK)
        {
            sys_mot->error_lvl1_motorL.status1.value = drv_mot2.registers.fault_status1.value;
            sys_mot->error_lvl1_motorL.status2.value = drv_mot2.registers.vgs_status2.value;
        }
        //h_drv8323s_clear_fault(&drv_mot2);
    }
    return ret;
}

return_t motor_check(st_system_motor_status_t *sys_mot)
{
    return_t ret = X_RET_OK;

    // Désactivation des drivers moteur
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_HIGH );
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_HIGH );
    delay_ms(100);

    motor_check_fault_pins();
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_SWITCH_CMD,BSP_IO_LEVEL_HIGH );
    // Desactivation des drivers moteurs
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_12V_EN,BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL1,BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL2,BSP_IO_LEVEL_LOW);
    // RAZ de la structure de gestion des erreurs matérielles (OVERCURRENT et FAULT moteurs)
    motor_error_sources_init();
    motor_check_fault_pins();
    // Activation des enable des moteurs. Normalement inutile car dans la mesure où on utilise la pin FAULT,
    // il n'est plus nécessaire de couper les enable en urgence lors d'un défaut.
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_HIGH );
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_HIGH );
    delay_ms(50);
    motor_check_fault_pins();

    // Configuration SPI des drivers
    ret = motor_config_spi_init();
    if(ret != X_RET_OK)
    {
        sys_mot->error_lvl1.bits.config_driver_h = TRUE;
        sys_mot->error_lvl1.bits.config_driver_l = TRUE;
    }

    // Configuration du drivers haut
    ret = motor_config_spi(&drv_mot1);
    if(ret != X_RET_OK)
        sys_mot->error_lvl1.bits.config_driver_h = TRUE;

    // Configuration du drivers bas
    ret = motor_config_spi(&drv_mot2);
    if(ret != X_RET_OK)
        sys_mot->error_lvl1.bits.config_driver_l = TRUE;

    // Si à ce stade nous avons déjà relevé une défaillance alors
    // on coupe les drivers et on retourne une erreur
    if(sys_mot->error_lvl1.value != 0x00)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_LOW );
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_LOW );
        return F_RET_MOTOR_CHECK_ERROR;
    }



    motor_check_fault_pins();

    // Vérification des pins indiquant une défaillance sur la partie moteurs.
    // Si une erreur est présente alors on coupe les drivers et on retourne une erreur
    if(motor_check_process_error_sources(sys_mot) != X_RET_OK)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_LOW );
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_LOW );
        return F_RET_MOTOR_CHECK_ERROR;
    }


    // Ouverture du FSP
    motor_init_fsp();
    motors_instance.motorH->motor_ctrl_instance->p_api->configSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorH);
    motors_instance.motorL->motor_ctrl_instance->p_api->configSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorL);


    // Activation et vérification du régulateur +12V
    delay_ms(50);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_12V_EN,BSP_IO_LEVEL_HIGH);
    delay_ms(10);
    bsp_io_level_t pgood_level;
    R_IOPORT_PinRead(&g_ioport_ctrl,IO_12V_PGOOD,&pgood_level);
    if(pgood_level == BSP_IO_LEVEL_LOW)
        sys_mot->error_lvl1.bits.vcc_12v  = TRUE;
    else
        sys_mot->error_lvl1.bits.vcc_12v  = FALSE;

    // Activation et vérification du +12V pour le codeur du moteur H
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL1,BSP_IO_LEVEL_HIGH);
    delay_ms(100);

    st_adc_t adc_snapshot;
    adc_get_snapshot(&adc_snapshot);
    if(adc_snapshot.vhall1 < 11000 || adc_snapshot.vhall1 > 13000)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL1,BSP_IO_LEVEL_LOW);
        delay_ms(10);
        sys_mot->error_lvl1.bits.vcc_hall_h = TRUE;
    }
    else
        sys_mot->error_lvl1.bits.vcc_hall_h = FALSE;

    // Activation et vérification du +12V pour le codeur du moteur B
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL2,BSP_IO_LEVEL_HIGH);
    delay_ms(100);
    adc_get_snapshot(&adc_snapshot);
    if(adc_snapshot.vhall2 < 11000 || adc_snapshot.vhall2 > 13000)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_EN_12V_HALL2,BSP_IO_LEVEL_LOW);
        delay_ms(10);
        sys_mot->error_lvl1.bits.vcc_hall_l = TRUE;
    }
    else
        sys_mot->error_lvl1.bits.vcc_hall_l = FALSE;



    // Si à ce stade nous avons déjà relevé une défaillance alors
    // on coupe les drivers et on retourne une erreur
    if(sys_mot->error_lvl1.value != 0x0)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT1_ENABLE,BSP_IO_LEVEL_LOW );
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_MOT2_ENABLE,BSP_IO_LEVEL_LOW );
        return F_RET_MOTOR_CHECK_ERROR;
    }


    // Procédure de calibrage
    drv_mot1.registers.csa_control.bits.CSA_CAL_A=1;
    drv_mot1.registers.csa_control.bits.CSA_CAL_B=1;
    drv_mot1.registers.csa_control.bits.CSA_CAL_C=1;
    ret = h_drv8323s_write_all_registers(&drv_mot1);
    ret = h_drv8323s_read_all_registers(&drv_mot1);
    tx_thread_sleep(10);
    drv_mot1.registers.csa_control.bits.CSA_CAL_A=0;
    drv_mot1.registers.csa_control.bits.CSA_CAL_B=0;
    drv_mot1.registers.csa_control.bits.CSA_CAL_C=0;
    ret = h_drv8323s_write_all_registers(&drv_mot1);
    ret = h_drv8323s_read_all_registers(&drv_mot1);
    adc_set_calibration_mode(TRUE);
    tx_thread_sleep(10);
    adc_set_calibration_mode(FALSE);

    return ret;
}

