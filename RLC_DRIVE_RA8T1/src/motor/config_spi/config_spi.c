/*
 * config_spi.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "config_spi.h"

h_drv8323s_t drv_mot1;
h_drv8323s_t drv_mot2;
i_spi_t interface_mot1;
i_spi_t interface_mot2;


return_t motor_config_spi_init(void)
{
  return_t ret = X_RET_OK;

  // Initialisation des interfaces SPI
  i_spi_init(&interface_mot1,&g_mutex_spi, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
  i_spi_init(&interface_mot2,&g_mutex_spi, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);
  // Initialisation des fonctions relatives aux drivers
  ret = h_drv8323s_init(&drv_mot1,&interface_mot1,TRUE);
  if(ret != X_RET_OK) return ret;
  ret = h_drv8323s_init(&drv_mot2,&interface_mot2,TRUE);
  if(ret != X_RET_OK) return ret;

  return ret;
}

return_t motor_config_spi(h_drv8323s_t *ptr)
{
    return_t ret = X_RET_OK;

    ret = h_drv8323s_read_all_registers(ptr);
    if(ret != X_RET_OK) return ret;

    ptr->registers.csa_control.bits.GAIN = 0x01;
    ptr->registers.gate_drive_hs.bits.IDRIVEN_HS = 0x2;
    ptr->registers.gate_drive_hs.bits.IDRIVEP_HS = 0x2;
    ptr->registers.gate_drive_ls.bits.IDRIVEN_LS = 0x2;
    ptr->registers.gate_drive_ls.bits.IDRIVEP_LS = 0x2;
    ptr->registers.gate_drive_ls.bits.TDRIVE = 0x03;
    ptr->registers.ocp_control.bits.OCP_MODE = 0;
    ptr->registers.csa_control.bits.VREF_DIV = 1;
    ret =  h_drv8323s_write_all_registers(ptr);
    return ret;
}
