#include "main_thread.h"
#include "log_thread.h"


#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_protected/c_protected.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_time/i_time.h>
#include <_hal/h_time/h_time.h>
#include <_hal/h_log/h_log.h>
#include <_hal/h_motors/h_drv8323s/h_drv8323s.h>
#include <_lib_impl_cust/impl_time/impl_time.h>
#include <_lib_impl_cust/impl_log/impl_log.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>
#include <rtc/rtc.h>
#include <files/lfs/lfs.h>
#include <files/lfs_impl.h>
#include <files/json_file.h>
#include <remotectrl/remotectrl.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main thread"

extern TX_THREAD log_thread;
i_time_t i_time_interface_t;


h_drv8323s_t drv_mot1;
i_spi_t interface_mot1;
h_drv8323s_t drv_mot2;
i_spi_t interface_mot2;

/* Main Thread entry function */
void main_thread_entry(void)
{
    volatile return_t ret = X_RET_OK;
    i_log.write_e = impl_log_write_e;
    i_log.write_d = impl_log_write_d;
    i_log.write_i = impl_log_write_i;
    i_log.write_w = impl_log_write_w;


    // Configuration de l'interface de gestion du temps
    i_time_init(&i_time_interface_t,impl_time_init, impl_time_update);
    h_time_init(&i_time_interface_t);




    rtc_init();
    /*LFS_Init();
    LFS_ParseFolders("/");
    LFS_ParseFolders((char*)dir_payloads);*/

    /*char *ptr_test = malloc(1024);
    memset(ptr_test,0x00,1024);
    sprintf(ptr_test,"Ceci est un test");
    json_file_add_to_queue(FILE_TYPE_PAYLOAD,ptr_test);*/


    i_spi_init(&interface_mot1,&g_mutex_spi, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
    i_spi_init(&interface_mot2,&g_mutex_spi, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT1_ENABLE, BSP_IO_LEVEL_HIGH);
    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_MOT2_ENABLE, BSP_IO_LEVEL_HIGH);
    tx_thread_sleep(10);
    volatile return_t ret_mot1 =  h_drv8323s_init(&drv_mot1,&interface_mot1,TRUE);
    volatile return_t ret_mot2 =  h_drv8323s_init(&drv_mot2,&interface_mot2,TRUE);

    drv_mot1.registers.csa_control.bits.GAIN = 0x00;
    ret_mot1 =  h_drv8323s_write_all_registers(&drv_mot1);

    // Demarrage du Thread dédié aux LOGs
    tx_thread_resume(&log_thread);


    LOG_I(LOG_STD,"Main thread start");




    /* TODO: add your own code here */
    while (1)
    {
        /*delay_ms(1000);
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED1,1 );
        delay_ms(1000);;
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED1,0 );*/
        remotectrl_process();
        tx_thread_sleep (10);
    }
}
