#include "main_thread.h"
#include "log_thread.h"
#include "modem_thread.h"

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
#include <remotectrl/remotectrl.h>
#include <motor/motor.h>
#include <motor/config_spi/config_spi.h>
#include <sht40_sensor/sht40.h>
#include <adc/adc.h>
#include <files/mqtt_file.h>
#include <exchanged_data/exchanged_data.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main thread"

extern TX_THREAD log_thread;
extern TX_THREAD motors_thread;
extern TX_THREAD modem_thread;
i_time_t i_time_interface_t;


/*h_drv8323s_t drv_mot1;
i_spi_t interface_mot1;
h_drv8323s_t drv_mot2;
i_spi_t interface_mot2;*/

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

    LOG_I(LOG_STD,"Main thread start");












    rtc_init();
    rtc_set(0);

    LFS_Init(FALSE);
    LFS_ParseFolders("/");
    LFS_ParseFolders((char*)dir_payloads);
    LFS_ParseFolders((char*)dir_json);


   /* lfs_dir_t dir;
    int err = lfs_dir_open(&lfs,&dir,dir_json);

    lfs_file_t file;
    err = lfs_file_open(&lfs, &file, "/JSON/1727873142052.json", LFS_O_RDWR | LFS_O_CREAT );
    if(err != 0)
    {
        LOG_E(LOG_STD,"ERROR");
    }
    lfs_file_close(&lfs, &file);
    lfs_dir_close(&lfs,&dir);

    LFS_ParseFolders((char*)dir_json);*/
    /*char *ptr_data = malloc(200);
    strcpy(ptr_data,"test rtc attr");
    json_file_add_to_queue(FILE_TYPE_PAYLOAD,ptr_data);*/



    // Demarrage du Thread dédié aux LOGs
    tx_thread_resume(&log_thread);
    delay_ms(1000);
    tx_thread_resume(&modem_thread);

    // Initialisation de la partie moteurs (partie logicielle)
    motor_structures_init();
    motor_init_type(MOTOR_TYPE_RM_ITOH_BRAKE);

    // Initialisation de la partie ADC


    ret = adc_init();
    if(ret != X_RET_OK){
        LOG_E(LOG_STD,"INIT ADC ERROR");}
    else{
        LOG_I(LOG_STD,"INIT ADC SUCCESS");}

    // Initialisation de la table d'échange
    exchanged_data_init();

    // Initialisation de la VEE (EEPROM virtuelle)
    vee_init();




    //exchdat_set_temperature_humidity(24.12f,45.78f);
    //mqtt_publish_temperature_humidity();



    /*volatile float temperature,rh;

    do
    {
        ret = sht40_read(&temperature,&rh);
        if(ret != X_RET_OK)
        {
            LOG_E(LOG_STD,"error");
        }
        else
        {
            LOG_I(LOG_STD,"temp=%0.2f  rh=%0.2f",temperature,rh);
        }
        tx_thread_sleep(1000);
    }while(1);*/

    //tx_thread_resume(&motors_thread);


    set_drive_mode(MOTOR_INIT_MODE);



     c_timespan_t ts;
     c_timespan_init(&ts);
     h_time_update(&ts);





    /* TODO: add your own code here */
    while (1)
    {
        remotectrl_process();

        bool_t elasped = FALSE;
        h_time_is_elapsed_ms(&ts, 2000, &elasped);
        if(elasped == TRUE)
        {
            h_time_update(&ts);
            //bsp_io_level_t lvl;
            //R_IOPORT_PinRead(&g_ioport_ctrl, IO_VM_SWITCH_CMD,&lvl );
            //LOG_D(LOG_STD,"VM CMD %d",lvl);
            //st_adc_t adc_snapshot;
            //adc_get_snapshot(&adc_snapshot);

            //LOG_D(LOG_STD,"%lu mV / %lu mA / %lu mA /%lu mA",adc_snapshot.vin,adc_snapshot.iin,adc_snapshot.mot1_iu,adc_snapshot.mot1_iw);
        }

        /*if(motor_emergency_is_error())
        {
             emergency_src_t emergency_data = motor_emergency_get_data();
             if(emergency_data.bits.motor1_fault)
             {
                 h_drv8323s_read_status_registers(&drv_mot1);
                 LOG_E(LOG_STD,"MOT1 %04X %04X",drv_mot1.registers.fault_status1.value,drv_mot1.registers.vgs_status2.value);
                 tx_thread_sleep (10);
                 h_drv8323s_clear_fault(&drv_mot1);
                 motor_emergency_init();
             }

             if(emergency_data.bits.motor2_fault)
              {
                  h_drv8323s_read_status_registers(&drv_mot2);
                  LOG_E(LOG_STD,"MOT2 %04X %04X",drv_mot2.registers.fault_status1.value,drv_mot2.registers.vgs_status2.value);
                  tx_thread_sleep (10);
                  h_drv8323s_clear_fault(&drv_mot2);
                  motor_emergency_init();
              }

        }*/

        tx_thread_sleep (1);
    }
}
