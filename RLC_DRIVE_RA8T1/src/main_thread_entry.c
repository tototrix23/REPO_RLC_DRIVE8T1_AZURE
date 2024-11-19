#include "main_thread.h"
#include "log_thread.h"
#include "modem_thread.h"

#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_protected/c_protected.h>
#include <_target/t_critical_section.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_time/i_time.h>
#include <_hal/h_time/h_time.h>
#include <_hal/h_log/h_log.h>
#include <_hal/h_motors/h_drv8323s/h_drv8323s.h>
#include <_lib_impl_cust/impl_time/impl_time.h>
#include <_lib_impl_cust/impl_log/impl_log.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>
#include <rtc/rtc.h>
#include <remotectrl/remotectrl.h>
#include <motor/motor.h>
#include <motor/config_spi/config_spi.h>
#include <i2c/sht40.h>
#include <adc/adc.h>
#include <files/mqtt_file.h>
#include <files/mqtt_publish_vars.h>
#include <exchanged_data/exchanged_data.h>
#include <flash/flash_stack.h>
#include <flash/flash_routines.h>
#include <time.h>
#include <my_malloc.h>
#include <i2c/eeprom.h>
#include <relay/relay.h>
#include <synchro/synchro.h>

#define MEDIA_SECTOR_HEADS_VALUE (1U)
#define MEDIA_SECTORS_PER_HEAD    (1U)

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

//volatile uint8_t tab[4096];



static char * current_heap_start = 0;
static char * current_heap_limit = 0;

void heap_init(void)
{
    extern char _Heap_Begin __asm("__HeapBase");  ///< Defined by the linker.

    extern char _Heap_Limit __asm("__HeapLimit");

    if(current_heap_start == 0)
    {
        current_heap_start = &_Heap_Begin;
    }
    if(current_heap_limit == 0)
    {
        current_heap_limit = &_Heap_Limit;
    }


    char *ptr = current_heap_start;
    uint32_t i=0;
    for(i=0;i<BSP_CFG_HEAP_BYTES;i++)
    {
        *ptr = 0x55;
        ptr++;
    }
}

uint32_t heap_usage(uint32_t *bytes,uint32_t *percent)
{
    T_CRITICAL_SECTION_DEFINE;
    T_CRITICAL_SECTION_ENTER;
    uint32_t ret = 0;

    volatile char *ptr = (current_heap_limit-1);

    char value;
    do
    {

        value = *ptr;

        if(ptr == current_heap_start)
            break;
        else
            ptr--;

        if(value != 0x55)
            break;


    }while(1);

    volatile uint32_t s = ptr-current_heap_start;

    uint32_t p = ((s*100))/BSP_CFG_HEAP_BYTES;
    *percent = p;

    *bytes = s;

    T_CRITICAL_SECTION_EXIT;

    return ret;
}




/* Main Thread entry function */
void main_thread_entry(void)
{
    volatile return_t ret = X_RET_OK;
    i_log.write_e = impl_log_write_e;
    i_log.write_d = impl_log_write_d;
    i_log.write_i = impl_log_write_i;
    i_log.write_w = impl_log_write_w;



    heap_init();

    /*flash_open();
    flash_erase_chip();
    flash_close();*/

    volatile UINT fx_ret_val = FX_SUCCESS;
        /* Initialize LevelX system */

    // Configuration de l'interface de gestion du temps
    i_time_init(&i_time_interface_t,impl_time_init, impl_time_update);
    h_time_init(&i_time_interface_t);


    delay_ms(2000);
    LOG_I(LOG_STD,"Main thread start");

    relay_init();
    eeprom_init();
    rtc_init();
    fs_initialise_only_one_time();
    ret = fs_open();
    if(ret == X_RET_OK)
    {
        ULONG flash_bytes_available;
        fs_bytes_available(&flash_bytes_available);
        LOG_D(LOG_STD,"%ul bytes available in Flash");
    }
    fs_close();


    synchro_init();

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

    // Initialisation des variables MQTT
    mqtt_vars_init();


    exchdat_set_firmware((char*)drive_firmware);
    exchdat_set_board_version(0);
    exchdat_set_lighting_enabled(FALSE);
    exchdat_set_scrolling_enabled(TRUE);


    tx_thread_resume(&motors_thread);

    set_drive_mode(MOTOR_INIT_MODE);

    c_timespan_t ts_sensor;
    c_timespan_init(&ts_sensor);
    h_time_update(&ts_sensor);


    c_timespan_t ts_adc;
    c_timespan_init(&ts_adc);
    h_time_update(&ts_adc);

    c_timespan_t ts_heap;
    c_timespan_init(&ts_heap);
    h_time_update(&ts_heap);

    c_timespan_t ts_relay;
    c_timespan_init(&ts_relay);
    h_time_update(&ts_relay);



    /* TODO: add your own code here */
    while (1)
    {

        remotectrl_process();

        bool_t elasped = FALSE;

        h_time_is_elapsed_ms(&ts_sensor, 10000, &elasped);

        if(elasped == TRUE)
        {
            h_time_update(&ts_sensor);
            st_sensor_t sensor_data;
            sht40_read(&sensor_data);
            exchdat_set_sensor(sensor_data);



            /*uint32_t bytes,percent;

            heap_usage(&bytes,&percent);

            LOG_D(LOG_STD,"%d bytes / %d percent",bytes,percent);*/

            /*tx_mutex_get(&g_flash_memory_mutex,TX_WAIT_FOREVER);

            ret = fs_open();

            if(ret == X_RET_OK)
            {

                fs_bytes_available(&flash_bytes_available);

            }

            fs_close();

            tx_mutex_put(&g_flash_memory_mutex);*/

            //bsp_io_level_t lvl;
            //R_IOPORT_PinRead(&g_ioport_ctrl, IO_VM_SWITCH_CMD,&lvl );
            //LOG_D(LOG_STD,"VM CMD %d",lvl);
            //st_adc_t adc_snapshot;
            //adc_get_snapshot(&adc_snapshot);

            //LOG_D(LOG_STD,"%lu mV / %lu mA / %lu mA /%lu mA",adc_snapshot.vin,adc_snapshot.iin,adc_snapshot.mot1_iu,adc_snapshot.mot1_iw);
        }


        h_time_is_elapsed_ms(&ts_adc, 1000, &elasped);
        if(elasped == TRUE)
        {
            h_time_update(&ts_adc);
            st_adc_t adc_snap;

            ret =  adc_get_snapshot(&adc_snap);
            if(ret == X_RET_OK)
            {
                float vinf = (float)(adc_snap.vin/1000.0f);
                float vbattf = (float)(adc_snap.vbatt/1000.0f);
                st_voltages_t v;
                v.main_voltage = vinf;
                v.battery_voltage = vbattf;
                exchdat_set_voltages(v);
                bool_t batt_detected = FALSE;
                if(vbattf > 5.0f)
                    batt_detected = TRUE;
                exchdat_set_battery_detected(batt_detected);
            }
        }

        h_time_is_elapsed_ms(&ts_relay, 1000, &elasped);
        {
            h_time_update(&ts_relay);
            relay_process();
        }

        /*h_time_is_elapsed_ms(&ts_heap, 10000, &elasped);
        if(elasped == TRUE)
        {
            h_time_update(&ts_heap);
            uint32_t b,p;
            heap_usage(&b, &p);
            LOG_D(LOG_STD,"heap %d bytes / %d percent",b,p);
        }*/


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
