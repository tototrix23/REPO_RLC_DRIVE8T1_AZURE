/*
 * adc.c
 *
 *  Created on: 24 oct. 2023
 *      Author: Ch.Leclercq
 */
#include <motors_thread.h>
#include <config.h>
#include <_core/c_protected/c_protected.h>
#include <_hal/h_time/h_time.h>
#include "adc.h"
#include <motor/motor.h>
#include <motor/emergency/emergency.h>
#include <motor/config_spi/config_spi.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "adc"


typedef struct adc_accumulator_t
{
    uint32_t accumulator;
   uint32_t counter;
}adc_accumulator_t;


typedef enum e_current_phase
{
    motor1_iu,
    motor1_iw,
    motor2_iu,
    motor2_iw
}current_phase_t;

typedef  struct adc_imotor_internal_t
{
   uint32_t accumulator;
   uint32_t counter;
   uint32_t accumulator2;
   uint32_t counter2;
   uint32_t accumulator_stopped;
   uint32_t counter_stopped;
   uint32_t counter_timeout;
   bool_t detect;
   bool_t motor_stopped;
}adc_imotor_internal_t;

typedef  struct adc_imotor_phase_t
{
    uint32_t calibration_accumulator;
    uint16_t calibration_adc;
    motor_120_driver_phase_pattern_t pattern1;
    motor_120_driver_phase_pattern_t pattern2;
    adc_imotor_internal_t internal;
    uint32_t *ptr_result;
}adc_imotor_phase_t;


typedef  struct adc_imotor_t
{
    adc_imotor_phase_t iu;
    adc_imotor_phase_t iw;
}adc_imotor_t;

st_adc_raw_t adc_raw_inst;
st_adc_t adc_inst;

bool_t adc_calibration_finished = FALSE;
bool_t adc_int=FALSE;
volatile float adc_iin=0.0;
volatile float adc_vin=0.0;
volatile float adc_hall1=0.0;
volatile float adc_hall2=0.0;
volatile float adc_vbatt=0.0;
uint16_t adc_uiin;

static bool_t calibration_mode = FALSE;
static uint32_t calibration_count = 0;


volatile adc_accumulator_t accumulator_iin;
volatile adc_accumulator_t accumulator_vin;
volatile adc_accumulator_t accumulator_vbatt;
volatile adc_accumulator_t accumulator_vhall_common;
volatile adc_accumulator_t accumulator_vhall1;
volatile adc_accumulator_t accumulator_vhall2;
volatile adc_accumulator_t accumulator_vm;

volatile adc_imotor_t motor1_currents;
volatile adc_imotor_t motor2_currents;

volatile adc_ballast_activated = FALSE;

bool_t process_current(adc_imotor_phase_t* ptr,motor_120_driver_instance_ctrl_t* drv_inst,uint16_t adc_value);


void adc_mot_callback(adc_callback_args_t *p_args);
adc_instance_t *ptr_adc_instance[2];

extern const adc_extended_cfg_t g_adc0_cfg_extend;
extern const adc_extended_cfg_t g_adc1_cfg_extend;

const adc_cfg_t g_adc0_custom_cfg =
{ .unit = 0, .mode = ADC_MODE_SINGLE_SCAN, .resolution = ADC_RESOLUTION_12_BIT, .alignment =
          (adc_alignment_t) ADC_ALIGNMENT_RIGHT,
  .trigger = (adc_trigger_t) 0xF, // Not used
  .p_callback = adc_mot_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = &g_motor_120_driver0,
#else
  .p_context = &NULL,
#endif
  .p_extend = &g_adc0_cfg_extend,
#if defined(VECTOR_NUMBER_ADC0_SCAN_END)
    .scan_end_irq        = VECTOR_NUMBER_ADC0_SCAN_END,
#else
  .scan_end_irq = FSP_INVALID_VECTOR,
#endif
  .scan_end_ipl = (5),
#if defined(VECTOR_NUMBER_ADC0_SCAN_END_B)
    .scan_end_b_irq      = VECTOR_NUMBER_ADC0_SCAN_END_B,
#else
  .scan_end_b_irq = FSP_INVALID_VECTOR,
#endif
  .scan_end_b_ipl = (BSP_IRQ_DISABLED), };


const adc_cfg_t g_adc1_custom_cfg =
{ .unit = 1, .mode = ADC_MODE_SINGLE_SCAN, .resolution = ADC_RESOLUTION_12_BIT, .alignment =
          (adc_alignment_t) ADC_ALIGNMENT_RIGHT,
  .trigger = (adc_trigger_t) 0xF, // Not used
  .p_callback = adc_mot_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = &g_motor_120_driver1,
#else
  .p_context = &NULL,
#endif
  .p_extend = &g_adc1_cfg_extend,
#if defined(VECTOR_NUMBER_ADC1_SCAN_END)
    .scan_end_irq        = VECTOR_NUMBER_ADC1_SCAN_END,
#else
  .scan_end_irq = FSP_INVALID_VECTOR,
#endif
  .scan_end_ipl = (5),
#if defined(VECTOR_NUMBER_ADC1_SCAN_END_B)
    .scan_end_b_irq      = VECTOR_NUMBER_ADC1_SCAN_END_B,
#else
  .scan_end_b_irq = FSP_INVALID_VECTOR,
#endif
  .scan_end_b_ipl = (BSP_IRQ_DISABLED), };

return_t adc_init(void)
{
    return_t ret = X_RET_OK;

    ptr_adc_instance[0] = (adc_instance_t*)&g_adc0;
    ptr_adc_instance[1] = (adc_instance_t*)&g_adc1;

    memset(&adc_inst,0x00,sizeof(st_adc_t));
    for(uint8_t i=0;i<2;i++)
    {
        adc_status_t status = {.state = ADC_STATE_SCAN_IN_PROGRESS};
        if(i==0)
            R_ADC_Open (ptr_adc_instance[i]->p_ctrl, &g_adc0_custom_cfg);
        else
            R_ADC_Open (ptr_adc_instance[i]->p_ctrl, &g_adc1_custom_cfg);

        R_ADC_ScanCfg (ptr_adc_instance[i]->p_ctrl, ptr_adc_instance[i]->p_channel_cfg);
        R_ADC_Calibrate(ptr_adc_instance[i]->p_ctrl,ptr_adc_instance[i]->p_cfg->p_extend);
        while (ADC_STATE_IDLE != status.state)
        {
            R_ADC_StatusGet(ptr_adc_instance[i]->p_ctrl, &status);
        }
        R_ADC_ScanStart(ptr_adc_instance[i]->p_ctrl);
    }

    memset(&accumulator_iin,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vin,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vbatt,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vhall_common,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vhall1,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vhall2,0x00,sizeof(adc_accumulator_t));
    memset(&accumulator_vm,0x00,sizeof(adc_accumulator_t));



    memset(&motor1_currents,0x00,sizeof(adc_imotor_t));
    memset(&motor2_currents,0x00,sizeof(adc_imotor_t));
    motor1_currents.iu.pattern1=MOTOR_120_DRIVER_PHASE_PATTERN_UP_PWM_WN_ON;
    motor1_currents.iu.pattern2=MOTOR_120_DRIVER_PHASE_PATTERN_WP_PWM_UN_ON;
    motor1_currents.iu.ptr_result = &adc_raw_inst.average.mot1_iu_uint;
    motor1_currents.iw.pattern1=MOTOR_120_DRIVER_PHASE_PATTERN_UP_PWM_WN_ON;
    motor1_currents.iw.pattern2=MOTOR_120_DRIVER_PHASE_PATTERN_WP_PWM_UN_ON;
    motor1_currents.iw.ptr_result = &adc_raw_inst.average.mot1_iw_uint;
    return ret;

}

return_t adc_is_ready(bool_t *result)
{
    return_t ret = X_RET_OK;
    *result = FALSE;

    bool_t adc_ready[2];

    for(uint8_t i=0;i<2;i++)
    {
        adc_instance_ctrl_t *adc_inst_ctrl = (adc_instance_ctrl_t*)ptr_adc_instance[i]->p_ctrl;
        if(adc_inst_ctrl->initialized != 0x00 && adc_inst_ctrl->opened != 0x00)
            adc_ready[i] = TRUE;
        else
            adc_ready[i] = FALSE;
    }
    if(adc_ready[0] == TRUE && adc_ready[1] == TRUE)
    {
        *result = TRUE;
    }
    else
    {
        *result = FALSE;
    }

    return ret;
}

return_t adc_capture(void)
{
    return_t ret = X_RET_OK;
    /*if(adc_calibration_finished == FALSE) return X_RET_OK;
    fsp_err_t err = R_ADC_B_ScanGroupStart(&g_adc_external_ctrl,ADC_GROUP_MASK_8);
    if(err != FSP_SUCCESS)
    {
        ERROR_LOG_AND_RETURN(F_RET_ERROR_GENERIC);
    }*/
    return ret;
}

return_t adc_set_calibration_mode(bool_t active)
{
    if(calibration_mode == FALSE && active == TRUE)
    {
        calibration_count = 0;
        motor1_currents.iu.calibration_accumulator=0;
        motor1_currents.iw.calibration_accumulator=0;
    }
    else if(calibration_mode == TRUE && active == FALSE)
    {
        motor1_currents.iu.calibration_accumulator= motor1_currents.iu.calibration_accumulator/calibration_count;
        motor1_currents.iu.calibration_adc = (uint16_t)motor1_currents.iu.calibration_accumulator;
        motor1_currents.iw.calibration_accumulator= motor1_currents.iw.calibration_accumulator/calibration_count;
        motor1_currents.iw.calibration_adc = (uint16_t)motor1_currents.iw.calibration_accumulator;
    }
    calibration_mode = active;
    return X_RET_OK;
}

return_t adc_get_snapshot(st_adc_t *res)
{
    return_t ret = X_RET_OK;
    st_adc_raw_t obj;
    c_protected_get_object(&adc_raw_inst, &obj, sizeof(st_adc_raw_t));
    memset(res,0x00,sizeof(st_adc_t));
    res->vhall1 = (uint16_t)ADC_VHALL1_ADAPT(obj.average.vhall1_uint);
    res->vhall2 = (uint16_t)ADC_VHALL2_ADAPT(obj.average.vhall2_uint);
    res->vin = (uint16_t)ADC_VIN_ADAPT(obj.average.vin_uint);
    res->vm = (uint16_t)ADC_VM_ADAPT(obj.average.vm_uint);
    res->iin = (uint16_t)ADC_IIN_ADAPT(obj.average.iin_uint);
    res->vbatt = (uint16_t)ADC_VBATT_ADAPT(obj.average.vbatt_uint);


    uint8_t gain;
    ret = h_drv8323s_get_gain(&drv_mot1,FALSE,&gain);
    if(ret != X_RET_OK)
    {
        return ret;
    }
    else
    {
        if(gain == 5)
        {
            res->mot1_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X5(obj.average.mot1_iu_uint);
            res->mot1_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X5(obj.average.mot1_iw_uint);
        }
        else if(gain == 10)
        {
            res->mot1_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X10(obj.average.mot1_iu_uint);
            res->mot1_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X10(obj.average.mot1_iw_uint);
        }
        else if(gain == 20)
        {
            res->mot1_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X20(obj.average.mot1_iu_uint);
            res->mot1_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X20(obj.average.mot1_iw_uint);
        }
        else if(gain == 40)
        {
            res->mot1_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X40(obj.average.mot1_iu_uint);
            res->mot1_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X40(obj.average.mot1_iw_uint);
        }
    }

    ret = h_drv8323s_get_gain(&drv_mot2,FALSE,&gain);
    if(ret != X_RET_OK)
    {
        return ret;
    }
    else
    {
        if(gain == 5)
        {
            res->mot2_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X5(obj.average.mot2_iu_uint);
            res->mot2_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X5(obj.average.mot2_iw_uint);
        }
        else if(gain == 10)
        {
            res->mot2_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X10(obj.average.mot2_iu_uint);
            res->mot2_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X10(obj.average.mot2_iw_uint);
        }
        else if(gain == 20)
        {
            res->mot2_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X20(obj.average.mot2_iu_uint);
            res->mot2_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X20(obj.average.mot2_iw_uint);
        }
        else if(gain == 40)
        {
            res->mot2_iu = (uint16_t)ADC_IMOT_ADAPT_GAIN_X40(obj.average.mot2_iu_uint);
            res->mot2_iw = (uint16_t)ADC_IMOT_ADAPT_GAIN_X40(obj.average.mot2_iw_uint);
        }
    }
    return ret;
}

return_t adc_get_snapshot_raw(st_adc_raw_t *res)
{
    st_adc_raw_t obj;
    c_protected_get_object(&adc_raw_inst, res, sizeof(st_adc_raw_t));
    return X_RET_OK;
}

bool_t process_current(adc_imotor_phase_t* ptr,motor_120_driver_instance_ctrl_t* drv_inst,uint16_t adc_value)
{
    volatile uint16_t adc_val = 0;

    if(drv_inst->pattern == ptr->pattern1 ||
       drv_inst->pattern == ptr->pattern2)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED_ERROR,BSP_IO_LEVEL_HIGH );
        ptr->internal.detect = 1;
        ptr->internal.counter_timeout = 0;
        if(adc_value> ptr->calibration_adc)
            adc_val = adc_value-ptr->calibration_adc;
        else
            adc_val = ptr->calibration_adc - adc_value;


        if(ptr->internal.counter < 2000)
        {
            ptr->internal.accumulator += adc_val;
            ptr->internal.counter++;
            ptr->internal.motor_stopped = FALSE;
            ptr->internal.counter_stopped = 0;
        }
        else
        {
            ptr->internal.counter = 0;
            ptr->internal.accumulator = 0;
            ptr->internal.counter2 = 0;
            ptr->internal.accumulator2 = 0;
            ptr->internal.motor_stopped = TRUE;
            ptr->internal.detect = FALSE;
            if(ptr->internal.counter_stopped < 1000)
            {
                ptr->internal.accumulator_stopped+=adc_val;
                ptr->internal.counter_stopped++;
            }
            else
            {
                ptr->internal.accumulator_stopped = ptr->internal.accumulator_stopped/ptr->internal.counter_stopped;
                ptr->internal.accumulator_stopped = ptr->internal.accumulator_stopped*drv_inst->duty_uint;
                ptr->internal.accumulator_stopped = ptr->internal.accumulator_stopped/drv_inst->u4_carrier_base;
                *ptr->ptr_result = ptr->internal.accumulator_stopped;
                ptr->internal.accumulator_stopped = 0;
                ptr->internal.counter_stopped = 0;
            }
        }
    }
    else
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED_ERROR,BSP_IO_LEVEL_LOW);
        if(ptr->internal.detect == 1)
        {
            ptr->internal.accumulator = ptr->internal.accumulator/ptr->internal.counter;
            ptr->internal.accumulator = ptr->internal.accumulator * drv_inst->duty_uint;
            ptr->internal.accumulator = ptr->internal.accumulator/drv_inst->u4_carrier_base;

            ptr->internal.accumulator2+=ptr->internal.accumulator;
            ptr->internal.counter2++;
            if(ptr->internal.counter2 >= 8)
            {
                ptr->internal.accumulator2 = ptr->internal.accumulator2/ptr->internal.counter2;
                *ptr->ptr_result = ptr->internal.accumulator2;
                ptr->internal.counter2 = 0;
                ptr->internal.accumulator2 = 0;
            }
            ptr->internal.detect = 0;
            ptr->internal.accumulator = 0;
            ptr->internal.counter = 0;
        }
        else
        {
            ptr->internal.counter_timeout++;
            if(ptr->internal.counter_timeout >= 2000)
            {
                *ptr->ptr_result = 0;
            }
        }
    }


    return ptr->internal.motor_stopped;
}

void adc_mot_callback(adc_callback_args_t *p_args)
{
    motor_120_driver_instance_t      * p_instance      = (motor_120_driver_instance_t *) p_args->p_context;
    motor_120_driver_instance_ctrl_t * p_instance_ctrl = (motor_120_driver_instance_ctrl_t *) p_instance->p_ctrl;
    motor_120_driver_callback_args_t   temp_args_t;

    uint16_t data[5];

    bsp_io_level_t mot1_fault;
    bsp_io_level_t mot2_fault;
    R_IOPORT_PinRead(&g_ioport_ctrl, IO_MOT1_FAULT,&mot1_fault );
    R_IOPORT_PinRead(&g_ioport_ctrl, IO_MOT2_FAULT,&mot2_fault );
    if(mot1_fault == 0)
        motor_emergency_set_motor1_fault();
    if(mot2_fault == 0)
        motor_emergency_set_motor2_fault();


    if(p_instance == &g_motor_120_driver0)
    {
        //R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED_ERROR,BSP_IO_LEVEL_HIGH );





        R_ADC_Read(&g_adc0_ctrl,8,&data[0]);
        R_ADC_Read(&g_adc0_ctrl,4,&data[1]);
        R_ADC_Read(&g_adc0_ctrl,2,&data[2]);
        R_ADC_Read(&g_adc0_ctrl,1,&data[3]);
        R_ADC_Read(&g_adc0_ctrl,0,&data[4]);
        if(calibration_mode == TRUE && calibration_count<2000)
        {
            motor1_currents.iu.calibration_accumulator += data[2];
            motor1_currents.iw.calibration_accumulator += data[3];
            calibration_count++;
        }



        adc_raw_inst.instantaneous.vm = ADC_VM_ADAPT(data[4]);
        accumulator_vm.accumulator+=data[4];
        accumulator_vm.counter++;
        if(accumulator_vm.counter>=1000)
        {
            adc_raw_inst.average.vhall1_uint = accumulator_vm.accumulator/accumulator_vm.counter;
            accumulator_vm.accumulator=0;
            accumulator_vm.counter = 0;
        }

        adc_raw_inst.instantaneous.vhall1 = (float)ADC_VHALL1_ADAPT(data[0]);
        accumulator_vhall1.accumulator+=data[0];
        accumulator_vhall1.counter++;
        if(accumulator_vhall1.counter>=1000)
        {
            adc_raw_inst.average.vhall1_uint = accumulator_vhall1.accumulator/accumulator_vhall1.counter;
            accumulator_vhall1.accumulator=0;
            accumulator_vhall1.counter = 0;
        }

        adc_raw_inst.instantaneous.vhall2 = (float)ADC_VHALL1_ADAPT(data[1]);
        accumulator_vhall2.accumulator+=data[1];
        accumulator_vhall2.counter++;
        if(accumulator_vhall2.counter>=1000)
        {
            adc_raw_inst.average.vhall2_uint = accumulator_vhall2.accumulator/accumulator_vhall2.counter;
            accumulator_vhall2.accumulator=0;
            accumulator_vhall2.counter = 0;
        }

        bool_t stopped = process_current(&motor1_currents.iu,p_instance_ctrl,data[2]);
        if(stopped)
            *motor1_currents.iw.ptr_result = 0;
        stopped = process_current(&motor1_currents.iw,p_instance_ctrl,data[3]);
        if(stopped)
            *motor1_currents.iu.ptr_result = 0;

        //R_IOPORT_PinWrite(&g_ioport_ctrl, IO_LED_ERROR,BSP_IO_LEVEL_LOW);

    }
    else if(p_instance == &g_motor_120_driver1)
    {
        R_ADC_Read(&g_adc1_ctrl,16,&data[0]);
        R_ADC_Read(&g_adc1_ctrl,17,&data[1]);
        R_ADC_Read(&g_adc1_ctrl,18,&data[2]);


        adc_raw_inst.instantaneous.vin = (float)ADC_VIN_ADAPT(data[1]);
        accumulator_vin.accumulator+=data[1];
        accumulator_vin.counter++;
        if(accumulator_vin.counter>=1000)
        {
            adc_raw_inst.average.vin_uint = accumulator_vin.accumulator/accumulator_vin.counter;
            accumulator_vin.accumulator=0;
            accumulator_vin.counter = 0;
        }

        adc_raw_inst.instantaneous.vbatt = (float)ADC_VBATT_ADAPT(data[0]);
        accumulator_vbatt.accumulator+=data[0];
        accumulator_vbatt.counter++;
        if(accumulator_vbatt.counter>=1000)
        {
            adc_raw_inst.average.vbatt_uint = accumulator_vbatt.accumulator/accumulator_vbatt.counter;
            accumulator_vbatt.accumulator=0;
            accumulator_vbatt.counter = 0;
        }

        adc_raw_inst.instantaneous.iin = (float)ADC_IIN_ADAPT(data[2]);
        accumulator_iin.accumulator+=data[2];
        accumulator_iin.counter++;
        if(accumulator_iin.counter>=10000)
        {
            adc_raw_inst.average.iin_uint = accumulator_iin.accumulator/accumulator_iin.counter;
            accumulator_iin.accumulator=0;
            accumulator_iin.counter = 0;
        }
    }

    if(adc_raw_inst.instantaneous.vm > adc_raw_inst.instantaneous.vin)
    {
        volatile float diff_vin_vm = adc_raw_inst.instantaneous.vm - adc_raw_inst.instantaneous.vin;
        if(adc_ballast_activated == FALSE && diff_vin_vm > 2000.0f)
        {
            LOG_D(LOG_STD,"1 -> %d",(uint32_t)diff_vin_vm);
            adc_ballast_activated = TRUE;
            //R_GPT_Open(&g_timer_ballast_ctrl, &g_timer_ballast_cfg);
            //(void) R_GPT_Start(&g_timer_ballast_ctrl);
            R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_BALLAST_CMD,BSP_IO_LEVEL_HIGH);
        }
        else if(adc_ballast_activated == TRUE && diff_vin_vm < 500.0f)
        {
            LOG_D(LOG_STD,"0 -> %d",(uint32_t)diff_vin_vm);
            adc_ballast_activated = FALSE;
            //(void) R_GPT_Stop(&g_timer_ballast_ctrl);
            //R_GPT_Close(&g_timer_ballast_ctrl);
            R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_BALLAST_CMD,BSP_IO_LEVEL_LOW);
        }
    }
    else if(adc_ballast_activated == TRUE)
    {
        adc_ballast_activated = FALSE;
        R_IOPORT_PinWrite(&g_ioport_ctrl, IO_VM_BALLAST_CMD,BSP_IO_LEVEL_LOW);
    }


    rm_motor_120_driver_cyclic(p_args);
}
