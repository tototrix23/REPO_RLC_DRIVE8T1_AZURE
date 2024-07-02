/*
 * adc.c
 *
 *  Created on: 24 oct. 2023
 *      Author: Ch.Leclercq
 */
#include <motors_thread.h>
#include <config.h>
#include <_core/c_protected/c_protected.h>
#include "adc.h"
#include <motor/motor.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "adc"

st_adc_t adc_inst;

bool_t adc_calibration_finished = FALSE;
bool_t adc_int=FALSE;
float adc_iin=0.0;
float adc_vin=0.0;
float adc_hall1=0.0;
float adc_hall2=0.0;
float adc_vbatt=0.0;
uint16_t adc_uiin;

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

void adc_mot_callback(adc_callback_args_t *p_args)
{
    motor_120_driver_instance_t      * p_instance      = (motor_120_driver_instance_t *) p_args->p_context;
    motor_120_driver_instance_ctrl_t * p_instance_ctrl = (motor_120_driver_instance_ctrl_t *) p_instance->p_ctrl;
    motor_120_driver_callback_args_t   temp_args_t;


    uint16_t data[5];


    if(p_instance == &g_motor_120_driver0)
    {
        R_ADC_Read(&g_adc0_ctrl,8,&data[0]);
        R_ADC_Read(&g_adc0_ctrl,4,&data[1]);


        volatile float v_vhall1 = (float)(((float)data[0]*3300.0f)/4096.0f);
        adc_hall1 = (adc_hall1*(ADC_VHALL1_AVERAGE-1.0f))/(ADC_VHALL1_AVERAGE);
        adc_hall1 = adc_hall1+(v_vhall1/ADC_VHALL1_AVERAGE);

        volatile float v_vhall2 = (float)(((float)data[1]*3300.0f)/4096.0f);
        adc_hall2 = (adc_hall2*(ADC_VHALL2_AVERAGE-1.0f))/(ADC_VHALL2_AVERAGE);
        adc_hall2 = adc_hall2+(v_vhall2/ADC_VHALL2_AVERAGE);

        adc_inst.instantaneous.vhall1 = (uint16_t)ADC_VHALL1_ADAPT(v_vhall1);
        adc_inst.instantaneous.vhall2 = (uint16_t)ADC_VHALL2_ADAPT(v_vhall2);
        adc_inst.average.vhall1 = (uint16_t)ADC_VHALL1_ADAPT(adc_hall1);
        adc_inst.average.vhall2 = (uint16_t)ADC_VHALL2_ADAPT(adc_hall2);

    }
    else if(p_instance == &g_motor_120_driver1)
    {
        R_ADC_Read(&g_adc1_ctrl,16,&data[0]);
        R_ADC_Read(&g_adc1_ctrl,17,&data[1]);
        R_ADC_Read(&g_adc1_ctrl,18,&data[2]);

        volatile float v_vin = (((float)data[1]*3300.0f)/4096.0f);
        adc_vin = (adc_vin*(ADC_VIN_AVERAGE-1.0f))/(ADC_VIN_AVERAGE);
        adc_vin = adc_vin+(v_vin/ADC_VIN_AVERAGE);

        volatile float v_vbatt = (float)(((float)data[0]*3300.0f)/4096.0f);
        adc_vbatt = (adc_vbatt*(ADC_VBATT_AVERAGE-1.0f))/(ADC_VBATT_AVERAGE);
        adc_vbatt = adc_vbatt+(v_vbatt/ADC_VBATT_AVERAGE);

        volatile float v_iin = (float)(((float)data[2]*3300.0f)/4096.0f);
        adc_iin = (adc_iin*(ADC_IIN_AVERAGE-1.0f))/(ADC_IIN_AVERAGE);
        adc_iin = adc_iin+(v_iin/ADC_IIN_AVERAGE);


        adc_inst.instantaneous.vin = (uint16_t)ADC_VIN_ADAPT(v_vin);
        adc_inst.instantaneous.iin = (uint16_t)ADC_IIN_ADAPT(v_iin);
        adc_inst.instantaneous.vbatt = (uint16_t)ADC_VBATT_ADAPT(v_vbatt);
        adc_inst.average.vin = (uint16_t)ADC_VIN_ADAPT(adc_vin);
        adc_inst.average.iin = (uint16_t)ADC_IIN_ADAPT(adc_iin);
        adc_inst.average.vbatt = (uint16_t)ADC_VBATT_ADAPT(adc_vbatt);

        volatile uint8_t end=1+adc_iin;
        end=0;

    }
    rm_motor_120_driver_cyclic(p_args);


}



/*void adc_interrupt(adc_callback_args_t *p_args)
{
    if (p_args->event == ADC_EVENT_SCAN_COMPLETE)
    {
        if (p_args->group_mask == ADC_GROUP_MASK_0)
        {
            p_args->p_context = motors_instance.motorH->motor_driver_instance;
            rm_motor_120_driver_cyclic(p_args);

            motor_120_driver_instance_ctrl_t* driverH_ctrl = (motor_120_driver_instance_ctrl_t*)motors_instance.motorH->motor_driver_instance->p_ctrl;
            adc_inst.motorH.iu_ad = (adc_inst.motorH.iu_ad*(ADC_IMOT_AVERAGE-1.0f))/(ADC_IMOT_AVERAGE);
            adc_inst.motorH.iu_ad = adc_inst.motorH.iu_ad + (driverH_ctrl->f_iu_ad/ADC_IMOT_AVERAGE);
            adc_inst.motorH.iw_ad = (adc_inst.motorH.iw_ad*(ADC_IMOT_AVERAGE-1.0f))/(ADC_IMOT_AVERAGE);
            adc_inst.motorH.iw_ad = adc_inst.motorH.iw_ad + (driverH_ctrl->f_iw_ad/ADC_IMOT_AVERAGE);



        }
        if (p_args->group_mask == ADC_GROUP_MASK_2)
        {
            p_args->p_context = motors_instance.motorL->motor_driver_instance;
            rm_motor_120_driver_cyclic(p_args);

            motor_120_driver_instance_ctrl_t* driverL_ctrl = (motor_120_driver_instance_ctrl_t*)motors_instance.motorL->motor_driver_instance->p_ctrl;
            adc_inst.motorL.iu_ad = (adc_inst.motorL.iu_ad*(ADC_IMOT_AVERAGE-1.0f))/(ADC_IMOT_AVERAGE);
            adc_inst.motorL.iu_ad = adc_inst.motorL.iu_ad + (driverL_ctrl->f_iu_ad/ADC_IMOT_AVERAGE);
            adc_inst.motorL.iw_ad = (adc_inst.motorL.iw_ad*(ADC_IMOT_AVERAGE-1.0f))/(ADC_IMOT_AVERAGE);
            adc_inst.motorL.iw_ad = adc_inst.motorL.iw_ad + (driverL_ctrl->f_iw_ad/ADC_IMOT_AVERAGE);

        }
        if (p_args->group_mask == ADC_GROUP_MASK_8)
        {

            uint16_t data[5] = {0};
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VIN,&data[0]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VBATT,&data[1]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_IIN,&data[2]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VHALL1,&data[3]);
            R_ADC_B_Read(&g_adc_external_ctrl, ADC_CHANNEL_VHALL2,&data[4]);


            volatile float v_vin = (((float)data[0]*3300.0f)/4096.0f);
            adc_vin = (adc_vin*(ADC_VIN_AVERAGE-1.0f))/(ADC_VIN_AVERAGE);
            adc_vin = adc_vin+(v_vin/ADC_VIN_AVERAGE);

            volatile float v_vbatt = (float)(((float)data[1]*3300.0f)/4096.0f);
            adc_vbatt = (adc_vbatt*(ADC_VBATT_AVERAGE-1.0f))/(ADC_VBATT_AVERAGE);
            adc_vbatt = adc_vbatt+(v_vbatt/ADC_VBATT_AVERAGE);

            volatile float v_iin = (float)(((float)data[2]*3300.0f)/4096.0f);
            adc_iin = (adc_iin*(ADC_IIN_AVERAGE-1.0f))/(ADC_IIN_AVERAGE);
            adc_iin = adc_iin+(v_iin/ADC_IIN_AVERAGE);

            volatile float v_vhall1 = (float)(((float)data[3]*3300.0f)/4096.0f);
            adc_hall1 = (adc_hall1*(ADC_VHALL1_AVERAGE-1.0f))/(ADC_VHALL1_AVERAGE);
            adc_hall1 = adc_hall1+(v_vhall1/ADC_VHALL1_AVERAGE);

            volatile float v_vhall2 = (float)(((float)data[4]*3300.0f)/4096.0f);
            adc_hall2 = (adc_hall2*(ADC_VHALL2_AVERAGE-1.0f))/(ADC_VHALL2_AVERAGE);
            adc_hall2 = adc_hall2+(v_vhall2/ADC_VHALL2_AVERAGE);


            adc_inst.instantaneous.vin = (uint16_t)ADC_VIN_ADAPT(v_vin);
            adc_inst.instantaneous.iin = (uint16_t)ADC_IIN_ADAPT(v_iin);
            adc_inst.instantaneous.vbatt = (uint16_t)ADC_VBATT_ADAPT(v_vbatt);
            adc_inst.instantaneous.vhall1 = (uint16_t)ADC_VHALL1_ADAPT(v_vhall1);
            adc_inst.instantaneous.vhall2 = (uint16_t)ADC_VHALL2_ADAPT(v_vhall2);
            adc_inst.average.vin = (uint16_t)ADC_VIN_ADAPT(adc_vin);
            adc_inst.average.iin = (uint16_t)ADC_IIN_ADAPT(adc_iin);
            adc_inst.average.vbatt = (uint16_t)ADC_VBATT_ADAPT(adc_vbatt);
            adc_inst.average.vhall1 = (uint16_t)ADC_VHALL1_ADAPT(adc_hall1);
            adc_inst.average.vhall2 = (uint16_t)ADC_VHALL2_ADAPT(adc_hall2);


            //R_IOPORT_PinWrite(&g_ioport_ctrl, LED,BSP_IO_LEVEL_LOW );

        }
    }
    else if(p_args->event == ADC_EVENT_CALIBRATION_COMPLETE || p_args->event == ADC_EVENT_CALIBRATION_REQUEST)
    {
        if(p_args->event == ADC_EVENT_CALIBRATION_COMPLETE) adc_calibration_finished = TRUE;
        //LOG_D(LOG_STD,"ADC EVENT CALIBRATION");
        return;
    }
    else
    {

    }
}*/
