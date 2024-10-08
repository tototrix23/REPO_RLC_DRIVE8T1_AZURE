/*
 * moteur.c
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#include <c_timespan/c_timespan.h>
#include <h_time/h_time.h>
#include <motor/motor.h>
#include <exchanged_data/exchanged_data.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "DRIVE"

motor_cfg_t g_user_motor_cfg;
motor_120_degree_extended_cfg_t g_user_motor_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor_120_driver_extended_cfg;

motor_cfg_t g_user_motor1_cfg;
motor_120_degree_extended_cfg_t g_user_motor1_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor1_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor1_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor1_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor1_120_driver_extended_cfg;

/*timer_instance_t mot0_g_timer0;
timer_cfg_t mot0_g_timer0_cfg;
timer_instance_t mot0_g_timer1;
timer_cfg_t mot0_g_timer1_cfg;
motor_120_driver_extended_cfg_t g_mot_120_driver0_extend;
motor_120_driver_cfg_t g_mot_120_driver0_cfg;
motor_120_driver_instance_ctrl_t g_mot_120_driver0_ctrl;
motor_120_driver_instance_t g_mot_120_driver0;
motor_120_control_hall_instance_ctrl_t g_mot_120_control_hall0_ctrl;
extern const motor_120_control_hall_extended_cfg_t g_motor_120_control_hall0_extend;
motor_120_control_hall_extended_cfg_t g_mot_120_control_hall0_extend;
motor_120_control_cfg_t g_mot_120_control_hall0_cfg;
motor_120_control_instance_t g_mot_120_control_hall0;
motor_120_degree_instance_ctrl_t g_mot_120_degree0_ctrl;
extern const motor_120_degree_extended_cfg_t g_motor_120_degree0_extend;
motor_120_degree_extended_cfg_t g_mot_120_degree0_extend;
motor_cfg_t g_mot_120_degree0_cfg;
motor_instance_t g_mot_120_degree0;*/
st_motor_t motor0;

/*timer_instance_t mot1_g_timer0;
timer_cfg_t mot1_g_timer0_cfg;
timer_instance_t mot1_g_timer1;
timer_cfg_t mot1_g_timer1_cfg;
motor_120_driver_extended_cfg_t g_mot_120_driver1_extend;
motor_120_driver_cfg_t g_mot_120_driver1_cfg;
motor_120_driver_instance_ctrl_t g_mot_120_driver1_ctrl;
motor_120_driver_instance_t g_mot_120_driver1;
motor_120_control_hall_instance_ctrl_t g_mot_120_control_hall1_ctrl;
extern const motor_120_control_hall_extended_cfg_t g_motor_120_control_hall1_extend;
motor_120_control_hall_extended_cfg_t g_mot_120_control_hall1_extend;
motor_120_control_cfg_t g_mot_120_control_hall1_cfg;
motor_120_control_instance_t g_mot_120_control_hall1;
motor_120_degree_instance_ctrl_t g_mot_120_degree1_ctrl;
extern const motor_120_degree_extended_cfg_t g_motor_120_degree1_extend;
motor_120_degree_extended_cfg_t g_mot_120_degree1_extend;
motor_cfg_t g_mot_120_degree1_cfg;
motor_instance_t g_mot_120_degree1;*/
st_motor_t motor1;

volatile st_drive_t motors_instance;

char motorh_serial[20]={0};
char motorl_serial[20]={0};


static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value);

void motor_structures_init(void)
{

    /*memset(&motors_instance,0x00,sizeof(st_drive_t));
    //---------------------------------------------------------------------
    // MOTOR0
    //---------------------------------------------------------------------
    memcpy(&mot0_g_timer0_cfg,&g_timer3_cfg,sizeof(timer_cfg_t));
    mot0_g_timer0_cfg.p_context = &g_mot_120_control_hall0;
    memcpy(&mot0_g_timer0,&g_timer3,sizeof(timer_instance_t));
    mot0_g_timer0.p_cfg  =&mot0_g_timer0_cfg;

    memcpy(&mot0_g_timer1_cfg,&g_timer4_cfg,sizeof(timer_cfg_t));
    mot0_g_timer1_cfg.p_context = &g_mot_120_control_hall0;
    memcpy(&mot0_g_timer1,&g_timer4,sizeof(timer_instance_t));
    mot0_g_timer1.p_cfg  =&mot0_g_timer1_cfg;

    // DRIVER
    memcpy(&g_mot_120_driver0_extend,&g_motor_120_driver0_extend,sizeof(motor_120_driver_extended_cfg_t));
    g_mot_120_driver0_extend.p_adc_instance = &g_adc_mot0;

    memcpy(&g_mot_120_driver0_cfg,&g_motor_120_driver0_cfg,sizeof(motor_120_driver_cfg_t));
    g_mot_120_driver0_cfg.p_extend = &g_mot_120_driver0_extend;
    g_mot_120_driver0_cfg.p_context = &g_mot_120_control_hall0;
    g_mot_120_driver0.p_ctrl = &g_mot_120_driver0_ctrl;
    g_mot_120_driver0.p_cfg = &g_mot_120_driver0_cfg;
    g_mot_120_driver0.p_api = &g_motor_120_driver_on_motor_120_driver;

    //HALL
    memcpy(&g_mot_120_control_hall0_extend,&g_motor_120_control_hall0_extend,sizeof(motor_120_control_hall_extended_cfg_t));
    g_mot_120_control_hall0_extend.p_motor_120_driver_instance = &g_mot_120_driver0;
    g_mot_120_control_hall0_extend.p_speed_cyclic_timer_instance = &mot0_g_timer0;
    g_mot_120_control_hall0_extend.p_speed_calc_timer_instance = &mot0_g_timer1;
    memcpy(&g_mot_120_control_hall0_cfg,&g_motor_120_control_hall0_cfg,sizeof(motor_120_control_cfg_t));

    g_mot_120_control_hall0_cfg.p_context = &g_mot_120_degree0;
    g_mot_120_control_hall0_cfg.p_extend = &g_mot_120_control_hall0_extend;

    memcpy(&g_mot_120_control_hall0,&g_motor_120_control_hall0,sizeof(motor_120_control_instance_t));
    g_mot_120_control_hall0.p_ctrl = &g_mot_120_control_hall0_ctrl;
    g_mot_120_control_hall0.p_cfg = &g_mot_120_control_hall0_cfg;
    g_mot_120_control_hall0.p_api = &g_motor_120_control_on_motor_120_control_hall;

    //MOTOR
    memcpy(&g_mot_120_degree0_extend,&g_motor_120_degree0_extend,sizeof(motor_120_degree_extended_cfg_t));
    g_mot_120_degree0_extend.p_motor_120_control_instance = &g_mot_120_control_hall0;

    memcpy(&g_mot_120_degree0_cfg,&g_motor_120_degree0_cfg,sizeof(motor_cfg_t));
    g_mot_120_degree0_cfg.p_extend = &g_mot_120_degree0_extend;

    g_mot_120_degree0.p_ctrl = &g_mot_120_degree0_ctrl;
    g_mot_120_degree0.p_cfg = &g_mot_120_degree0_cfg;
    g_mot_120_degree0.p_api =  &g_motor_on_motor_120_degree;*/

    motor0.motor_ctrl_instance = &g_motor_120_degree0;
    motor0.motor_driver_instance = &g_motor_120_driver0;
    motor0.motor_hall_instance = &g_motor_120_control_hall0;
    motor0.hall_vars = (motor_120_control_hall_instance_ctrl_t *)(&g_motor_120_control_hall0_ctrl);

/*
    //---------------------------------------------------------------------
    // MOTOR1
    //---------------------------------------------------------------------
    memcpy(&mot1_g_timer0_cfg,&g_timer8_cfg,sizeof(timer_cfg_t));
    mot1_g_timer0_cfg.p_context = &g_mot_120_control_hall1;
    memcpy(&mot1_g_timer0,&g_timer8,sizeof(timer_instance_t));
    mot1_g_timer0.p_cfg  =&mot1_g_timer0_cfg;

    memcpy(&mot1_g_timer1_cfg,&g_timer9_cfg,sizeof(timer_cfg_t));
    mot1_g_timer1_cfg.p_context = &g_mot_120_control_hall1;
    memcpy(&mot1_g_timer1,&g_timer9,sizeof(timer_instance_t));
    mot1_g_timer1.p_cfg  =&mot1_g_timer1_cfg;
    // DRIVER
    memcpy(&g_mot_120_driver1_extend,&g_motor_120_driver1_extend,sizeof(motor_120_driver_extended_cfg_t));
    g_mot_120_driver1_extend.p_adc_instance = &g_adc_mot1;

    memcpy(&g_mot_120_driver1_cfg,&g_motor_120_driver1_cfg,sizeof(motor_120_driver_cfg_t));
    g_mot_120_driver1_cfg.p_extend = &g_mot_120_driver1_extend;
    g_mot_120_driver1_cfg.p_context = &g_mot_120_control_hall1;

    g_mot_120_driver1.p_ctrl = &g_mot_120_driver1_ctrl;
    g_mot_120_driver1.p_cfg = &g_mot_120_driver1_cfg;
    g_mot_120_driver1.p_api = &g_motor_120_driver_on_motor_120_driver;

    //HALL
    memcpy(&g_mot_120_control_hall1_extend,&g_motor_120_control_hall1_extend,sizeof(motor_120_control_hall_extended_cfg_t));
    g_mot_120_control_hall1_extend.p_motor_120_driver_instance = &g_mot_120_driver1;
    g_mot_120_control_hall1_extend.p_speed_cyclic_timer_instance = &mot1_g_timer0;
    g_mot_120_control_hall1_extend.p_speed_calc_timer_instance = &mot1_g_timer1;
    memcpy(&g_mot_120_control_hall1_cfg,&g_motor_120_control_hall1_cfg,sizeof(motor_120_control_cfg_t));
    g_mot_120_control_hall1_cfg.p_context = &g_mot_120_degree1;
    g_mot_120_control_hall1_cfg.p_extend = &g_mot_120_control_hall1_extend;

    memcpy(&g_mot_120_control_hall1,&g_motor_120_control_hall1,sizeof(motor_120_control_instance_t));
    g_mot_120_control_hall1.p_ctrl = &g_mot_120_control_hall1_ctrl;
    g_mot_120_control_hall1.p_cfg = &g_mot_120_control_hall1_cfg;
    g_mot_120_control_hall1.p_api = &g_motor_120_control_on_motor_120_control_hall;

    //MOTOR
    memcpy(&g_mot_120_degree1_extend,&g_motor_120_degree1_extend,sizeof(motor_120_degree_extended_cfg_t));
    g_mot_120_degree1_extend.p_motor_120_control_instance = &g_mot_120_control_hall1;

    memcpy(&g_mot_120_degree1_cfg,&g_motor_120_degree1_cfg,sizeof(motor_cfg_t));
    g_mot_120_degree1_cfg.p_extend = &g_mot_120_degree1_extend;

    g_mot_120_degree1.p_ctrl = &g_mot_120_degree1_ctrl;
    g_mot_120_degree1.p_cfg = &g_mot_120_degree1_cfg;
    g_mot_120_degree1.p_api =  &g_motor_on_motor_120_degree;*/

    motor1.motor_ctrl_instance = &g_motor_120_degree1;
    motor1.motor_driver_instance = &g_motor_120_driver1;
    motor1.motor_hall_instance = &g_motor_120_control_hall1;
    motor1.hall_vars = (motor_120_control_hall_instance_ctrl_t *)(&g_motor_120_control_hall1_ctrl);

    motor0.current_drive_mode = -1;
    motor1.current_drive_mode = -1;

    motors_instance.motorH = &motor0;
    motors_instance.motorL = &motor1;
    motors_instance.motors[0] =  motors_instance.motorH;
    motors_instance.motors[1] =  motors_instance.motorL;
    exchdat_set_drive_mode(MOTOR_MANUAL_MODE);
}

void motor_deinit_fsp(void)
{
    motor_120_degree_instance_ctrl_t * p_instance_ctrl;
    p_instance_ctrl = (motor_120_degree_instance_ctrl_t*)g_motor_120_degree0.p_ctrl;
    if(p_instance_ctrl->open != 0x0)
       g_motor_120_degree0.p_api->close(g_motor_120_degree0.p_ctrl);
    p_instance_ctrl = (motor_120_degree_instance_ctrl_t*)g_motor_120_degree1.p_ctrl;
    if(p_instance_ctrl->open != 0x0)
       g_motor_120_degree1.p_api->close(g_motor_120_degree1.p_ctrl);


    R_GPT_THREE_PHASE_Stop(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Stop(g_three_phase1.p_ctrl);
}

void motor_init_fsp(void)
{
    /* Motor application (120_degree) */
    g_motor_120_degree0.p_api->open(g_motor_120_degree0.p_ctrl, g_motor_120_degree0.p_cfg);
    g_motor_120_degree1.p_api->open(g_motor_120_degree1.p_ctrl, g_motor_120_degree1.p_cfg);

    /* POEG */
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);

    R_GPT_THREE_PHASE_Stop(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Stop(g_three_phase1.p_ctrl);
    R_GPT_THREE_PHASE_Reset(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Reset(g_three_phase1.p_ctrl);
    gpt_periodset(g_timer0.p_ctrl,g_timer0.p_cfg->period_counts,(uint32_t)(g_timer0.p_cfg->period_counts));
    gpt_periodset(g_timer1.p_ctrl,g_timer1.p_cfg->period_counts,(uint32_t)(g_timer1.p_cfg->period_counts));
    gpt_periodset(g_timer2.p_ctrl,g_timer2.p_cfg->period_counts,(uint32_t)(g_timer2.p_cfg->period_counts));
    gpt_periodset(g_timer5.p_ctrl,g_timer5.p_cfg->period_counts,(uint32_t)((float)g_timer5.p_cfg->period_counts*1.5f));
    gpt_periodset(g_timer6.p_ctrl,g_timer6.p_cfg->period_counts,(uint32_t)((float)g_timer6.p_cfg->period_counts*1.5f));
    gpt_periodset(g_timer7.p_ctrl,g_timer7.p_cfg->period_counts,(uint32_t)((float)g_timer7.p_cfg->period_counts*1.5f));
    R_GPT_THREE_PHASE_Start(g_three_phase0.p_ctrl);
    R_GPT_THREE_PHASE_Start(g_three_phase1.p_ctrl);

    /* ELC */
    g_elc.p_api->open(g_elc.p_ctrl, g_elc.p_cfg);
    g_elc.p_api->enable(g_elc.p_ctrl);


    /* RMW */
    g_user_motor_cfg = *(g_motor_120_degree0_ctrl.p_cfg);
    g_user_motor_120_degree_extended_cfg = *(motor_120_degree_extended_cfg_t *)g_user_motor_cfg.p_extend;
    g_user_motor_cfg.p_extend = &g_user_motor_120_degree_extended_cfg;
    g_motor_120_degree0_ctrl.p_cfg = &g_user_motor_cfg;

    g_user_motor_120_control_cfg = *(g_motor_120_control_hall0_ctrl.p_cfg);
    g_user_motor_120_control_extended_cfg =
        *(motor_120_control_hall_extended_cfg_t *)g_user_motor_120_control_cfg.p_extend;
    g_user_motor_120_control_cfg.p_extend = &g_user_motor_120_control_extended_cfg;

    g_user_motor_120_driver_cfg = *(g_motor_120_driver0_ctrl.p_cfg);
    g_user_motor_120_driver_extended_cfg = *(motor_120_driver_extended_cfg_t *)g_user_motor_120_driver_cfg.p_extend;
    g_user_motor_120_driver_cfg.p_extend = &g_user_motor_120_driver_extended_cfg;




    g_user_motor1_cfg = *(g_motor_120_degree1_ctrl.p_cfg);
    g_user_motor1_120_degree_extended_cfg = *(motor_120_degree_extended_cfg_t *)g_user_motor1_cfg.p_extend;
    g_user_motor1_cfg.p_extend = &g_user_motor1_120_degree_extended_cfg;
    g_motor_120_degree1_ctrl.p_cfg = &g_user_motor1_cfg;

    g_user_motor1_120_control_cfg = *(g_motor_120_control_hall1_ctrl.p_cfg);
    g_user_motor1_120_control_extended_cfg =
        *(motor_120_control_hall_extended_cfg_t *)g_user_motor1_120_control_cfg.p_extend;
    g_user_motor1_120_control_cfg.p_extend = &g_user_motor1_120_control_extended_cfg;

    g_user_motor1_120_driver_cfg = *(g_motor_120_driver1_ctrl.p_cfg);
    g_user_motor1_120_driver_extended_cfg = *(motor_120_driver_extended_cfg_t *)g_user_motor1_120_driver_cfg.p_extend;
    g_user_motor1_120_driver_cfg.p_extend = &g_user_motor1_120_driver_extended_cfg;


    g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);
    g_motor_120_degree1.p_api->reset(g_motor_120_degree1.p_ctrl);


    motor0.motor_ctrl_instance->p_api->statusGet(motor0.motor_ctrl_instance->p_ctrl,&motor0.status);
    motor1.motor_ctrl_instance->p_api->statusGet(motor1.motor_ctrl_instance->p_ctrl,&motor1.status);
    motor0.motor_ctrl_instance->p_api->errorCheck(motor0.motor_ctrl_instance->p_ctrl,&motor0.error);
    motor1.motor_ctrl_instance->p_api->errorCheck(motor1.motor_ctrl_instance->p_ctrl,&motor1.error);
}

void motor_log_speed(st_motor_t *mot)
{

}


void motor_log_api(void)
{

}



return_t motor_is_speed_achieved(st_motor_t *mot,bool_t *res)
{
    return_t ret = X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(mot  != NULL)
    ASSERT(res  != NULL)
#endif

	volatile motor_120_control_instance_t *mot_inst = (motor_120_control_instance_t*)mot->motor_hall_instance;

    volatile motor_120_control_hall_instance_ctrl_t * p_instance_ctrl = (motor_120_control_hall_instance_ctrl_t *) (mot_inst->p_ctrl);

    const float delta_value = 0.003f;
    float speedGet=0.0;
    float speedSet=0.0;
    motor_120_control_cfg_t *pcfg = 0;

    if(p_instance_ctrl->active == MOTOR_120_CONTROL_STATUS_ACTIVE)
    {
        if(p_instance_ctrl->extSettings->active == TRUE)
        {
            float diff = fabsf(p_instance_ctrl->f4_v_ref - p_instance_ctrl->extSettings->voltage);
            if(diff <= 0.02f)
            {
                *res = TRUE;
            }
            else
            {
                *res = FALSE;
            }
        }
        else
        {

            float delta = p_instance_ctrl->f4_ref_speed_rad * delta_value;
            if(  (p_instance_ctrl->f4_speed_rad >= (p_instance_ctrl->f4_ref_speed_rad-delta)) &&
                 (p_instance_ctrl->f4_speed_rad <= (p_instance_ctrl->f4_ref_speed_rad+delta)))
            {
                //LOG_D(LOG_STD,"!!!! delta:%f sref:%f  s:%f vref:%f",p_instance_ctrl->f4_ref_speed_rad * delta_value,p_instance_ctrl->f4_ref_speed_rad,p_instance_ctrl->f4_speed_rad,p_instance_ctrl->f4_v_ref);
                *res = TRUE;
            }
            else
            {

                mot_inst->p_api->speedGet(mot_inst->p_ctrl,&speedGet);

                speedSet  =p_instance_ctrl->f4_ref_speed_rad/p_instance_ctrl->f_rpm2rad;


            	pcfg = (motor_120_control_cfg_t*)(mot_inst->p_cfg);


            	if( (p_instance_ctrl->f4_v_ref <= pcfg->f4_min_drive_v+0.05f) ||
                    (p_instance_ctrl->f4_v_ref >= pcfg->f4_max_drive_v-0.05f))
            	{
            	    //LOG_D(LOG_STD,"!!! vref:%f vmax:%f",p_instance_ctrl->f4_v_ref,pcfg->f4_max_drive_v);
            	    *res = TRUE;
            	}

            	else
                *res = FALSE;

            }
        }
    }
    else
    {
        *res = FALSE;
    }


    if(*res == TRUE)
    {
        /*pcfg = (motor_120_control_cfg_t*)(mot_inst->p_cfg);
        mot_inst->p_api->speedGet(mot_inst->p_ctrl,&speedGet);
        speedSet  =p_instance_ctrl->f4_ref_speed_rad/p_instance_ctrl->f_rpm2rad;

        LOG_D(LOG_STD,"delta:%f sref:%f  s:%f vref:%f",p_instance_ctrl->f4_ref_speed_rad * delta_value,p_instance_ctrl->f4_ref_speed_rad,p_instance_ctrl->f4_speed_rad,p_instance_ctrl->f4_v_ref);
        LOG_D(LOG_STD,"speedSet:%f speedGet:%f  s:%f",speedSet,speedGet);
        LOG_D(LOG_STD,"vref:%f vmax:%f",p_instance_ctrl->f4_v_ref,pcfg->f4_max_drive_v);*/
    }



    return ret;
}

return_t motor_wait_stop(st_motor_t *mot)
{
	return_t ret=X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(mot  != NULL)
#endif

	c_timespan_t ts;
    h_time_update(&ts);
	motor_120_control_wait_stop_flag_t flg_wait_stop = MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET;

    while (MOTOR_120_CONTROL_WAIT_STOP_FLAG_SET == flg_wait_stop)
	{
    	mot->motor_ctrl_instance->p_api->waitStopFlagGet(mot->motor_ctrl_instance->p_ctrl, &flg_wait_stop);
    	bool_t res=FALSE;
    	h_time_is_elapsed_ms(&ts, 1000, &res);
    	if(res == TRUE)
    	{
    	    LOG_E(LOG_STD,"motor_wait_stop timeout");
    	    ERROR_LOG_AND_RETURN(F_RET_MOTOR_STOP_FLAG_TIMEOUT);
    	}
    	h_time_update(&ts);
	}
	return ret;
}

return_t motor_wait_driver_init(st_motor_t *mot)
{
    return_t ret=X_RET_OK;
    c_timespan_t ts;
    h_time_update(&ts);
    uint8_t init_finished = 0;

    while (init_finished == 0)
    {
        mot->motor_ctrl_instance->p_api->driver_init_finished(mot->motor_ctrl_instance->p_ctrl, &init_finished);
        bool_t res=FALSE;
        h_time_is_elapsed_ms(&ts, 1000, &res);
        if(res == TRUE)
        {
            LOG_E(LOG_STD,"motor_wait_driver_init timeout");
            ERROR_LOG_AND_RETURN(F_RET_MOTOR_STOP_FLAG_TIMEOUT);
        }
        h_time_update(&ts);
    }
    return ret;
}


static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value)
{
    gpt_instance_ctrl_t * p_instance_ctrl = (gpt_instance_ctrl_t *) p_ctrl;

    p_instance_ctrl->p_reg->GTPBR = period_counts;          /* Set period to buffer register */
    p_instance_ctrl->p_reg->GTPR = (uint32_t)(value);
}

void return_motor_cplx_update(return_motor_cplx_t *ptr,return_t code)
{
    ptr->code = code;
    ptr->fsp_motorH_error_code = motors_instance.motorH->error;
    ptr->fsp_motorL_error_code = motors_instance.motorL->error;
}


void mtr0_callback_120_degree(motor_callback_args_t * p_args)
{
    motor0.motor_ctrl_instance->p_api->statusGet(motor0.motor_ctrl_instance->p_ctrl,&motor0.status);
    switch (p_args->event)
    {
        case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
        {
            /* Do nothing */


            motor_120_driver_instance_ctrl_t * p_instance_ctrl = (motor_120_driver_instance_ctrl_t *) motor0.motor_driver_instance->p_ctrl;



        }
        break;

        case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
        {
            if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != motor0.status)
            {
                motor0.motor_ctrl_instance->p_api->errorCheck(motor0.motor_ctrl_instance->p_ctrl, &motor0.error);

                if(motor0.error != 0x00)
                    LOG_E(LOG_STD,"Motor0 error 0x%x",motor0.error);

                /*if(motor0.error == 0x80)
                {
                    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
                    motor0.motor_ctrl_instance->p_api->errorCheck(motor0.motor_ctrl_instance->p_ctrl, &motor0.error);
                    volatile motor_120_driver_instance_t *mot_inst = (motor_120_driver_instance_t*)motors_instance.motorH->motor_driver_instance;
                    volatile motor_120_driver_instance_ctrl_t * p_instance_ctrl = (motor_120_driver_instance_ctrl_t *) (mot_inst->p_ctrl);
                    LOG_E(LOG_STD,"VADC0 %f", p_instance_ctrl->f_vdc_ad);

                }*/
            }
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_BACKWARD:
        {
            /* Do nothing */
        }
        break;



        default:
        {
            /* Do nothing */
        }
        break;
    }
} /* End of function mtr_callback_120_degree */



void mtr1_callback_120_degree(motor_callback_args_t * p_args)
{
    motor1.motor_ctrl_instance->p_api->statusGet(motor1.motor_ctrl_instance->p_ctrl,&motor1.status);
    switch (p_args->event)
    {
        case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
        {

            uint16_t current_error_code = motor1.error;
            motor1.motor_ctrl_instance->p_api->errorCheck(motor1.motor_ctrl_instance->p_ctrl, &motor1.error);
            if(motor1.error != current_error_code && motor1.error != 0x00)
            {
                LOG_E(LOG_STD,"Motor1 error 0x%x",motor1.error);

                /*if(motor1.error == 0x80)
                                {
                                    volatile motor_120_driver_instance_t *mot_inst = (motor_120_driver_instance_t*)motors_instance.motorL->motor_driver_instance;
                                    volatile motor_120_driver_instance_ctrl_t * p_instance_ctrl = (motor_120_driver_instance_ctrl_t *) (mot_inst->p_ctrl);
                                    LOG_E(LOG_STD,"VADC1 %f", p_instance_ctrl->f_vdc_ad);

                                }*/

            }

            /*if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != motor1.status)
            {
                motor1.motor_ctrl_instance->p_api->errorCheck(motor1.motor_ctrl_instance->p_ctrl, &motor1.error);

                if(motor0.error != 0x00)
                    LOG_E(LOG_STD,"Motor1 error 0x%x",motor1.error);
            }*/

            //mtr_ics_interrupt_process();
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
        {
            /* Do nothing */
        }
        break;

        case MOTOR_CALLBACK_EVENT_CYCLE_BACKWARD:
        {
            /* Do nothing */
        }
        break;



        default:
        {
            /* Do nothing */
        }
        break;
    }
} /* End of function mtr_callback_120_degree */

