#include "motors_thread.h"
#include <remotectrl/remotectrl.h>
#include <_core/c_common.h>
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include <_hal/h_log/h_log.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main thread"


float       g_f4_speed_ref = 0.0F;
uint8_t     g_u1_motor_status;            /* Motor status */
uint8_t     com_u1_sw_userif;             /* User interface switch */
uint8_t     g_u1_sw_userif;               /* User interface switch */
uint8_t     com_u1_mode_system;           /* System mode */
uint8_t     g_u1_mode_system;             /* System mode */
float       g_f4_max_speed_rpm;
uint8_t     g_u1_stop_req;
uint16_t    g_u2_chk_error;
uint16_t    g_u2_vr1_ad;
uint16_t    g_u2_conf_hw;
uint16_t    g_u2_conf_sw;
uint16_t    g_u2_conf_tool;
uint8_t     g_u1_conf_motor_type_len;
uint8_t     g_u1_conf_control_len;
uint8_t     g_u1_conf_inverter_len;
uint8_t     g_u1_reset_req;                       /* Reset request flag */
uint8_t     g_u1_sw_cnt;                          /* Counter to remove chattering */
uint8_t     g_u1_oc_u_cnt;
uint8_t     g_u1_oc_w_cnt;

motor_cfg_t g_user_motor_cfg;
motor_120_degree_extended_cfg_t g_user_motor_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor_120_driver_extended_cfg;

uint8_t     g_u1_motor1_status;
uint16_t    g_u2_chk_error1;
uint8_t     g_u1_reset_req1;

volatile motor_120_control_cfg_t *ptr;
motor_cfg_t g_user_motor1_cfg;
motor_120_degree_extended_cfg_t g_user_motor1_120_degree_extended_cfg;
motor_120_control_cfg_t g_user_motor1_120_control_cfg;
motor_120_control_hall_extended_cfg_t g_user_motor1_120_control_extended_cfg;
motor_120_driver_cfg_t g_user_motor1_120_driver_cfg;
motor_120_driver_extended_cfg_t g_user_motor1_120_driver_extended_cfg;

motor_wait_stop_flag_t g_wait_flag;


float i_mot0 = 0.0f;
float i_mot0_avr = 0.0f;
float i_mot0_max = 0.0f;
float i_mot0_accu = 0.0f;
uint16_t i_cnt = 0;
float save_iu=0.0f;


static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value);
static void mtr_adc_remove_spike(void);

void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
        // Desactivation de la tension moteur
        //R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
        // Flag indiquant le défaut
        //flag_overcurrent_vm = TRUE;

        g_motor_120_degree0.p_api->errorSet(g_motor_120_degree0.p_ctrl, MOTOR_ERROR_OVER_CURRENT_HW);
        g_u2_chk_error |= MOTOR_ERROR_OVER_CURRENT_HW;
    }
}


void mtr0_callback_120_degree(motor_callback_args_t * p_args)
{
    static uint8_t detect = 0;


    switch (p_args->event)
        {
            case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
            {
                /*float f4_temp_iu = g_motor_120_driver0_ctrl.f_iu_ad - g_motor_120_driver0_ctrl.f_offset_iu;
                float f4_temp_iw = g_motor_120_driver0_ctrl.f_iw_ad - g_motor_120_driver0_ctrl.f_offset_iw;

                float i_phase = 0.577350269f * (f4_temp_iu-f4_temp_iw);
                if(i_phase < 0.0f)
                    i_phase = i_phase*-1.0f;


                //MOTOR_120_DRIVER_PHASE_PATTERN_UP_PWM_VN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_VP_ON_UN_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_VP_PWM_WN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_WP_ON_VN_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_WP_PWM_UN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_UP_ON_WN_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_VP_ON_W_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_W_PWM_VN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_WP_ON_U_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_U_PWM_WN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_UP_ON_V_PWM
                //MOTOR_120_DRIVER_PHASE_PATTERN_V_PWM_UN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_U_PWM_VN_ON

                //MOTOR_120_DRIVER_PHASE_PATTERN_W_PWM_UN_ON
                //MOTOR_120_DRIVER_PHASE_PATTERN_VP_ON_U_PWM
                if(g_motor_120_driver0_ctrl.pattern == MOTOR_120_DRIVER_PHASE_PATTERN_W_PWM_UN_ON)
                       // g_motor_120_driver0_ctrl.pattern == MOTOR_120_DRIVER_PHASE_PATTERN_VP_ON_U_PWM
                   //MOTOR_120_DRIVER_PHASE_PATTERN_W_PWM_UN_ON)//MOTOR_120_DRIVER_PHASE_PATTERN_VP_ON_U_PWM
                {

                    detect = 1;
                    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_LED_ERROR, 1);




                    if(i_phase > i_mot0_max)
                        i_mot0_max = i_phase;

                }
                else
                {
                    if(detect == 1)
                    {

                        detect = 0;
                        //i_mot0 = i_mot0_accu/i_cnt;
                        i_mot0 = i_mot0_max;


                        i_mot0_max = 0.0f;

                        i_mot0_accu += i_mot0;
                        i_cnt++;
                        if(i_cnt >= 100)
                        {

                            i_mot0_avr = i_mot0_accu / i_cnt;
                            i_mot0_accu = 0.0f;
                            i_cnt = 0x00;
                        }
                    }
                    R_IOPORT_PinWrite (&g_ioport_ctrl, IO_LED_ERROR, 0);
                }*/




                /* Do nothing */





            }
            break;

            case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
            {
                if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != g_u1_motor_status)
                {
                    mtr_adc_remove_spike();
                    g_motor_120_degree0.p_api->errorCheck(g_motor_120_degree0.p_ctrl, &g_u2_chk_error);
                }

            }
            break;

            case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
            {
                /* Do nothing */
                //g_u2_vr1_ad = get_vr1();
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
    switch (p_args->event)
        {
            case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
            {

            }
            break;

            case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
            {
                if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != g_u1_motor1_status)
                {
                    //mtr_adc_remove_spike();
                    g_motor_120_degree1.p_api->errorCheck(g_motor_120_degree1.p_ctrl, &g_u2_chk_error1);
                }

            }
            break;

            case MOTOR_CALLBACK_EVENT_CYCLE_FORWARD:
            {

                //g_u2_vr1_ad = get_vr1();
            }
            break;

            case MOTOR_CALLBACK_EVENT_CYCLE_BACKWARD:
            {

            }
            break;

            default:
            {

            }
            break;
        }
}


static void software_init(void)
{
    g_u1_motor_status            = MOTOR_120_DEGREE_CTRL_STATUS_STOP;
    g_f4_max_speed_rpm           = 1500.0f;//MTR_MAX_SPEED_RPM;
    g_u1_mode_system             = MOTOR_120_DEGREE_CTRL_EVENT_STOP;
    g_u1_reset_req               = 0;
    g_u1_stop_req                = 1;
} /* End of function software_init */

static void motor_fsp_init(void)
{



    /* Motor application (120_degree) */
    g_motor_120_degree0.p_api->open(g_motor_120_degree0.p_ctrl, g_motor_120_degree0.p_cfg);
    g_motor_120_degree1.p_api->open(g_motor_120_degree1.p_ctrl, g_motor_120_degree1.p_cfg);

    //tx_thread_sleep(10);
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

} /* End of function motor_fsp_init */


static void board_ui0(void)
{
    uint8_t u1_temp_sw_signal;

    motor_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_WAIT_STOP_FLAG_SET;

    /* Get status of motor control system */
    g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &g_u1_motor_status);
    switch (g_u1_motor_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:


            /* Check SW1 */
            if (m12_auto == 1)
            {
                while (MOTOR_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
                {
                    /* waiting for motor stop */
                    g_motor_120_degree0.p_api->waitStopFlagGet(g_motor_120_degree0.p_ctrl, &u1_temp_flg_wait_stop);
                }
                g_motor_120_degree0.p_api->run(g_motor_120_degree0.p_ctrl);
            }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:


            /* Check SW1 */
            if (m12_auto == 0)
            {
                g_motor_120_degree0.p_api->stop(g_motor_120_degree0.p_ctrl);
            }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
            /* check SW2 & reset request flag */

            if (m12_enrh == 1 && g_u1_reset_req == 0)
            {
                g_u1_reset_req = 1;
            }
            else if (m12_enrh == 0 && g_u1_reset_req == 1)
            {
                g_u1_reset_req = 0;
                g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);
            }
            else
            {
                /* Do nothing */
            }
        break;

        default:
            /* Do nothing */
        break;
    }

    /*=============================*/
    /*      Set speed reference    */
    /*=============================*/

    g_motor_120_degree0.p_api->speedSet(g_motor_120_degree0.p_ctrl, 2000.0f);



} /* End of function board_ui */


static void board_ui1(void)
{
    uint8_t u1_temp_sw_signal;

    motor_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_WAIT_STOP_FLAG_SET;

    /* Get status of motor control system */
    g_motor_120_degree1.p_api->statusGet(g_motor_120_degree1.p_ctrl, &g_u1_motor1_status);
    switch (g_u1_motor1_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:


            /* Check SW1 */
            if (m12_auto == 1)
            {
                while (MOTOR_WAIT_STOP_FLAG_SET == u1_temp_flg_wait_stop)
                {
                    /* waiting for motor stop */
                    g_motor_120_degree1.p_api->waitStopFlagGet(g_motor_120_degree1.p_ctrl, &u1_temp_flg_wait_stop);
                }
                g_motor_120_degree1.p_api->run(g_motor_120_degree1.p_ctrl);
            }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:


            /* Check SW1 */
            if (m12_auto == 0)
            {
                g_motor_120_degree1.p_api->stop(g_motor_120_degree1.p_ctrl);
            }
        break;

        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
            /* check SW2 & reset request flag */

            if (m12_enrh == 1 && g_u1_reset_req == 0)
            {
                g_u1_reset_req1 = 1;
            }
            else if (m12_enrh == 0 && g_u1_reset_req1 == 1)
            {
                g_u1_reset_req1 = 0;
                g_motor_120_degree1.p_api->reset(g_motor_120_degree1.p_ctrl);
            }
            else
            {
                /* Do nothing */
            }
        break;

        default:
            /* Do nothing */
        break;
    }

    /*=============================*/
    /*      Set speed reference    */
    /*=============================*/

    g_motor_120_degree1.p_api->speedSet(g_motor_120_degree1.p_ctrl, 2000.0f);



} /* End of function board_ui */


void dac_example (void)
{
    fsp_err_t err;
    uint16_t  value;
    /* Pin configuration: Output enable DA0 as Analog. */
    /* Initialize the DAC channel */
    err = R_DAC_Open(&g_dac_ctrl, &g_dac_cfg);
    /* Handle any errors. This function should be defined by the user. */
    assert(FSP_SUCCESS == err);
    value = (uint16_t) 2048;
    err = R_DAC_Write(&g_dac_ctrl, value);
    assert(FSP_SUCCESS == err);
    err = R_DAC_Start(&g_dac_ctrl);
    assert(FSP_SUCCESS == err);
}

/* Motors Thread entry function */
void motors_thread_entry(void)
{
   // dac_example();



    motor_fsp_init();
    tx_thread_sleep(30);
    software_init();                              /* Initialize private global variables */

    /* Execute reset event */
    //g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);

    c_timespan_t ts;
    c_timespan_init(&ts);
    h_time_update(&ts);


    /* TODO: add your own code here */
    while (1)
    {
        board_ui0();

        /*bool_t elasped;
        h_time_is_elapsed_ms(&ts, 200, &elasped);
        if(elasped)
        {
            FSP_CRITICAL_SECTION_DEFINE;
                FSP_CRITICAL_SECTION_ENTER;
            float f4_temp_iu = g_motor_120_driver0_ctrl.f_iu_ad - g_motor_120_driver0_ctrl.f_offset_iu;
            float f4_temp_iw = g_motor_120_driver0_ctrl.f_iw_ad - g_motor_120_driver0_ctrl.f_offset_iw;
            FSP_CRITICAL_SECTION_EXIT;

            LOG_D(LOG_STD,"%f      %f",i_mot0,i_mot0_avr);

            h_time_update(&ts);

        }*/


        board_ui1();
        tx_thread_sleep (1);
    }
}

static void gpt_periodset (timer_ctrl_t * const p_ctrl, uint32_t const period_counts, uint32_t const value)
{
    gpt_instance_ctrl_t * p_instance_ctrl = (gpt_instance_ctrl_t *) p_ctrl;

    p_instance_ctrl->p_reg->GTPBR = period_counts;          /* Set period to buffer register */
    p_instance_ctrl->p_reg->GTPR = (uint32_t)(value);
}


static void mtr_adc_remove_spike(void)
{
    float f4_temp_iu = g_motor_120_driver0_ctrl.f_iu_ad - g_motor_120_driver0_ctrl.f_offset_iu;
    float f4_temp_iw = g_motor_120_driver0_ctrl.f_iw_ad - g_motor_120_driver0_ctrl.f_offset_iw;
    float f4_temp_oc_limit = g_user_motor_120_degree_extended_cfg.f_overcurrent_limit * 0.9F;
    if(g_motor_120_driver0_ctrl.u1_flag_offset_calc != 0)
    {

        if ((f4_temp_iu > f4_temp_oc_limit) || (f4_temp_iu < -f4_temp_oc_limit))
        {
            g_u1_oc_u_cnt++;
            if (g_u1_oc_u_cnt < 128)
            {
                g_motor_120_driver0_ctrl.f_iu_ad = 0.0F;
            }
        }
        else
        {
            g_u1_oc_u_cnt = 0;
        }
        if ((f4_temp_iw > f4_temp_oc_limit) || (f4_temp_iw < -f4_temp_oc_limit))
        {
            g_u1_oc_w_cnt++;
            if (g_u1_oc_w_cnt < 128)
            {
                g_motor_120_driver0_ctrl.f_iw_ad = 0.0F;
            }
        }
        else
        {
            g_u1_oc_w_cnt = 0;
        }

    }
}
