#include "motors_thread.h"
#include <remotectrl/remotectrl.h>

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
motor_wait_stop_flag_t g_wait_flag;


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
    switch (p_args->event)
        {
            case MOTOR_CALLBACK_EVENT_ADC_FORWARD:
            {
                /* Do nothing */
            }
            break;

            case MOTOR_CALLBACK_EVENT_ADC_BACKWARD:
            {
                if (MOTOR_120_DEGREE_CTRL_STATUS_ERROR != g_u1_motor_status)
                {
                    //mtr_adc_remove_spike();
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

    /* POEG */
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);

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

} /* End of function motor_fsp_init */


static void board_ui(void)
{
    uint8_t u1_temp_sw_signal;

    motor_wait_stop_flag_t u1_temp_flg_wait_stop = MOTOR_WAIT_STOP_FLAG_SET;

    /* Get status of motor control system */
    g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &g_u1_motor_status);
    switch (g_u1_motor_status)
    {
        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:


            /* Check SW1 */
            if (m12_auto == 0)
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

/* Motors Thread entry function */
void motors_thread_entry(void)
{

    motor_fsp_init();
    software_init();                              /* Initialize private global variables */

    /* Execute reset event */
    g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);


   /* while(1)
    {
        g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &g_u1_motor_status);
        g_motor_120_degree0.p_api->stop(g_motor_120_degree0.p_ctrl);
        g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);
        TEMPO_S(1);
        //g_motor_120_degree0.p_api->speedSet(g_motor_120_degree0.p_ctrl, 500.0f);
        TEMPO_MS(100);
        g_motor_120_degree0.p_api->run(g_motor_120_degree0.p_ctrl);
        TEMPO_MS(100);
        g_motor_120_degree0.p_api->speedSet(g_motor_120_degree0.p_ctrl, 500.0f);
        TEMPO_S(1);

    }
*/

    /* TODO: add your own code here */
    while (1)
    {
        board_ui();
        tx_thread_sleep (1);
    }
}
