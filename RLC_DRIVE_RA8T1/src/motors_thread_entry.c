#include "motors_thread.h"



void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
        // Desactivation de la tension moteur
        //R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
        // Flag indiquant le défaut
        //flag_overcurrent_vm = TRUE;
    }
}


/* Motors Thread entry function */
void motors_thread_entry(void)
{
    /* TODO: add your own code here */
    while (1)
    {
        tx_thread_sleep (1);
    }
}
