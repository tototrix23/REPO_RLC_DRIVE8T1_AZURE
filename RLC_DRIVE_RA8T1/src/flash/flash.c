/*
 * flash.c
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */
#include "flash.h"
#include "hal_data.h"
#include <_core/c_common.h>
int flash_read_status(uint8_t *st);
int flash_write_and_read(uint8_t *tx_b,uint8_t *rx_b,uint32_t count);
int flash_write_enable(void);
int flash_write_disable(void);
int flash_write_align(ULONG flash_address, ULONG *source, ULONG words);

ULONG spi_sector_buffer[4096];

ospi_b_instance_ctrl_t g_ospi1_ctrl;

static const spi_flash_erase_command_t g_ospi1_erase_command_list[] =
{
#if ((0x2121 > 0) && (4096 > 0))
  { .command = 0x2121, .size = 4096 },
#endif
#if ((0xDCDC > 0) && (262144 > 0))
  { .command = 0xDCDC, .size = 262144 },
#endif
#if (0x6060 > 0)
  { .command = 0x6060, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE },
#endif
        };

static ospi_b_timing_setting_t g_ospi1_timing_settings =
{ .command_to_command_interval = OSPI_B_COMMAND_INTERVAL_CLOCKS_2, .cs_pullup_lag =
          OSPI_B_COMMAND_CS_PULLUP_CLOCKS_NO_EXTENSION,
  .cs_pulldown_lead = OSPI_B_COMMAND_CS_PULLDOWN_CLOCKS_NO_EXTENSION };

#if !(0)

#if (0)
            static const spi_flash_erase_command_t g_ospi0_high_speed_erase_command_list[] =
            {
              #if ((0 > 0) && (4096 > 0))
                {.command = 0,     .size = 4096 },
              #endif
              #if ((0 > 0) && (4096 > 0))
                {.command = 0,      .size = 262144  },
              #endif
              #if (0 > 0)
                {.command = 0,       .size  = SPI_FLASH_ERASE_SIZE_CHIP_ERASE        },
              #endif
            };
             #endif

const ospi_b_xspi_command_set_t g_ospi1_high_speed_command_set =
        { .protocol = SPI_FLASH_PROTOCOL_8D_8D_8D,
          .command_bytes = OSPI_B_COMMAND_BYTES_2,
          .read_command = 0xEEEE,
          .page_program_command = 0x1212,
          .write_enable_command = 0x0606,
          .status_command = 0x0505,
          .read_dummy_cycles = 20,
          .program_dummy_cycles = 0, /* Unused by OSPI Flash */
          .status_dummy_cycles = 3,
#if (0)
                .p_erase_command_list      = g_ospi0_high_speed_erase_command_list,
                .erase_command_list_length = sizeof(g_ospi0_high_speed_erase_command_list)/sizeof(g_ospi0_high_speed_erase_command_list[0]),
            #else
          .p_erase_command_list = NULL, /* Use the erase commands specified in spi_flash_cfg_t */
#endif
        };
#endif

#if OSPI_B_CFG_DOTF_SUPPORT_ENABLE
            extern uint8_t g_ospi_dotf_iv[];
            extern uint8_t g_ospi_dotf_key[];

            static ospi_b_dotf_cfg_t g_ospi_dotf_cfg=
            {
                .key_type       = OSPI_B_DOTF_AES_KEY_TYPE_128,
                .p_start_addr   = (uint32_t *)0x90000000,
                .p_end_addr     = (uint32_t *)0x90001FFF,
                .p_key          = (uint32_t *)g_ospi_dotf_key,
                .p_iv           = (uint32_t *)g_ospi_dotf_iv,
            };
            #endif

static const ospi_b_extended_cfg_t g_ospi1_extended_cfg =
{ .channel = (ospi_b_device_number_t) 0, .data_latch_delay_clocks = 0x08, .p_timing_settings = &g_ospi1_timing_settings,
#if (0)
                .p_xspi_command_set_list                 = ,
                .xspi_command_set_list_length            = 0,
            #else
  .p_xspi_command_set_list = &g_ospi1_high_speed_command_set,
  .xspi_command_set_list_length = 1U,
#endif
  .p_autocalibration_preamble_pattern_addr = (uint8_t*) 0x00,
#if OSPI_B_CFG_DMAC_SUPPORT_ENABLE
                .p_lower_lvl_transfer                    = &RA_NOT_DEFINED,
            #endif
#if OSPI_B_CFG_DOTF_SUPPORT_ENABLE
                .p_dotf_cfg                              = &g_ospi_dotf_cfg,
            #endif
  .read_dummy_cycles = 0,
  .program_dummy_cycles = 0, /* Unused by OSPI Flash */
  .status_dummy_cycles = 0, };
const spi_flash_cfg_t g_ospi1_cfg =
{ .spi_protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,
  .read_mode = SPI_FLASH_READ_MODE_STANDARD, /* Unused by OSPI Flash */
  .address_bytes = SPI_FLASH_ADDRESS_BYTES_3,
  .dummy_clocks = SPI_FLASH_DUMMY_CLOCKS_DEFAULT, /* Unused by OSPI Flash */
  .page_program_address_lines = (spi_flash_data_lines_t) 0U, /* Unused by OSPI Flash */
  .page_size_bytes = 64,
  .write_status_bit = 0,
  .write_enable_bit = 1,
  .page_program_command = 0x12,
  .write_enable_command = 0x06,
  .status_command = 0x05,
  .read_command = 0x13,
#if OSPI_B_CFG_XIP_SUPPORT_ENABLE
                .xip_enter_command           = 0,
                .xip_exit_command            = 0,
            #else
  .xip_enter_command = 0U,
  .xip_exit_command = 0U,
#endif
  .erase_command_list_length = sizeof(g_ospi1_erase_command_list) / sizeof(g_ospi1_erase_command_list[0]),
  .p_erase_command_list = &g_ospi1_erase_command_list[0],
  .p_extend = &g_ospi1_extended_cfg, };
/** This structure encompasses everything that is needed to use an instance of this interface. */
const spi_flash_instance_t g_ospi1 =
{ .p_ctrl = &g_ospi1_ctrl, .p_cfg = &g_ospi1_cfg, .p_api = &g_spi_b_on_spi_flash, };
rm_levelx_nor_spi_instance_ctrl_t g_rm_levelx_nor_spi1_ctrl;

#define RA_NOT_DEFINED 0xFFFFFFFF
rm_levelx_nor_spi_cfg_t g_rm_levelx_nor_spi1_cfg =
{
#if (RA_NOT_DEFINED != RA_NOT_DEFINED)
    .p_lower_lvl        = &RA_NOT_DEFINED,
    .base_address       = BSP_FEATURE_QSPI_DEVICE_START_ADDRESS,
#elif (RA_NOT_DEFINED != RA_NOT_DEFINED)
    .p_lower_lvl        = &RA_NOT_DEFINED,
    .base_address       = BSP_FEATURE_OSPI_DEVICE_RA_NOT_DEFINED_START_ADDRESS,
#else
  .p_lower_lvl = &g_ospi1,
  .base_address = 0x00,//BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS,
#endif
  .address_offset = 0,
  .size = 4194304, .poll_status_count = 0xFFFFFFFF, .p_context = &g_rm_filex_levelx_nor1_ctrl, .p_callback =
          rm_filex_levelx_nor_spi_callback };
#undef RA_NOT_DEFINED

#ifndef LX_DIRECT_READ
#define FSP_LX_READ_BUFFER_SIZE_WORDS (128U)
ULONG g_rm_levelx_nor_spi1_read_buffer[FSP_LX_READ_BUFFER_SIZE_WORDS] =
{ 0 };
#endif

/** WEAK system error call back */
#if defined(__ICCARM__)
#define g_rm_levelx_nor_spi0_system_error_WEAK_ATTRIBUTE
#pragma weak g_rm_levelx_nor_spi1_system_error  = g_rm_levelx_nor_spi1_system_error_internal
#elif defined(__GNUC__)
#define g_rm_levelx_nor_spi1_system_error_WEAK_ATTRIBUTE   \
        __attribute__ ((weak, alias("g_rm_levelx_nor_spi1_system_error_internal")))
#endif

UINT g_rm_levelx_nor_spi1_system_error(UINT error_code)
g_rm_levelx_nor_spi1_system_error_WEAK_ATTRIBUTE;
/*****************************************************************************************************************//**
 * @brief      This is a weak example initialization error function.  It should be overridden by defining a user  function
 *             with the prototype below.
 *             - void g_rm_levelx_nor_spi0_system_error(UINT error_code)
 *
 * @param[in]  error_code represents the error that occurred.
 **********************************************************************************************************************/
UINT g_rm_levelx_nor_spi1_system_error_internal(UINT error_code);
UINT g_rm_levelx_nor_spi1_system_error_internal(UINT error_code)
{
    FSP_PARAMETER_NOT_USED (error_code);

    /** An error has occurred. Please check function arguments for more information. */
    BSP_CFG_HANDLE_UNRECOVERABLE_ERROR (0);

    return LX_ERROR;
}

/* LevelX NOR instance "Read Sector" service */
static UINT g_rm_levelx_nor_spi1_read(ULONG *flash_address, ULONG *destination, ULONG words);
static UINT g_rm_levelx_nor_spi1_read(ULONG *flash_address, ULONG *destination, ULONG words)
{
    fsp_err_t err;
    //err = RM_LEVELX_NOR_SPI_Read (&g_rm_levelx_nor_spi1_ctrl, flash_address, destination, words);
    err = flash_read(flash_address, destination, words);
    if (err != 0x00)
    {
        return LX_ERROR;
    }

    return LX_SUCCESS;
}

/* LevelX NOR instance "Write Sector" service */
static UINT g_rm_levelx_nor_spi1_write(ULONG *flash_address, ULONG *source, ULONG words);
static UINT g_rm_levelx_nor_spi1_write(ULONG *flash_address, ULONG *source, ULONG words)
{
    fsp_err_t err;

    //err = RM_LEVELX_NOR_SPI_Write (&g_rm_levelx_nor_spi1_ctrl, flash_address, source, words);
    err = flash_write(flash_address, source, words);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }

    return LX_SUCCESS;
}

/* LevelX NOR instance "Block Erase" service */
static UINT g_rm_levelx_nor_spi1_block_erase(ULONG block, ULONG block_erase_count);
static UINT g_rm_levelx_nor_spi1_block_erase(ULONG block, ULONG block_erase_count)
{
    fsp_err_t err;

    err = RM_LEVELX_NOR_SPI_BlockErase (&g_rm_levelx_nor_spi1_ctrl, block, block_erase_count);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }

    return LX_SUCCESS;
}

/* LevelX NOR instance "Block Erased Verify" service */
static UINT g_rm_levelx_nor_spi1_block_erased_verify(ULONG block);
static UINT g_rm_levelx_nor_spi1_block_erased_verify(ULONG block)
{
    fsp_err_t err;

    err = RM_LEVELX_NOR_SPI_BlockErasedVerify (&g_rm_levelx_nor_spi1_ctrl, block);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }

    return LX_SUCCESS;
}

/* LevelX NOR instance "Driver Initialization" service */
UINT g_rm_levelx_nor_spi1_initialize(LX_NOR_FLASH *p_nor_flash)
{
    fsp_err_t err;

    g_rm_levelx_nor_spi1_cfg.p_lx_nor_flash = p_nor_flash;

    /* Open the rm_levelx_nor_spi driver */

    /*err = R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }*/

#ifndef LX_DIRECT_READ
    /** lx_nor_flash_sector_buffer is used only when LX_DIRECT_READ disabled */
    p_nor_flash->lx_nor_flash_sector_buffer = &spi_sector_buffer[0];
#endif

    p_nor_flash->lx_nor_flash_driver_read = g_rm_levelx_nor_spi1_read;
    p_nor_flash->lx_nor_flash_driver_write = g_rm_levelx_nor_spi1_write;
    p_nor_flash->lx_nor_flash_driver_block_erase = g_rm_levelx_nor_spi1_block_erase;
    p_nor_flash->lx_nor_flash_driver_block_erased_verify = g_rm_levelx_nor_spi1_block_erased_verify;
    p_nor_flash->lx_nor_flash_driver_system_error = g_rm_levelx_nor_spi1_system_error;

    err = RM_LEVELX_NOR_SPI_Open (&g_rm_levelx_nor_spi1_ctrl, &g_rm_levelx_nor_spi1_cfg);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }

    return LX_SUCCESS;
}

/* LevelX NOR instance "Driver Close" service */
fsp_err_t g_rm_levelx_nor_spi1_close()
{
    //return RM_LEVELX_NOR_SPI_Close (&g_rm_levelx_nor_spi1_ctrl);
    return R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
}
LX_NOR_FLASH g_lx_nor1;
rm_filex_levelx_nor_instance_ctrl_t g_rm_filex_levelx_nor1_ctrl;

const rm_filex_levelx_nor_cfg_t g_rm_filex_levelx_nor1_cfg =
{ .close = g_rm_levelx_nor_spi1_close, .nor_driver_initialize = g_rm_levelx_nor_spi1_initialize, .p_nor_flash =
          &g_lx_nor1,
  .p_nor_flash_name = "g_rm_filex_levelx_nor_1", .p_callback = g_rm_filex_levelx_nor_1_callback, .p_context = NULL };

const rm_filex_levelx_nor_instance_t g_rm_filex_levelx_nor1_instance =
{ .p_ctrl = &g_rm_filex_levelx_nor1_ctrl, .p_cfg = &g_rm_filex_levelx_nor1_cfg };






void g_rm_filex_levelx_nor_1_callback(rm_filex_levelx_nor_callback_args_t *p_args)
{
    if (p_args->event == RM_FILEX_LEVELX_NOR_EVENT_BUSY)
    {
        /* Put the thread to sleep while waiting for operation to complete. */
        tx_thread_sleep(1);
    }
}


void g_rm_filex_levelx_nor_0_callback(rm_filex_levelx_nor_callback_args_t *p_args)
{
    if (p_args->event == RM_FILEX_LEVELX_NOR_EVENT_BUSY)
    {
        /* Put the thread to sleep while waiting for operation to complete. */
        tx_thread_sleep(1);
    }
}


volatile bool g_transfer_complete = false;

void sci_b_spi_lfs_callback (spi_callback_args_t * p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_transfer_complete = true;
    }
    else
    {
        volatile ULONG xxx = p_args->event;
        xxx=0;
        xxx=0;
    }
}


int flash_read_status(uint8_t *st)
{
    int ret = 0;
    uint8_t cmd_buffer[2];
    uint8_t in_buffer[2];
    cmd_buffer[0] = 0x05;
    cmd_buffer[1] = 0xFF;

    ret = flash_write_and_read(cmd_buffer,in_buffer,2);
    if(ret != 0) return ret;
    *st = in_buffer[1];
    return 0;
}

int flash_write_and_read(uint8_t *tx_b,uint8_t *rx_b,uint32_t count)
{
    fsp_err_t err = FSP_SUCCESS;
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    err = R_SCI_B_SPI_WriteRead(&g_sci_spi_lfs_ctrl, tx_b,rx_b, count, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    return -1;
}

int flash_write_enable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x06;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}

int flash_write_disable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x04;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}

int flash_read(ULONG *flash_address, ULONG *destination, ULONG words)
{
    fsp_err_t err = FSP_SUCCESS;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    ULONG *dest = destination;
    ULONG addr = (ULONG)flash_address;

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = 0x03;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);


    uint8_t tab[4];
    ULONG i = 0;

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;
    for(i=0;i<words;i++)
    {
        g_transfer_complete = false;
        err = R_SCI_B_SPI_Read(&g_sci_spi_lfs_ctrl, tab, 4, SPI_BIT_WIDTH_8_BITS);
        if(err != FSP_SUCCESS) goto error;
        while (false == g_transfer_complete);

        ULONG value = (ULONG)(tab[0]<<24);
        value += (ULONG)(tab[1]<<16);
        value += (ULONG)(tab[2]<<8);
        value += (ULONG)(tab[3]);

        *dest = value;
        dest++;


    }


    g_transfer_complete = false;

    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    tx_mutex_put(&g_mutex_spi);
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    tx_mutex_put(&g_mutex_spi);
    return -1;
}


int flash_write(ULONG *flash_address, ULONG *source, ULONG words)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);

    fsp_err_t err = FSP_SUCCESS;

    volatile uint8_t tab[4];
    ULONG i = 0;

    volatile ULONG addr = (ULONG)flash_address;
    ULONG *src = source;

    volatile ULONG words_operation = 0;
    volatile ULONG words_count=0;
    bool_t end = FALSE;
    do
    {
        volatile ULONG sector_start = (addr + (words_count * 4))/4096;
        volatile ULONG words_left = (words - words_count);

        if(words_left > 64) words_operation = 64;
        else words_operation = words_left;


        volatile ULONG sector_end = (addr + (words_operation * 4))/4096;

        if(sector_start != sector_end)
        {
             volatile ULONG stop = 0;
             stop = 0;
             stop = 1;
             stop = 0;
              stop = 1;

        }
        else
        {
            err = flash_write_align(addr,source+words_count,words_operation);
            if(err != 0)
            {
                goto error;
            }
        }

        addr += (words_operation*4);
        words_count += words_operation;
        if(words_count == words) end = TRUE;

    }while(!end);

    return 0;


    error:

    tx_mutex_put(&g_mutex_spi);
    return -1;
}


int flash_write_align(ULONG flash_address, ULONG *source, ULONG words)
{
    int ret = 0;
    ret = flash_write_enable();
    if(ret != 0)
    {
        return ret;
    }

    fsp_err_t err = FSP_SUCCESS;

    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    volatile uint8_t tab[4];
    ULONG i = 0;

    volatile ULONG addr = (ULONG)flash_address;
    ULONG *src = source;

    uint8_t cmd_buffer[4];

    cmd_buffer[0] = 0x02;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;

    for(i=0;i<words;i++)
    {

        ULONG value = *src;
        tab[0] = (uint8_t)(value>>24);
        tab[1] = (uint8_t)(value>>16);
        tab[2] = (uint8_t)(value>>8);
        tab[3] = (uint8_t)(value & 0x000000FF);


        g_transfer_complete = false;
        err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, tab, 4, SPI_BIT_WIDTH_8_BITS);
        if(err != FSP_SUCCESS) goto error;
        __NOP();
        __NOP();
        while (false == g_transfer_complete);
        g_transfer_complete = false;




        src++;
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    __NOP();
    __NOP();

    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
            return ret;
        }
        if((st & 0x02) == 0x00)
        {
            return 0;
        }
        __NOP();
        __NOP();
    }

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    return -1;
}



int flash_erase(ULONG block)
{

    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    ret = flash_write_enable();
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    ULONG addr = (ULONG)((block * 4096));
    uint8_t cmd_buffer[4];
    uint8_t in_buffer[4];

    cmd_buffer[0] = 0x20;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    ret = flash_write_and_read(cmd_buffer,in_buffer,4);
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    __NOP();
    __NOP();

    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
           tx_mutex_put(&g_mutex_spi);
           return ret;
        }
        if((st & 0x02) == 0x00)
        {
           tx_mutex_put(&g_mutex_spi);
           return 0;
        }
    }
    return 0;
}


int flash_erase_chip(void)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    ret = flash_write_enable();
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0xC7;
    ret = flash_write_and_read(cmd_buffer,in_buffer,1);
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    volatile uint32_t tempo = 1000;
    while(tempo>0)tempo--;

    while(1)
    {
        uint8_t st = 0;
        ret = flash_read_status(&st);
        if(ret != 0)
        {
           tx_mutex_put(&g_mutex_spi);
           return ret;
        }
        if((st & 0x02) == 0x00)
        {
           tx_mutex_put(&g_mutex_spi);
           return 0;
        }
    }
    return 0;
}

int flash_open(void)
{
    return R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
}
int flash_close(void)
{
    return R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
}
