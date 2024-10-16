/*
 * flash.c
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */
#include "flash.h"




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
  .address_bytes = SPI_FLASH_ADDRESS_BYTES_4,
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
  .base_address = BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS,
#endif
  .address_offset = 0,
  .size = 33554432, .poll_status_count = 0xFFFFFFFF, .p_context = &g_rm_filex_levelx_nor1_ctrl, .p_callback =
          rm_filex_levelx_nor_spi_callback };
#undef RA_NOT_DEFINED

#ifndef LX_DIRECT_READ
#define FSP_LX_READ_BUFFER_SIZE_WORDS (128U)
ULONG g_rm_levelx_nor_spi0_read_buffer[FSP_LX_READ_BUFFER_SIZE_WORDS] =
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

    err = RM_LEVELX_NOR_SPI_Read (&g_rm_levelx_nor_spi1_ctrl, flash_address, destination, words);
    if (FSP_SUCCESS != err)
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

    err = RM_LEVELX_NOR_SPI_Write (&g_rm_levelx_nor_spi1_ctrl, flash_address, source, words);
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
    err = RM_LEVELX_NOR_SPI_Open (&g_rm_levelx_nor_spi1_ctrl, &g_rm_levelx_nor_spi1_cfg);
    if (FSP_SUCCESS != err)
    {
        return LX_ERROR;
    }

#ifndef LX_DIRECT_READ
    /** lx_nor_flash_sector_buffer is used only when LX_DIRECT_READ disabled */
    p_nor_flash->lx_nor_flash_sector_buffer = g_rm_levelx_nor_spi1_ReadBuffer;
#endif

    p_nor_flash->lx_nor_flash_driver_read = g_rm_levelx_nor_spi1_read;
    p_nor_flash->lx_nor_flash_driver_write = g_rm_levelx_nor_spi1_write;
    p_nor_flash->lx_nor_flash_driver_block_erase = g_rm_levelx_nor_spi1_block_erase;
    p_nor_flash->lx_nor_flash_driver_block_erased_verify = g_rm_levelx_nor_spi1_block_erased_verify;
    p_nor_flash->lx_nor_flash_driver_system_error = g_rm_levelx_nor_spi1_system_error;

    return LX_SUCCESS;
}

/* LevelX NOR instance "Driver Close" service */
fsp_err_t g_rm_levelx_nor_spi1_close()
{
    return RM_LEVELX_NOR_SPI_Close (&g_rm_levelx_nor_spi1_ctrl);
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


