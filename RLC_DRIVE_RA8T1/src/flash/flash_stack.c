/*
 * flash.c
 *
 *  Created on: 16 oct. 2024
 *      Author: Christophe
 */
#include "flash_block_media_api.h"
#include "hal_data.h"
#include <_core/c_common.h>
#include <flash/flash_stack.h>







ospi_b_instance_ctrl_t g_flash_ctrl;

static const spi_flash_erase_command_t g_flash_erase_command_list[] =
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

static ospi_b_timing_setting_t g_flash_timing_settings =
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

const ospi_b_xspi_command_set_t g_flash_high_speed_command_set =
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

static const ospi_b_extended_cfg_t g_flash_extended_cfg =
{ .channel = (ospi_b_device_number_t) 0, .data_latch_delay_clocks = 0x08, .p_timing_settings = &g_flash_timing_settings,
#if (0)
                .p_xspi_command_set_list                 = ,
                .xspi_command_set_list_length            = 0,
            #else
  .p_xspi_command_set_list = &g_flash_high_speed_command_set,
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
const spi_flash_cfg_t g_flash_cfg =
{ .spi_protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,
  .read_mode = SPI_FLASH_READ_MODE_STANDARD, /* Unused by OSPI Flash */
  .address_bytes = SPI_FLASH_ADDRESS_BYTES_4,
  .dummy_clocks = SPI_FLASH_DUMMY_CLOCKS_DEFAULT, /* Unused by OSPI Flash */
  .page_program_address_lines = (spi_flash_data_lines_t) 0U, /* Unused by OSPI Flash */
  .page_size_bytes = 256,
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
  .erase_command_list_length = sizeof(g_flash_erase_command_list) / sizeof(g_flash_erase_command_list[0]),
  .p_erase_command_list = &g_flash_erase_command_list[0],
  .p_extend = &g_flash_extended_cfg, };
/** This structure encompasses everything that is needed to use an instance of this interface. */
const spi_flash_instance_t g_flash =
{ .p_ctrl = &g_flash_ctrl, .p_cfg = &g_flash_cfg, .p_api = &g_spi_b_on_spi_flash, };
rm_block_media_spi_instance_ctrl_t g_rm_block_media1_ctrl;

#define RA_NOT_DEFINED 0xFFFFFFFF
static const rm_block_media_spi_extended_cfg_t g_rm_block_media1_extended_cfg =
{
#if (RA_NOT_DEFINED != RA_NOT_DEFINED)
                .p_spi             = &RA_NOT_DEFINED,
                .base_address      = BSP_FEATURE_QSPI_DEVICE_START_ADDRESS + 0,
            #elif (RA_NOT_DEFINED != RA_NOT_DEFINED)
                .p_spi             = &RA_NOT_DEFINED,
                .base_address      = BSP_FEATURE_OSPI_DEVICE_RA_NOT_DEFINED_START_ADDRESS + 0,
            #else
  .p_spi = &g_flash,
  .base_address = 0,//BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS + 0,
#endif
  .block_count_total = 1024,
  .block_size_bytes = 4096, };
#undef RA_NOT_DEFINED

const rm_block_media_cfg_t g_rm_block_media1_cfg =
{ .p_callback = rm_filex_block_media_memory_callback, .p_context = &g_rm_filex_block_media_1_ctrl, .p_extend =
          &g_rm_block_media1_extended_cfg };
rm_block_media_instance_t g_rm_block_media1 =
{ .p_ctrl = &g_rm_block_media1_ctrl,

.p_cfg = &g_rm_block_media1_cfg,
  .p_api = &g_rm_block_media_on_spi_custom };//g_rm_block_media_on_spi_custom//g_rm_block_media_on_spi
rm_filex_block_media_instance_ctrl_t g_rm_filex_block_media_1_ctrl;

const rm_filex_block_media_cfg_t g_rm_filex_block_media_1_cfg =
{ .p_lower_lvl_block_media = (rm_block_media_instance_t*) &g_rm_block_media1, .partition =
          RM_FILEX_BLOCK_MEDIA_PARTITION0,
  .p_callback = g_rm_filex_block_media_1_callback };

const rm_filex_block_media_instance_t g_rm_filex_block_media_1_instance =
{ .p_ctrl = &g_rm_filex_block_media_1_ctrl, .p_cfg = &g_rm_filex_block_media_1_cfg, .p_api = &g_filex_on_block_media };



void g_rm_filex_block_media_0_callback(rm_filex_block_media_callback_args_t *p_args)
{

}
void g_rm_filex_block_media_1_callback(rm_filex_block_media_callback_args_t *p_args)
{

    switch (p_args->event)
    {
        /*case RM_BLOCK_MEDIA_EVENT_MEDIA_INSERTED:
         {
         tx_event_flags_set(&my_event_flags_group, RM_BLOCK_MEDIA_EVENT_MEDIA_INSERTED, TX_OR);
         }
         break;
         case RM_BLOCK_MEDIA_EVENT_MEDIA_REMOVED:
         {
         tx_event_flags_set(&my_event_flags_group, RM_BLOCK_MEDIA_EVENT_MEDIA_REMOVED, TX_OR);
         }
         break;
         case RM_BLOCK_MEDIA_EVENT_WAIT_END:
         {

         tx_event_flags_set(&my_event_flags_group, RM_BLOCK_MEDIA_EVENT_WAIT_END, TX_OR);
         }*/
        break;
        case RM_BLOCK_MEDIA_EVENT_POLL_STATUS:
        case RM_BLOCK_MEDIA_EVENT_MEDIA_SUSPEND:
        case RM_BLOCK_MEDIA_EVENT_MEDIA_RESUME:
        case RM_BLOCK_MEDIA_EVENT_WAIT:
        default:
        {
            break;
        }
    }
}









