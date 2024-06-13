/*
 * lfs_impl.c
 *
 *  Created on: 6 juin 2024
 *      Author: Christophe
 */

#include "lfs_impl.h"

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "lfs_impl"

void sci_b_spi_lfs_callback (spi_callback_args_t * p_args);
int LFS_WriteRead(uint8_t *tx_b,uint8_t *rx_b,uint32_t count);
int LFS_WriteEnable(void);
int LFS_WriteDisable(void);
int LFS_ReadStatus(uint8_t *st);

const char dir_payloads[] = "/PAYLOADS";
const char dir_events[] = "/EVENTS";
const char dir_firmware[] = "/FIRMWARE";


bool_t lfs_init_success = FALSE;
lfs_t lfs;

const struct lfs_config cfg = {
    // block device operations
    .read  = LFS_ImplRead,//user_provided_block_device_read,
    .prog  = LFS_ImplProg,//user_provided_block_device_prog,
    .erase = LFS_ImplErase,//user_provided_block_device_erase,
    .sync  = LFS_ImplSync,//user_provided_block_device_sync,

    // block device configuration
    .read_size = 256,//16,
    .prog_size = 256,//16,
    .block_size = 4096,
    .block_count = 1024,
    .cache_size = 256,//16,
    .lookahead_size = 256,//16,
    .block_cycles = 500,
};


static volatile bool g_transfer_complete = false;

void sci_b_spi_lfs_callback (spi_callback_args_t * p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_transfer_complete = true;
    }
}

int LFS_Init(void)
{



    lfs_init_success = FALSE;
    int err = lfs_mount(&lfs, &cfg);

    if (err) {
        err = lfs_format(&lfs, &cfg);
        err = lfs_mount(&lfs, &cfg);
        if(err)
        {

            LOG_E(LOG_STD,"Error mounting LFS");
            LFS_DeInit();
            return err;
        }
    }

    lfs_dir_t dir;
    err = lfs_dir_open(&lfs,&dir,dir_payloads);
    if(err)
    {
        err = lfs_mkdir(&lfs,dir_payloads);
        if(err)
        {

            LOG_E(LOG_STD,"Error creating 'payloads' directory");
            LFS_DeInit();
            return err;
        }
    }
    lfs_dir_close(&lfs,&dir);

    err = lfs_dir_open(&lfs,&dir,dir_events);
    if(err)
    {
        err = lfs_mkdir(&lfs,dir_events);
        if(err)
        {

            LOG_E(LOG_STD,"Error creating 'events' directory");
            LFS_DeInit();
            return err;
        }
    }
    lfs_dir_close(&lfs,&dir);

    err = lfs_dir_open(&lfs,&dir,dir_firmware);
    if(err)
    {
        err = lfs_mkdir(&lfs,dir_firmware);
        if(err)
        {
            tx_mutex_put(&g_mutex_spi);
            LOG_E(LOG_STD,"Error creating 'firmware' directory");
            LFS_DeInit();
            return err;
        }
    }
    lfs_dir_close(&lfs,&dir);

    lfs_init_success = TRUE;

    return 0;
}


int LFS_ParseFolders(char *dir)
{
   if(lfs_init_success != TRUE) return -1;

   volatile int err = 0;
   lfs_dir_t lfs_dir;
   err = lfs_dir_open(&lfs,&lfs_dir,dir);
   if(err < 0)
   {
       LOG_E(LOG_STD,"Error Opening '%s' folder",dir);
       return err;
   }


   LOG_I(LOG_STD,"'%s' folder",dir);

   struct lfs_info dir_info;
   do{
      err = lfs_dir_read(&lfs,&lfs_dir,&dir_info);
      if(err == 1)
      {
          if(dir_info.type == LFS_TYPE_REG)
          {
              LOG_I(LOG_STD,"--- '%s' (%lu bytes)",dir_info.name,dir_info.size);
          }
          else if(dir_info.type == LFS_TYPE_DIR)
          {

          }
      }
   }while(err > 0);

   lfs_dir_close(&lfs,&lfs_dir);
   volatile int i=0;


}

int LFS_DeInit(void)
{
    int err = lfs_unmount(&lfs);
    if(err)
    {
        LOG_E(LOG_STD,"Error unmounting LFS");
        return err;
    }
    return 0;
}

int LFS_WriteRead(uint8_t *tx_b,uint8_t *rx_b,uint32_t count)
{

    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    if(err != FSP_SUCCESS) return -1;
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    err = R_SCI_B_SPI_WriteRead(&g_sci_spi_lfs_ctrl, tx_b,rx_b, count, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    return -1;
}

int LFS_ReadStatus(uint8_t *st)
{
    int ret = 0;
    uint8_t cmd_buffer[2];
    uint8_t in_buffer[2];
    cmd_buffer[0] = 0x05;
    cmd_buffer[1] = 0xFF;

    ret = LFS_WriteRead(cmd_buffer,in_buffer,2);
    if(ret != 0) return ret;
    *st = in_buffer[1];
    return 0;
}

int LFS_WriteEnable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x06;
    ret = LFS_WriteRead(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}

int LFS_WriteDisable(void)
{
    int ret = 0;
    uint8_t cmd_buffer[1];
    uint8_t in_buffer[1];
    cmd_buffer[0] = 0x04;
    ret = LFS_WriteRead(cmd_buffer,in_buffer,1);
    if(ret != 0) return ret;
    return 0;
}







int LFS_ImplRead(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size)
{
    fsp_err_t err = FSP_SUCCESS;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    err = R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    if(err != FSP_SUCCESS)
    {
        tx_mutex_put(&g_mutex_spi);
        return -1;
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    uint32_t addr = (uint32_t)((c->block_size * block) + off);

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = 0x03;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;
    err = R_SCI_B_SPI_Read(&g_sci_spi_lfs_ctrl, buffer, size, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;

    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return 0;

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return -1;
}

int LFS_ImplProg(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size)
{
    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    ret = LFS_WriteEnable();
    if(ret != 0)
    {
        tx_mutex_put(&g_mutex_spi);
        return ret;
    }

    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_Open(&g_sci_spi_lfs_ctrl, &g_sci_spi_lfs_cfg);
    if(err != FSP_SUCCESS)
    {
        tx_mutex_put(&g_mutex_spi);
        return -1;
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_LOW);
    g_transfer_complete = false;

    uint32_t addr = (uint32_t)((c->block_size * block) + off);

    uint8_t cmd_buffer[4];

    cmd_buffer[0] = 0x02;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, cmd_buffer, 4, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;


    err = R_SCI_B_SPI_Write(&g_sci_spi_lfs_ctrl, buffer, size, SPI_BIT_WIDTH_8_BITS);
    if(err != FSP_SUCCESS) goto error;
    while (false == g_transfer_complete);
    g_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    volatile uint32_t tempo = 1000;
    while(tempo>0)tempo--;

    while(1)
    {
        uint8_t st = 0;
        ret = LFS_ReadStatus(&st);
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

    error:
    R_IOPORT_PinWrite(&g_ioport_ctrl, IO_FLASH_CS, BSP_IO_LEVEL_HIGH);
    err = R_SCI_B_SPI_Close(&g_sci_spi_lfs_ctrl);
    tx_mutex_put(&g_mutex_spi);
    return -1;


    return 0;
}


int LFS_ImplErase(const struct lfs_config *c, lfs_block_t block)
{

    int ret = 0;
    tx_mutex_get(&g_mutex_spi,TX_WAIT_FOREVER);
    uint32_t addr = (uint32_t)((c->block_size * block));
    uint8_t cmd_buffer[4];
    uint8_t in_buffer[4];

    cmd_buffer[0] = 0x20;
    cmd_buffer[1] = (uint8_t)(addr>>16);
    cmd_buffer[2] = (uint8_t)(addr>>8);
    cmd_buffer[3] = (uint8_t)(addr & 0x00FF);

    ret = LFS_WriteRead(cmd_buffer,in_buffer,4);
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
        ret = LFS_ReadStatus(&st);
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

int LFS_ImplSync(const struct lfs_config *c)
{

    return 0;
}

