/*
 * eeprom.c
 *
 *  Created on: 6 nov. 2024
 *      Author: Christophe
 */

#include <exchanged_data/exchanged_data.h>
#include <status/motor_status.h>
#include <i2c/eeprom.h>
#include "i2c_callback.h"

#define EEPROM_HW_ADDR     0x50
#define EEPROM_BLOCK1      0
#define EEPROM_BLOCK2      8
#define EEPROM_WRITE_PAGE  16
#define EEPROM_VAR_SIZE    32
#define EEPROM_VAR_MAX     8


#define CRC16 0x8005

#define EEPROM_PATTERN_SIZE  8
const char eeprom_pattern_ref[EEPROM_PATTERN_SIZE] = "VEE0004";
// Declaration des varibles à entretenir dans l'EEPROM virtuelle
static char eeprom_vee_pattern[EEPROM_PATTERN_SIZE];

typedef struct eeprom_var_t
{
    // Identifiant de la variable dans le tableau.
    uint16_t id;
    // Pointeur vers les données
    void *ptr_data;
    // Taille de la donnnée à stocker.
    uint32_t size;
    // Optionel - permet de faire appel à un mutex de protection
    TX_MUTEX*mutex;
} eeprom_var_t;


eeprom_var_t eeprom_var_array[EEPROM_VAR_COUNT] =
{
 {
  .id = EEPROM_PATTERN,
  .ptr_data = eeprom_vee_pattern,
  .size = sizeof(eeprom_vee_pattern),
  .mutex = NULL
 },
 {
  .id = EEPROM_MOTOR_STATUS,
  .ptr_data = &exchanged_data.motor_status,
  .size = sizeof(st_system_motor_status_t),
  .mutex = &g_exchanged_data_mutex
 }
};



uint16_t gen_crc16(const uint8_t *data, uint16_t size);

return_t eeprom_read(uint16_t addr,uint8_t *dest,uint16_t bytes);
return_t eeprom_write(uint16_t addr,uint8_t *src,uint16_t bytes);

return_t eeprom_read_block_integrity(uint16_t block,uint8_t *dest);
return_t eeprom_write_block_integrity(uint16_t block,uint8_t *src);

return_t eeprom_read_block(uint16_t block,uint8_t *dest,bool_t *crc_valid);
return_t eeprom_write_block(uint16_t block,uint8_t *src,bool_t verify);


uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}



return_t eeprom_init(void)
{
    return_t ret = X_RET_OK;

    tx_mutex_get(&g_mutex_vee,TX_WAIT_FOREVER);

    uint8_t i = 0;

    ret = eeprom_update_from_eeprom(EEPROM_PATTERN);
    if(ret == X_RET_OK)
    {
        char *ptr = strcmp(eeprom_pattern_ref,eeprom_vee_pattern);
        if(ptr != 0x0)
        {
            strcpy(eeprom_vee_pattern,eeprom_pattern_ref);

            for(i=1;i<EEPROM_VAR_COUNT;i++)
            {
                memset(eeprom_var_array[i].ptr_data,0x00,eeprom_var_array[i].size);
                ret = eeprom_update_to_eeprom(i);
                if(ret != X_RET_OK)
                    goto end;
            }

            ret = eeprom_update_to_eeprom(EEPROM_PATTERN);
            if(ret != X_RET_OK)
                    goto end;
        }
        else
        {
            for(i=1;i<EEPROM_VAR_COUNT;i++)
            {
                ret = eeprom_update_from_eeprom(i);
                if(ret != X_RET_OK)
                    goto end;
            }
        }
    }
    end:
    tx_mutex_put(&g_mutex_vee);
    return ret;
}


return_t eeprom_update_from_eeprom(uint16_t id)
{
    return_t ret = X_RET_OK;
    if(id >= EEPROM_VAR_COUNT) return X_RET_PARAM_RANGE;

    uint8_t buffer[EEPROM_VAR_SIZE];
    ret = eeprom_read_block_integrity(id,buffer);
    if(ret == X_RET_OK)
    {
       memcpy(eeprom_var_array[id].ptr_data,buffer,eeprom_var_array[id].size);
    }
    return ret;
}

return_t eeprom_update_to_eeprom(uint16_t id)
{
    return_t ret = X_RET_OK;
    if(id >= EEPROM_VAR_COUNT) return X_RET_PARAM_RANGE;

    uint8_t buffer[EEPROM_VAR_SIZE];
    memset(buffer,0x00,EEPROM_VAR_SIZE);
    memcpy(buffer,eeprom_var_array[id].ptr_data,eeprom_var_array[id].size);
    ret = eeprom_write_block_integrity(id,buffer);
    if(ret == X_RET_OK)
    {
        ret = eeprom_read_block_integrity(id,buffer);
        return ret;
    }
    return ret;
}

return_t eeprom_write_block_integrity(uint16_t block,uint8_t *src)
{
    uint16_t blockA = block;
    uint16_t blockB = block + EEPROM_BLOCK2;
    uint8_t retry=0;
    do
    {
        return_t retB = eeprom_write_block(blockB,(uint8_t*)src,TRUE);
        return_t retA = eeprom_write_block(blockA,(uint8_t*)src,TRUE);

        if(retA == X_RET_OK && retB == X_RET_OK)
        {
           return 0;
        }

        retry++;
        if(retry>5)
        {
            return F_RET_EEPROM;
        }
    }while(1);
}


return_t eeprom_read_block_integrity(uint16_t block,uint8_t *dest)
{
    uint8_t bufferA[EEPROM_VAR_SIZE];
    uint8_t bufferB[EEPROM_VAR_SIZE];

    bool_t crcA=FALSE;
    bool_t crcB=FALSE;
    return_t retA,retB;
    uint8_t retry=0;
    uint16_t blockA = block;
    uint16_t blockB = block + EEPROM_BLOCK2;


    eeprom_start_read_block_integrity:
    do{
        retA = eeprom_read_block(blockA,bufferA,&crcA);
        retB = eeprom_read_block(blockB,bufferB,&crcB);

        if(retA == X_RET_OK && retB == X_RET_OK )
        {
            if(crcA==TRUE && crcB==TRUE)
            {

               uint16_t crcA_value=0;
               uint16_t crcB_value=0;
               crcA_value = bufferA[EEPROM_VAR_SIZE-2];
               crcA_value = (crcA_value<<8);
               crcA_value = crcA_value + bufferA[EEPROM_VAR_SIZE-1];

               crcB_value = bufferB[EEPROM_VAR_SIZE-2];
               crcB_value = (crcB_value<<8);
               crcB_value = crcB_value + bufferA[EEPROM_VAR_SIZE-1];

               if(crcA_value == crcB_value)
               {
                   memcpy(dest,&bufferB,EEPROM_VAR_SIZE);
                   return 0;
               }
               else
               {
                   memcpy(&bufferA,&bufferB,EEPROM_VAR_SIZE);
                   eeprom_write_block(blockA,(uint8_t*)&bufferA,TRUE);
                   goto eeprom_error_read_block_integrity;
               }

            }
            else if(crcA==TRUE && crcB==FALSE)
            {
               memcpy(&bufferB,&bufferA,EEPROM_VAR_SIZE);
               eeprom_write_block(blockB,(uint8_t*)&bufferB,TRUE);
               goto eeprom_error_read_block_integrity;
            }
            else if(crcA==FALSE && crcB==TRUE)
            {
               memcpy(&bufferA,&bufferB,EEPROM_VAR_SIZE);
               eeprom_write_block(blockA,(uint8_t*)&bufferA,TRUE);
               goto eeprom_error_read_block_integrity;

            }
            else
            {
               memset(&bufferA,0x00,EEPROM_VAR_SIZE);
               memset(&bufferB,0x00,EEPROM_VAR_SIZE);
               retA = eeprom_write_block(blockA,(uint8_t*)&bufferA,TRUE);
               retB = eeprom_write_block(blockB,(uint8_t*)&bufferB,TRUE);
               goto eeprom_error_read_block_integrity;
            }
        }


        eeprom_error_read_block_integrity:
        retry++;
        if(retry>5)
        {
            return F_RET_EEPROM;
        }
        else
        {
            goto eeprom_start_read_block_integrity;
        }
    }while(1);
    return 0;
}

return_t eeprom_write_block(uint16_t block,uint8_t *src,bool_t verify)
{
    return_t ret = X_RET_OK;
    uint8_t buffer[EEPROM_VAR_SIZE];
    memcpy(buffer,src,EEPROM_VAR_SIZE);
    uint16_t crc_block_calc = gen_crc16(buffer,EEPROM_VAR_SIZE-2);
    buffer[EEPROM_VAR_SIZE-2] = (uint8_t)(crc_block_calc>>8);
    buffer[EEPROM_VAR_SIZE-1] = (uint8_t)(crc_block_calc & 0x00FF);
    uint16_t addr = block * EEPROM_VAR_SIZE;

    uint8_t retry=0;
    bool_t end = FALSE;

    uint8_t buffer_read[EEPROM_WRITE_PAGE];
    bool_t crc_valid;

    do{
           ret = eeprom_write(addr,buffer,EEPROM_WRITE_PAGE);
           if(ret != X_RET_OK)
               goto retry_label;


           if(verify == TRUE)
           {
               ret = eeprom_read(addr,buffer_read,EEPROM_WRITE_PAGE);
               if(ret != X_RET_OK)
                   goto retry_label;

               int cmp = memcmp(buffer,buffer_read,EEPROM_WRITE_PAGE);
               if(cmp != 0)
                   goto retry_label;
           }


           ret = eeprom_write(addr+EEPROM_WRITE_PAGE,buffer+EEPROM_WRITE_PAGE,EEPROM_WRITE_PAGE);
           if(ret != X_RET_OK)
               goto retry_label;

           if(verify == TRUE)
           {
               ret = eeprom_read(addr+EEPROM_WRITE_PAGE,buffer_read,EEPROM_WRITE_PAGE);
               if(ret != X_RET_OK)
                   goto retry_label;

               int cmp = memcmp(buffer+EEPROM_WRITE_PAGE,buffer_read,EEPROM_WRITE_PAGE);
               if(cmp != 0)
                   goto retry_label;
           }





           return X_RET_OK;


           retry_label:
           retry++;
           if(retry>10)
           {
              end = true;
           }
    }while(!end);


    return ret;
}

return_t eeprom_read_block(uint16_t block,uint8_t *dest,bool_t *crc_valid)
{
    return_t ret = X_RET_OK;

    *crc_valid = FALSE;
    uint16_t addr = block * EEPROM_VAR_SIZE;
    ret = eeprom_read(addr,dest,EEPROM_VAR_SIZE);
    if(ret == X_RET_OK)
    {
        uint16_t crc_block_calc = gen_crc16(dest,EEPROM_VAR_SIZE-2);
        uint16_t crc_block_read = dest[EEPROM_VAR_SIZE-2];
        crc_block_read<<=8;
        crc_block_read += dest[EEPROM_VAR_SIZE-1];
        if(crc_block_calc == crc_block_read)
        {
            *crc_valid = TRUE;
        }
    }
    return ret;
}





return_t eeprom_read(uint16_t addr,uint8_t *dest,uint16_t bytes)
{
    volatile return_t ret = X_RET_OK;
    volatile fsp_err_t err;
    tx_mutex_get(&g_i2c_mutex,TX_WAIT_FOREVER);
    err = R_SCI_B_I2C_Open(&g_i2c0_ctrl, &g_i2c0_cfg);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }


    uint16_t new_addr = EEPROM_HW_ADDR;
    if((addr & 0x100) == 0x100)
    {
        new_addr++;
    }

    err = R_SCI_B_I2C_SlaveAddressSet(&g_i2c0_ctrl,new_addr,I2C_MASTER_ADDR_MODE_7BIT);
    if(err != FSP_SUCCESS)
    {
        ret = -2;
        goto end;
    }

    uint8_t mem_addr = (uint8_t)(addr& 0xFF);


    iic_event = I2C_MASTER_EVENT_ABORTED;
    err = R_SCI_B_I2C_Write(&g_i2c0_ctrl, &mem_addr, 1, false);
    if(err != FSP_SUCCESS)
    {
        ret = -3;
        goto end;
    }

    uint16_t timeout_ms = 10;
    while ((I2C_MASTER_EVENT_RX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        ret = -4;
        goto end;
    }



    iic_event = I2C_MASTER_EVENT_ABORTED;
    err = R_SCI_B_I2C_Read(&g_i2c0_ctrl, dest, bytes, false);
    if(err != FSP_SUCCESS)
    {
        ret = -5;
        goto end;
    }

    timeout_ms = 10;
    while ((I2C_MASTER_EVENT_RX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        ret = -6;
        goto end;
    }




    end:
    R_SCI_B_I2C_Close(&g_i2c0_ctrl);
    tx_mutex_put(&g_i2c_mutex);
    return ret;
}


return_t eeprom_write(uint16_t addr,uint8_t *src,uint16_t bytes)
{
    volatile return_t ret = X_RET_OK;
    volatile fsp_err_t err;
    tx_mutex_get(&g_i2c_mutex,TX_WAIT_FOREVER);
    err = R_SCI_B_I2C_Open(&g_i2c0_ctrl, &g_i2c0_cfg);
    if(err != FSP_SUCCESS)
    {
        ret = -1;
        goto end;
    }


    uint16_t new_addr = EEPROM_HW_ADDR;
    if((addr & 0x100) == 0x100)
    {
        new_addr++;
    }

    err = R_SCI_B_I2C_SlaveAddressSet(&g_i2c0_ctrl,new_addr,I2C_MASTER_ADDR_MODE_7BIT);
    if(err != FSP_SUCCESS)
    {
        ret = -2;
        goto end;
    }

    uint8_t mem_addr = (uint8_t)(addr& 0xFF);


    uint8_t buffer_write[32];
    buffer_write[0] = mem_addr;
    uint8_t i=0;
    uint8_t *ptr = src;
    for(i=0;i<bytes;i++)
    {
        buffer_write[i+1] = *ptr;
        ptr++;
    }


    iic_event = I2C_MASTER_EVENT_ABORTED;
    err = R_SCI_B_I2C_Write(&g_i2c0_ctrl, &buffer_write, bytes+1, false);
    if(err != FSP_SUCCESS)
    {
        ret = -3;
        goto end;
    }

    uint16_t timeout_ms = 10;
    while ((I2C_MASTER_EVENT_RX_COMPLETE != iic_event) && timeout_ms)
    {
        tx_thread_sleep(1);
        timeout_ms--;
    }

    if (I2C_MASTER_EVENT_ABORTED == iic_event)
    {
        ret = -4;
        goto end;
    }



    end:
    R_SCI_B_I2C_Close(&g_i2c0_ctrl);
    tx_mutex_put(&g_i2c_mutex);
    return ret;
}
