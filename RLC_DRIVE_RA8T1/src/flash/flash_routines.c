/*
 * fs_routines.c
 *
 *  Created on: 22 oct. 2024
 *      Author: Christophe
 */


#include "flash_routines.h"
#include "return_codes.h"
#include <rtc/rtc.h>
#include <time.h>
#include <fx_api.h>


#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "flash routines"

static FX_MEDIA g_fx_media1;
static uint8_t g_fx_media1_media_memory[G_FX_MEDIA0_MEDIA_MEMORY_SIZE];


uint8_t g_fx_fault_tolerant_memory[4096];


char dir_data[] = "/DATA\0";

return_t fs_initialise_only_one_time(void)
{
    fx_system_initialize ();


    return_t ret = X_RET_OK;

    ret = fs_open();
    if(ret != X_RET_OK) return ret;

    char write_array[256];
    strcpy(write_array,"test");
    fs_set_directory(dir_data);
    fs_file_delete("test");
    ret = fs_file_create_and_write("test", write_array, strlen(write_array));
    if(ret != X_RET_OK)
    {
        LOG_E(LOG_STD,"Error initialising media, formatting");
        ret = fs_format();
        if(ret != X_RET_OK)
            goto end;

        ret = fs_open();
        if(ret != X_RET_OK)
            goto end;
    }
    else
    {
        ret = fs_file_delete("test");
        if(ret != X_RET_OK)
            goto end;
    }

    end:
    ret = fs_close();
    return ret;
}

return_t fs_open()
{
    return_t ret = X_RET_OK;
    volatile fsp_err_t err = RM_FILEX_BLOCK_MEDIA_Open (&g_rm_filex_block_media_1_ctrl, &g_rm_filex_block_media_1_cfg);
    if(err != FSP_SUCCESS)
    {
        LOG_E(LOG_STD,"Error opening block media");
        return F_RET_FS_INIT;
    }


    err = fx_media_open(&g_fx_media1, "&g_fx_media1", RM_FILEX_BLOCK_MEDIA_BlockDriver,
                        (void* ) &g_rm_filex_block_media_1_instance, g_fx_media1_media_memory,
                        G_FX_MEDIA0_MEDIA_MEMORY_SIZE);
    if(err != FSP_SUCCESS)
    {
        ret = fs_format();
        if(ret != X_RET_OK)
            return ret;


        err = fx_media_open(&g_fx_media1, "&g_fx_media1", RM_FILEX_BLOCK_MEDIA_BlockDriver,
                                (void* ) &g_rm_filex_block_media_1_instance, g_fx_media1_media_memory,
                                G_FX_MEDIA0_MEDIA_MEMORY_SIZE);
        if(err != FSP_SUCCESS)
        {
            LOG_E(LOG_STD,"Error opening media");
            return F_RET_FS_INIT_MEDIA;
        }
    }

    err = fx_fault_tolerant_enable(&g_fx_media1,g_fx_fault_tolerant_memory,sizeof(g_fx_fault_tolerant_memory));
    if(err != FSP_SUCCESS)
        {
            LOG_E(LOG_STD,"Error opening media");
            return F_RET_FS_INIT_MEDIA;
        }


    ULONG flash_bytes_available;
    fs_bytes_available(&flash_bytes_available);

    if(flash_bytes_available < 1000000)
    {
        LOG_E(LOG_STD,"Error media avaialable size");
        ret = fx_media_close(&g_fx_media1);
        if(ret != X_RET_OK)
            return ret;
        ret = fs_format();
        if(ret != X_RET_OK)
            return ret;
        err = fx_media_open(&g_fx_media1, "&g_fx_media1", RM_FILEX_BLOCK_MEDIA_BlockDriver,
                            (void* ) &g_rm_filex_block_media_1_instance, g_fx_media1_media_memory,
                            G_FX_MEDIA0_MEDIA_MEMORY_SIZE);
        if(ret != FSP_SUCCESS)
        {
            LOG_E(LOG_STD,"Error opening media");
            return F_RET_FS_INIT_MEDIA;
        }
        else
        {
            LOG_I(LOG_STD,"Success formating and opening");
        }

    }


    fs_set_timestamp();

    UINT attributes;
    err = fx_directory_attributes_read (&g_fx_media1, dir_data, &attributes);
    if(err == FX_NOT_FOUND)
    {
        err = fx_directory_create (&g_fx_media1, dir_data);
        if(err != FX_SUCCESS)
        {
            LOG_E(LOG_STD,"Error creating directory with code %d",err);
            ret =  F_RET_FS_CREATE_DIRECTORY;
            goto error;
        }
    }
    else if(err != FX_SUCCESS)
    {
        LOG_E(LOG_STD,"Error reading directory with code %d",err);
        ret =  F_RET_FS_READ_DIRECTORY;
        goto error;
    }
    return X_RET_OK;
    error:
    return ret;
}

return_t fs_format()
{
    fsp_err_t err = fx_media_format (&g_fx_media1, RM_FILEX_BLOCK_MEDIA_BlockDriver,
                             (void*) &g_rm_filex_block_media_1_instance, g_fx_media1_media_memory,
                             G_FX_MEDIA0_MEDIA_MEMORY_SIZE,
                             G_FX_MEDIA0_VOLUME_NAME,
                             G_FX_MEDIA0_NUMBER_OF_FATS,
                             G_FX_MEDIA0_DIRECTORY_ENTRIES,
                             G_FX_MEDIA0_HIDDEN_SECTORS,
                             G_FX_MEDIA0_TOTAL_SECTORS,
                             G_FX_MEDIA0_BYTES_PER_SECTOR,
                             G_FX_MEDIA0_SECTORS_PER_CLUSTER, 0, 0);
    if(err != FSP_SUCCESS)
    {
        LOG_E(LOG_STD,"Error formating media [%d]",err);
        return F_RET_FS_FORMAT;
    }
    return X_RET_OK;
}

return_t fs_close()
{
    fx_media_close(&g_fx_media1);
    return X_RET_OK;
}

return_t fs_set_timestamp()
{
    st_rtc_t r = rtc_get();
    if(r.configured == TRUE)
    {
        time_t timestamp = r.time_ms / 1000;
        volatile struct tm * timeInfos = gmtime( & timestamp );
        fx_system_date_set(timeInfos->tm_year+1900,timeInfos->tm_mon+1,timeInfos->tm_mday);
        fx_system_time_set(timeInfos->tm_hour,timeInfos->tm_min,timeInfos->tm_sec);
    }
    return X_RET_OK;
}


return_t fs_is_directory_exist( char *dir, bool_t *result)
{

    UINT attributes;
    fsp_err_t fsp_err = fx_directory_attributes_read (&g_fx_media1, dir, &attributes);
    if (fsp_err == FSP_SUCCESS)
    {
        *result = TRUE;
        return X_RET_OK;
    }
    else if (fsp_err == FX_NOT_FOUND)
    {
        *result = FALSE;
        return X_RET_OK;
    }
    else
        return F_RET_FS_GENERIC;

    return X_RET_OK;
}

return_t fs_get_current_directory(char **dir)
{

    fsp_err_t fsp_err = fx_directory_default_get (&g_fx_media1, dir);
    if (fsp_err != FSP_SUCCESS)
        return F_RET_FS_GENERIC;
    return X_RET_OK;
}

return_t fs_set_directory(char *dir)
{
    fsp_err_t err_fsp = fx_directory_default_set (&g_fx_media1, dir);
    if (err_fsp != FSP_SUCCESS)
        return F_RET_FS_SET_DIRECTORY;
    return X_RET_OK;
}

return_t fs_get_directory(char **dir)
{
    fsp_err_t err_fsp = fx_directory_default_get (&g_fx_media1, dir);
    if (err_fsp != FSP_SUCCESS)
        return F_RET_FS_GET_DIRECTORY;
    return X_RET_OK;
}

return_t fs_create_directory(char *dir)
{
    fsp_err_t err_fsp = fx_directory_create (&g_fx_media1, dir);
    if (err_fsp == FSP_SUCCESS || err_fsp == FX_ALREADY_CREATED)
    {
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error creating dir %s [%d]",dir,err_fsp);
        return F_RET_FS_CREATE_DIRECTORY;
    }
}

return_t fs_delete_directory(char *dir)
{
    fsp_err_t err_fsp = fx_directory_delete (&g_fx_media1, dir);
    if (err_fsp == FSP_SUCCESS)
    {
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error deleting dir %s [%d]",dir,err_fsp);
        return F_RET_FS_DELETE_DIRECTORY;
    }
}

return_t fs_file_create(char *name)
{
    fsp_err_t err_fsp = fx_file_create (&g_fx_media1, name);
    if (err_fsp == FSP_SUCCESS || err_fsp == FX_ALREADY_CREATED)
    {
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error creating file %s [%d]",name,err_fsp);
        return F_RET_FS_CREATE_FILE;
    }
    return X_RET_OK;
}

return_t fs_file_delete( char *name)
{
    fsp_err_t err_fsp = fx_file_delete (&g_fx_media1, name);
    if (err_fsp == FSP_SUCCESS)
    {
        return fs_flush();
        //return X_RET_OK;
    }
    else if (err_fsp == FX_NOT_FOUND)
    {
        LOG_W(LOG_STD,"Error finding file %s",name);
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error deleting file %s [%d]",name,err_fsp);
        return F_RET_FS_DELETE_FILE;
    }
    return X_RET_OK;
}

return_t fs_file_close(FX_FILE *file_ptr)
{
    fsp_err_t err_fsp = fx_file_close (file_ptr);
    if (err_fsp == FSP_SUCCESS)
    {
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error closing file %s [%d]",file_ptr->fx_file_name,err_fsp);
        return F_RET_FS_CLOSE_FILE;
    }

    return X_RET_OK;
}

return_t fs_file_open(FX_FILE *file_ptr, char *name, uint16_t open_mode)
{
    fsp_err_t err_fsp = fx_file_open(&g_fx_media1, file_ptr, name, open_mode);
    if (err_fsp == FSP_SUCCESS)
    {

        err_fsp = fx_file_seek(file_ptr, 0);
        if (err_fsp != FSP_SUCCESS)
        {
            LOG_E(LOG_STD,"Error seek file %s [%d]",name,err_fsp);
            return F_RET_FS_OPEN_FILE;
        }
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error opening file %s [%d]",name,err_fsp);
        return F_RET_FS_OPEN_FILE;
    }
    return X_RET_OK;
}

return_t fs_file_read(FX_FILE *file_ptr, void *ptr, uint64_t requested_size, uint64_t *actual_size)
{
    ULONG as;
    fsp_err_t err_fsp = fx_file_read (file_ptr, ptr, (ULONG) requested_size, (ULONG*) &as);
    if (err_fsp == FSP_SUCCESS)
    {
        *actual_size = as;
        return X_RET_OK;
    }
    else if (err_fsp == FX_END_OF_FILE)
    {
        return F_RET_FS_READ_FILE_EOF;
    }
    else
        return F_RET_FS_READ_FILE;
    return X_RET_OK;
}

return_t fs_file_write(FX_FILE *file_ptr, void *ptr, uint64_t size)
{
    fsp_err_t err_fsp = fx_file_write (file_ptr, ptr, (ULONG) size);
    if (err_fsp != FSP_SUCCESS)
    {
        LOG_E(LOG_STD,"Error wrinting file %s [%d]",file_ptr->fx_file_name,err_fsp);
        return F_RET_FS_WRITE_FILE;
    }
    return X_RET_OK;
}

return_t fs_flush()
{
    //
    fsp_err_t err_fsp = fx_media_flush (&g_fx_media1);
    if (FSP_SUCCESS != err_fsp)
    {
        return F_RET_FS_GENERIC;
    }
    return X_RET_OK;
}

return_t fs_bytes_available(ULONG *bytes)
{

    ULONG b=0;
    fsp_err_t err_fsp = fx_media_space_available (&g_fx_media1,&b);
    if(err_fsp)
    {
        return F_RET_FS_GENERIC;
    }

    *bytes=b;

    //LOG_D(LOG_STD,"FS %d bytes available",b);
    return X_RET_OK;
}

return_t fs_file_create_and_write(char *name, void *ptr, uint64_t size)
{
    FX_FILE file;
    volatile return_t ret = fs_file_create (name);
    if (ret != X_RET_OK)
        return ret;

    ret = fs_file_open (&file, name, FX_OPEN_FOR_WRITE);
    if (ret != X_RET_OK)
        return ret;


    ret = fs_file_write(&file,ptr,size);
    if(ret != X_RET_OK)
    {
        fs_file_close (&file);
        fs_file_delete(name);
        //fs_flush ();
        return ret;
    }

    volatile uint8_t xxx=0;
    xxx=1;

    ret = fs_file_close (&file);
    if (ret != X_RET_OK)
        return ret;

    ret = fs_flush ();
    if (ret != X_RET_OK)
        return ret;
    return X_RET_OK;

}


return_t fs_file_seek(FX_FILE *file_ptr, uint64_t offset)
{
    fsp_err_t err_fsp = fx_file_seek (file_ptr, (ULONG) offset);
    if (err_fsp == FSP_SUCCESS)
    {
        return X_RET_OK;
    }
    else
        return F_RET_FS_GENERIC;

    return X_RET_OK;
}

return_t fs_file_rename(char *old, char *new)
{
    fsp_err_t err_fsp = fx_file_rename (&g_fx_media1, old, new);
    if (err_fsp == FSP_SUCCESS)
    {
        return X_RET_OK;
    }
    else
        return F_RET_FS_RENAME_FILE;
    return X_RET_OK;
}


return_t fs_first_file_find(char *file_name,ULONG *size,UINT *year,UINT* month,UINT* day,UINT* hour,UINT* minut,UINT* second)
{

    UINT attributes;

    fsp_err_t err_fsp = fx_directory_first_full_entry_find (&g_fx_media1, file_name, &attributes, size, year, month,
                                                            day, hour, minut, second);

    if (err_fsp == FSP_SUCCESS)
    {
        if ((attributes & 0x10) != 0x10)
            return X_RET_OK;
        else
        {
            do
            {
                err_fsp = fx_directory_next_full_entry_find (&g_fx_media1, file_name, &attributes, size, year, month,
                                                             day, hour, minut, second);
                if (err_fsp == FSP_SUCCESS)
                {
                    if ((attributes & 0x10) != 0x10)
                        return X_RET_OK;
                }
                else if (err_fsp == FX_NO_MORE_ENTRIES)
                {
                    return F_RET_FS_NO_MORE_ENTRIES;
                }
                else
                {
                    return F_RET_FS_GENERIC;
                }

            }
            while (attributes & 0x10);
        }
    }
    else if (err_fsp == FX_NO_MORE_ENTRIES)
    {
        return F_RET_FS_NO_MORE_ENTRIES;
    }
    else
    {
        return F_RET_FS_GENERIC;
    }

    return X_RET_OK;
}

return_t fs_next_file_find(char *file_name,ULONG *size,UINT *year,UINT* month,UINT* day,UINT* hour,UINT* minut,UINT* second)
{

    UINT attributes;


    do
    {
        fsp_err_t err_fsp = fx_directory_next_full_entry_find (&g_fx_media1, file_name, &attributes, size, year,
                                                               month, day, hour, minut, second);

        if (err_fsp == FSP_SUCCESS)
        {
            if ((attributes & 0x10) != 0x10)
                return X_RET_OK;
        }
        else if (err_fsp == FX_NO_MORE_ENTRIES)
        {
            return F_RET_FS_NO_MORE_ENTRIES;
        }
        else
        {
            return F_RET_FS_GENERIC;
        }
    }
    while (attributes & 0x10);
    return X_RET_OK;
}


return_t fs_file_date_time_set(char *file_name,UINT year,UINT month,UINT day,UINT hour,UINT minut,UINT second)
{
    return_t ret = X_RET_OK;
    fsp_err_t err_fsp = fx_file_date_time_set(&g_fx_media1, file_name,year,month,day,hour,minut,second);
    if (err_fsp == FSP_SUCCESS)
    {
        return X_RET_OK;
    }
    else
    {
        return F_RET_FS_GENERIC;
    }
    return X_RET_OK;
}
