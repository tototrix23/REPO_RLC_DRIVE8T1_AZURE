/*
 * fs_routines.h
 *
 *  Created on: 22 oct. 2024
 *      Author: Christophe
 */

#ifndef FLASH_FLASH_ROUTINES_H_
#define FLASH_FLASH_ROUTINES_H_

#include <_core/c_common.h>
#include "flash_stack.h"
#include "flash_impl.h"



extern char dir_data[];

return_t fs_initialise_only_one_time(void);
return_t fs_open();
return_t fs_format();
return_t fs_close();
return_t fs_set_timestamp();
return_t fs_flush();
return_t fs_bytes_available(ULONG *bytes);
return_t fs_is_directory_exist( char *dir, bool_t *result);
return_t fs_get_current_directory(char **dir);
return_t fs_set_directory(char *dir);
return_t fs_get_directory(char **dir);
return_t fs_create_directory(char *dir);
return_t fs_delete_directory(char *dir);
return_t fs_file_create(char *name);
return_t fs_file_delete( char *name);
return_t fs_file_close(FX_FILE *file_ptr);
return_t fs_file_open(FX_FILE *file_ptr, char *name, uint16_t open_mode);
return_t fs_file_read(FX_FILE *file_ptr, void *ptr, uint64_t requested_size, uint64_t *actual_size);
return_t fs_file_write(FX_FILE *file_ptr, void *ptr, uint64_t size);
return_t fs_file_create_and_write(char *name, void *ptr, uint64_t size);
return_t fs_file_seek(FX_FILE *file_ptr, uint64_t offset);
return_t fs_file_rename(char *old, char *new);
return_t fs_first_file_find(char *file_name,ULONG *size,UINT *year,UINT* month,UINT* day,UINT* hour,UINT* minut,UINT* second);
return_t fs_next_file_find(char *file_name,ULONG *size,UINT *year,UINT* month,UINT* day,UINT* hour,UINT* minut,UINT* second);
return_t fs_file_date_time_set(char *file_name,UINT year,UINT month,UINT day,UINT hour,UINT minut,UINT second);


#endif /* FLASH_FLASH_ROUTINES_H_ */
