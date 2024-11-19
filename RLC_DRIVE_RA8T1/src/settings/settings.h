/*
 * settings.h
 *
 *  Created on: 5 nov. 2024
 *      Author: Christophe
 */

#ifndef SETTINGS_SETTINGS_H_
#define SETTINGS_SETTINGS_H_

#include <_core/c_common.h>

#define SETTINGS_MAX_COUNT   10

typedef enum e_settings_mode
{
    setting_mode_force_off = 0,
    setting_mode_force_on = 1,
    setting_mode_force_auto = 2,
}e_settings_mode;

typedef enum e_settings_type
{
    setting_scrolling = 0,
    setting_lighting = 1,
}e_settings_type;



typedef struct st_time_settings_t
{
    uint64_t start;
    uint64_t stop;
}st_time_settings_t;


typedef struct st_settings_with_id_t
{
    char id[64];
    e_settings_mode mode;
    struct st_time_settings_t array[SETTINGS_MAX_COUNT];
}st_settings_with_id_t;




bool_t settings_is_valid(st_settings_with_id_t *ptr);
return_t settings_is_in_range(st_settings_with_id_t *ptr,bool_t *res);





#endif /* SETTINGS_SETTINGS_H_ */
