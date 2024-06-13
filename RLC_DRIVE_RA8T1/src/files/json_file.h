/*
 * file.h
 *
 *  Created on: 7 juin 2024
 *      Author: Christophe
 */

#ifndef FILES_JSON_FILE_H_
#define FILES_JSON_FILE_H_

#include <_core/c_common.h>

#define FILE_TYPE_PAYLOAD  0
#define FILE_TYPE_EVENT    1

typedef struct st_json_file_t
{
    uint8_t type;
    char *ptr_data;
}
__attribute__((packed))      // Remove interfield padding.
__attribute__((aligned(4)))  // Set alignment and add tail padding.
json_file_t;


return_t json_file_add_to_queue(uint8_t type,char *ptr);


#endif /* FILES_JSON_FILE_H_ */
