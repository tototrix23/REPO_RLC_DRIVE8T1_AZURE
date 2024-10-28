/*
 * my_malloc.h
 *
 *  Created on: 25 oct. 2024
 *      Author: Christophe
 */

#ifndef MY_MALLOC_H_
#define MY_MALLOC_H_

#include <_core/c_common.h>
#include <stdio.h>


void *my_malloc(uint32_t s,char *f,uint32_t l);
void my_free(void *p,char *f,uint32_t l);

#define MALLOC(x)    my_malloc(x,__FILE__,__LINE__);
#define FREE2(x)    my_free(x,__FILE__,__LINE__);

void my_free2(void **p,char *f,uint32_t l);
#define FREE(x)    my_free2(x,__FILE__,__LINE__);


#endif /* MY_MALLOC_H_ */
