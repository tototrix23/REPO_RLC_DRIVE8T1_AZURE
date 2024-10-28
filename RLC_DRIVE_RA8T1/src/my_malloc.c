/*
 * my_malloc.c
 *
 *  Created on: 25 oct. 2024
 *      Author: Christophe
 */


#include "my_malloc.h"

volatile char call_file[64];
volatile uint32_t call_line=0;
volatile uint32_t call_size=0;


volatile char free_file[64];
volatile uint32_t free_line=0;
volatile void* free_ptr=0;


void *my_malloc(uint32_t s,char *f,uint32_t l)
{
    strcpy(call_file,f);
    call_line = l;
    call_size = s;
    return malloc(s);
}


void my_free(void *p,char *f,uint32_t l)
{
    strcpy(free_file,f);
    free_line = l;
    free_ptr = p;
    free(p);
}


void my_free2(void **p,char *f,uint32_t l)
{
    strcpy(free_file,f);
    free_line = l;
    free_ptr = *p;
    free(*p);
    *p = 0x0;
}
