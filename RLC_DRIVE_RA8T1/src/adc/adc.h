/*
 * adc.h
 *
 *  Created on: 24 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <return_codes.h>




typedef struct st_adc_t
{
    uint16_t iin;
    uint16_t vin;
    uint16_t vm;
    uint16_t vbatt;
    uint16_t vhall_comm;
    uint16_t vhall1;
    uint16_t vhall2;
    uint16_t mot1_iu;
    uint16_t mot1_iw;
    uint16_t mot2_iu;
    uint16_t mot2_iw;
}st_adc_t;


typedef struct st_adc_raw_t
{
    struct
    {
        uint32_t vm_uint;
        uint32_t iin_uint;
        uint32_t vin_uint;
        uint32_t vbatt_uint;
        uint32_t vhall_common_uint;
        uint32_t vhall1_uint;
        uint32_t vhall2_uint;
        uint32_t mot1_iu_uint;
        uint32_t mot1_iw_uint;
        uint32_t mot2_iu_uint;
        uint32_t mot2_iw_uint;
    }average;
    struct
    {
        float vm;
        float vin;
        float vbatt;
        float iin;
        float vhall_common;
        float vhall1;
        float vhall2;
        float mot1_iu;
        float mot1_iw;
    }instantaneous;
}st_adc_raw_t;

extern st_adc_t adc_inst;


return_t adc_init(void);
return_t adc_capture(void);
return_t adc_is_ready(bool_t *result);
return_t adc_set_calibration_mode(bool_t active);
return_t adc_get_snapshot(st_adc_t *res);
return_t adc_get_snapshot_raw(st_adc_raw_t *res);
#endif /* ADC_ADC_H_ */
