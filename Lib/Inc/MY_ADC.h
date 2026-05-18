#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

extern uint16_t ADC_DMA[5];

typedef struct{
    uint16_t Raw_U;
    uint16_t Raw_V;
    uint16_t Raw_W;

    float Curr_A;
    float Curr_B;
    float Curr_C;

    float Curr_Bus;
    float Vol_Bus;

    uint16_t Offset_U;
    uint16_t Offset_V;
    uint16_t Offset_W;

    uint32_t Offset_Sum_U;
    uint32_t Offset_Sum_V;
    uint32_t Offset_Sum_W;

    float Adc_to_Vol;
    float Adc_to_Curr;
    float Adc_to_Bus;

}ADC_t;

typedef struct
{
    float ADC_ref;
    float ADC_max;
    float Shunt_Phrase;
    float Shunt_Bus;
    float Curr_Gain;
    float Vol_Gain;
}ADC_Config_t;

extern ADC_Config_t ADC_Config;
extern ADC_t ADC_Value;

void ADC_Calibration(void);

#endif
