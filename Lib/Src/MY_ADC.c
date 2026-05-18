#include "MY_ADC.h"


#define ADC_OFFSET_NUM    1000U

uint16_t ADC_DMA[5] = {0};


ADC_Config_t ADC_Config =
{
    .ADC_ref = 3.3f,
    .ADC_max = 4095.0f,
    .Shunt_Phrase = 0.002f,
    .Shunt_Bus = 0.002f,
    .Curr_Gain = 20.0f,
    .Vol_Gain = 105.1f / 5.1f,
};

ADC_t ADC_Value = 
{
    .Raw_U = 0,
    .Raw_V = 0,
    .Raw_W = 0,

    .Curr_A = 0.0f,
    .Curr_B = 0.0f,
    .Curr_C = 0.0f,

    .Curr_Bus = 0.0f,
    .Vol_Bus = 0.0f,

    .Offset_U = 0,
    .Offset_V = 0,
    .Offset_W = 0,

    .Offset_Sum_U = 0,
    .Offset_Sum_V = 0,
    .Offset_Sum_W = 0,

    .Adc_to_Vol = 0.0f,
    .Adc_to_Curr = 0.0201465f,
    .Adc_to_Bus = 0.0f,

};
void ADC_Calibration(void)
{
    
}



