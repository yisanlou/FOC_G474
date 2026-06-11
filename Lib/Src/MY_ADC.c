#include "MY_ADC.h"
#include "MotorCofig.h"


#define ADC_OFFSET_NUM    1000U

volatile uint16_t ADC_DMA[5] = {0};
ADC_DmaDiag_t ADC_DmaDiag = {0};


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

    .Offset_U = 1980,
    .Offset_V = 2007,
    .Offset_W = 1967,

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

void Current_Samp()
{
    ADC_Value.Raw_U = ADC_DMA[0];
    ADC_Value.Raw_V = ADC_DMA[1];
    ADC_Value.Raw_W = ADC_DMA[2];

    float curr_u = ((float)ADC_Value.Raw_U - (float)ADC_Value.Offset_U) * ADC_Value.Adc_to_Curr;
    float curr_v = ((float)ADC_Value.Raw_V - (float)ADC_Value.Offset_V) * ADC_Value.Adc_to_Curr;
    float curr_w = ((float)ADC_Value.Raw_W - (float)ADC_Value.Offset_W) * ADC_Value.Adc_to_Curr;

#if (PHASE_SEQUENCE == PHASE_POSITIVE)
    ADC_Value.Curr_A = curr_u;
    ADC_Value.Curr_B = curr_v;
    ADC_Value.Curr_C = curr_w;
#elif (PHASE_SEQUENCE == PHASE_NEGATIVE)
    ADC_Value.Curr_A = curr_u;
    ADC_Value.Curr_B = curr_w;
    ADC_Value.Curr_C = curr_v;
#else
#error "Invalid PHASE_SEQUENCE"
#endif

    
}




