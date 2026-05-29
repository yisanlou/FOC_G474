#include "MY_Tim.h"
#include "FOC.h"
#include "MY_ADC.h"
#include "Control.h"
#include "VOFA.h"

#include "TEST.h"



volatile uint32_t CNT = 0;

CCMRAM void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim->Instance == HRTIM1) && (TimerIdx == HRTIM_TIMERINDEX_MASTER))
  {
    CNT++;
    //ADC Update
    //Clarke/park
    // CurrLoop PI
    // SVPWM
    // Update CMP
    ADC_Value.Raw_U = ADC_DMA[0];
    ADC_Value.Raw_V = ADC_DMA[1];
    ADC_Value.Raw_W = ADC_DMA[2];

    ADC_Value.Curr_A = ((float)ADC_Value.Raw_U - (float)ADC_Value.Offset_U) * ADC_Value.Adc_to_Curr;
    ADC_Value.Curr_B = ((float)ADC_Value.Raw_V - (float)ADC_Value.Offset_V) * ADC_Value.Adc_to_Curr;
    ADC_Value.Curr_C = ((float)ADC_Value.Raw_W - (float)ADC_Value.Offset_W) * ADC_Value.Adc_to_Curr;
    
    CLARKE(ADC_Value.Curr_A, ADC_Value.Curr_B, ADC_Value.Curr_C, &FOC_State.Ialpha, &FOC_State.Ibeta);
    ParkTransform(FOC_State.Ialpha, FOC_State.Ibeta,FOC_State.sinVal, FOC_State.cosVal, &FOC_State.Id, &FOC_State.Iq);
    if(CNT >= 1000) {
        // SpdLoop_Run();
       //
        Vofa_Send(1);
        CNT = 0;
    }
    // CurrLoop_Run();
   OpenloopVol(1.6f, 50.0f, 5.0f);


    
    

  }
}
