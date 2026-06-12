#include "MY_Init.h"
#include "Encoder.h"
#include "fdcan.h"
#include "MY_CAN.h"

//TimeerB -> U  TimeerA -> V    TimeerE -> W



void My_Init(void)
{

  FDCAN2_Config();
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_DMA[0], 1);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&ADC_DMA[1], 1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC_DMA[2], 1);
  HAL_ADC_Start_DMA(&hadc4, (uint32_t *)&ADC_DMA[3], 2);



  __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
  __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
  if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_B|HRTIM_TIMERID_TIMER_E) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  HRTIM1 -> sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 8500;
  HRTIM1 -> sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = 8500;
  HRTIM1 -> sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = 8500;



}



