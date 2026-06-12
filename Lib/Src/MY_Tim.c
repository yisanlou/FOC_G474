#include "MY_Tim.h"
#include "FOC.h"
#include "MY_ADC.h"
#include "Control.h"
#include "VOFA.h"
#include "MotorCofig.h"
#include "TEST.h"
#include "Encoder.h"
#include "arm_math.h"
#include "tim.h"
#include <stdint.h>
#include <stdio.h>

#define TIM6_SPEED_LOOP_TS  (1.0f / 1000.0f)
#define TIM6_MIT_LOOP_DIV   5U

uint32_t Encoder_Offset_Data[13];

float Angle_Expt = 0.0f;
uint8_t Offset_num = 0;
volatile uint32_t CNT = 0;
volatile uint32_t Com_CNT = 0;

CCMRAM void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{

	if ((hhrtim->Instance == HRTIM1) && (TimerIdx == HRTIM_TIMERINDEX_MASTER))
  {
    Start_Angle_Read();
		CNT++;

    if(MOTOR_MODE == MOTOR_MODE_OFFSET)
    {
      static float VolD_Expt = 1.2f;
      static uint8_t Offset_Done = 0;

      if((CNT >= 8000) && (Offset_Done == 0))
      {
        char tx_buf[64];

        snprintf(tx_buf, sizeof(tx_buf), "Etheta=%.3f, Enc=%lu\r\n",
                 Angle_Expt, (unsigned long)Encoder_Value.Raw_Num);
        UART_Send(tx_buf);

        if(Offset_num < 13)
        {
          Encoder_Offset_Data[Offset_num] = Encoder_Value.Raw_Num;
          Offset_num++;
        }

        if(Offset_num >= 13)
        {
          float Expt_Step = (float)ENC_RES / (float)Motor->Pole_pairs;
          int32_t real_step_signed = (int32_t)(Encoder_Offset_Data[12] - Encoder_Offset_Data[0]);
          float Real_Step;
          float Error_Encoder;

          if(real_step_signed < 0)
          {
            real_step_signed = -real_step_signed;
          }

          Real_Step = (float)real_step_signed;
          Error_Encoder = (Real_Step - Expt_Step) / Expt_Step;

          snprintf(tx_buf, sizeof(tx_buf), "Real=%.1f, Expt=%.1f, Err=%.3f%%\r\n",
                   Real_Step, Expt_Step, Error_Encoder * 100.0f);
          UART_Send(tx_buf);

          VolD_Expt = 0.0f;
          Offset_Done = 1;
          Angle_Expt = 0.0f;
        }
        else
        {
          Angle_Expt += PI/6.0f;
        }

        CNT = 0;
      }

      Correct_Offset(VolD_Expt, Angle_Expt, Motor->Pole_pairs);
    }
    
    
    //ADC Update
    //Clarke/park
    // CurrLoop PI
    // SVPWM
    // Update CMP
    else if(MOTOR_MODE == MOTOR_MODE_WORK)
    {
    Current_Samp();
    
    if(CNT >= 100)
    {
      Vofa_Send(8);
      CNT = 0;
    }

    float etheta = FOC_State.Etheta;
    float sin_val = arm_sin_f32(etheta);
    float cos_val = arm_cos_f32(etheta);

    FOC_State.sinVal = sin_val;
    FOC_State.cosVal = cos_val;

    CLARKE(ADC_Value.Curr_A, ADC_Value.Curr_B, ADC_Value.Curr_C, &FOC_State.Ialpha, &FOC_State.Ibeta);
    ParkTransform(FOC_State.Ialpha, FOC_State.Ibeta, sin_val, cos_val, &FOC_State.Id, &FOC_State.Iq);

    CurrLoop_Run(FOC_State.Iq, FOC_State.Id, Expt.Iq, 0.0f,&FOC_State.Vol_Q, &FOC_State.Vol_D);


    INVERSEPARK(FOC_State.Vol_Q, FOC_State.Vol_D,
                cos_val, sin_val,
                &FOC_State.Ualpha, &FOC_State.Ubeta);
    SVPWM(FOC_State.Ualpha, FOC_State.Ubeta, FOC_GetVbus());

    FOC_UpdatePwmCompare();
    // OpenloopVol(1.4f, 0.0f, 10.0f, Motor->Pole_pairs);

    // Correct_Offset(1.0f, 0.0f, Motor->Pole_pairs);

    }

    
    

  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM6) && (MOTOR_MODE == MOTOR_MODE_WORK))
  {
    static uint8_t mit_loop_cnt = 0U;
    // static uint8_t pos_loop_cnt =0;

    Encoder_CalcSpeed(TIM6_SPEED_LOOP_TS);

    mit_loop_cnt++;
    if (mit_loop_cnt >= TIM6_MIT_LOOP_DIV)
    {
      MITModeLoop_Run(MITMode.pos_ref,
                      MITMode.vel_ref,
                      MITMode.torque_ref,
                      MITMode.kp,
                      MITMode.kd);
      mit_loop_cnt = 0U;
    }

  //   pos_loop_cnt++;
  //   if (pos_loop_cnt >= 10U)
  //   {
  //     PosLoop_Run(Real.Mech_Pos, Expt.Mech_Pos, &Expt.Mech_Vel_RPM);
  //     pos_loop_cnt = 0U;
  //   }

  //   Expt.Mech_Vel_RPM = OutputLimitation(500.0f, -500.0f, Expt.Mech_Vel_RPM);

  //   SpdLoop_Run(Real.Mech_Vel_RPM, Expt.Mech_Vel_RPM, &Expt.Iq);
   }



}
