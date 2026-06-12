#include "Control.h"
#include "FOC.h"
#include "MY_ADC.h"
#include "main.h"
#include "MotorCofig.h"
#include "VOFA.h"
#include "MY_Tim.h"

#define CURRENT_LOOP_DEFAULT_VBUS 48.0f
#define CURRENT_LOOP_TS           (1.0f / (float)PWM_FREQ)
#define CURRENT_LOOP_VOLT_LIMIT   (CURRENT_LOOP_DEFAULT_VBUS / SQRT3)
#define CURRENT_LOOP_BW_RAD       1000.0f
#define SPD_IQ_RISE_STEP_A        0.02f
#define SPD_IQ_FALL_STEP_A        0.05f
#define MIT_MODE_TS               0.005f
#define MIT_TORQUE_CONSTANT_NM_A  0.15f
#define MIT_IQ_RAMP_RATE_A_S      20.0f


Ctrl_t IqLoop = {
    .out_max = CURRENT_LOOP_VOLT_LIMIT,
    .out_min = -CURRENT_LOOP_VOLT_LIMIT,
    .Ts = CURRENT_LOOP_TS,
};

Ctrl_t IdLoop = {
    .out_max = CURRENT_LOOP_VOLT_LIMIT,
    .out_min = -CURRENT_LOOP_VOLT_LIMIT,
    .Ts = CURRENT_LOOP_TS,
};

Ctrl_t SpdLoop = {
    .Ts = 10.0f * CURRENT_LOOP_TS,
    .out_max = 5.0f,
    .out_min = -5.0f,

};
Ctrl_t PosLoop = {
    .Ts = 10.0f * 10.0f * CURRENT_LOOP_TS,
    .out_max = 500.0f,
    .out_min = -500.0f,
};
Ctrl_Value_t Expt = {0};
Ctrl_Value_t Real = {0};
MITMode_t MITMode = {
    .torque_max = 5.0f,
    .torque_min = -5.0f,
    .kp = 0.0f,
    .kd = 0.0f,
    
};

static inline float Ctrl_GetTs(const Ctrl_t *loop)
{
    if (loop->Ts > 0.0f)
    {
        return loop->Ts;
    }

    return CURRENT_LOOP_TS;
}

static inline void Ctrl_ResetState(Ctrl_t *loop)
{
    loop->err = 0.0f;
    loop->last_err = 0.0f;
    loop->integral = 0.0f;
    loop->derivative = 0.0f;
    loop->out = 0.0f;
}

static inline void CurrLoop_UpdateLimit(void)
{
    float vbus = CURRENT_LOOP_DEFAULT_VBUS;

    if (ADC_Value.Vol_Bus > FOC_VDC_MIN)
    {
        vbus = ADC_Value.Vol_Bus;
    }

    IqLoop.out_max = vbus / SQRT3;
    IqLoop.out_min = -IqLoop.out_max;
    IdLoop.out_max = IqLoop.out_max;
    IdLoop.out_min = IqLoop.out_min;
}

static void CurrLoop_InitGain(void)
{
    static uint8_t init_done = 0;

    if (init_done != 0U)
    {
        return;
    }

    IqLoop.kp = Motor->Q_Ind * PWM_FREQ / 3 ;
    IqLoop.ki = Motor->PP_Res * PWM_FREQ / 3;
    IdLoop.kp = Motor->D_Ind * PWM_FREQ/ 3 ;
    IdLoop.ki = Motor->PP_Res * PWM_FREQ / 3;
    SpdLoop.kp = 0.0095f;
    SpdLoop.ki = 0.055f;
    PosLoop.kp = 60.0f;
    PosLoop.ki = 0.0f;

    init_done = 1U;
}

static float PI_Run(Ctrl_t *loop, float ref, float fdb)
{
    float integral_last = loop->integral;
    float out_raw;

    loop->err = ref - fdb;
    loop->integral = loop->integral + loop->err *  loop->Ts;

    out_raw = loop->kp * loop->err + loop->ki * loop->integral ;
    loop->out = OutputLimitation(loop->out_max, loop->out_min, out_raw);

    if (loop->out != out_raw)
    {
      loop->integral = integral_last;
    }

    loop->last_err = loop->err;
    return loop->out;

}

static float SpdLoop_LimitIqSlew(float target)
{
    static float iq_slew = 0.0f;
    float delta = target - iq_slew;
    float step = SPD_IQ_RISE_STEP_A;

    if ((target * target) < (iq_slew * iq_slew))
    {
        step = SPD_IQ_FALL_STEP_A;
    }
    else if (target * iq_slew < 0.0f)
    {
        step = SPD_IQ_FALL_STEP_A;
    }

    if (delta > step)
    {
        iq_slew += step;
    }
    else if (delta < -step)
    {
        iq_slew -= step;
    }
    else
    {
        iq_slew = target;
    }

    return iq_slew;
}

static float MITMode_LimitIqRamp(float target)
{
    static float iq_ramp = 0.0f;
    float step = MIT_IQ_RAMP_RATE_A_S * MIT_MODE_TS;
    float delta = target - iq_ramp;

    if (delta > step)
    {
        iq_ramp += step;
    }
    else if (delta < -step)
    {
        iq_ramp -= step;
    }
    else
    {
        iq_ramp = target;
    }

    return iq_ramp;
}


void CurrLoop_Run(float RealQ, float RealD, float ExptQ, float ExptD, float *Volq, float *Vold)
{
    if ((Volq == 0) || (Vold == 0))
    {
        return;
    }

    CurrLoop_InitGain();

    CurrLoop_UpdateLimit();

    *Volq = PI_Run(&IqLoop, ExptQ, RealQ);
    *Vold = PI_Run(&IdLoop, ExptD, RealD);
}

void SpdLoop_Run(float RealVel, float ExptVel, float *ExptQ)
{
    float pi_out;
    float iq_ref;

    if (ExptQ == 0)
    {
        return;
    }

    CurrLoop_InitGain();

    pi_out = PI_Run(&SpdLoop, ExptVel, RealVel);
    iq_ref = SpdLoop_LimitIqSlew(pi_out);

    SpdLoop.out = iq_ref;
    *ExptQ = iq_ref;

}

void PosLoop_Run(float RealPos, float ExptPos, float *ExptVel)
{
    if (ExptVel == 0)
    {
        return;
    }

    CurrLoop_InitGain();

    *ExptVel = PI_Run(&PosLoop, ExptPos, RealPos);
}

void MITModeLoop_Run(float ExptPos, float ExptVel, float ExptTorque, float Kp, float Kd)
{
    //Ctrl DATA
    //DATA0 7-0 Kp High 8 Bytes
    //DATA1 7-4 kp Low  4 Bytes  0-3 Kd High 8bytes
    //DATA2 7-0 Kd Low  8 Bytes
    //DATA3 7-0 Pos High 8 Bytes
    //DATA4 7-0 Pos Low  8 Bytes
    //DATA5 7-0 Vel High 8 Bytes
    //DATA6 7-4 Vel Low  4 Bytes 3-0 Torque High 4Bytes
    //DATA7 0-7 Torque Low 8 Bytes



    //Feedback DATA
    //DATA0 Pos High 8 Bytes
    //DATA1 Pos Low 8 Bytes
    //DATA2 Vel High 8 Bytes
    //DATA3 Vel Low 8 Bytes
    //DATA4 Iq High 8 Bytes
    //DATA5 Iq Low 8 Bytes
    //DATA6 Tem 
    //DATA7 Err 
    float torque_target;
    float iq_target;
    float iq_ramp;

    MITMode.pos_ref = ExptPos;
    MITMode.vel_ref = ExptVel;
    MITMode.torque_ref = ExptTorque;
    MITMode.kp = Kp;
    MITMode.kd = Kd;

    MITMode.pos_fdb = Real.Mech_Pos;
    MITMode.vel_fdb = Real.Mech_Vel_Rad;

    torque_target = Kp * (MITMode.pos_ref - MITMode.pos_fdb)
                  + Kd * (MITMode.vel_ref - MITMode.vel_fdb)
                  + MITMode.torque_ref;
    torque_target = OutputLimitation(MITMode.torque_max,
                                     MITMode.torque_min,
                                     torque_target);

    iq_target = torque_target / MIT_TORQUE_CONSTANT_NM_A;
    iq_ramp = MITMode_LimitIqRamp(iq_target);

    Expt.Iq = iq_ramp;
    MITMode.torque_out = iq_ramp * MIT_TORQUE_CONSTANT_NM_A;


}



