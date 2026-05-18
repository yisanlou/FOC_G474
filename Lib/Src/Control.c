#include "Control.h"
#include "FOC.h"
#include "MY_ADC.h"
#include "main.h"


#define CURRENT_LOOP_DEFAULT_VBUS 48.0f
#define CURRENT_LOOP_TS           (1.0f / (float)PWM_FREQ)
#define CURRENT_LOOP_VOLT_LIMIT   (CURRENT_LOOP_DEFAULT_VBUS / SQRT3)


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

Ctrl_t SpdLoop = {0};
Ctrl_t PosLoop = {0};

MITMode_t MITMode = {0};

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

static float PI_Run(Ctrl_t *loop, float ref, float fdb)
{
    float integral_last = loop->integral;
    float out_raw;

    loop->err = ref - fdb;
    loop->integral = loop->integral + loop->err * loop->Ts;

    out_raw = loop->kp * loop->err + loop->ki * loop->integral ;
    loop->out = OutputLimitation(loop->out_max, loop->out_min, out_raw);

    if (loop->out != out_raw)
    {
      loop->integral = integral_last;
    }

    loop->last_err = loop->err;
    return loop->out;

}


void CurrLoop_Run(float RealQ, float RealD, float ExptQ, float ExptD, float *Volq, float *Vold)
{
    if ((Volq == 0) || (Vold == 0))
    {
        return;
    }



    // CurrLoop_UpdateLimit();

    *Volq = PI_Run(&IqLoop, ExptQ, RealQ);
    *Vold = PI_Run(&IdLoop, ExptD, RealD);
}

void SpdLoop_Run(void)
{
}

void PosLoop_Run(void)
{
}

void MITModeLoop_Run(void)
{
}



