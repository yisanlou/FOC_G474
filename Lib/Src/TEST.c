#include "FOC.h"
#include "MY_ADC.h"
#include "arm_math.h"
#include "hrtim.h"
#include "main.h"

#define OPENLOOP_TWO_PI       6.28318530718f

float Theta = 0.0f;

void OpenloopVol(float Volq, float Vold, float Spd, float Poles)
{
    float Valpha;
    float Vbeta;
    float theta_step;

    if (Poles <= 0.0f)
    {
        SVPWM(0.0f, 0.0f, FOC_GetVbus());
        return;
    }

    theta_step = Spd * Poles / (float)PWM_FREQ;
    Theta = Theta + theta_step;

    while (Theta >= OPENLOOP_TWO_PI)
    {
        Theta = Theta - OPENLOOP_TWO_PI;
    }

    while (Theta < 0.0f)
    {
        Theta = Theta + OPENLOOP_TWO_PI;
    }

    FOC_State.Etheta = Theta;
    FOC_State.cosVal = arm_cos_f32(Theta);
    FOC_State.sinVal = arm_sin_f32(Theta);

    INVERSEPARK(Volq, Vold, FOC_State.cosVal, FOC_State.sinVal, &Valpha, &Vbeta);
    SVPWM(Valpha, Vbeta, FOC_GetVbus());

    FOC_UpdatePwmCompare();
}

void Correct_Offset(float Vold, float Etheta, float Poles)
{

    float cosVal = arm_cos_f32(Etheta);
    float sinVal = arm_sin_f32(Etheta);

    INVERSEPARK(0.0f, Vold, cosVal, sinVal, &FOC_State.Ualpha, &FOC_State.Ubeta);
    SVPWM(FOC_State.Ualpha, FOC_State.Ubeta, FOC_GetVbus());

    FOC_UpdatePwmCompare();

}

