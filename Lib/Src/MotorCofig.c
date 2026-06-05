#include "MotorCofig.h"


static const Motor_t Motor_Table[MOTOR_MODEL_COUNT] =
{
    [MOTOR_GIM_6010_8] =
    {
        .Nominal_Voltage = 48.f,
        .Nominal_Torque = 4.6f,
        .Stall_Torque = 17.91f,
        .Gear_Rate = 8.f,
        .Nominal_Current = 2.8f,
        .Stall_Current = 17.19f,
        .PP_Res = 1.45f,
        .PP_Ind = 0.00039f,
        .Q_Ind = 0.000195f,
        .D_Ind = 0.000195f,
        .Pole_pairs = 14,
    },
};

const Motor_t * const Motor = &Motor_Table[ACTIVE_MOTOR_MODEL];
