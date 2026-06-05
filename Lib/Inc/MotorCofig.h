#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include <stdint.h>
#include "Encoder.h"

#define MOTOR_MODE_OFFSET      0
#define MOTOR_MODE_WORK        1

#define PHASE_POSITIVE         0
#define PHASE_NEGATIVE         1


#define MOTOR_MODE             MOTOR_MODE_WORK
#define PHASE_SEQUENCE         PHASE_POSITIVE


typedef enum
{
    MOTOR_GIM_6010_8 = 0,

    MOTOR_MODEL_COUNT
}MotorModel_t;

#define ACTIVE_MOTOR_MODEL     MOTOR_GIM_6010_8

typedef struct
{
    float Nominal_Voltage;
    float Nominal_Torque;
    float Stall_Torque;
    float Gear_Rate;
    float Nominal_Current;
    float Stall_Current;
    float PP_Res;
    float PP_Ind;
    float Q_Ind;
    float D_Ind;
    uint8_t Pole_pairs;
}Motor_t;




extern const Motor_t * const Motor;


#endif
