#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"


#define ENC_RES      2097152UL          // 2^21
#define ENC_MASK     (ENC_RES - 1)      // 0x1FFFFF
#define POLE_PAIRS   14

#define TWO_PI       6.283185307179586f
#define PI           3.141592653589793f


extern volatile uint8_t Angle_Ready;

void Start_Angle_Read(void);
void Encoder_GetElectricalRad(uint32_t encoder_raw);
void Encoder_CalcSpeed(void);

typedef struct{
    uint32_t Raw_Num;
    uint32_t Offset;
    uint16_t Elec_Angle_16;

    uint32_t elec_raw;

    float Elec_Angle_pi;
    float Mech_Angle;
    float Mech_pi;



} Encoder_t ;


extern Encoder_t Encoder_Value;

#endif
