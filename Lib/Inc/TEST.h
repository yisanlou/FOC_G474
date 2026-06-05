#ifndef __TEST_H_
#define __TEST_H_


#include "FOC.h"
#include "main.h"




void OpenloopVol(float Volq, float Vold, float Spd, float Poles);
void Correct_Offset(float Vold, float Etheta, float Poles);


#endif