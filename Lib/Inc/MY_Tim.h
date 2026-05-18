#ifndef _MY_TIM_H_
#define _MY_TIM_H_

#include "hrtim.h"

#ifndef CCMRAM
#define CCMRAM __attribute__((section(".RamFunc")))
#endif

#endif 
