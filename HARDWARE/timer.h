#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f4xx.h"
#include "sys.h"

void TIM3_GetData_Init(u16 arr,u16 psc);
void TIM5_PWM_Init(u32 arr,u32 psc);
void TIM1_PWM_Init(u32 arr,u32 psc);

//#define DEBUG_PROBE1_ON 	PFout(0)=1
//#define DEBUG_PROBE1_OFF 	PFout(0)=0
//#define DEBUG_PROBE2_ON 	PFout(1)=1
//#define DEBUG_PROBE2_OFF 	PFout(1)=0

#endif
