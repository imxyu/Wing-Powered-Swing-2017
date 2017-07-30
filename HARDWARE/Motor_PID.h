#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H
#include "Struct_Defined.h"

void MotorPID_Struct_Init(void);
int PWM_Calc_M1(float NextPoint);
int PWM_Calc_M2(float NextPoint);

void Set_SetPoint_M1(float point);
void Set_SetPoint_M2(float point);

void Set_Kp_M1(float kp);
void Set_Kp_M2(float kp);

void Set_Ki_M1(float ki);
void Set_Ki_M2(float ki);

void Set_Kd_M1(float kd);
void Set_Kd_M2(float kd);

void Mode_1(void);
void Mode_2(void);
void Mode_3(void);
void Mode_4(void);
void Mode_5(void);
#endif

