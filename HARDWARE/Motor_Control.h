#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
void Motor_Init(void);
void MA_Forward(int pwm);
void MA_Backward(int pwm);
void MC_Forward(int pwm);
void MC_Backward(int pwm);

void MB_Forward(int pwm);
void MB_Backward(int pwm);
void MD_Forward(int pwm);
void MD_Backward(int pwm);

void MA_Stop(void);
void MB_Stop(void);
void MC_Stop(void);
void MD_Stop(void);

void MotorMove1(int pwm);
void MotorMove2(int pwm);

#define MAIN1 PEout(1)
#define MAIN2 PEout(2)
#define MCIN1 PEout(3)
#define MCIN2 PEout(4)

#define MBIN1 PEout(12)
#define MBIN2 PEout(13)
#define MDIN1 PEout(14)
#define MDIN2 PEout(15)
#endif 

