#include "Motor_Control.h"
#include "stm32f4xx.h"
#include "pwm.h"
#include "math.h"
#include "sys.h"

#define PWM_MAX 2400

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOF,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;       
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;        
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOE,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}
void MotorMove1(int pwm)
{
	if (pwm>PWM_MAX)
	{
		pwm=PWM_MAX;
	}
	if (pwm>0)
	{
		MA_Forward(pwm);
		MC_Backward(pwm);
	}
	else if (pwm<0)
	{
		MA_Backward(abs(pwm));
		MC_Forward(abs(pwm));
	}
}

void MotorMove2(int pwm)
{
	if (pwm>PWM_MAX)
	{
		pwm=PWM_MAX;
	}
	if (pwm>0)
	{
		MB_Forward(pwm);
		MD_Backward(pwm);
	}
	else if (pwm<0)
	{
		MB_Backward(abs(pwm));
		MD_Forward(abs(pwm));
	}
}

void MA_Forward(int pwm)
{
	MAIN1=1,MAIN2=0;
	TIM_SetCompare1(TIM5,pwm);
}

void MA_Backward(int pwm)
{
	MAIN1=0,MAIN2=1;
	TIM_SetCompare1(TIM5,pwm);
}

void MC_Forward(int pwm)
{
	MCIN1=1,MCIN2=0;
	TIM_SetCompare2(TIM5,pwm);
}

void MC_Backward(int pwm)
{
	MCIN1=0,MCIN2=1;
	TIM_SetCompare2(TIM5,pwm);
}

void MB_Backward(int pwm)
{
	MBIN1=1,MBIN2=0;
	TIM_SetCompare3(TIM5,pwm);
}

void MB_Forward(int pwm)
{
	MBIN1=0,MBIN2=1;
	TIM_SetCompare3(TIM5,pwm);
}

void MD_Backward(int pwm)
{
	MDIN1=1,MDIN2=0;
	TIM_SetCompare4(TIM5,pwm);
}

void MD_Forward(int pwm)
{
	MDIN1=0,MDIN2=1;
	TIM_SetCompare4(TIM5,pwm);
}

void MA_Stop()
{
	TIM_SetCompare1(TIM5,0);
	MAIN1=0,MAIN2=0;
}

void MB_Stop()
{
	TIM_SetCompare2(TIM5,0);
	MBIN1=0,MBIN2=0;
}

void MC_Stop()
{
	TIM_SetCompare3(TIM5,0);
	MCIN1=0,MCIN2=0;
}

void MD_Stop()
{
	TIM_SetCompare4(TIM5,0);
	MDIN1=0,MDIN2=0;
}