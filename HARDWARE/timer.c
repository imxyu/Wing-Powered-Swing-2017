#include "timer.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "Struct_Defined.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "Motor_PID.h"
#include "Motor_Control.h"
#include "oled.h"
#include "ds18b20.h"

extern int mode;
extern EulerAngleTypeDef EulerAngle;
extern MotorTypeDef M1,M2;
extern PIDTypeDef M1PID,M2PID;
char pwm_buff1[20],pwm_buff2[20];
/*******************************************
TIM3��ʼ������
��ʱ5ms��ÿ���ж�������ݲɼ�
*******************************************/
void TIM3_GetData_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/********************************************
��ʱ��3�жϷ�����
��mpu_dmp��ȡ���ݲ���ʾ��OLED
********************************************/
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		mpu_dmp_get_data(&EulerAngle.Pitch,&EulerAngle.Roll,&EulerAngle.Yaw);		//��ȡŷ����
		OLED_ShowAxis(EulerAngle.Pitch,EulerAngle.Roll,EulerAngle.Yaw);
		OLED_ShowNum(0,6,DS18B20_Get_Temp(),3,16);	
		//д��λ��
		M1.CurPos = EulerAngle.Pitch; 
		M2.CurPos = EulerAngle.Roll;	
		
		//�����ٶ�
//		M1.CurSpeed = M1.CurPos - M1.PrevPos;
//		M1.PrevPos = M1.CurPos;				
//		
//		M2.CurSpeed = M2.CurPos - M2.PrevPos;
//		M2.PrevPos = M2.CurPos;
		switch(mode)
		{
			case 1: Mode_1();break;
			case 2: Mode_2();break;
			case 3: Mode_3();break;
			case 4: Mode_4();break;
			case 5: Mode_5();break;
		}
		
		MotorMove1(PWM_Calc_M1(M1.CurPos));
		MotorMove2(PWM_Calc_M2(M2.CurPos));
		
//		sprintf(pwm_buff1,"%d",M1PID.SumError);
//		OLED_ShowString(0,6,pwm_buff1);
//		sprintf(pwm_buff2,"%d",M2PID.SumError);
//		OLED_ShowString(64,6,pwm_buff2);
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 					 										//����жϱ�־λ

}
