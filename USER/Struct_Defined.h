#ifndef __STRUCT_DEFINED_H
#define __STRUCT_DEFINED_H
#include "stm32f4xx.h"
/*************************************
MPU6050����ԭʼ����
*************************************/
typedef struct
{
	short   AccX;	
	short		AccY;	
	short		AccZ;
	short		GyroX;
	short		GyroY;
	short		GyroZ;		
}MPU6050_AxisTypeDef;
/*************************************
ŷ��������
*************************************/
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}EulerAngleTypeDef;

typedef struct
{
	    float Offset;	  //����ƫ����
			float CurPos;
			float PrevPos;
			float CurAcc;
			float PrevSpeed;

	volatile float SetXPos;	  //�趨λ��
	volatile float SetYPos;	  //�趨λ��
	volatile float SetSpeed;  //�趨�ٶ�
	
	volatile float CurXPos;	  //��ǰλ��
	volatile float CurYPos;	  //��ǰλ��
	volatile float CurSpeed;  //��ǰ�ٶ�ʸ��

	volatile int32_t  PWM;	      //PWM
	volatile uint8_t  ShootFlag;
	volatile uint8_t  AdjustFlag;
	volatile uint8_t  ErrFlag;

	volatile uint32_t SetMaxPos;	  	//����趨���λ��
	volatile uint32_t SetMaxPower;	  //����趨�������
	volatile int32_t  SetMaxSpeed;	  //����趨����ٶ�
		
}MotorTypeDef;

typedef struct
{
	float  SetPoint; 	//  �趨Ŀ�� Desired Value 
	double  SumError;		//	����ۼ� 
		
	float  Proportion;      //  �������� Proportional Const 
	float  Integral;        //  ���ֳ��� Integral Const
	float  Derivative;      //  ΢�ֳ��� Derivative Const

	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]

}PIDTypeDef;


#endif

