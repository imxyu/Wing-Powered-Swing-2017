#ifndef __STRUCT_DEFINED_H
#define __STRUCT_DEFINED_H
#include "stm32f4xx.h"
/*************************************
MPU6050六轴原始数据
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
欧拉角数据
*************************************/
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}EulerAngleTypeDef;

typedef struct
{
	    float Offset;	  //允许偏差量
			float CurPos;
			float PrevPos;
			float CurAcc;
			float PrevSpeed;

	volatile float SetXPos;	  //设定位置
	volatile float SetYPos;	  //设定位置
	volatile float SetSpeed;  //设定速度
	
	volatile float CurXPos;	  //当前位置
	volatile float CurYPos;	  //当前位置
	volatile float CurSpeed;  //当前速度矢量

	volatile int32_t  PWM;	      //PWM
	volatile uint8_t  ShootFlag;
	volatile uint8_t  AdjustFlag;
	volatile uint8_t  ErrFlag;

	volatile uint32_t SetMaxPos;	  	//软件设定最大位置
	volatile uint32_t SetMaxPower;	  //软件设定最大力量
	volatile int32_t  SetMaxSpeed;	  //软件设定最大速度
		
}MotorTypeDef;

typedef struct
{
	float  SetPoint; 	//  设定目标 Desired Value 
	double  SumError;		//	误差累计 
		
	float  Proportion;      //  比例常数 Proportional Const 
	float  Integral;        //  积分常数 Integral Const
	float  Derivative;      //  微分常数 Derivative Const

	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]

}PIDTypeDef;


#endif

