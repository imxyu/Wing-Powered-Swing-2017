#include "Motor_PID.h"
#include "stm32f4xx.h"
#include "Struct_Defined.h"
#include "Motor_Control.h"
#include "math.h"

#define PWM_MAX 	2400
#define PI 				3.14159
PIDTypeDef M1PID,M2PID;
extern MotorTypeDef M1,M2;
extern EulerAngleTypeDef EulerAngle;

/***********************************
功能：初始化M1PID、M2PID，
所有值归零
***********************************/
void MotorPID_Struct_Init()
{
	M1PID.SumError=0;
	M1PID.LastError=0;
	M1PID.PrevError=0;
	M1PID.Derivative=0;
	M1PID.Integral=0;
	M1PID.Proportion=0;
	M1PID.SetPoint=0;
	
	M2PID.SumError=0;
	M2PID.LastError=0;
	M2PID.PrevError=0;
	M2PID.Derivative=0;
	M2PID.Integral=0;
	M2PID.Proportion=0;
	M2PID.SetPoint=0;
}
/***********************************
功能：将设定值通过PID算法转换成
M1组（MA、MC）的PWM值
未加入积分限幅
***********************************/
int PWM_Calc_M1(float NextPoint)
{
	float iError,dError;
	iError=M1PID.SetPoint-NextPoint;
	M1PID.SumError+=iError;
	dError=iError-M1PID.LastError;
	M1PID.LastError=iError;
//	if (M1PID.SumError>2300)
//		M1PID.SumError=2300;
//	if (M1PID.SumError<-2300)
//		M1PID.SumError=-2300;
	
	return (int)		M1PID.Proportion * iError
								+	M1PID.Integral * M1PID.SumError
								+ M1PID.Derivative * dError;
}
/***********************************
功能：将设定值通过PID算法转换成
M2组（MB、MD）的PWM值
未加入积分限幅
***********************************/
int PWM_Calc_M2(float NextPoint)
{
	float iError,dError;
	iError=M2PID.SetPoint-NextPoint;
	M2PID.SumError+=iError;
	dError=iError-M2PID.LastError;
	M2PID.LastError=iError;
	if (M2PID.SumError>2300)
		M2PID.SumError=2300;
	if (M2PID.SumError<-2300)
		M2PID.SumError=-2300;
	
	return (int)		M2PID.Proportion * iError
								+	M2PID.Integral * M2PID.SumError
								+ M2PID.Derivative * dError;
}
/***********************************
功能：为M1、M2电机组的
SetPoint、Kp、Ki、Kd赋值
***********************************/
void Set_SetPoint_M1(float point)
{
	M1PID.SetPoint=point;
}
	
void Set_Kp_M1(float kp)
{
	M1PID.Proportion=kp;
}
	
void Set_Ki_M1(float ki)
{
	M1PID.Integral=ki;
}
	
void Set_Kd_M1(float kd)
{
	M1PID.Derivative=kd;
}

void Set_SetPoint_M2(float point)
{
	M2PID.SetPoint=point;
}
	
void Set_Kp_M2(float kp)
{
	M2PID.Proportion=kp;
}
	
void Set_Ki_M2(float ki)
{
	M2PID.Integral=ki;
}
	
void Set_Kd_M2(float kd)
{
	M2PID.Derivative=kd;
}

/***********************************
第一问
要求：从静止开始，15s内控制风力摆做类似
自由摆运动，使激光笔稳定地在地面上画出一
条长度不短于cm，的直线段其线性度偏差不大
于+-2.5cm，并具有较好的稳定性。

实现：线段大于1m，偏差略大于2.5cm
**********************************/
void Mode_1()
{
	const float priod=1000.0;					//计算值1617
	static unsigned int MoveTimeCnt=0;
	float set_y=0.0;
	float A=0.0;
	float Normalization=0.0;
	float Omega=0.0;
	
	MoveTimeCnt+=5;
	Normalization=(float)MoveTimeCnt/priod;
	Omega=2.0*PI*Normalization;
	A=atan((25.0/(65+11.5)))*57.2958f;
	set_y=1.11*A*sin(Omega);
	
	Set_SetPoint_M1(set_y);
	Set_Kp_M1(35);
	Set_Ki_M1(0.04);
	Set_Kd_M1(5000);
	
	Set_SetPoint_M2(0);
	Set_Kp_M2(35);
	Set_Ki_M2(0.04);
	Set_Kd_M2(5000);
}

void Mode_2()
{
	Set_SetPoint_M1(15);
	Set_Kp_M1(35);
	Set_Ki_M1(0.040);
	Set_Kd_M1(5000);
}
/***********************************
第三问
要求：可设定摆动方向，从静止开始15秒内
按照设定方向摆动，线段不短于20cm

实现：可以
**********************************/
void Mode_3()
{
	const float priod=1000.0;					//计算值1617
	static unsigned int MoveTimeCnt=0;
	float set_y=0.0,set_x=0.0;
	float Angle=30;
	float A=0.0,Ax=0.0,Ay=0.0;
	float Normalization=0.0;
	float Omega=0.0;
	
	MoveTimeCnt+=5;
	Normalization=(float)MoveTimeCnt/priod;
	Omega=2.0*PI*Normalization;
	A=atan((25.0/(65+11.5)))*57.2958f;
	Ax=A*cos(Angle*0.017453);
	Ay=A*sin(Angle*0.017453);
	set_x=Ax*sin(Omega);
	set_y=Ay*sin(Omega);
	
	Set_SetPoint_M1(set_y);
	Set_Kp_M1(35);
	Set_Ki_M1(0.04);
	Set_Kd_M1(5000);
	
	Set_SetPoint_M2(set_x);
	Set_Kp_M2(35);
	Set_Ki_M2(0.04);
	Set_Kd_M2(5000);
}

/***********************************
第四问
要求：5s内从30°至45°回到原点并静止

实现：相当可以
**********************************/
void Mode_4()
{
	Set_SetPoint_M1(0);
	Set_SetPoint_M2(0);
	
	Set_Kd_M1(15000);
	Set_Kd_M2(15000);
	
	Set_Ki_M1(0.0);
	Set_Ki_M2(0.0);
	
	Set_Kp_M1(35);
	Set_Kp_M2(35);

}
/***********************************
第五问
要求：圆的半径在15-35cm范围内可设置，
30s内须重复3次，轨迹在指定半径在+-2.5cm
的圆环内。

实现：可以满足要求。需要加入一定的相位
补偿来画的更圆。抗干扰能力较强。
设定半径25cm，实测26cm
**********************************/
void Mode_5()
{
	const float priod=1618.16;												//通过单摆周期公式算得的周期 1618.16ms
	const float Phase_Backward=PI/2.0;					//逆时针
	const float Phase_Forward=3*PI/2.0;					//顺时针

	float deltaPhase[]={0,0.0873,0.1745,0.2619,0.3492};	//相位补偿值
	static unsigned int MoveTimeCnt=0;
	float set_y=0.0,set_x=0.0;
	float A=0.0;
	float R=35.0;																			//画圆半径，单位cm
	float Normalization=0.0;
	float Omega=0.0;

	MoveTimeCnt+=5;
	Normalization=(float)MoveTimeCnt/priod;
	Omega=2.0*3.141592*Normalization;
	A=atan((R/(65+11.5)))*57.2958f;
	set_y=A*sin(Omega+deltaPhase[1]);
	set_x=A*sin(Omega+Phase_Forward);
	Set_SetPoint_M1(set_y);
	Set_Kp_M1(70);
	Set_Ki_M1(0.1);
	Set_Kd_M1(8000);
	
	Set_SetPoint_M2(set_x);
	Set_Kp_M2(70);
	Set_Ki_M2(0.1);
	Set_Kd_M2(8000);
}

