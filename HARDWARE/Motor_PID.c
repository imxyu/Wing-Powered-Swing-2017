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
���ܣ���ʼ��M1PID��M2PID��
����ֵ����
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
���ܣ����趨ֵͨ��PID�㷨ת����
M1�飨MA��MC����PWMֵ
δ��������޷�
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
���ܣ����趨ֵͨ��PID�㷨ת����
M2�飨MB��MD����PWMֵ
δ��������޷�
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
���ܣ�ΪM1��M2������
SetPoint��Kp��Ki��Kd��ֵ
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
��һ��
Ҫ�󣺴Ӿ�ֹ��ʼ��15s�ڿ��Ʒ�����������
���ɰ��˶���ʹ������ȶ����ڵ����ϻ���һ
�����Ȳ�����cm����ֱ�߶������Զ�ƫ���
��+-2.5cm�������нϺõ��ȶ��ԡ�

ʵ�֣��߶δ���1m��ƫ���Դ���2.5cm
**********************************/
void Mode_1()
{
	const float priod=1000.0;					//����ֵ1617
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
������
Ҫ�󣺿��趨�ڶ����򣬴Ӿ�ֹ��ʼ15����
�����趨����ڶ����߶β�����20cm

ʵ�֣�����
**********************************/
void Mode_3()
{
	const float priod=1000.0;					//����ֵ1617
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
������
Ҫ��5s�ڴ�30����45��ص�ԭ�㲢��ֹ

ʵ�֣��൱����
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
������
Ҫ��Բ�İ뾶��15-35cm��Χ�ڿ����ã�
30s�����ظ�3�Σ��켣��ָ���뾶��+-2.5cm
��Բ���ڡ�

ʵ�֣���������Ҫ����Ҫ����һ������λ
���������ĸ�Բ��������������ǿ��
�趨�뾶25cm��ʵ��26cm
**********************************/
void Mode_5()
{
	const float priod=1618.16;												//ͨ���������ڹ�ʽ��õ����� 1618.16ms
	const float Phase_Backward=PI/2.0;					//��ʱ��
	const float Phase_Forward=3*PI/2.0;					//˳ʱ��

	float deltaPhase[]={0,0.0873,0.1745,0.2619,0.3492};	//��λ����ֵ
	static unsigned int MoveTimeCnt=0;
	float set_y=0.0,set_x=0.0;
	float A=0.0;
	float R=35.0;																			//��Բ�뾶����λcm
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

