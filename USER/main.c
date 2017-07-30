#include "stm32f4xx.h"
#include "delay.h"
#include "key.h"
#include "oled.h"
#include "pwm.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "stdio.h"
#include "dac.h"
#include "Struct_Defined.h"
#include "timer.h"
#include "Motor_PID.h"
#include "ds18b20.h"

MotorTypeDef M1,M2;
EulerAngleTypeDef EulerAngle;
unsigned int mode;
int main(void)
{
	mode=0;
	
	delay_init(168);
	KEY_Init();
	OLED_Init();
	OLED_ShowString(0,0,"Loading...");
	delay_ms(200);
	OLED_Clear();
	MPU_Init();
	while(mpu_dmp_init())
	{
		OLED_ShowString(0,0,"MPU_DMP_INIT...");
 		delay_ms(200);
	}
	OLED_Clear();
	while(DS18B20_Init())
	{
		OLED_ShowString(0,0,"DS18B20 INIT...");
 		delay_ms(200);
	}
	OLED_Clear();
	MotorPID_Struct_Init();
	TIM3_GetData_Init(50-1,8400-1);
	TIM5_PWM_Init(2400-1,3-1);
	Motor_Init();
	while(1)
	{
		mode=3;
		//mode=KEY_Scan(1);
		//OLED_ShowNum(0,6,DS18B20_Get_Temp(),10,16);	
	}
}


