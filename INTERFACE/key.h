#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9 ) //PG9
#define KEY1 		GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_11) //PG11
#define KEY2 		GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_13) //PG13
#define KEY3  	GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15) //PG15

#define BOMA0 	GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_6)  //PG6
#define BOMA1 	GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8)  //PG8
#define BOMA2 	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)  //PC9
#define BOMA3 	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)  //PC7


#define KEY0_PRES 	0
#define KEY1_PRES	  1
#define KEY2_PRES	  2
#define KEY3_PRES   3

void KEY_Init(void);	//IO��ʼ��
void BOMA_Init(void);
int  KEY_Scan(int);  		//����ɨ�躯��	
void Mode_Switch(int mode);

#endif
