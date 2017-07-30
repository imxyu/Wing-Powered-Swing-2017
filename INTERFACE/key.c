#include "key.h"
#include "delay.h" 
#include "oled.h"

//������ʼ������
void KEY_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15; //KEY0 KEY1 KEY2 KEY3��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOG9,11,13,15
} 

void BOMA_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOG,Cʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8; //BOMA0 BOMA1��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOG6,8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_7; //BOMA2 BOMA3��Ӧ����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC9,7
}
/***************************************
����ɨ��
int KEY_Scan(int mode)
mode:0,��֧��������;1,֧��������
ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
***************************************/
int KEY_Scan(int mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)
		key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY0==0)
			return 1;
		else if(KEY1==0)
			return 2;
		else if(KEY2==0)
			return 3;
		else if(KEY3==0)
			return 4;
	}
	else if(KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1)
		key_up=1; 	    
 	return 0;// �ް�������
}

/***************************************
���ݰ���ѡ��ģʽ
void Mode_Switch(void)
***************************************/
void Mode_Switch(int mode)
{
	mode = KEY_Scan(1);
	switch(mode)
	{
		case 1:
		{
			//KEY0���º�ִ�еĴ���
			break;
		}
		case 2:
		{
			//KEY1���º�ִ�еĴ���
			break;
		}
		case 3:
		{
			//KEY2���º�ִ�еĴ���
			break;
		}
		case 4:
		{
			//KEY3���º�ִ�еĴ���
			break;
		}
		default:
			break;
	}
}


















