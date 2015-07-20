#include "TIMER/timer.h"
#include "usart/usart.h"
#include "PID/PID.h"
#include "LED/led.h"

double x_TargetAngle=0,y_TargetAngle=0;		//��ʵ��Ӧ��ʱ��������ֵ��ȫ�ֱ���������Χ�豸���͸������ڣ�������ֲʱ��ע��������ֵ����Դ
//��ʱ��3�жϷ������,ÿ5ms����һ�ζ�ʱ���ж�
void TIM3_IRQHandler(void)
{ 	
    static u16 _500ms;	
	if(TIM3->SR&0X0001)//����ж�
	{				
		if(_500ms == 25)
		{
			_500ms = 0;
			R_LED=~R_LED;     //��������ָʾ
		}
		_500ms++;
	}
	TIM3->SR&=~(1<<0);//����жϱ�־λ 
}
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void Timer3_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr-1;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM3->PSC=psc-1;  //Ԥ��Ƶ��
	TIM3->DIER|=1<<0;   //��������ж�				  
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  	MY_NVIC_Init(2,3,TIM3_IRQn);//��ռ1�������ȼ�3����2									 
}














