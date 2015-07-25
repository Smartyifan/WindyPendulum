#include "TIMER/timer.h"
#include "usart/usart.h"
#include "PID/PID.h"
#include "LED/led.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"
#include "JY901/jy901.h"
#include "delay/delay.h"

float x_TargetAngle=0.0,y_TargetAngle=0.0;		//��ʵ��Ӧ��ʱ��������ֵ��ȫ�ֱ���������Χ�豸���͸������ڣ�������ֲʱ��ע��������ֵ����Դ
//��ʱ��3�жϷ������,ÿ5ms����һ�ζ�ʱ���ж�
void TIM3_IRQHandler(void)
{ 	
     static u16 _1s,_2s;	
	if(TIM3->SR&0X0001)//����ж�
	{			

// 		SimplePlotSend(&HC05,x_PendPID.PIDout,y_CurrentError,0,0);
 		if(_1s == 20)
 		{
 			_1s = 0;
			_2s++;
			//��������ָʾ
 			R_LED=~R_LED;     
			//���������ʾ
// 			if(MotorStart == ENABLE)  B_LED=~R_LED;
 		}
		if(_2s == 2)
 		{
			_2s = 0;   
			//���������ʾ
			if(MotorStart == ENABLE)
			{
			/*��ɫLED��˸ ----------------------------------------------------*/
			B_LED=~B_LED;           //��ʾ������ƴ򿪣�ÿ2s������˸����
			}
 		}
 		_1s++;
	}
	TIM3->SR&=~(1<<0);		//����жϱ�־λ 
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
  	MY_NVIC_Init(0,0,TIM3_IRQn);//��ռ1�������ȼ�3����2									 
}














