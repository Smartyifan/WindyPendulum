#include "PWM/pwm.h"
#include "sys/sys.h"
#include "delay/delay.h"
#include "usart/usart.h"
//psc=8
//arr=999
//f=72MHz/((59999+1)*(17+1))=50Hz
//初始占空比  50%
/*
*		ch1 = PB6
*       ch2 = PB7
*		ch3 = PB8
*		ch4 = PB9
*/
void Timer4_PWM_Init(u16 arr,u16 psc)
{
	u16 ALLCH_CRR = 3000;		//初始占空比
	
	RCC->APB1ENR |= 1<<2;		//TIM4
	RCC->APB2ENR |= 1<<3;		//GPIOB
	
	GPIOB->CRL&=0X00FFFFFF;	//PB6/7/8/9输出
	GPIOB->CRL|=0XBB000000;	//复用功能输出 
	GPIOB->CRH&=0XFFFFFF00;	
	GPIOB->CRH|=0X000000BB;

	TIM4->ARR = arr-1;
	TIM4 ->PSC = psc-1;
	TIM4->EGR=0x0001;  

	TIM4->CCMR1 |= 6<<4;		//PWM模式1
	TIM4->CCMR1 |= 1<<3;		//使能预装载
	TIM4->CCMR1 |= 6<<12;
	TIM4->CCMR1 |= 1<<11;
	
	TIM4->CCMR2 |= 6<<4;		
	TIM4->CCMR2 |= 1<<3;
	TIM4->CCMR2 |= 6<<12;
	TIM4->CCMR2 |= 1<<11;
	
	TIM4->CCER |= 1<<0;			//使能Timer2 PWM输出
	TIM4->CCER |= 1<<4;
	TIM4->CCER |= 1<<8;
	TIM4->CCER |= 1<<12;
	
	TIM4->CCR1 = ALLCH_CRR-1;				//初始占空比均为 5%
	TIM4->CCR2 = ALLCH_CRR-1;
	TIM4->CCR3 = ALLCH_CRR-1;
	TIM4->CCR4 = ALLCH_CRR-1;
	
	TIM4->CR1 = 0x0080;			//使能ARR重装载
	TIM4->CR1 |= 1<<0;			//使能定时器
}

void PWM_SET(s16 CH1_CCR,s16 CH2_CCR,s16 CH3_CCR,s16 CH4_CCR)
{
	if(CH1_CCR>4800)CH1_CCR = 4800;
	if(CH2_CCR>4800)CH2_CCR = 4800;
	if(CH3_CCR>4800)CH3_CCR = 4800;
	if(CH4_CCR>4800)CH4_CCR = 4800;

	if(CH1_CCR<=10)CH1_CCR = 10;
	if(CH2_CCR<=10)CH2_CCR = 10;
	if(CH3_CCR<=10)CH3_CCR = 10;
	if(CH4_CCR<=10)CH4_CCR = 10;

	TIM4->CR1&=~(1<<0);        //关闭定时器4
	
	TIM4->CCR1 = CH1_CCR-1;		//设置占空比ZKB
	TIM4->CCR2 = CH2_CCR-1;
	TIM4->CCR3 = CH3_CCR-1;
	TIM4->CCR4 = CH4_CCR-1;
	

	TIM4->CR1|=0x0001;   //打开定时器4，开始输出PWM波

}


void Motor_Start_Up(void)
{
	PWM_SET(0,0,0,0);
}

void Motor_Init(void)
{
	Timer4_PWM_Init(4800,1); //空心杯电机的一般驱动频率为10K-20K,故取15KHz
	Motor_Start_Up();
}

void Motor_Stop(void)
{
	TIM4->CR1&=~(1<<0);        //关闭定时器4
}

