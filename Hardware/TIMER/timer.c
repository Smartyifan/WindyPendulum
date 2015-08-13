#include "TIMER/timer.h"
#include "usart/usart.h"
#include "PID/PID.h"
#include "LED/led.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"
#include "JY901/jy901.h"
#include "delay/delay.h"
#include "MotionCtr/motionctr.h"

//定时器3中断服务程序,每20ms进入一次定时器中断
void TIM3_IRQHandler(void)
{ 	
	if(TIM3->SR&0X0001)//溢出中断
	{
		if(DetectZeroDrift == DISABLE && DriftDetected == SUCCESS && MotorStart == ENABLE){
			if((MontionControl.MotionMode == SinglePend || MontionControl.MotionMode == DoublePend) && MontionControl.SinglePendParam.Charged == SUCCESS)
				(*MontionControl.CtrlFun)(0,0);		//单摆或双摆控制函数
		}
	}
	TIM3->SR&=~(1<<0);		//清除中断标志位 
}

//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void Timer3_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;//TIM3时钟使能    
 	TIM3->ARR=arr-1;  //设定计数器自动重装值   
	TIM3->PSC=psc-1;  //预分频器
	TIM3->DIER|=1<<0;   //允许更新中断				  
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,0,TIM3_IRQn);//抢占1，子优先级0，组2									 
}














