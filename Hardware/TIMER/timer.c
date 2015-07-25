#include "TIMER/timer.h"
#include "usart/usart.h"
#include "PID/PID.h"
#include "LED/led.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"
#include "JY901/jy901.h"
#include "delay/delay.h"

float x_TargetAngle=0.0,y_TargetAngle=0.0;		//在实际应用时，这两个值是全局变量，有外围设备发送给动力摆，所以移植时，注意这两个值的来源
//定时器3中断服务程序,每5ms进入一次定时器中断
void TIM3_IRQHandler(void)
{ 	
     static u16 _1s,_2s;	
	if(TIM3->SR&0X0001)//溢出中断
	{			

// 		SimplePlotSend(&HC05,x_PendPID.PIDout,y_CurrentError,0,0);
 		if(_1s == 20)
 		{
 			_1s = 0;
			_2s++;
			//程序运行指示
 			R_LED=~R_LED;     
			//电机启动提示
// 			if(MotorStart == ENABLE)  B_LED=~R_LED;
 		}
		if(_2s == 2)
 		{
			_2s = 0;   
			//电机启动提示
			if(MotorStart == ENABLE)
			{
			/*蓝色LED闪烁 ----------------------------------------------------*/
			B_LED=~B_LED;           //表示电机控制打开，每2s快速闪烁两次
			}
 		}
 		_1s++;
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
 	TIM3->ARR=arr-1;  //设定计数器自动重装值//刚好1ms    
	TIM3->PSC=psc-1;  //预分频器
	TIM3->DIER|=1<<0;   //允许更新中断				  
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(0,0,TIM3_IRQn);//抢占1，子优先级3，组2									 
}














