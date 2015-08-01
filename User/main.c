/**
  ******************************************************************************
  * @file    STM32工程/工程模版/User
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * No attention
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "sys/sys.h"  		
#include "delay/delay.h"	//延时
#include "string.h"			//该头文件，定义很多字符操作
#include "LED/led.h"		//LED
#include "TIMER/timer.h"	//定时器
#include "PWM/pwm.h"		//电机控制
#include "PID/PID.h"		//PID计算
#include "HC05/hc05.h"		//HC-05蓝牙串口
#include "JY901/jy901.h"	//JY-901九轴姿态传感器
#include "MotionCtr/motionctr.h"	//运动控制

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*--------------------------------------------------
**PID复位，包括PID中P\I\D三个参数的设置以及偏差清零
**无入口参数
**无出口参数
---------------------------------------------------*/

void ParamSet(){
	
	/* HC-05 --------------------------------------*/
	HC05.USARTBASE = USART1;	//使用串口1驱动	
	
	/* MPU6050 ------------------------------------*/
	JY901.USARTBASE = USART2;   //蓝牙陀螺仪即JY901波特率为115200时，输出速率为100HZ,波特率为9600时，输出速率为20HZ

	/* x_PendPID ----------------------------------*/
	x_PendPID.Kp = 0;	    //Kp---OK	0.85-0.90
	x_PendPID.Ki = 0;			//Ki
	x_PendPID.Kd = 0;			//Kd        0.1-1.0
// 	x_PendPID.PIDout_H = 10;    //PID计算值输出限制
// 	x_PendPID.PIDout_L = -10;   //PID计算值输出限制
	PIDParamInit(&x_PendPID);	//清空输出

	/* y_PendPID ---------------------------------*/
	y_PendPID.Kp = 0;
	y_PendPID.Ki = 0;
	y_PendPID.Kd = 0;
// 	y_PendPID.PIDout_H = 10;
// 	y_PendPID.PIDout_L = -10;     //PID计算值输出限制
	PIDParamInit(&y_PendPID);
	
	/* 运动参数设置 ----------------------------*/
	MotionCtrParamInit(&MontionControl);	
}
/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void Initial()
{
	/* 参数设置 ----------------------------------------*/
	ParamSet();
	
	/* 中断分组与延时器初始化 ---------------------------*/
	NVIC_Configuration();	//中断分组2
	delay_init();
    
	/* SWD初始化 ---------------------------------------*/
	JTAG_Set(1);                         //关闭JTAG,开启SWD

	/* 三色LED初始化 -----------------------------------*/
	LED_Init();                      
	delay_ms(10);   
	
	
	/* HC05初始化 --------------------------------------*/
	HC05Init(&HC05);	
	delay_ms(10);
	
	/* JY-901初始化 ------------------------------------*/	
	JY901Init(&JY901);   	//用串口2驱动 
	delay_ms(10);
	
	/* 电机PID初始化 ------------------------------------*/
		/* 电机PWM初始化，用定时器4产生PWM波，但在此函数中
		未启动定时器4，且初始PWM占空比均为1/4800*/
 	Motor_Init();           
	delay_ms(10);						
	/* 定时器中断初始化 ---------------------------------*/
	//用于定时查看PIDout，使用HC-05发送至XJI魔幻上位机画图
 	Timer3_Init(5000,720);	//定时器3初始化 T = (720*5000)/72 000 000 = 50 ms
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    Initial();
	
    while(1){

	}
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
