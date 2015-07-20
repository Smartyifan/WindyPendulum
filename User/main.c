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
#include "delay/delay.h"
#include "string.h" //该头文件，定义很多字符操作
#include "LED/led.h"
#include "TIMER/timer.h"
#include "PWM/pwm.h"
#include "PID/PID.h"
#include "HC05/hc05.h"
#include "JY901/jy901.h"
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
	x_PendPID.Kp = 10;			//Kp	
	x_PendPID.Ki = 0;			//Ki
	x_PendPID.Kd = 0;			//Kd
	x_PendPID.PIDout_H = 10;    //PID计算值输出限制
	x_PendPID.PIDout_L = -10;   //PID计算值输出限制
	PIDParamInit(&x_PendPID);	//清空输出

	/* y_PendPID ---------------------------------*/
	y_PendPID.Kp = 0;
	y_PendPID.Ki = 0;
	y_PendPID.Kd = 0;
	y_PendPID.PIDout_H = 10;
	x_PendPID.PIDout_L = -10;     //PID计算值输出限制
	PIDParamInit(&y_PendPID);
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
// 	JTAG_Set(1);                         //关闭JTAG,开启SWD

	/* 三色LED初始化 -----------------------------------*/
	LED_Init();                      
	delay_ms(10);   
	
	
	/* HC05初始化 --------------------------------------*/
	HC05Init(&HC05);	
	delay_ms(10);
	
	/* MPU6050初始化 ------------------------------------*/	
	JY901Init(&JY901);   	//用串口2驱动 
	delay_ms(10);
	
	/* 电机PID初始化 ------------------------------------*/
	Motor_Init();           //电机PWM初始化，用定时器4产生PWM波
	delay_ms(10);						
	
	/* 定时器中断初始化 ---------------------------------*/
	Timer3_Init(1999,719);	//定时器3初始化，每20ms一次中断，未使能定时器，只有当偏差计算完成时才启动定时器
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    Initial();
    while(1);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
