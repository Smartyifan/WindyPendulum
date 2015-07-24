/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\PID\PID.c
  * @author  贾一帆
  * @version V3.5.0
  * @date    2015-07-02 16:01:43
  * @brief   通用增量式PID计算
  ******************************************************************************
  * @attention
  *	使用步骤：
  *		1.定义相应结构体并使用PIDParamInit初始化相关参数
  *		2.在使用前先给Kp\Ki\Kd赋值
  *		3.通过PIDCalculater得到PIDout输出
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "PID/PID.h"
#include "usart/usart.h"
#include "PWM/pwm.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PIDStruct x_PendPID,y_PendPID;		//摆杆PID控制结构体
/* Private function prototypes -----------------------------------------------*/
void PIDGetError(PIDStruct * PID,float error);
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   PIDCalculatre
  *@param   None;
  *@retval  None
  */
void PIDParamInit(PIDStruct * PID){
	
	/* PIDout Init------------*/
    PID->PIDout = 0;	//PIDout初始化为0	
	/* errors Init------------*/
    PID->error[0] = 0;	//清空偏差数组
	PID->error[1] = 0;
    PID->error[2] = 0;
}

/**
  *@brief   PIDCalculater   增量式PID计算
  *@param   PIDStruct * PID	指向PID结构体的指针
  *@retval  None
  */
// extern double abs(double __x); //不加该声明语句使用abs时会出现警告
void PIDCalculater(PIDStruct * PID,float error){  
	
	/* 得到偏差量 ----------------------------------------------------*/
    PIDGetError(PID,error);		//获得偏差并更新偏差量数组
	/* 计算PID输出 ---------------------------------------------------*/
    PID->Pout = PID->Kp*(PID->error[0] - PID->error[1]);	//计算Pout
	
	/* 分离积分 ------------------------------------------------------*/
	if((PID->error[0] > -0.5)  &&  (PID->error[0] < 0.5)){
		PID->Iout = PID->Ki* PID->error[0];						//计算Iout
	}
	
    PID->Dout = PID->Kd*(PID->error[0] + PID->error[2] -2*PID->error[1]);	//计算Dout
	//下面PID输出的写法是不是有问题？
    PID->PIDout += PID->Pout + PID->Iout + PID->Dout;		//得到PIDout
    /* 高低阈值限制 --------------------------------------------------*/
// 	if(PID->PIDout > PID->PIDout_H)PID->PIDout = PID->PIDout_H;		//PIDout最高值限制
// 	if(PID->PIDout < PID->PIDout_L)PID->PIDout = PID->PIDout_L;		//PIDout最高值限制
}

/**
  *@brief   PIDGetError		
  *@param   PIDStruct * PID	指向PID结构体的指针
  *			double error	当前偏差量			
  *@retval  None
  */
void PIDGetError(PIDStruct * PID,float error){
	PID->error[2] = PID->error[1];	//数组移位
	PID->error[1] = PID->error[0];
    PID->error[0] = error;
}

/**
  *@brief   PIDControl	根据PIDout控制外部设备，内容根据实际情况书写
  *@param   PIDStruct * PID	指向PID结构体的指针	
  *@retval  None
  */

s16 motor1=0,motor2=0,motor3=0,motor4=0;
void PIDControl(void){
	//计算每个电机的PWM变化是升高还是降低//代码移植时要改动
	motor1 = TIM4->CCR1 +((s16)(y_PendPID.PIDout));
	motor2 = TIM4->CCR2 +((s16)(x_PendPID.PIDout));
	motor3 = TIM4->CCR3 -((s16)(y_PendPID.PIDout));
	motor4 = TIM4->CCR4 -((s16)(x_PendPID.PIDout));

	PWM_SET(motor1,motor2,motor3,motor4);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
