/**
  ******************************************************************************
  * @file    	E:\GitRepo\WindyPendulum\Hardware\MotionCtr\motionctr.c
  * @author  	贾一帆
  * @version	V0.0
  * @date  		2015-08-01 20:00:29
  * @brief    	风力摆的运动控制
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */  
#include "MotionCtr/motionctr.h"
#include <math.h>
/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "PID/pid.h"
#include "PWM/pwm.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define _2Pi  6.28
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MotionCtrStr MontionControl;		//运动控制结构体
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void MotionCtrParamInit(MotionCtrStr * MotionCtrl){
	MotionCtrl->MotionMode = SinglePend;	//初始化模式为单摆运动模式
	
	/* 单摆模式参数初始化 -------------------------------*/
	MotionCtrl->SinglePendParam.Pend = Rol;	
	MotionCtrl->SinglePendParam.Period = 0;
	MotionCtrl->SinglePendParam.Amplitude = 0;
	
	/* 双摆模式参数初始化 -----------------------------*/
	MotionCtrl->DoublePendParam.RolPeriod = 0;
	MotionCtrl->DoublePendParam.RolAmplitude = 0;
	MotionCtrl->DoublePendParam.PitchPeriod = 0;
	MotionCtrl->DoublePendParam.PitchAmplitude = 0;
	
	/* 稳定点模式参数初始化 -----------------------------*/
	MotionCtrl->StableParam.PitchExpect = 0;
	MotionCtrl->StableParam.RolExpect = 0;
	
	/* 轨迹跟踪模式初始化 -------------------------------*/
}


/**
  *@brief   SinglePendCtrl		单摆模式下的控制函数
  *@param   None
  *@retval  None
  */
void SinglePendCtrl(float RolCule,float PitchCule){
	static u16 Tick;
	float Force;		//驱动力
	/* 改变时间 -------------------------------------------------*/
	Tick++;
	if(Tick >= MontionControl.SinglePendParam.Period)Tick = 0;
	
	/* 计算驱动力 -----------------------------------------------*/
	if(MontionControl.SinglePendParam.Period <5){			//周期在5s之内
		Force = MontionControl.SinglePendParam.Amplitude * sin(Tick*(_2Pi/MontionControl.SinglePendParam.Period));
	}else{													//周期在5s以上	
		
	}
	
	
	switch(MontionControl.SinglePendParam.Pend){
		case Rol:{
			if(Force >=0){				//Rol
			motor4 = 0;
			motor2 = Force;
		}else{
			motor4 = -Force;
			motor2 = 0;
		}
			break;
		}
		case Pitch:{
		if(Force >= 0){			//Pitch
			motor3 = 0;
			motor1 = Force;
		}else{
			motor3 = -Force;
			motor1 = 0;
		}

			break;
		}
	}
	
	/* 根据motor值驱动电机 -------------------------*/
	PWM_SET(motor1,motor2,motor3,motor4);

}


/**
  *@brief   ConePendCtrl	双摆模式下的控制函数
  *@param   None
  *@retval    None
  */
void DoublePendCtrl(float RolCule,float PitchCule,u16 Tick){

}

/**
  *@brief   StablePlotCtrl	稳定点模式下的控制函数
  *@param   None
  *@retval    None
  */
void StablePlotCtrl(float RolCule,float PitchCule,u16 Tick){

}


/**
  *@brief   TrackedCtrl		轨迹跟踪模式下的控制函数
  *@param   None
  *@retval    None
  */
void TrackedCtrl(float RolCule,float PitchCule,u16 Tick){
	
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
