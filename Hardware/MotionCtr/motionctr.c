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
#include "JY901/jy901.h"
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
  *@param   MotionCtrStr * MotionCtrl	运动控制结构体
  *@retval  None
  */
void MotionCtrParamInit(MotionCtrStr * MotionCtrl){
	MotionCtrl->MotionMode = Stop;	//初始化模式为停止运动模式
	
	/* 单摆模式参数初始化 -------------------------------*/
	MotionCtrl->SinglePendParam.Angle = 0;
	MotionCtrl->SinglePendParam.RolPeriod = 0;
	MotionCtrl->SinglePendParam.RolAmplitude = 0;
	MotionCtrl->SinglePendParam.PitchPeriod = 0;
	MotionCtrl->SinglePendParam.PitchAmplitude = 0;
	
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
  *@param   
  *@retval  None
  */
void SinglePendCtrl(float RolCule,float PitchCule){
	static u16 RolTick = 0,PitchTick = 0;	//时刻
	float RolpForce,RolnForce,PitchpForce,PitchnForce;				//驱动力
		
	/* 改变时间 -------------------------------------------------*/
	RolTick++;	PitchTick++;
	if(RolTick 	>= 	MontionControl.SinglePendParam.RolPeriod)			RolTick = 0;
	if(PitchTick	>=	MontionControl.SinglePendParam.PitchPeriod)		PitchTick = 0;
	if(MontionControl.MotionMode	==	Stop)							{RolTick = 0;PitchTick = 0;}

	/* 计算驱动力 -----------------------------------------------*/
	if(RolTick < MontionControl.SinglePendParam.RolPeriod/2){
		RolpForce = RolpPendPID.PIDout * 
						sin(	RolTick *	(_2Pi/MontionControl.SinglePendParam.RolPeriod)	);
	}else if(RolTick >= MontionControl.SinglePendParam.RolPeriod/2){
		RolnForce =  -RolnPendPID.PIDout * 
						sin(	RolTick *	(_2Pi/MontionControl.SinglePendParam.RolPeriod)	);
	}
	
	if(PitchTick < MontionControl.SinglePendParam.PitchPeriod/2){
		PitchpForce = PitchpPendPID.PIDout * 
						sin(	PitchTick *	(_2Pi/MontionControl.SinglePendParam.PitchPeriod)	);
	}else if(PitchTick >= MontionControl.SinglePendParam.PitchPeriod/2){
		PitchnForce = -PitchnPendPID.PIDout * 
						sin(	PitchTick *	(_2Pi/MontionControl.SinglePendParam.PitchPeriod)	);
	}
	
	/* 设置motor值 ----------------------------------------------*/
	motor1 = PitchpForce;
	motor2 = RolnForce;
	motor3 = PitchnForce;
	motor4 = RolpForce;
	
	/* 根据motor值驱动电机 --------------------------------------*/
	PWM_SET(motor1,motor2,motor3,motor4);
}


/**
  *@brief   ConePendCtrl	双摆模式下的控制函数
  *@param   float RolCule	当前Rol角度
			float PitchCule	当前Pitch角度
  *@retval  None
  */
void DoublePendCtrl(float RolCule,float PitchCule){
	static u16 RolTick = 0,PitchTick = 0;	//时刻
	float RolForce,PitchForce;				//驱动力
	
	/* 改变时间 -------------------------------------------------*/
	RolTick++;PitchTick++;
	if(RolTick >= MontionControl.DoublePendParam.RolPeriod)RolTick = 0;
	if(PitchTick >= MontionControl.DoublePendParam.PitchPeriod)PitchTick = 0;
	if(MontionControl.MotionMode == Stop){RolTick = 0;PitchTick = 0;}
	/* 计算驱动力 -----------------------------------------------*/
	RolForce = MontionControl.DoublePendParam.RolAmplitude * sin(RolTick*(_2Pi/MontionControl.DoublePendParam.RolPeriod));
	PitchForce = MontionControl.DoublePendParam.PitchAmplitude * sin(PitchTick*(_2Pi/MontionControl.DoublePendParam.PitchPeriod));
	
	if(RolForce >0){ 
		motor2 = RolForce;
		motor4 = 0;
	}else{
		motor2 = 0;
		motor4 = -RolForce;
	}
	
	if(PitchForce>0){
		motor1 = PitchForce;
		motor3 = 0;
	}else{
		motor1 = 0;
		motor3 = -PitchForce;
	}
	
	/* 根据motor值驱动电机 --------------------------------------*/
	PWM_SET(motor1,motor2,motor3,motor4);
}

/**
  *@brief   StablePlotCtrl	稳定点模式下的控制函数
  *@param   float RolCule	当前Rol角度
			float PitchCule	当前Pitch角度
  *@retval  None
  */
void StablePlotCtrl(float RolCule,float PitchCule){
	/* 定义Rol,Pitch角度的偏差量--------------------------------------------------------------------*/
// 	float Rol_CurrentError,Pitch_CurrentError;      //目标值减去实际角度值（已减去零漂）

	/* 得到偏差角度 --------------------------------------------------------------------------------*/			
	//Rol_CurrentError =  目标值  -  (当前实际值 - 零漂)
// 	Rol_CurrentError = MontionControl.StableParam.RolExpect - RolCule + JY901.ZeroDirft.RolZeroDirft;
// 	Pitch_CurrentError = MontionControl.StableParam.PitchExpect - PitchCule + JY901.ZeroDirft.PitchZeroDirft;
	
	/* 计算PIDoout ---------------------------------------------------------------------------------*/
	/* 计算Pout -----------------------------------------------------------------------*/
// 	StaRol_PID.Pout = StaRol_PID.Kp * Rol_CurrentError;			//Rol
// 	StaPitch_PID.Pout = StaPitch_PID.Kp * Pitch_CurrentError;			//Pitch
// 	
// 	/* 计算Iout -----------------------------------------------------------------------*/
// 	if(Rol_CurrentError>-1.5 && Rol_CurrentError <1.5){			//Rol
// 		StaRol_PID.Iout += StaRol_PID.Ki * Rol_CurrentError;
// 	}else StaRol_PID.Iout = 0;
// 	
// 	if(Pitch_CurrentError>-1.5 && Pitch_CurrentError <1.5){			//Pitch
// 		StaPitch_PID.Iout += StaPitch_PID.Ki * Pitch_CurrentError;
// 	}else StaPitch_PID.Iout = 0;
// 	
// 	/* 计算Dout ------------------------------------------------------------------------*/
// 	StaRol_PID.Dout = -StaRol_PID.Kd * JY901.WxCuled.Rol;		//Rol
// 	StaPitch_PID.Dout = -StaPitch_PID.Kd * JY901.WxCuled.Pitch;	//Pitch	

// 	/* 将PIDout输出至TIM4控制电机 ------------------------------------------------------------------*/
// 	//Pout 与 Iout 
// 	motor1 = TIM4->CCR1 -((s16)(StaPitch_PID.Pout + StaPitch_PID.Iout));	
// 	motor2 = TIM4->CCR2 -((s16)(StaRol_PID.Pout + StaRol_PID.Iout));
// 	motor3 = TIM4->CCR3 +((s16)(StaPitch_PID.Pout + StaPitch_PID.Iout));
// 	motor4 = TIM4->CCR4 +((s16)(StaRol_PID.Pout + StaRol_PID.Iout));

// 	if(motor1>4800)motor1 = 4800;
// 	if(motor2>4800)motor2 = 4800;
// 	if(motor3>4800)motor3 = 4800;
// 	if(motor4>4800)motor4 = 4800;

// 	if(motor1<=10)motor1 = 10;
// 	if(motor2<=10)motor2 = 10;
// 	if(motor3<=10)motor3 = 10;
// 	if(motor4<=10)motor4 = 10;

// 	//Dout
// 	motor1 -= StaPitch_PID.Dout;
// 	motor2 -= StaRol_PID.Dout;
// 	motor3 += StaPitch_PID.Dout;
// 	motor4 += StaRol_PID.Dout;

// 	PWM_SET(motor1,motor2,motor3,motor4);

}	


/**
  *@brief   TrackedCtrl		轨迹跟踪模式下的控制函数
  *@param   float RolCule	当前Rol角度
			float PitchCule	当前Pitch角度
  *@retval  None
  */
void TrackedCtrl(float RolCule,float PitchCule){
	
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
