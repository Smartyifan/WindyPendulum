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
/* 模糊控制参数 ----------------------------------*/
float Ke = 0.4,Kc = 4.5,Ku = 1;
s8 PlusFuzCtrlArray[13][13] = 
{
 {-60, -60, -60, -60, -60, -60, -60, -50, -40, -30, -20, -10, 0},
 {-60, -60, -60, -55, -50, -50, -50, -40, -30, -20, -10, -5, 0},
 {-60, -60, -60, -50, -40, -40, -40, -30, -20, -10, 0, 0, 0},
 {-60, -55, -50, -45, -40, -35, -30, -20, -10, -5, 0, 5, 10},
 {-60, -50, -40, -40, -40, -30, -20, -10, 0, 0, 0, 10, 20},
 {-50, -40, -30, -30, -30, -20, -10, 0, 10, 10, 10, 20, 30},
 {-40, -30, -20, -20, -20, -10, 0, 10, 20, 20, 20, 30, 40},
 {-30, -20, -10, -10, -10, 0, 10, 20, 30, 30, 30, 40, 50},
 {-20, -10, 0, 0, 0, 10, 20, 30, 40, 40, 40, 50, 60},
 {-10, -5, 0, 5, 10, 20, 30, 35, 40, 45, 50, 55, 60},
 {0, 0, 0, 10, 20, 30, 40, 40, 40, 50, 60, 60, 60},
 {0, 5, 10, 20, 30, 40, 50, 50, 50, 55, 60, 60, 60},
 {0, 10, 20, 30, 40, 50, 60, 60, 60, 60, 60, 60, 60}
};

s8 NegFuzCtrlArray[13][13] = 
{
 {60, 60, 60, 60, 60, 60, 60, 50, 40, 30, 20, 10, 0},
 {60, 60, 60, 55, 50, 50, 50, 40, 30, 20, 10, 5, 0},
 {60, 60, 60, 50, 40, 40, 40, 30, 20, 10, 0, 0, 0},
 {60, 55, 50, 45, 40, 35, 30, 20, 10, 5, 0, -5, -10},
 {60, 50, 40, 40, 40, 30, 20, 10, 0, 0, 0, -10, -20},
 {60, 50, 40, 35, 30, 20, 10, 0, -10, -15, -20, -30, -40},
 {60, 50, 40, 30, 20, 10, 0, -10, -20, -30, -40, -50, -60},
 {40, 30, 20, 15, 10, 0, -10, -20, -30, -35, -40, -50, -60},
 {20, 10, 0, 0, 0, -10, -20, -30, -40, -40, -40, -50, -60},
 {10, 5, 0, -5, -10, -20, -30, -35, -40, -45, -50, -55, -60},
 {0, 0, 0, -10, -20, -30, -40, -40, -40, -50, -60, -60, -60},
 {0, -5, -10, -20, -30, -40, -50, -50, -50, -55, -60,-60, -60},
 {0, -10, -20, -30, -40, -50, -60, -60, -60, -60, -60, -60, -60}
};

/* Private function prototypes -----------------------------------------------*/
short angle2ZKB(float angle){return (short)(-0.4273*angle*angle + 33.941*angle -11.2875);}	//角度转换成占空比->更换结构时需要重新测定
void FuzzyGetError(float (*e)[2],float error);
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
	MotionCtrl->SinglePendParam.Period = 0;
	MotionCtrl->SinglePendParam.RolAmplitude = 0;
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
	static u16 Tick = 0;	//时刻
	float RolpForce,RolnForce,PitchpForce,PitchnForce;				//驱动力
		
	/* 改变时间 -------------------------------------------------*/
	Tick++;	
	if(Tick 	>= 	MontionControl.SinglePendParam.Period ||
	   MontionControl.MotionMode	==	Stop)			Tick = 0;

	
	/* 计算驱动力 -----------------------------------------------*/
	/* 周期2s，为固有周期，可用周期性驱动力 -----------------------------------------------*/
	if(MontionControl.SinglePendParam.Period	==	2){
		if(Tick < MontionControl.SinglePendParam.Period/2){					//RolForce
			RolpForce = RolpPendPID.PIDout * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	);
		}else if(Tick >= MontionControl.SinglePendParam.Period/2){
			RolnForce =  -RolnPendPID.PIDout * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	);
		}
		
		if(Tick < MontionControl.SinglePendParam.Period/2){					//PitchForce		
			PitchpForce = PitchpPendPID.PIDout * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	);
		}else if(Tick >= MontionControl.SinglePendParam.Period/2){
			PitchnForce = -PitchnPendPID.PIDout * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	);
		}

		
	/* 周期不为2s，需要使用拟合驱动力实现 ---------------------------------------------------*/
	}else{
		if(Tick < MontionControl.SinglePendParam.Period/2){					//RolForce
			RolpForce = angle2ZKB(
							MontionControl.SinglePendParam.RolAmplitude * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	));
		}else if(Tick >= MontionControl.SinglePendParam.Period/2){
			RolnForce =  angle2ZKB(
						  -	MontionControl.SinglePendParam.RolAmplitude * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	));
		}
		
		if(Tick < MontionControl.SinglePendParam.Period/2){					//PitchForce		
			PitchpForce = angle2ZKB(
							MontionControl.SinglePendParam.PitchAmplitude * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	));
		}else if(Tick >= MontionControl.SinglePendParam.Period/2){
			PitchnForce = angle2ZKB(
						 -	MontionControl.SinglePendParam.PitchAmplitude * 
							sin(	Tick *	(_2Pi/MontionControl.SinglePendParam.Period)	));
		}


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
  *@param   float RolCule	当前Rol偏差
			float PitchCule	当前Pitch偏差
  *@retval  None
  */
void StablePlotCtrl(float Role,float Pitche){
	s8 plusdeltuRol   = 0,		negdeluRol   = 0;
	s8 plusdeltuPitch = 0,		negdeluPitch = 0;
	
	static float Rolerror[2],	Pitcherror[2];
	
	s8 eSetRol 	 = 0,	deSetRol   = 0;
	s8 eSetPitch = 0,	deSetPitch = 0;

	/* 偏差移位 ----------------------------------------------------------------*/
	FuzzyGetError(&Rolerror,Role);
	FuzzyGetError(&Pitcherror,Pitche);
	
	/* 将物理论域转换为模糊论域 ------------------------------------------------*/
	eSetRol  = 	(s8)	(Ke * Rolerror[0] + 0.5); 	//四舍五入取整		Rol
	deSetRol =  (s8)	(Kc * (Rolerror[0] - Rolerror[1]) + 0.5);	
	
	eSetPitch  = (s8)	(Ke * Pitcherror[0] + 0.5); 	//四舍五入取整	Pitch
	deSetPitch = (s8)	(Kc * (Pitcherror[0] - Pitcherror[1]) + 0.5);

	/* 判断所属模糊集 ----------------------------------------------------------*/
	if(eSetRol > 6) eSetRol = 6;				//限制范围为-6~6	Rol
	else if(eSetRol < -6) eSetRol = -6;
	
	if(deSetRol > 6) deSetRol = 6;
	else if(deSetRol < -6) deSetRol = -6;

	if(eSetPitch > 6) eSetPitch = 6;				//限制范围为-6~6	Pitch
	else if(eSetPitch < -6) eSetPitch = -6;
	
	if(deSetPitch > 6) deSetPitch = 6;
	else if(deSetPitch < -6) deSetPitch = -6;

	/* 查表得出模糊输出 -----------------------------------*/
	plusdeltuRol = 	PlusFuzCtrlArray[eSetRol+6][deSetRol+6];			//Rol
	negdeluRol	 = 	NegFuzCtrlArray[eSetRol+6][deSetRol+6];
	
	plusdeltuPitch = 	PlusFuzCtrlArray[eSetPitch+6][deSetPitch+6];	//Pitch
	negdeluPitch   = 	NegFuzCtrlArray[eSetPitch+6][deSetPitch+6];

	/* 乘以增益系数 ---------------------------------------*/
	plusdeltuRol   *= Ku;			//Rol
	negdeluRol	   *= Ku;

	plusdeltuPitch *= Ku;			//Pitch
	negdeluPitch   *= Ku;
	
	/* 将增量加入控制 ---------------------------------------------------------*/
	motor1 += (s16)plusdeltuPitch;
	motor2 += (s16)negdeluRol;
	motor3 += (s16)negdeluPitch;
	motor4 += (s16)plusdeltuRol;

	/* 根据油门控制电机 -------------------------------------------------------*/
	PWM_SET(motor1,motor2,motor3,motor4);
}	
/**
  *@brief   
  *@param   None
  *@retval    None
  */
void FuzzyGetError(float (*e)[2],float error){
	//数组移位
	(*e)[1] = (*e)[0];
    (*e)[0] = error;
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
