/**
  ******************************************************************************
  * @file    	E:\GitRepo\WindyPendulum\Hardware\MotionCtr\motionctr.h
  * @author  	贾一帆
  * @version 	V0.0
  * @date    	2015-08-01 20:02:22
  * @brief   	
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef  _MOTIONCTR_H
#define  _MOTIONCTR_H
/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>

/* Define --------------------------------------------------------------------*/
/* TypeDefine ----------------------------------------------------------------*/
typedef enum {SinglePend = 0, DoublePend = 1,StabelPlot = 2,Tracked = 3,Stop = 4} MotionModeTypedef;	//运动模式的选择
typedef enum {Rol = 0,Pitch = 1}SinglePendSel;												//单摆轴的选择
typedef struct {
	MotionModeTypedef MotionMode;	//运动模式
	
	void (*CtrlFun) (float,float);		//控制函数	
	
	float Rolp_Amplitude;		//测量Rol正摆幅
	float Roln_Amplitude;		//测量Rol负摆幅
	float Pitchp_Amplitude;		//测量Pitch正摆幅
	float Pitchn_Amplitude;		//测量Pitch负摆幅
	
	struct {					//单摆参数
		float Angle;			//与x轴的角度
		float Amplitude;		//摆幅
		float RolPeriod;		//Rol周期
		float RolAmplitude;		//Rol摆幅
		float PitchPeriod;		//Pitch周期
		float PitchAmplitude;	//Pitch摆幅
	}SinglePendParam;
	
	struct{						//双摆参数
		float RolPeriod;		//Rol周期
		float RolAmplitude;		//Rol摆幅
		float PitchPeriod;		//Pitch周期
		float PitchAmplitude;	//Pitch摆幅
	}DoublePendParam;
	
	struct{						//稳定点参数
		float RolExpect;		//Rol期望
		float PitchExpect;		//Pitch期望
	}StableParam;
}MotionCtrStr;

/* extern Variables ----------------------------------------------------------*/
extern MotionCtrStr MontionControl;		//运动控制结构体
/* extern function------------------------------------------------------------*/
void MotionCtrParamInit(MotionCtrStr * MotionCtrl);		//参数初始化
extern void SinglePendCtrl(float RolCule,float PitchCule);		//单摆模式下的控制函数
extern void DoublePendCtrl(float RolCule,float PitchCule);			//圆锥摆模式下的控制函数
extern void StablePlotCtrl(float RolCule,float PitchCule);		//稳定点模式下的控制函数
extern void TrackedCtrl(float RolCule,float PitchCule);			//轨迹跟踪模式下的控制函数

#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
