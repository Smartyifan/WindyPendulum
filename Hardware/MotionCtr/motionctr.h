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
typedef enum {SinglePend = 0, ConePend = 1,StabelPlot = 2,Tracked = 3} MotionModeTypedef;	//运动模式的选择
typedef enum {Rol = 0,Pitch = 1}SinglePendSel;												//单摆轴的选择
typedef struct {
	MotionModeTypedef MotionMode;	//运动模式
	
	void (*CtrlFun) (void);		//控制函数	
	
	struct {					//单摆参数
		SinglePendSel Pend;		//摆动轴
		float Period;			//周期
		float Amplitude;		//摆幅
	}SinglePendParam;
	
	struct{						//圆锥摆参数
		float Period;			//周期
		float Amplitude;		//摆幅	
	}ConePendParam;
	
	struct{						//稳定点参数
		float RolExpect;		//Rol期望
		float PitchExpect;		//Pitch期望
	}StableParam;
}MotionCtrStr;

/* extern Variables ----------------------------------------------------------*/
extern MotionCtrStr MontionControl;		//运动控制结构体
/* extern function------------------------------------------------------------*/
void MotionCtrParamInit(MotionCtrStr * MotionCtrl);		//参数初始化
extern void SinglePendCtrl(void);		//单摆模式下的控制函数
extern void ConePendCtrl(void);			//圆锥摆模式下的控制函数
extern void StablePlotCtrl(void);		//稳定点模式下的控制函数
extern void TrackedCtrl(void);			//轨迹跟踪模式下的控制函数

#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
