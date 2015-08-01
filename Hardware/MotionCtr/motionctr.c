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
/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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
	
	/* 圆锥摆模式参数初始化 -----------------------------*/
	MotionCtrl->ConePendParam.Period = 0;
	MotionCtrl->ConePendParam.Amplitude = 0;
	
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
void SinglePendCtrl(void){
	
}


/**
  *@brief   ConePendCtrl	圆锥摆模式下的控制函数
  *@param   None
  *@retval    None
  */
void ConePendCtrl(void){

}

/**
  *@brief   StablePlotCtrl	稳定点模式下的控制函数
  *@param   None
  *@retval    None
  */
void StablePlotCtrl(void){

}


/**
  *@brief   TrackedCtrl		轨迹跟踪模式下的控制函数
  *@param   None
  *@retval    None
  */
void TrackedCtrl(void){
	
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
