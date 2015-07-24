/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef  _UPMAC_H
#define  _UPMAC_H
/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include "sys/sys.h"
#include "HC05/hc05.h"
/* Define --------------------------------------------------------------------*/
/* TypeDefine ----------------------------------------------------------------*/

/* extern Variables ----------------------------------------------------------*/

/* extern function------------------------------------------------------------*/
extern void SimplePlotSend(HC05Str * HC05,float A,float B,float C,float D);	 	//在XJI上位机中画图
extern void Motor_Start_Up(void);			//启动电机
extern void Motor_Stop(void);				//停止电机
extern void StartDetectZeroDrift(void);		//启动检测零漂
extern void DetectCmd(void);				//识别上位机命令
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
