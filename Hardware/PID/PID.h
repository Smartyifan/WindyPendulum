/**
  ******************************************************************************
  * @file    
  * @author  贾一帆
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef _PID_H
#define _PID_H
/* Includes ------------------------------------------------------------------*/
#include "sys/sys.h"
/* Define --------------------------------------------------------------------*/

typedef struct PIDStruct{          
	float Kp;			//计算参数
    float Ki;
    float Kd;
    
    float error[3];    //存放偏差量的数组  
        
    float Pout;
    float Iout;
    float Dout;
    	
    float PIDout;		//PIDout
    float PIDout_H;		//PIDout阈值限制
    float PIDout_L;
}PIDStruct;
/* extern variable ----------------------------------------------------------*/
extern PIDStruct SigRol_PID,SigPitch_PID;		//单摆PID控制结构体
extern PIDStruct DobRol_PID,DobPitch_PID;		//双摆PID控制结构体
extern PIDStruct StaRol_PID,StaPitch_PID;		//稳定点PID控制结构体
extern s16 motor1,motor2,motor3,motor4;			//电机
extern float * pSigPeakValue;					//峰值指针
/*extern function-------------------------------------------------------------*/
extern void PIDParamInit(PIDStruct * PID);				//PID参数初始化
extern void PIDGetError(PIDStruct * PID,float error);	//存放偏差		
extern void PIDCalculater(PIDStruct * PID,float error);	//计算PIDout		
extern void PIDControl(void);							//PID控制输出	
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
