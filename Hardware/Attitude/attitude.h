/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\Attitude\attitude.h
  * @author  	贾一帆
  * @version 	V0.0
  * @date    	2015-07-12 15:59:43
  * @brief   	
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef  _ATTITUDE_H
#define  _ATTITUDE_H
/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include "sys/sys.h"

#include "JY901/jy901.h"
/* Define --------------------------------------------------------------------*/
/* TypeDefine ----------------------------------------------------------------*/
typedef struct {
	struct{				//姿态角	
		short Rol;		//横滚
		short Pitch;	//俯仰
		short Yaw;		//偏航
	}attang;
	
	struct{				//平移
		short x;
		short y;
		short z;
	}tanslation;	
	
	long Height;		//高度
	
	struct{				//电机控制量
		vs16 Throttle;  //油门
		u16 Motor1;		
		u16 Motor2;
		u16 Motor3;
		u16 Motor4;
	}Motor;
}AttitudeStr;

typedef struct{
	float Kp;			//计算参数
    float Ki;
    float Kd;
	
	float IMax;			//最大积分量
	
    float Pout;
    float Iout;
    float Dout;
    float PIDout;		//PIDout

}AngPID;
/* extern Variables ----------------------------------------------------------*/
/* extern function------------------------------------------------------------*/
extern void AttiangPID(JY901Str * JY901,AttitudeStr * Attitude);
		 				    
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
