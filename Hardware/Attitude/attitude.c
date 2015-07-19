/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\Attitude\attitude.c
  * @author  	贾一帆
  * @version	V0.0
  * @date  		2015-07-12 15:35:42
  * @brief   	姿态控制机
  ******************************************************************************
  * @attention
  * 电机位置
  *        2			    
  * 	   |              
  *   3 —   —  1        -->  y
  *        |            |
  *        4            +x
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "sys/sys.h"
#include <math.h> 
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "Attitude/attitude.h"
#include "JY901/jy901.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
AttitudeStr Attitude;				//姿态控制
AngPID RolPID,PitchPID,YawPID;		//姿态角PID
/* Private function prototypes -----------------------------------------------*/
void GetMiMx(float num,float max,float min);
void AttiangPID(JY901Str * JY901,AttitudeStr * Attitude);			//计算角度的PID
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   AttiInit		姿态控制参数的初始化
  *@param   AttitudeStr * Attitude	//姿态控制结构体
  *@retval  None
  */
void AttiInit(AttitudeStr * Attitude){
	/* 姿态角 -----------------------------------*/
	Attitude->attang.Rol = 0;
	Attitude->attang.Pitch = 0;
	Attitude->attang.Yaw = 0;

	/* 水平偏移量 -------------------------------*/
	Attitude->tanslation.x = 0;
	Attitude->tanslation.y = 0;
	Attitude->tanslation.z = 0;
	
	/* 高度 ------------------------------------*/
	Attitude->Height = 0;
	
	/* 电机 ------------------------------------*/
	Attitude->Motor.Motor1 = 0;
	Attitude->Motor.Motor2 = 0;
	Attitude->Motor.Motor3 = 0;
	Attitude->Motor.Motor4 = 0;
}

/**
  *@brief   AttiangPID		姿态角PID控制
  *@param   JY901Str * JY901		//JY901读取的实际姿态角数据
			AttitudeStr * Attitude	//控制目标
  *@retval    None
  */
void AttiangPID(JY901Str * JY901,AttitudeStr * Attitude){
	/* 计算偏差量 ---------------------------------------------------------------*/
	short rol = Attitude->attang.Rol - JY901->Ang.Rol;		//偏差量
	short pit = Attitude->attang.Pitch - JY901->Ang.Pitch;
	short yaw = Attitude->attang.Yaw - JY901->Ang.Yaw;
	
	/* 设置Rol和Pitch的IMax -----------------------------------------------------*/
	RolPID.IMax = Attitude->Motor.Throttle/2;
	GetMiMx(RolPID.IMax,1000,0);					//油门值在0~1000之间
	PitchPID.IMax = RolPID.IMax;
	
	/* 计算P --------------------------------------------------------------------*/
	RolPID.Pout = rol * RolPID.Kp;		
	PitchPID.Pout = pit * PitchPID.Kp;
	YawPID.Pout = yaw * YawPID.Kp;
	
	/* 计算I --------------------------------------------------------------------*/
	if(Attitude->Motor.Throttle >300){
		if(fabs(RolPID.Iout) < Attitude->Motor.Throttle) {		//比油门还大时不积分
			RolPID.Iout += RolPID.Ki * rol;			//计算积分
		}

		if(fabs(PitchPID.Iout) < Attitude->Motor.Throttle) {	//比油门还大时不积分
			PitchPID.Iout += PitchPID.Ki * pit;			//计算积分
		}
		
		if(fabs(YawPID.Iout) < Attitude->Motor.Throttle) {		//比油门还大时不积分
			YawPID.Iout += YawPID.Ki * yaw;			//计算积分
		}

	}else if(Attitude->Motor.Throttle < 200){
		RolPID.Iout = 0;
		PitchPID.Iout = 0;
		YawPID.Iout = 0;
	}
	
	/* 计算D --------------------------------------------------------------------*/
	RolPID.Dout = RolPID.Kd * JY901->Wx.Rol;			//横滚
	PitchPID.Dout = PitchPID.Kd * JY901->Wx.Pitch;		//俯仰
	YawPID.Dout = YawPID.Kd * JY901->Wx.Yaw;			//偏航
}

void GetMiMx(float num,float max,float min)
{
	if(num>max)
		num = max;
	else if(num<min)
		num = min;
	else
		num = num;
}
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
