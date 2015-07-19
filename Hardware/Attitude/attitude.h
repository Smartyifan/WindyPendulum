/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\Attitude\attitude.h
  * @author  	��һ��
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
	struct{				//��̬��	
		short Rol;		//���
		short Pitch;	//����
		short Yaw;		//ƫ��
	}attang;
	
	struct{				//ƽ��
		short x;
		short y;
		short z;
	}tanslation;	
	
	long Height;		//�߶�
	
	struct{				//���������
		vs16 Throttle;  //����
		u16 Motor1;		
		u16 Motor2;
		u16 Motor3;
		u16 Motor4;
	}Motor;
}AttitudeStr;

typedef struct{
	float Kp;			//�������
    float Ki;
    float Kd;
	
	float IMax;			//��������
	
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
