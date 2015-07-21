/**
  ******************************************************************************
  * @file    
  * @author  ��һ��
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
	float Kp;			//�������
    float Ki;
    float Kd;
    
    float error[3];    //���ƫ����������  
        
    float Pout;
    float Iout;
    float Dout;
    
    float PIDout;		//PIDout
    float PIDout_H;	//PIDout��ֵ����
    float PIDout_L;
}PIDStruct;
/*extern function-------------------------------------------------------------*/
extern PIDStruct x_PendPID,y_PendPID;       //�ڸ�PID
extern s16 motor1,motor2,motor3,motor4;
extern void PIDParamInit(PIDStruct * PID);
extern void PIDCalculater(PIDStruct * PID,float error);
extern void PIDControl(void);
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
