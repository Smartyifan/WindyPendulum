/**
  ******************************************************************************
  * @file    E:\GitRepo\InverPend\Hardware\PID\PID.c
  * @author  ��һ��
  * @version V3.5.0
  * @date    2015-07-02 16:01:43
  * @brief   ͨ������ʽPID����
  ******************************************************************************
  * @attention
  *	ʹ�ò��裺
  *		1.������Ӧ�ṹ�岢ʹ��PIDParamInit��ʼ����ز���
  *		2.��ʹ��ǰ�ȸ�Kp\Ki\Kd��ֵ
  *		3.ͨ��PIDCalculater�õ�PIDout���
  ******************************************************************************
  */  
  
/* Includes ------------------------------------------------------------------*/
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "PID/PID.h"
#include "usart/usart.h"
#include "PWM/pwm.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PIDStruct x_PendPID,y_PendPID;		//�ڸ�PID���ƽṹ��
/* Private function prototypes -----------------------------------------------*/
void PIDGetError(PIDStruct * PID,float error);
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   PIDCalculatre
  *@param   None;
  *@retval  None
  */
void PIDParamInit(PIDStruct * PID){
	
	/* PIDout Init------------*/
    PID->PIDout = 0;	//PIDout��ʼ��Ϊ0	
	/* errors Init------------*/
    PID->error[0] = 0;	//���ƫ������
	PID->error[1] = 0;
    PID->error[2] = 0;
}

/**
  *@brief   PIDCalculater   ����ʽPID����
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��
  *@retval  None
  */
// extern double abs(double __x); //���Ӹ��������ʹ��absʱ����־���
void PIDCalculater(PIDStruct * PID,float error){  
	
	/* �õ�ƫ���� ----------------------------------------------------*/
    PIDGetError(PID,error);		//���ƫ�����ƫ��������
	/* ����PID��� ---------------------------------------------------*/
    PID->Pout = PID->Kp*(PID->error[0] - PID->error[1]);	//����Pout
	
	/* ������� ------------------------------------------------------*/
	if((PID->error[0] > -0.5)  &&  (PID->error[0] < 0.5)){
		PID->Iout = PID->Ki* PID->error[0];						//����Iout
	}
	
    PID->Dout = PID->Kd*(PID->error[0] + PID->error[2] -2*PID->error[1]);	//����Dout
	//����PID�����д���ǲ��������⣿
    PID->PIDout += PID->Pout + PID->Iout + PID->Dout;		//�õ�PIDout
    /* �ߵ���ֵ���� --------------------------------------------------*/
// 	if(PID->PIDout > PID->PIDout_H)PID->PIDout = PID->PIDout_H;		//PIDout���ֵ����
// 	if(PID->PIDout < PID->PIDout_L)PID->PIDout = PID->PIDout_L;		//PIDout���ֵ����
}

/**
  *@brief   PIDGetError		
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��
  *			double error	��ǰƫ����			
  *@retval  None
  */
void PIDGetError(PIDStruct * PID,float error){
	PID->error[2] = PID->error[1];	//������λ
	PID->error[1] = PID->error[0];
    PID->error[0] = error;
}

/**
  *@brief   PIDControl	����PIDout�����ⲿ�豸�����ݸ���ʵ�������д
  *@param   PIDStruct * PID	ָ��PID�ṹ���ָ��	
  *@retval  None
  */

s16 motor1=0,motor2=0,motor3=0,motor4=0;
void PIDControl(void){
	//����ÿ�������PWM�仯�����߻��ǽ���//������ֲʱҪ�Ķ�
	motor1 = TIM4->CCR1 +((s16)(y_PendPID.PIDout));
	motor2 = TIM4->CCR2 +((s16)(x_PendPID.PIDout));
	motor3 = TIM4->CCR3 -((s16)(y_PendPID.PIDout));
	motor4 = TIM4->CCR4 -((s16)(x_PendPID.PIDout));

	PWM_SET(motor1,motor2,motor3,motor4);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
