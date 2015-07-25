/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\UpperMachine\upmac.c
  * @author  	��һ��
  * @version	V0.0
  * @date  		2015-07-10 10:38:53
  * @brief   	������UpperMachine��λ��ͨѶ
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <string.h>
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "sys/sys.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"
#include "JY901/jy901.h"
#include "LED/led.h"
#include "delay/delay.h"
#include "PID/pid.h"
#include "TIMER/timer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
/**
  *@brief   
  *@param   None
  *@retval    None
  */
void SimplePlotSend(HC05Str * HC05,float A,float B,float C,float D){
	union {
		float f;
		char c[4];
	} temp;

	HC05->TxData[0] = 0xAA;
	HC05->TxData[1] = 0xEE;
	
	temp.f = A;
	HC05->TxData[2] = temp.c[0];
	HC05->TxData[3] = temp.c[1];
	HC05->TxData[4] = temp.c[2];
	HC05->TxData[5] = temp.c[3];

	temp.f = B;
	HC05->TxData[6] = temp.c[0];
	HC05->TxData[7] = temp.c[1];
	HC05->TxData[8] = temp.c[2];
	HC05->TxData[9] = temp.c[3];
	
	temp.f = C;
	HC05->TxData[10] = temp.c[0];
	HC05->TxData[11] = temp.c[1];
	HC05->TxData[12] = temp.c[2];
	HC05->TxData[13] = temp.c[3];
	
	temp.f = D;
	HC05->TxData[14] = temp.c[0];
	HC05->TxData[15] = temp.c[1];
	HC05->TxData[16] = temp.c[2];
	HC05->TxData[17] = temp.c[3];
	UARTxDMASend(HC05,18);
}




/**
  *@brief   DetectCmd	ʶ����λ�����͵�ָ��
  *@param   None
  *@retval  None
  */
void DetectCmd(void){
	union{
		float f;
		u8 c[4];
	}temp;

	if(HC05.RxData[0] == 0xAA){
		switch(HC05.RxData[1]){
			
			/* ������ָ�� -------------------------------------------*/
			case 0x51: {				//ֹͣ���
				Motor_Stop();
				HC05printf(&HC05,"Motor_Stop\r\n");
				break;
			}
			case 0x52:{					//�������
				Motor_Start_Up();
				HC05printf(&HC05,"Motor_Start_Up\r\n");
				break;
			}
			case 0x53:{					//�����Ư
				StartDetectZeroDrift();
				HC05printf(&HC05,"StartDetectZeroDrift\r\n");
				break;
			}
			
			
			/* ������ָ�� ---------------------------------------------*/
			case 0xA1:{					//Rol.Kp
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				x_PendPID.Kp = temp.f;
				HC05printf(&HC05,"x_PendPID.Kp = %f\r\n",x_PendPID.Kp);
				break;
			}
			case 0xA2:{					//Rol.Ki
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				x_PendPID.Ki = temp.f;
				HC05printf(&HC05,"x_PendPID.Ki = %f\r\n",x_PendPID.Ki);				
				break;
			}
			case 0xA3:{					//Rol.Kd
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				x_PendPID.Kd = temp.f;
				HC05printf(&HC05,"x_PendPID.Kd = %f\r\n",x_PendPID.Kd);
				break;
			}
			
			case 0xA4:{					//Pitch.Kp
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				y_PendPID.Kp = temp.f;
				HC05printf(&HC05,"y_PendPID.Kp = %f\r\n",y_PendPID.Kp);
				break;
			}
			case 0xA5:{					//Pitch.Ki
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				y_PendPID.Ki = temp.f;
				HC05printf(&HC05,"y_PendPID.Ki = %f\r\n",y_PendPID.Ki);
				break;
			}
			case 0xA6:{					//Pitch.Kd
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				y_PendPID.Kd = temp.f;
				HC05printf(&HC05,"y_PendPID.Kd = %f\r\n",y_PendPID.Kd);
				break;
			}
			case 0xA7:{					//x_TargetAngle
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				x_TargetAngle = temp.f;
				HC05printf(&HC05,"x_TargetAngle = %f\r\n",x_TargetAngle);
				break;
			}
			case 0xA8:{					//y_TargetAngle
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//����������
				y_TargetAngle = temp.f;
				HC05printf(&HC05,"y_TargetAngle = %f\r\n",y_TargetAngle);
				break;
			}

		}
	}else return;
}





/**
  *@brief   Motor_Stop		�رյ�����
  *@param   None
  *@retval  None
  */
void Motor_Stop(void)
{
	MotorStart = DISABLE;
	PIDParamInit(&x_PendPID);
	PIDParamInit(&y_PendPID);
	
	TIM4->CCR1 = 1;		//����ռ�ձ�ZKB
	TIM4->CCR2 = 1;
	TIM4->CCR3 = 1;
	TIM4->CCR4 = 1;
	TIM4->CCER &=~(1<<0);			//ʹ��Timer4 PWM�����ֹ
	TIM4->CCER &=~(1<<4);
	TIM4->CCER &=~(1<<8);
	TIM4->CCER &=~(1<<12);
	TIM4->CR1&=~(1<<0);        //�رն�ʱ��4
}


/**
  *@brief   Motor_Start_Up		����������
  *@param   None
  *@retval    None
  */
void Motor_Start_Up(void)
{
	MotorStart = ENABLE;
	TIM4->CCER |= 1<<0;			//ʹ��Timer4 PWM���
	TIM4->CCER |= 1<<4;
	TIM4->CCER |= 1<<8;
	TIM4->CCER |= 1<<12;
	TIM4->CR1|=0x0001;   //�򿪶�ʱ��4����ʼ���PWM��
}

/**
  *@brief   StartDetectZeroDrift	���������Ư
  *@param   None
  *@retval    None
  */
void StartDetectZeroDrift(void){
	i=0;
	DetectZeroDrift = ENABLE;
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
