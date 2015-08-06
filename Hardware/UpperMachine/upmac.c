/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\UpperMachine\upmac.c
  * @author  	贾一帆
  * @version	V0.0
  * @date  		2015-07-10 10:38:53
  * @brief   	与UpperMachine上位机通讯
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
#include "MotionCtr/motionctr.h"
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
  *@brief   DetectCmd	检测收到的指令
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
			
			/* 控制类指令-------------------------------------------*/
			case 0x51: {				//停止电机
				Motor_Stop();
				MontionControl.MotionMode = Stop;
				HC05printf(&HC05," Motor_Stop\r\n");
				break;
			}
			case 0x52:{					//启动电机
				Motor_Start_Up();
				HC05printf(&HC05," Motor_Start_Up\r\n");
				break;
			}
			case 0x53:{					//校准零偏
				StartDetectZeroDrift();
				HC05printf(&HC05," StartDetectZeroDrift\r\n");
				break;
			}
			case 0x54:{					//单摆模式
				MontionControl.MotionMode = SinglePend;
				MontionControl.CtrlFun = SinglePendCtrl;
				HC05printf(&HC05," SinglePend Mode...\r\n");
				break;
			}
			case 0x55:{					//单摆Rol模式
				MontionControl.SinglePendParam.Pend = Rol;
				HC05printf(&HC05," SinglePend Mode in Rol...\r\n");
				break;
			}
			case 0x56:{					//单摆Pitch模式
				MontionControl.SinglePendParam.Pend = Pitch;
				HC05printf(&HC05," SinglePend Mode in Pitch...\r\n");
				break;
			}
			case 0x57:{					//双摆模式
				MontionControl.MotionMode = DoublePend;
				MontionControl.CtrlFun = DoublePendCtrl;
				HC05printf(&HC05," ConePend Mode...\r\n");
				break;
			}
			case 0x58:{					//稳定点模式
				MontionControl.MotionMode = StabelPlot;
				MontionControl.CtrlFun = StablePlotCtrl;
				HC05printf(&HC05," StabelPlot Mode...\r\n");
				break;
			}
			case 0x59:{					//轨迹跟踪模式
				MontionControl.MotionMode = Tracked;
				MontionControl.CtrlFun = TrackedCtrl;
				HC05printf(&HC05," Tracked Mode...\r\n");
				break;
			}
			
			/* 运动类指令 --------------------------------------------*/
			/* 单摆运动 -----------------------------*/
			case 0xB1:{									//单摆摆幅	
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.SinglePendParam.Amplitude = temp.f;
				HC05printf(&HC05," Amplitude = %f\r\n",MontionControl.SinglePendParam.Amplitude);
				break;
			}
			case 0xB2:{
				memcpy(&temp.c[0],&HC05.RxData[2],4);	//单摆周期	
				MontionControl.SinglePendParam.Period = temp.f;
				HC05printf(&HC05," Preiod = %f\r\n",MontionControl.SinglePendParam.Period);
				break;
			}
			
			/* 双摆运动 --------------------------*/
			case 0xC1:{									//Rol周期
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.DoublePendParam.RolPeriod = temp.f;
				HC05printf(&HC05," RolPeriod = %f\r\n",MontionControl.DoublePendParam.RolPeriod);
				break;
			}
			case 0xC2:{									//Rol幅值
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.DoublePendParam.RolAmplitude = temp.f;
				HC05printf(&HC05," RolAmplitude = %f\r\n",MontionControl.DoublePendParam.RolAmplitude);
				break;
			}
			case 0xC3:{									//Pitch周期
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.DoublePendParam.PitchPeriod = temp.f;
				HC05printf(&HC05," PitchPeriod = %f\r\n",MontionControl.DoublePendParam.PitchPeriod);
				break;
			}
			case 0xC4:{									//Pitch幅值	
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.DoublePendParam.PitchAmplitude = temp.f;
				HC05printf(&HC05," PitchAmplitude = %f\r\n",MontionControl.DoublePendParam.PitchAmplitude);
				break;
			}
			
			/* 稳定点运动 ------------------------*/
			case 0xD1:{									//稳定点Rol期望
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.StableParam.RolExpect = temp.f;
				HC05printf(&HC05," StableRolExpect = %f\r\n",MontionControl.StableParam.RolExpect);
				break;
			}
			case 0xD2:{									//稳定点Pitch期望
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				MontionControl.StableParam.PitchExpect = temp.f;
				HC05printf(&HC05," StablePitchExpect = %f\r\n",MontionControl.StableParam.PitchExpect);
				break;
			}
			
			
			/* 调参类指令 --------------------------------------------*/
			case 0xA1:{					//Rol.Kp
				memcpy(&temp.c[0],&HC05.RxData[2],4);		//拷贝数组
				StaRol_PID.Kp = temp.f;
				HC05printf(&HC05," StaRol_PID.Kp = %f\r\n",StaRol_PID.Kp);
				break;
			}
			case 0xA2:{					//Rol.Ki
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				StaRol_PID.Ki = temp.f;
				HC05printf(&HC05," StaRol_PID.Ki = %f\r\n",StaRol_PID.Ki);				
				break;
			}
			case 0xA3:{					//Rol.Kd
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				StaRol_PID.Kd = temp.f;
				HC05printf(&HC05," StaRol_PID.Kd = %f\r\n",StaRol_PID.Kd);
				break;
			}
			
			case 0xA4:{					//Pitch.Kp
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				StaPitch_PID.Kp = temp.f;
				HC05printf(&HC05," StaPitch_PID.Kp = %f\r\n",StaPitch_PID.Kp);
				break;
			}
			case 0xA5:{					//Pitch.Ki
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				StaPitch_PID.Ki = temp.f;
				HC05printf(&HC05," StaPitch_PID.Ki = %f\r\n",StaPitch_PID.Ki);
				break;
			}
			case 0xA6:{					//Pitch.Kd
				memcpy(&temp.c[0],&HC05.RxData[2],4);		
				StaPitch_PID.Kd = temp.f;
				HC05printf(&HC05," StaPitch_PID.Kd = %f\r\n",StaPitch_PID.Kd);
				break;
			}
		}
	}else return;
}





/**
  *@brief   Motor_Stop		
  *@param   None
  *@retval  None
  */
void Motor_Stop(void)
{
	MotorStart = DISABLE;
	PIDParamInit(&StaRol_PID);
	PIDParamInit(&StaRol_PID);
	
	PIDParamInit(&SigPitch_PID);
	PIDParamInit(&SigRol_PID);
	
	PIDParamInit(&DobPitch_PID);
	PIDParamInit(&DobRol_PID);

	TIM4->CCR1 = 1;		//设置占空比ZKB
	TIM4->CCR2 = 1;
	TIM4->CCR3 = 1;
	TIM4->CCR4 = 1;
	TIM4->CCER &=~(1<<0);			//使能Timer4 PWM输出禁止
	TIM4->CCER &=~(1<<4);
	TIM4->CCER &=~(1<<8);
	TIM4->CCER &=~(1<<12);
	TIM4->CR1&=~(1<<0);        //关闭定时器4
}


/**
  *@brief   Motor_Start_Up		启动电机输出
  *@param   None
  *@retval    None
  */
void Motor_Start_Up(void)
{
	MotorStart = ENABLE;
	TIM4->CCER |= 1<<0;			//使能Timer4 PWM输出
	TIM4->CCER |= 1<<4;
	TIM4->CCER |= 1<<8;
	TIM4->CCER |= 1<<12;
	TIM4->CR1|=0x0001;   //打开定时器4，开始输出PWM波
}

/**
  *@brief   StartDetectZeroDrift	启动检测零漂
  *@param   None
  *@retval    None
  */
void StartDetectZeroDrift(void){
	i=0;
	DetectZeroDrift = ENABLE;
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
