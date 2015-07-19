/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\Attitude\attitude.c
  * @author  	��һ��
  * @version	V0.0
  * @date  		2015-07-12 15:35:42
  * @brief   	��̬���ƻ�
  ******************************************************************************
  * @attention
  * ���λ��
  *        2			    
  * 	   |              
  *   3 ��   ��  1        -->  y
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
AttitudeStr Attitude;				//��̬����
AngPID RolPID,PitchPID,YawPID;		//��̬��PID
/* Private function prototypes -----------------------------------------------*/
void GetMiMx(float num,float max,float min);
void AttiangPID(JY901Str * JY901,AttitudeStr * Attitude);			//����Ƕȵ�PID
/* Private functions ---------------------------------------------------------*/
/**
  *@brief   AttiInit		��̬���Ʋ����ĳ�ʼ��
  *@param   AttitudeStr * Attitude	//��̬���ƽṹ��
  *@retval  None
  */
void AttiInit(AttitudeStr * Attitude){
	/* ��̬�� -----------------------------------*/
	Attitude->attang.Rol = 0;
	Attitude->attang.Pitch = 0;
	Attitude->attang.Yaw = 0;

	/* ˮƽƫ���� -------------------------------*/
	Attitude->tanslation.x = 0;
	Attitude->tanslation.y = 0;
	Attitude->tanslation.z = 0;
	
	/* �߶� ------------------------------------*/
	Attitude->Height = 0;
	
	/* ��� ------------------------------------*/
	Attitude->Motor.Motor1 = 0;
	Attitude->Motor.Motor2 = 0;
	Attitude->Motor.Motor3 = 0;
	Attitude->Motor.Motor4 = 0;
}

/**
  *@brief   AttiangPID		��̬��PID����
  *@param   JY901Str * JY901		//JY901��ȡ��ʵ����̬������
			AttitudeStr * Attitude	//����Ŀ��
  *@retval    None
  */
void AttiangPID(JY901Str * JY901,AttitudeStr * Attitude){
	/* ����ƫ���� ---------------------------------------------------------------*/
	short rol = Attitude->attang.Rol - JY901->Ang.Rol;		//ƫ����
	short pit = Attitude->attang.Pitch - JY901->Ang.Pitch;
	short yaw = Attitude->attang.Yaw - JY901->Ang.Yaw;
	
	/* ����Rol��Pitch��IMax -----------------------------------------------------*/
	RolPID.IMax = Attitude->Motor.Throttle/2;
	GetMiMx(RolPID.IMax,1000,0);					//����ֵ��0~1000֮��
	PitchPID.IMax = RolPID.IMax;
	
	/* ����P --------------------------------------------------------------------*/
	RolPID.Pout = rol * RolPID.Kp;		
	PitchPID.Pout = pit * PitchPID.Kp;
	YawPID.Pout = yaw * YawPID.Kp;
	
	/* ����I --------------------------------------------------------------------*/
	if(Attitude->Motor.Throttle >300){
		if(fabs(RolPID.Iout) < Attitude->Motor.Throttle) {		//�����Ż���ʱ������
			RolPID.Iout += RolPID.Ki * rol;			//�������
		}

		if(fabs(PitchPID.Iout) < Attitude->Motor.Throttle) {	//�����Ż���ʱ������
			PitchPID.Iout += PitchPID.Ki * pit;			//�������
		}
		
		if(fabs(YawPID.Iout) < Attitude->Motor.Throttle) {		//�����Ż���ʱ������
			YawPID.Iout += YawPID.Ki * yaw;			//�������
		}

	}else if(Attitude->Motor.Throttle < 200){
		RolPID.Iout = 0;
		PitchPID.Iout = 0;
		YawPID.Iout = 0;
	}
	
	/* ����D --------------------------------------------------------------------*/
	RolPID.Dout = RolPID.Kd * JY901->Wx.Rol;			//���
	PitchPID.Dout = PitchPID.Kd * JY901->Wx.Pitch;		//����
	YawPID.Dout = YawPID.Kd * JY901->Wx.Yaw;			//ƫ��
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
