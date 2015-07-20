/**
  ******************************************************************************
  * @file    STM32����/����ģ��/User
  * @author  ��һ��
  * @version V3.5.0
  * @date    2014-10-4
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * No attention
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "sys/sys.h"  
#include "delay/delay.h"
#include "string.h" //��ͷ�ļ�������ܶ��ַ�����
#include "LED/led.h"
#include "TIMER/timer.h"
#include "PWM/pwm.h"
#include "PID/PID.h"
#include "HC05/hc05.h"
#include "JY901/jy901.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*--------------------------------------------------
**PID��λ������PID��P\I\D���������������Լ�ƫ������
**����ڲ���
**�޳��ڲ���
---------------------------------------------------*/
void PIDReset(void)
{
	x_PendPID.Kp = 10;
	x_PendPID.Ki = 0;
	x_PendPID.Kd = 0;
	x_PendPID.PIDout_H = 10;     //PID����ֵ�������
	x_PendPID.PIDout_L = -10;     //PID����ֵ�������
	PIDParamInit(&x_PendPID);
	
	y_PendPID.Kp = 0;
	y_PendPID.Ki = 0;
	y_PendPID.Kd = 0;
	y_PendPID.PIDout_H = 10;
	x_PendPID.PIDout_L = -10;     //PID����ֵ�������
	PIDParamInit(&y_PendPID);
}
/**
  *@brief   Initial
  *@param   None;
  *@retval  None
  */
void Initial()
{
	NVIC_Configuration();	//�жϷ���2
	delay_init();
    
	JTAG_Set(1);                         //�ر�JTAG,����SWD
	LED_Init();                          //LED��ʼ��
	delay_ms(10);   
	//HC05��ʼ����ʹ�ô���1����
	HC05.USARTBASE = USART1;		
	HC05Init(&HC05);	
	delay_ms(10);
	//MPU6050��ʼ�����ô���2����	
	JY901.USARTBASE = USART2;      //���������Ǽ�JY901������Ϊ115200ʱ���������Ϊ100HZ,������Ϊ9600ʱ���������Ϊ20HZ
	JY901Init(&JY901);    
	delay_ms(10);
	Motor_Init();                        //���PWM��ʼ�����ö�ʱ��4����PWM��
	delay_ms(10);						
	Timer3_Init(1999,719);                //��ʱ��3��ʼ����ÿ20msһ���жϣ�δʹ�ܶ�ʱ����ֻ�е�ƫ��������ʱ��������ʱ��
	PIDReset();                          //PID������λ
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    Initial();
    while(1);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
