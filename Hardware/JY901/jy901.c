/**
  ******************************************************************************
  * @file    	E:\ButterFly\Hardware\JY901\jy901.c
  * @author  	��һ��
  * @version	V0.0
  * @date  		2015-07-11 09:52:33
  * @brief   	JY-901������̬������
  ******************************************************************************
  * @attention
  * 2015-07-11 09:52:54
  * ͨ�����ڶ�ȡ���ݣ�ʹ��DMAͨ��
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include <string.h>
#include <stdlib.h>
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#include "sys/sys.h"
#include "delay/delay.h"

#include "JY901/jy901.h"
#include "HC05/hc05.h"
#include "UpperMachine/upmac.h"
#include "PID/PID.h"
#include "TIMER/timer.h"
#include "LED/led.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
JY901Str JY901;
FunctionalState DetectZeroDrift = DISABLE;	//���������Ư
ErrorStatus DriftDetected = ERROR;			//��Ư������
FunctionalState MotorStart = DISABLE;       //����������

/* Private function prototypes -----------------------------------------------*/
void UARTRxDMARec(JY901Str * JY901);							//����һ��DMA����
void JY901UartInit(USART_TypeDef * USARTBASE,u32 BaudRate);		//���ڳ�ʼ��
void UARTRxNVICInit(USART_TypeDef * USARTBASE);					//���ڵ�NVIC����
void JY901DMAInit(JY901Str * JY901);							//DMA��ʼ��
void DMATCNVICInit(DMA_Channel_TypeDef * DMAChannelRx);			//DMA����ͨ��NVIC����
/* Private functions ---------------------------------------------------------*/

/**
  *@brief   JY901Init
  *@param   None
  *@retval    None
  */
void JY901Init(JY901Str * JY901){
	
	/* ȷ��DMAͨ���� ------------------------------------------------------------------------------------*/
	if(JY901->USARTBASE == USART1){
		JY901->DMAChannelTx = DMA1_Channel4;	//Txͨ�� 
		JY901->DMAChannelRx = DMA1_Channel5;	//Rxͨ��
	}else if(JY901->USARTBASE == USART2){
		JY901->DMAChannelTx = DMA1_Channel7;	//Txͨ�� 
		JY901->DMAChannelRx = DMA1_Channel6;	//Rxͨ��
	}else if(JY901->USARTBASE == USART3){
		JY901->DMAChannelTx = DMA1_Channel2;	//Txͨ�� 
		JY901->DMAChannelRx = DMA1_Channel3;	//Rxͨ��
	}

	/* ���ڳ�ʼ�� ----------------------------------------------------------------------------------------*/
	JY901UartInit(JY901->USARTBASE,115200);
	
	/* DMA��ʼ�� -----------------------------------------------------------------------------------------*/	
	USART_DMACmd(JY901->USARTBASE,USART_DMAReq_Tx,ENABLE);	//ʹ�ܴ��ڷ���DMA
	USART_DMACmd(JY901->USARTBASE,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ��ڽ���DMA

	JY901DMAInit(JY901);		//��ʼ��DMA
}


/**
  *@brief   	USART2_IRQHandler	����2�жϺ��������ڼ��JY901���ݰ�ͷ 0x55 0x51
  *@param   	None
  *@retval  	None
  *@attention	��ռ���ȼ� 0 �����ȼ� 1
  */
void USART2_IRQHandler(void){
	static u8 FH[2];		//֡ͷ����
	static u8 pFH = 0;		//�����±�
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){	//���յ�����
		FH[pFH] = (unsigned char)JY901.USARTBASE->DR;
		
		if(pFH == 0 && FH[0] == 0x55){		//����һ���ֽ�	
			pFH++;
		}else if(pFH == 1){					//���ڶ����ֽ�
			if(FH[1]==0x51){
				USART_ITConfig(JY901.USARTBASE,USART_IT_RXNE,DISABLE);	//�رմ��ڽ����ж�
				UARTRxDMARec(&JY901);									//����һ��DMA����
			}
			pFH = 0;
		}
	}	
}

/**
  *@brief   	DMA1_Channel6_IRQHandler  DMAͨ��6�жϺ��������ڶ�MPU6050֮�������ش���
  *@param   	None
  *@retval    	None
  *@attention 	ִ��ʱ�� 46.14us�������Ư���֮��		��������:ST-Link_V2
				��ռ���ȼ� 1 �����ȼ� 0
  */
u16 i=0;  
void DMA1_Channel6_IRQHandler(void){
	/* ����x,y�Ƕȵ�ƫ����--------------------------------------------------------------------*/
	float x_CurrentError,y_CurrentError;      //Ŀ��ֵ��ȥʵ�ʽǶ�ֵ���Ѽ�ȥ��Ư��
   
	static float RolZeroDirftAll=0,PitchZeroDirftAll=0;//��Ư�ۼƱ���
		
	
	if(DMA_GetITStatus(DMA1_IT_TC6) == SET){	

		/* Use JY-901 ----------------------------------------------------------------------*/
		memcpy(&(JY901.Ax),&JY901.RxData[1],8);		//���ٶ�
		memcpy(&(JY901.Wx),&JY901.RxData[12],6);	//���ٶ�
		memcpy(&(JY901.Ang),&JY901.RxData[23],6);	//�Ƕ�	
		
		/* Use MPU6050 ---------------------------------------------------------------------*/
// 		memcpy(&(JY901.Ax),&JY901.RxData[1],6);		//���ٶ�
// 		memcpy(&(JY901.Wx),&JY901.RxData[12],6);	//���ٶ�	
// 		memcpy(&(JY901.Ang),&JY901.RxData[23],6);	//�Ƕ�
		/* ��ʾ�Ƕ� ------------------------------------------------------------------------*/

		/* �������ĽǶ�ת��-----------------------------------------------------------------------*/
		if(abs(JY901.Ang.Rol) > 273 && abs(JY901.Ang.Rol <13653))		JY901.AngCuled.RolCuled = (float)JY901.Ang.Rol/32768*180;
		if(abs(JY901.Ang.Pitch) > 273 && abs(JY901.Ang.Pitch <13653)) 	JY901.AngCuled.PitchCuled = (float)JY901.Ang.Pitch/32768*180; 

		/* ��Ư����-----------------------------------------------------------------------*/
		/*CuledFlagΪ0ʱƫ��δ������ɣ�Ϊ1ʱ��ʾƫ��������*/
		if(DetectZeroDrift == ENABLE) 
		{
			if(i == ZeroDirftCulNum) //����ƫ��ֵֵ
			{
				/* ������Ư ------------------------------------------------------*/
				JY901.ZeroDirft.RolZeroDirft = RolZeroDirftAll/ZeroDirftCulNum;
				JY901.ZeroDirft.PitchZeroDirft = PitchZeroDirftAll/ZeroDirftCulNum;
				
				/* ���ñ�־λ -----------------------------------------------------*/
				DetectZeroDrift = DISABLE;		//�������Ư
				DriftDetected = SUCCESS;		//��Ư������
				
				/* ��ɫLED��˸ ----------------------------------------------------*/
				G_LED=0;            			//��ʾ��ʼ��������ɣ�LED�̵ƿ�����˸2��
				delay_ms(100);
				G_LED=1;
				delay_ms(100);
				G_LED=0;            
				delay_ms(100);
				G_LED=1;
			}
			else 
			{
				if (i == 0)      //����ƫ��ֵ
				{
					JY901.ZeroDirft.RolZeroDirft=0;
					JY901.ZeroDirft.PitchZeroDirft=0;
				}
				RolZeroDirftAll += JY901.AngCuled.RolCuled;
				PitchZeroDirftAll += JY901.AngCuled.PitchCuled;					
			}
			i++;
		}
		
		/* PID����-----------------------------------------------------------------------*/
		else if(DetectZeroDrift == DISABLE && DriftDetected == SUCCESS && MotorStart == ENABLE)  //ֻ�е�����ƫ��������ʱ������PID����
		{
			//��Ӳ�������֪��PID����Ƶ�ʴ��Ϊ51Hz
			//x_CurrentError =  Ŀ��ֵ  -  ��ǰʵ��ֵ - ��Ư
			/* �õ�ƫ��Ƕ� --------------------------------------------------------------*/
			x_CurrentError = x_TargetAngle - JY901.AngCuled.RolCuled + JY901.ZeroDirft.RolZeroDirft;
			y_CurrentError = y_TargetAngle - JY901.AngCuled.PitchCuled + JY901.ZeroDirft.PitchZeroDirft;
			
			/* ����PIDoout --------------------------------------------------------------*/
			PIDCalculater(&x_PendPID,x_CurrentError);
			PIDCalculater(&y_PendPID,y_CurrentError);
			
			/* ��PIDout�����TIM4���Ƶ�� -------------------------------------------------*/
			PIDControl();		

			/* ��XJI��λ����������ͼ�� ----------------------------------------------------*/
 			SimplePlotSend(&HC05,JY901.AngCuled.RolCuled,x_CurrentError,(float)TIM4->CCR2/480,0);		//ִ��ʱ�� 7.92us �� 8us
			
		}
		
		
		/* �������� ----------------------------------------------------------------------*/
		USART_ITConfig(JY901.USARTBASE,USART_IT_RXNE,ENABLE);	//�򿪴��ڽ����ж�

		JY901.DMAChannelRx->CCR&=~1;      		//�ر�DMA���� 

		DMA_ClearITPendingBit(DMA1_IT_GL6);
	}
	
}

/**
  *@brief   UARTRxDMARec		����һ�δ��ڵ�DMA����
  *@param   JY901Str * JY901		//JY901�ṹ��
  *@retval    None
  */
void UARTRxDMARec(JY901Str * JY901){
	JY901->DMAChannelRx->CCR&=~1;      		//�ر�DMA���� 
	JY901->DMAChannelRx->CNDTR=JY901RxLen;  //DMA,���������� 
	JY901->DMAChannelRx->CCR|=1;       		//����DMA����
}

/**
  *@brief   JY901DMAInit
  *@param   JY901Str * JY901		//JY901�ṹ��
  *@retval    None
  */
void JY901DMAInit(JY901Str * JY901){
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	delay_us(5);

	/* UARTx  Tx DMAͨ����ʼ�� ---------------------------------------------------------------------------------*/
    DMA_DeInit(JY901->DMAChannelTx);   									//��DMA��ͨ��x�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(JY901->USARTBASE)->DR);  //DMA����USART->DR����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&JY901->TxData;  		//DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 			 		//���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = JY901TxLen;  					//DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  	//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  			//�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 	//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  						//��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 			//DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  						//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(JY901->DMAChannelTx, &DMA_InitStructure);  				//����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USARTx_Tx_DMA_Channex����ʶ�ļĴ���
	
	
	
	/* UARTx  Rx DMAͨ����ʼ�� ---------------------------------------------------------------------------------*/
	DMA_DeInit(JY901->DMAChannelRx);   									//��DMA��ͨ��x�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(JY901->USARTBASE)->DR);  //DMA����USART->DR����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&JY901->RxData;  		//DMA�ڴ����ַ  RxData����
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 			 		//���ݴ��䷽�򣬴������ȡ�����͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = JY901RxLen;  					//DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  	//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  			//�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 	//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  						//��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 				//DMAͨ�� xӵ�и����ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  						//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(JY901->DMAChannelRx, &DMA_InitStructure);  				//����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USARTx_Rx_DMA_Channex����ʶ�ļĴ���

	DMA_ITConfig(JY901->DMAChannelRx, DMA_IT_TC, ENABLE);				//������������ж�
	DMATCNVICInit(JY901->DMAChannelRx);									//����NVIC			
	
	DMA_SetCurrDataCounter(JY901->DMAChannelTx,0);		//TxDMAͨ�����������
	DMA_SetCurrDataCounter(JY901->DMAChannelRx,0);		//RxDMAͨ�����������
}

/**
  *@brief   JY901UartInit	HC-05ʹ�õĴ��ڳ�ʼ��
  *@param   USART_TypeDef * USARTBASE	���ں�
  *			u32 BaudRate	������
  *@retval    None
  */
void JY901UartInit(USART_TypeDef * USARTBASE,u32 BaudRate){
	u16 GPIO_Pin_Tx;			//Tx����
	u16 GPIO_Pin_Rx;			//Rx����
	GPIO_TypeDef * GPIOBase;	//GPIOBase
	
	/* �����ʼ���ṹ�� --------------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ���ò��� ----------------------------------------------------------------*/
	if(USARTBASE == USART1){
		GPIOBase = GPIOA;
		GPIO_Pin_Tx = GPIO_Pin_9;
		GPIO_Pin_Rx = GPIO_Pin_10;
	}else if(USARTBASE == USART2){
		GPIOBase = GPIOA;
		GPIO_Pin_Tx = GPIO_Pin_2;
		GPIO_Pin_Rx = GPIO_Pin_3;
	}else if(USARTBASE == USART3){
		GPIOBase = GPIOB;
		GPIO_Pin_Tx = GPIO_Pin_10;
		GPIO_Pin_Rx = GPIO_Pin_11;
	}

	/* ʹ��ʱ�� ----------------------------------------------------------------*/
	if(USARTBASE == USART1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//ʹ��UART1ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	}else if(USARTBASE == USART2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//ʹ��UART2ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	}else if(USARTBASE == USART3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ��UART3ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOBʱ��
	}

 	USART_DeInit(USARTBASE);  								//��λ����

	/* GPIO�˿����� -------------------------------------------------------------*/
	//USARTx_TX   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx; 			//TXD
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
    GPIO_Init(GPIOBase, &GPIO_InitStructure); 			//��ʼ��TxD

	//USARTx_RX	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Rx;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��������
    GPIO_Init(GPIOBase, &GPIO_InitStructure);  				//��ʼ��RxD

	/* USARTx ��ʼ������ --------------------------------------------------*/
	USART_InitStructure.USART_BaudRate = BaudRate;					//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//�ֳ�Ϊ8λ���ݸ�ʽ	��Ĭ��ģʽ��
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;				//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USARTBASE, &USART_InitStructure); 					//��ʼ������x
	
	/* �ж����� -----------------------------------------------------------*/
	USART_ITConfig(USARTBASE,USART_IT_TC,DISABLE);
	USART_ITConfig(USARTBASE,USART_IT_RXNE,ENABLE);					//���ڽ����ж�
    USART_ITConfig(USARTBASE,USART_IT_IDLE,DISABLE);				
	
	
	USART_Cmd(USARTBASE, ENABLE);                  					//ʹ�ܴ���x
	
	/* NVIC ���� ----------------------------------------------------------*/
	UARTRxNVICInit(USARTBASE);
	
	USART_ClearFlag(USARTBASE, USART_FLAG_TC);						//������ͳɹ���־
	
}

/**
  *@brief   RxNVICInit
  *@param   USART_TypeDef * USARTBASE	//���ں�
  *@retval    None
  */
void UARTRxNVICInit(USART_TypeDef * USARTBASE){
	IRQn_Type IRQChannel;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ȷ�ϲ��� ---------------------------------------------------------------*/
	if(USARTBASE == USART1){
		IRQChannel = USART1_IRQn;
	}else if(USARTBASE == USART2){
		IRQChannel = USART2_IRQn;
	}else if(USARTBASE == USART3){
		IRQChannel = USART3_IRQn;
	}
	
	/* NVIC��ʼ�� -------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 				//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ���жϺ�
	NVIC_Init(&NVIC_InitStructure);									//��ʼ��NVIC
}

/**
  *@brief   DMATCNVICInit
  *@param   DMA_Channel_TypeDef * DMAChannelRx	//DMAͨ����
  *@retval    None
  */
void DMATCNVICInit(DMA_Channel_TypeDef * DMAChannelRx){
	IRQn_Type IRQChannel;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ȷ�ϲ��� ---------------------------------------------------------------*/
	if(DMAChannelRx == DMA1_Channel5){
		IRQChannel = DMA1_Channel5_IRQn;
	}else if(DMAChannelRx == DMA1_Channel6){
		IRQChannel = DMA1_Channel6_IRQn;
	}else if(DMAChannelRx == DMA1_Channel3){
		IRQChannel = DMA1_Channel3_IRQn;
	}
	
	/* NVIC��ʼ�� -------------------------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 				//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ���жϺ�
	NVIC_Init(&NVIC_InitStructure);									//��ʼ��NVIC
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
