/**
  ******************************************************************************
  * @file    E:\ButterFly\Hardware\JY901\jy901.h
  * @author  贾一帆
  * @version V0.0
  * @date    2015-07-11 09:54:46
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  
  
#ifndef  _JY901_H
#define  _JY901_H
/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include "sys/sys.h"

/* Define --------------------------------------------------------------------*/
#define RxDataNum	5			//要接受的数据帧数
#define JY901TxLen  5			//发送一个指令的字节数
#define JY901RxLen  10+11*(RxDataNum-1)	//接收数据长度
#define ZeroDirftCulNum   300   //计算零漂采样数目
/* TypeDefine ----------------------------------------------------------------*/
typedef struct{
	USART_TypeDef * USARTBASE;

	DMA_Channel_TypeDef * DMAChannelTx;		//DMA接收通道号
	DMA_Channel_TypeDef * DMAChannelRx;		//DMA发送通道号

	u8 TxData[JY901TxLen];		//发送数组
	u8 RxData[JY901RxLen];		//接收数据
	
	struct {			//加速度
		short x;		//不做转换，16位数据
		short y;
		short z;
	}Ax;
	
	struct {			//角速度
		short x;		//不做转换，16位数据
		short y;
		short z;
	}Wx;
	
	struct {			//角度
		short Rol;		//横滚
		short Pitch;	//俯仰
		short Yaw;		//偏航
	}Ang;
	
	struct {			//角度转化为度 单位
		float RolCuled;		//横滚
		float PitchCuled;	//俯仰
	}AngCuled;
	
	struct {			//零漂
		float RolZeroDirft;		//横滚零漂
		float PitchZeroDirft;	//俯仰零漂
	}ZeroDirft;	
	
} JY901Str;
/* extern Variables ----------------------------------------------------------*/
extern JY901Str JY901;
/* extern function------------------------------------------------------------*/
extern void JY901Init(JY901Str * JY901);
		 				    
#endif
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
