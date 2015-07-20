#include "sys/sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//系统中断分组设置化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/10
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************  
void NVIC_Configuration(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

}


void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel)	 
{ 
	NVIC_InitTypeDef NVICInitStr;
	
	NVICInitStr.NVIC_IRQChannel = NVIC_Channel;
	NVICInitStr.NVIC_IRQChannelPreemptionPriority = NVIC_PreemptionPriority;
	NVICInitStr.NVIC_IRQChannelSubPriority = NVIC_SubPriority;
	NVICInitStr.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInitStr);
}

//JTAG模式设置,用于设置JTAG的模式
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;
//CHECK OK	
//100818		  
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 
