#include "LED/led.h"
	   	    
//LED IO初始化
void LED_Init(void)
{  	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PE端口时钟

	//RLED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //RedLED-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0
		
	GPIO_SetBits(GPIOA,GPIO_Pin_7);							 //PA.7 输出高

	
	//GLED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //RedLED-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1);							 //PA.1 输出高
	
	//BLED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //RedLED-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0

	GPIO_SetBits(GPIOA,GPIO_Pin_0);							 //PA.0 输出高

}






