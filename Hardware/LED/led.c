#include "LED/led.h"
	   	    
//LED IO初始化
void LED_Init(void)
{  	 
	JTAG_Set(1);   //关闭JTAG,JTAG 与LED公用引脚B3 B4	
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
	   	 
	GPIOB->CRL&=0xFF000FFF; 
	GPIOB->CRL|=0x00333000;//PB3 PB4 PB5 推挽输出   	 
    GPIOB->ODR|=0x0038; //PB3 PB4 PB5 输出高										
}






