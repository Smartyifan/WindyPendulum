#include "LED/led.h"
	   	    
//LED IO��ʼ��
void LED_Init(void)
{  	 
	JTAG_Set(1);   //�ر�JTAG,JTAG ��LED��������B3 B4	
	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��	
	   	 
	GPIOB->CRL&=0xFF000FFF; 
	GPIOB->CRL|=0x00333000;//PB3 PB4 PB5 �������   	 
    GPIOB->ODR|=0x0038; //PB3 PB4 PB5 �����										
}






