#ifndef __LED_H
#define __LED_H	 
#include "sys/sys.h"
//Mini STM32开发板
//LED驱动代码			 
//正点原子@ALIENTEK
//2010/5/27

//LED端口定义
#define R_LED PBout(3) // PB3
#define G_LED PBout(4)// PB4	
#define B_LED PBout(5)// PB5	

void LED_Init(void);//初始化		 				    
#endif

















