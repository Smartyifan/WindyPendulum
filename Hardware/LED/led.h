#ifndef __LED_H
#define __LED_H	 
#include "sys/sys.h"
//Mini STM32开发板
//LED驱动代码			 
//正点原子@ALIENTEK
//2010/5/27

//LED端口定义
#define R_LED PAout(7) 	// PA.7
#define G_LED PAout(1)	// PA.1
#define B_LED PAout(0)	// PA.0

void LED_Init(void);//初始化		 				    
#endif

















