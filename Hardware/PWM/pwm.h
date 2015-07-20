#ifndef _PWM_H
#define _PWM_H
#include "sys/sys.h"

void Timer4_PWM_Init(u16 arr,u16 psc);
void PWM_SET(s16 CH1_CCR,s16 CH2_CCR,s16 CH3_CCR,s16 CH4_CCR);
void Motor_Start_Up(void);
void Motor_Init(void);
void Motor_Stop(void);
#endif

