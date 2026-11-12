#ifndef __PWM__
#define __PWM__

#include "main.h"

/*===| 初始化PWM TIM1 TIM8 |===*/
void PWM_Init(void);

/*===| 设置各个通道的占空比 |===*/
void PWM_SetEmpty(uint8_t Channel, float Empty);

/*===| 设置舵机角度 |===*/
void PWM_Set_Servo_Angle(uint8_t Channel, float Angle);

#endif
