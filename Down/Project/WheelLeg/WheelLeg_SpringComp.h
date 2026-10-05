#ifndef __WHEELLEG_SPRINGCOMP_H__
#define __WHEELLEG_SPRINGCOMP_H__

#include "main.h"

void WheelLeg_SpringComp_Init(void);

void WheelLeg_SpringComp(float Leg_Length_Feedback,float Leg_Length_Target,float *GasSpring_Output,uint8_t Leg_LR);

uint8_t WheelLeg_SpringComp_Measure_Task
(const uint8_t Leg_LR,
const volatile uint8_t * const Enable_Output,
void (* const Fun_Motor_Joint_Wheel_Output)(float,float,float,float,float,float),
const volatile Motor_Data_StructTypeDef * const Motor1,
const volatile Motor_Data_StructTypeDef * const Motor2,
const volatile Motor_Data_StructTypeDef * const Motor3,
const volatile Motor_Data_StructTypeDef * const Motor4);
#endif