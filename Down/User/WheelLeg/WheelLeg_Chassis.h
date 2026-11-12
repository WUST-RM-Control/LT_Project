#ifndef _WheelLeg_Chassis_
#define _WheelLeg_Chassis_

#include "main.h"
#include "Chassis.h"
typedef struct
{
/*===| 离地检测 |===*/
    //单腿支撑力(N)
    float Leg_Left_Support_Force;
    float Leg_Right_Support_Force;
    //单腿离地标志位:触地0/离地1
    uint8_t Leg_Left_Off_ground;
    uint8_t Leg_Right_Off_ground;
} WheelLeg_Chassis_Off_ground_StructTypeDef;

int8_t Chassis_Smooth_Restand_Fun(Chassis_Control_StructTypedef *Chassis_Control);
int8_t Chassis_OFF_Slow_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct, float Smooth_Start_K);
int8_t Chassis_Jump_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct);
int8_t Chassis_Up_Step_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct);
int8_t Chassis_Off_Ground_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct);
#endif
