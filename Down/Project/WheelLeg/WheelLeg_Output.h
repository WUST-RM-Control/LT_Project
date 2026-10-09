#ifndef __WHEELLEG_OUTPUT_H__
#define __WHEELLEG_OUTPUT_H__

#include "main.h"
#include "Chassis.h"

// 单条腿最终关节输出 
typedef struct
{
//Leg

    /*===| VMC控关节PID参数 |===*/
    //腿摆角位置环PID
    PID_Struct_TypeDef Leg_Angle_PID;
    //腿摆角速度环PID
    PID_Struct_TypeDef Leg_Angle_Speed_PID;
    //腿长位置环PID
    PID_Struct_TypeDef Leg_Length_PID;
    //腿长速度环PID
    PID_Struct_TypeDef Leg_Length_Speed_PID;

//Motor
    //关节电机力矩的期望力矩NM(未限幅)
    //车头
    float Motor_Target_Torque13;
    float Motor_Target_Torque24;   //车尾

    //关节电机力矩的发送力矩NM(限幅)
    //车头
    float Motor_Send_Torque13;
    float Motor_Send_Torque24;   //车尾

    //轮电机发送扭矩
    float Motor_Send_Torque;

} WheelLeg_Output_Leg_StructTypedef;

// 整个关节输出
typedef struct
{
//Leg
    WheelLeg_Output_Leg_StructTypedef Leg_Left;
    WheelLeg_Output_Leg_StructTypedef Leg_Right;

} WheelLeg_Output_StructTypedef;


//关节电机限幅
#define WheelLeg_Joint_Output_Torque_Limit_NM    40.0f
//轮电机扭矩to电流
#define WheelLeg_Wheel_Output_Current2Torque     3.138094644e+3f

void WheelLeg_Output_Init(void);

int8_t WheelLeg_Output(Chassis_Control_StructTypedef *Chassis_Control);


#endif