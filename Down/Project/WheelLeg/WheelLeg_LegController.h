#ifndef __WHEELLEG_LEGCONTROLLER_H__
#define __WHEELLEG_LEGCONTROLLER_H__

#include "main.h"

typedef struct 
{
    //目标腿长
    float target_length_m;

    //当前腿长 
    float current_length_m;

    //腿长误差
    float length_error_m;

    //腿长误差PID
    PID_Struct_TypeDef length_position_pid;

    //腿长位置环输出 输出腿长变化速度给速度环
    float target_length_velocity_m_s;

    //当前腿长速度
    float current_length_velocity_m_s;

    //腿长内环PID
    PID_Struct_TypeDef length_velocity_pid;

    //腿长速度误差
    float length_velocity_error_m_s;

    //速度环输出
    float force_pid_n;   

    //重力补偿
    float gravity_comp_n;

    //氮气弹簧补偿
    float spring_comp_n;

    //最终虚拟里
    float force_total_n;

    //控制器有效标识
    uint8_t valid;

}WheelLeg_LegController_Leg;

typedef struct 
{
    WheelLeg_LegController_Leg left;
    WheelLeg_LegController_Leg right;

    // Roll横滚补偿 
    PID_Struct_TypeDef roll_pid;

    float target_roll_deg;
    float current_roll_deg;

    // 左右腿差动支撑补偿力
    float roll_comp_n;

}WheelLeg_LegController;


extern WheelLeg_LegController wheelLeg_legController;

void WheelLeg_LegController_Init(void);

void WheelLeg_LegController_SetTarget(float left_length_m,float right_length_m);

void WheelLeg_LegController_Update(void);
#endif