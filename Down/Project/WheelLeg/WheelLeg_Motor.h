#ifndef _WheelLeg_Motor_
#define _WheelLeg_Motor_

#include "main.h"

//**********关节电机反馈数据*********
typedef struct
{

    float raw_angle_deg;          // 达妙反馈单圈角度 [deg]
    float raw_total_angle_deg;    // 达妙累计角度 [deg]

    float raw_velocity_deg_s;     //达妙角速度 单位deg/s 注：原Motor_Data_Struct中的Speed_RPM对达妙而言实际是 deg/s，并不是RPM
    float raw_torque;             // 达妙原始力矩反馈

    int8_t temperature;
    uint8_t online;
    uint8_t error_id;

    //统一为国标单位
    float position_rad;           // 单圈位置 [rad]
    float total_position_rad;     // 累计位置 [rad]
    float velocity_rad_s;         // 角速度 [rad/s]

} WheelLeg_JointMotor;


//*********轮电机反馈数据********
typedef struct
{

    float rotor_angle_deg;
    float rotor_total_angle_deg;
    float rotor_speed_rpm;

    int16_t current_feedback_raw;

    int8_t temperature;
    uint8_t online;

	
    float position_rad;           // 输出轴累计角度rad
    float velocity_rad_s;         // 输出轴角速度 rad/s


    float displacement_m;         // 轮子滚动距离 m
    float linear_velocity_m_s;    // 车体前进方向速度m/s

}WheelLeg_WheelMotor;


//********整体轮腿电机数据**********
typedef struct
{
    //左腿关节 
    WheelLeg_JointMotor left_front;
    WheelLeg_JointMotor left_back;

    //右腿关节 
    WheelLeg_JointMotor right_front;
    WheelLeg_JointMotor right_back;

    //两个轮子
    WheelLeg_WheelMotor left_wheel;
    WheelLeg_WheelMotor right_wheel;

} WheelLeg_Motor;

extern WheelLeg_Motor wheelLeg_motor;

void WheelLeg_Motor_Update(void);

void WheelLeg_Motor_SendWheelTorque(float left_torque_nm,float right_torque_nm);

#endif
