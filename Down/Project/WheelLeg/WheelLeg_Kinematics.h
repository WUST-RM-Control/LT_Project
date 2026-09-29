#ifndef _WHEELLEG_KINEMATICS_H_
#define _WHEELLEG_KINEMATICS_H_

#include "main.h"

//虚拟退运动学
typedef struct
{
    
    //方向统一后的两个关节角  单位：rad
    float q_front_rad;
    float q_back_rad;

    //五连杆内部角度
    float phi1_rad;
    float phi2_rad;
    float phi3_rad;
    float phi4_rad;

    //虚拟腿末端点 C 坐标 单位：m
    float endpoint_x_m;
    float endpoint_y_m;

    //虚拟腿长度 单位：m
    float length_m;
 
	//虚拟腿连续角度
	float angle_total_rad;
	
	//C点雅可比
	float jacobian_xy[2][2];
	
	//虚拟腿雅可比
	float jacobian_virtual[2][2];
	
	//雅可比有效标志位 
	uint8_t jacobian_valid;
	
	//关节角速度
	float phi1_velocity_rad_s;
	float phi4_velocity_rad_s;
	
	//虚拟腿速度
	float length_velocity_m_s;
    float angle_velocity_rad_s;
	
	//上一阶段数据
    float last_angle_rad;
	
	//第一次计算标志位
	uint8_t angle_initialized;
	
    //虚拟腿角度  单位：rad 当前定义：腿竖直向下 ≈ 0 rad
    float angle_rad;

  //运动学计算是否有效
    uint8_t valid;

} WheelLeg_LegKinematics;

//左右腿运动学
typedef struct
{
    WheelLeg_LegKinematics left;
    WheelLeg_LegKinematics right;
} WheelLeg_Kinematics;

extern WheelLeg_Kinematics wheelLeg_kinematics;

void WheelLeg_Kinematics_Update(float dt);

#endif 
