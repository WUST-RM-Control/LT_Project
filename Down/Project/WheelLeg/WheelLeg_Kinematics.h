#ifndef _WHEELLEG_KINEMATICS_H_
#define _WHEELLEG_KINEMATICS_H_

#include "main.h"

/*===| 虚拟腿运动学数据 |===*/
typedef struct
{
//滤波前数据(滤波后数据直接赋值到Chassis_Control_Struct里面)
    //腿长 m，
    float Length_Feedback_No_kalman;
    //腿摆角 °，
    float Angle_Feedback_No_kalman;

    /*===| 雅可比力矩阵 |===*/
	float Jt[2][2];

} WheelLeg_LegKinematics_StructTypedef;

//左右腿运动学
typedef struct
{
    WheelLeg_LegKinematics_StructTypedef Left;
    WheelLeg_LegKinematics_StructTypedef Right;
} WheelLeg_Kinematics_StructTypedef;

void WheelLeg_Kinematics_Update(Chassis_Control_StructTypedef *Chassis_Control_Struct,const float dt)

#endif 
