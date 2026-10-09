#ifndef __WHEELLEG_LQR_H__
#define __WHEELLEG_LQR_H__

#include "main.h"

typedef struct
{
    //轮子的未滤波数据
    float Wheel_Speed_Feedback;	//m/s，与底盘机械正方向一致(轮速度，存在打滑)
    float Displacement_Now;		//m，当前轮位移,与轮方向一致
    float Displacement_Init;	//m，开始LQR运控时轮位移,与轮方向一致
    float Displacement;			//m，用于LQR运控的轮位移，与底盘机械正方向一致，+-(Displacement_Now-Displacement_Init)
	
} WheelLeg_LQR_Displacement_Date;

typedef struct
{
	//LQR
		/*===| 8个状态量 |===*/
		//0:无
		//1:L_杆与竖直方向夹角
		//2:L_杆与竖直方向夹角dot
		//3:位移
		//4:位移dot
		//5:Pitch
		//6:Pitch_Dot
		//7:R_杆与竖直方向夹角
		//8:R_杆与竖直方向夹角dot
		float State[9];

        float LQR_K_Left[12];

        float LQR_K_Right[12];


    //底盘位移
    float Wheel_Displacement_Feedback;  //底盘位移，与底盘机械正方向一致
    float Wheel_Displacement_Target;    //RoboControl_Struct目标速度积分得到

    float Wheel_Speed_Feedback;         //底盘速度(滤波后)，与底盘机械正方向一致
    float Wheel_Speed_Target;           //m/s，RoboControl_Struct赋值(Vy)


    float Wheel_displacement_add;       //目标速度积分目标位移得到
    float Wheel_displacement_init;      //正常运动开始时的位移，恢复正常运动时的Wheel_Displacement_Feedback赋值
    float Wheel_Speed_Target_Last;

    //LQR运控PID补丁
        //转向&小陀螺
	    PID_Struct_TypeDef Wheel_Wz_Speed_PID;
        //防劈叉
	    PID_Struct_TypeDef Leg_Splits_PID;
        

} WheelLeg_LQR_StructTypeDef;

void WheelLeg_LQR_Init(void);

void WheelLeg_LQR_Output(Chassis_Control_StructTypedef *Chassis_Control);

#endif