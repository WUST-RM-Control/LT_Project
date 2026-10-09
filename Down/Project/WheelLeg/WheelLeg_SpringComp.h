#ifndef __WHEELLEG_SPRINGCOMP_H__
#define __WHEELLEG_SPRINGCOMP_H__

#include "main.h"

typedef enum
{
    Down =  1,
    Up   = -1
}Start_State_EnumTypedef;

typedef struct 
{
    //0正转(初始值转到末端值)，1反转
    uint8_t Caled_State;
    //初始转动的目标值(UP:一开始目标值减小，Down:一开始目标值增大)
    Start_State_EnumTypedef Start_State;

    float Compensation_Speed;//每秒改变的目标值
    int8_t Compensation_Round;
    float Compensation_Distance_UP;
    float Compensation_Distance_Down;
    //区间大小：一般360(略大于或等于最大的Compensation_Distance_UP-Compensation_Distance_Down，最大指排除机械限位后可能的角度)
    float Compensation_Distance_Range;
    
    float Distance_Target;
    float Distance_Feedback; 
    float SpeedRPM_Feedback; 
    float Current_Output;
    
    PID_Struct_TypeDef Distance_PID;
    PID_Struct_TypeDef Speed_PID;
    
    float* Data;
    
    uint32_t DWT_Counter;
    float Dt;

    float Friction_K;
    float Friction_Out;
    float Friction_ACC;
    float Friction_DEC;


    //Leg数据
        Chassis_Leg_RL_State_EnumTypedef Leg_RL_State;
		float L0_Target;//一阶倒立摆腿长
		float L0_Feedback;
		float L0_Last;
		float L0_Speed;
		float L0_Speed_Last;
		
		float A0_Target;//一阶倒立摆腿角度
		float A0_Feedback;
		float A0_Last;
		float A0_Round;//圈数

		float Total_A0_Target;//一阶倒立摆腿总角度
		float Total_A0_Feedback;
		float Total_A0_Last;
		float Total_A0_Speed;
	
	/*===| 雅可比力矩阵 |===*/
	float Jt[2][2];

	//关节电机目标力矩
	float T_Target[2];
	//关节电机VMC
	//0L:沿杆方向的，1T关节为绕轴力矩
	float L_T_Target[2];


} Motor_Compensation_Config_StructTypedef;

typedef struct
{
    float L0_Leg_L_Cal_Data[2048];
    float L0_Leg_R_Cal_Data[2048];
} Config_StructTypedef;


void WheelLeg_SpringComp_Init(void);

void WheelLeg_SpringComp(Chassis_Leg_StructTypedef *Leg);

uint8_t WheelLeg_SpringComp_Measure_Task
(const uint8_t Leg_LR,
const volatile uint8_t * const Enable_Output,
void (* const Fun_Motor_Joint_Wheel_Output)(float,float,float,float,float,float),
const volatile Motor_Data_StructTypeDef * const Motor1,
const volatile Motor_Data_StructTypeDef * const Motor2,
const volatile Motor_Data_StructTypeDef * const Motor3,
const volatile Motor_Data_StructTypeDef * const Motor4);
#endif