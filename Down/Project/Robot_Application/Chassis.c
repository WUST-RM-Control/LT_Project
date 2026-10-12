/**
 * @file    Chassis.c[底盘控制]
 * @brief   轮腿底盘控制
 * @details 统一接口,外部控制底盘数据和传感器数据统一从 Chassis_Task 传入；
 *          Chassis_Task 调用 WheelLeg 文件的上层函数实现底盘的控制；
 *          Chassis_Control_Struct中存储需要输入到WheelLeg文件任务函数的参数
 * 			底层相关的底盘数据定义在各个 WheelLeg 前缀文件中
 */

#include "Chassis.h"
#include "RoboControl.h"
#include "Power_Limit.h"
#include "CAN_Driver.h"
#include "Motor_DJI_Driver.h"
#include "PID.h"
#include "USART_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Referee_Unpack.h"
#include "Vofa.h"
#include "INS.h"
#include "Function.h"
#include "Buzzer.h"
#include "arm_math.h"  // CMSIS-DSP 库
#include "user_lib.h"
#include "Remote_Control.h"
#include "kalman_filter.h"
#include <math.h>   
#include <stdbool.h>


#include "WheelLeg_SpringComp.h"
#include "WheelLeg_Kinematics.h"
#include "WheelLeg_LQR.h"
#include "WheelLeg_Output.h"
#include "WheelLeg_Chassis.h"

Chassis_Control_StructTypedef Chassis_Control_Struct = 
{
    .Leg_Left.Leg_LR  = Leg_L,
    .Leg_Right.Leg_LR = Leg_L
};


void Chassis_Task(void const * argument)
{
	osDelay(100);

    /*===| 默认模块赋值 |===*/
    Chassis_Control_Struct.Chassis_Control.Chassis_State = Chassis_OFF;
    Chassis_Control_Struct.Chassis_Control.Jump_Level = 1;
    Chassis_Control_Struct.Chassis_Control.Leg_Length_Feebback = Leg_Length_Max;
    DWT_GetDeltaT(&Chassis_Control_Struct.Chassis_Count);

    //运动学解算初始化(卡尔曼滤波器参数)
    WheelLeg_Kinematics_Init();
    //LQR运控初始化
    WheelLeg_LQR_Init();
    //电流输出初始化
    WheelLeg_Output_Init();
    //氮气弹簧补偿初始化
    WheelLeg_SpringComp_Init();
    for(;;)
    {
        Chassis_Control_Struct.Chassis_Dt = DWT_GetDeltaT(&Chassis_Control_Struct.Chassis_Count);
        
        /*===| 获得外部数据 |===*/
        Get_Control_Data(&Chassis_Control_Struct);
        
        /*===| 翻倒自起 |===*/
        Chassis_Smooth_Restand(&Chassis_Control_Struct);

        /*===| 底盘运动学解算 |===*/
        WheelLeg_Kinematics_Update(&Chassis_Control_Struct);
        
        /*===| LQR运控解算 |===*/
        WheelLeg_LQR_Output(&Chassis_Control_Struct);

        /*===| 氮气弹簧补偿 |===*/
        WheelLeg_SpringComp(&Chassis_Control_Struct.Leg_Left);
        WheelLeg_SpringComp(&Chassis_Control_Struct.Leg_Right);

        /*===| 输出 |===*/
        WheelLeg_Output(&Chassis_Control_Struct);

        osDelay(1);
    }
}

/**
 * @brief 获得外部数据
 * @note RoboControl_Struct赋值控制数据
 * @note INS赋值状态数据
 * 
 */
void Get_Control_Data(Chassis_Control_StructTypedef *Chassis_Control_Struct)
{
/*===| RoboControl_Struct数据赋值 |===*/
    //地盘状态赋值
    Chassis_Control_Struct->Chassis_Control.Chassis_State = RoboControl_Struct.Chassis_State;
    if(RoboControl_Struct.Chassis_Control_Flag.Chassis_RESET_Slow_FLag)
    {
        Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag = RoboControl_Struct.Chassis_Control_Flag.Chassis_RESET_Slow_FLag;
        RoboControl_Struct.Chassis_Control_Flag.Chassis_RESET_Slow_FLag = 0;
    }
    //目标运动速度
    Chassis_Control_Struct->Chassis_Control.Vy_Target = RoboControl_Struct.Robo_Target_Vy;
    Chassis_Control_Struct->Chassis_Control.Vx_Target = RoboControl_Struct.Robo_Target_Vx;
    Chassis_Control_Struct->Chassis_Control.Wz_Speed_Target = RoboControl_Struct.Robo_Target_Wz;

    Chassis_Control_Struct->Chassis_Control.Jump_Level = RoboControl_Struct.Jump_Level;
    Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = RoboControl_Struct.Leg_Length;
    if(Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_FOLLOW
    || Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_SPIN)
    {
        Chassis_Control_Struct->Leg_Left.Length_Target  = Chassis_Control_Struct->Chassis_Control.Leg_Length_Target;
        Chassis_Control_Struct->Leg_Right.Length_Target = Chassis_Control_Struct->Chassis_Control.Leg_Length_Target;
    }
    else if(Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_OFF)
    {
        Chassis_Control_Struct->Leg_Left.Length_Target  = Chassis_Control_Struct->Leg_Left.Length_Feedback;
        Chassis_Control_Struct->Leg_Right.Length_Target = Chassis_Control_Struct->Leg_Right.Length_Feedback;
    }
    
/*===| INS数据赋值 |===*/
    State_Filter(Chassis_Control_Struct,INS.MotionAccel_n[2]);
    Chassis_Control_Struct->Chassis_Control.Acceleration_Y = -INS.MotionAccel_n[1];
    Chassis_Control_Struct->Chassis_Control.Pitch_Last  = Chassis_Control_Struct->Chassis_Control.Pitch;
    Chassis_Control_Struct->Chassis_Control.Pitch       = -INS.Pitch * Angle2Radain;
    Chassis_Control_Struct->Chassis_Control.Pitch_dot   = (Chassis_Control_Struct->Chassis_Control.Pitch-Chassis_Control_Struct->Chassis_Control.Pitch_Last)/Chassis_Control_Struct->Chassis_Dt;
    Chassis_Control_Struct->Chassis_Control.Yaw_Speed   = INS.Gyro[Z];
}


void State_Filter(Chassis_Control_StructTypedef *Chassis_Control_Struct, float INS_MotionAccel_n_Z)
{
////	Chassis_Control_Struct.Left_Leg_Angle_dot_last  = Filter_Left_Leg_Angle_dot;
////	Chassis_Control_Struct.Right_Leg_Angle_dot_last = Filter_Right_Leg_Angle_dot;
////	Chassis_Control_Struct.Pitch_Speed_fdb_last		= Filter_Pitch_Speed_fdb;
////	Chassis_Control_Struct.Volecity_Fdb_last		= Filter_Volecity_Fdb;
	
////	Filter_Left_Leg_Angle_dot  = K_Lowpass_fliter*Chassis_Control_Struct.Left_Leg_Angle_dot  + (1.0f-K_Lowpass_fliter)*Chassis_Control_Struct.Left_Leg_Angle_dot_last;
////	Filter_Right_Leg_Angle_dot = K_Lowpass_fliter*Chassis_Control_Struct.Right_Leg_Angle_dot + (1.0f-K_Lowpass_fliter)*Chassis_Control_Struct.Right_Leg_Angle_dot_last;
////	Filter_Pitch_Speed_fdb     = K_Lowpass_fliter*Chassis_Control_Struct.Pitch_Speed_fdb     + (1.0f-K_Lowpass_fliter)*Chassis_Control_Struct.Pitch_Speed_fdb_last;
////	Filter_Volecity_Fdb		   = K_Lowpass_fliter*Chassis_Control_Struct.Volecity_Fdb        + (1.0f-K_Lowpass_fliter)*Chassis_Control_Struct.Volecity_Fdb_last;
	
    static float MotionAccel_n_Z_last = 0.0f, Filter_MotionAccel_n_Z =0.0f;
	MotionAccel_n_Z_last = Filter_MotionAccel_n_Z;

//================== Z轴加速度：中值滤波(去毛刺) + 低通滤波(去波动) ==================
	static float z_buf[3] = {0.0f, 0.0f, 0.0f}; // 存储3次原始数据
	static uint8_t z_idx = 0;
	static uint8_t z_init_cnt = 0;
	float Median_MotionAccel_n_Z;
	float z_a, z_b, z_c;

	// 1. 更新滑动窗口
	z_buf[z_idx] = INS_MotionAccel_n_Z;
	z_idx++;
	if(z_idx >= 3) z_idx = 0;

	// 2. 启动保护：前3次直接透传，防止初始0.0f污染中值
	if(z_init_cnt < 3) 
	{
		Median_MotionAccel_n_Z = INS_MotionAccel_n_Z;
		z_init_cnt++;
	}
	else 
	{
		// 3. 提取窗口数据
		z_a = z_buf[0];
		z_b = z_buf[1];
		z_c = z_buf[2];

		// 4. 纯逻辑判断求中值（无排序、无额外函数调用，最高效）
		if (z_a > z_b) 
		{
			if (z_b > z_c) Median_MotionAccel_n_Z = z_b;
			else if (z_a > z_c) Median_MotionAccel_n_Z = z_c;
			else Median_MotionAccel_n_Z = z_a;
		} 
		else 
		{
			if (z_a > z_c) Median_MotionAccel_n_Z = z_a;
			else if (z_b > z_c) Median_MotionAccel_n_Z = z_c;
			else Median_MotionAccel_n_Z = z_b;
		}
	}

	// 5. 一阶低通滤波（输入源从原始数据改为中值数据）
	Filter_MotionAccel_n_Z = 0.05f * Median_MotionAccel_n_Z + 0.95f * MotionAccel_n_Z_last;//中值滤波+低通滤波
//	Filter_MotionAccel_n_Z = Median_MotionAccel_n_Z;//中值滤波
//	Filter_MotionAccel_n_Z = K_Lowpass_fliter*INS_MotionAccel_Z + (1.0f-K_Lowpass_fliter)*MotionAccel_n_Z_last;//低通滤波
	
	Chassis_Control_Struct->Chassis_Control.Acceleration_Z = Filter_MotionAccel_n_Z;
//=========================================================================
}
