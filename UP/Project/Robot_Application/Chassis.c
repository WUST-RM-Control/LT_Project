//===========================================================================
//                      Chassis[底盘控制]
//      根据云台发下来的Vx,Vy,Wz,YawErr进行运动解算，实现底盘的各种运动状态
//      以及根据裁判系统反馈数据和[超电状态]实现功率控制
//===========================================================================

#include "Chassis.h"
#include "RoboControl.h"
#include "Power_Limit.h"
#include "Referee_Unpack.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"

/*===| 底盘控制数据结构体 |===*/
Chassis_Control_StructTypeDef Chassis_Control_Struct;

/**
 * @brief 底盘任务
 */
void Chassis_Task(void *argument)
{
    osDelay(1000);
    /*===| 底盘PID参数初始化 |===*/
    PID_Init(&Chassis_Control_Struct.MotorID_PID_Struct[0], Chassis_PID_Kp, Chassis_PID_Ki, Chassis_PID_Kd, 0, Chassis_PID_I_Output_Max, Chassis_PID_Output_Max);
    PID_Init(&Chassis_Control_Struct.MotorID_PID_Struct[1], Chassis_PID_Kp, Chassis_PID_Ki, Chassis_PID_Kd, 0, Chassis_PID_I_Output_Max, Chassis_PID_Output_Max);
    PID_Init(&Chassis_Control_Struct.MotorID_PID_Struct[2], Chassis_PID_Kp, Chassis_PID_Ki, Chassis_PID_Kd, 0, Chassis_PID_I_Output_Max, Chassis_PID_Output_Max);
    PID_Init(&Chassis_Control_Struct.MotorID_PID_Struct[3], Chassis_PID_Kp, Chassis_PID_Ki, Chassis_PID_Kd, 0, Chassis_PID_I_Output_Max, Chassis_PID_Output_Max);

    PID_Init(&Chassis_Control_Struct.Gimbal_Vx_PID_Struct, 0.1, 0.01, 1, 0, 0.2*Robo_Vx_MaxSpeed, Robo_Vx_MaxSpeed);
    PID_Init(&Chassis_Control_Struct.Gimbal_Vy_PID_Struct, 0.1, 0.01, 1, 0, 0.2*Robo_Vy_MaxSpeed, Robo_Vy_MaxSpeed);
    PID_Init(&Chassis_Control_Struct.Gimbal_Wz_PID_Struct, 0.1, 0.01, 1, 0, 0.2*Robo_Wz_MaxSpeed, Robo_Wz_MaxSpeed);
       
    {
        /*===| 里程计积分 |===*/
        float Dt = DWT_GetDeltaT(&Chassis_Control_Struct.Odometer_DWT_Count);
        Chassis_Control_Struct.Odometer_Chassis_Vx = (Chassis_RPM_to_m_s * 0.25f) * (- Motor.Chassis1.Speed_RPM - Motor.Chassis2.Speed_RPM + Motor.Chassis3.Speed_RPM + Motor.Chassis4.Speed_RPM);
        Chassis_Control_Struct.Odometer_Chassis_Vy = (Chassis_RPM_to_m_s * 0.25f) * (- Motor.Chassis1.Speed_RPM + Motor.Chassis2.Speed_RPM - Motor.Chassis3.Speed_RPM + Motor.Chassis4.Speed_RPM);
        Chassis_Control_Struct.Odometer_Chassis_Wz = (Chassis_RPM_to_m_s * 0.25f) * (- Motor.Chassis1.Speed_RPM - Motor.Chassis2.Speed_RPM - Motor.Chassis3.Speed_RPM - Motor.Chassis4.Speed_RPM) / Chassis_R_Distance;
        Chassis_Control_Struct.Odometer_Chassis_X += Chassis_Control_Struct.Odometer_Chassis_Vx * Dt;
        Chassis_Control_Struct.Odometer_Chassis_Y += Chassis_Control_Struct.Odometer_Chassis_Vy * Dt;
        Chassis_Control_Struct.Odometer_Chassis_Angle += Chassis_Control_Struct.Odometer_Chassis_Wz * Dt * (180.0f / PI);
        
        Chassis_Control_Struct.Odometer_Gimbal_Vy = Chassis_Control_Struct.Odometer_Chassis_Vx * cosf(Chassis_Control_Struct.Yaw_ErrAngle) + Chassis_Control_Struct.Odometer_Chassis_Vy * sinf(Chassis_Control_Struct.Yaw_ErrAngle);
        Chassis_Control_Struct.Odometer_Gimbal_Vx = Chassis_Control_Struct.Odometer_Chassis_Vx * sinf(Chassis_Control_Struct.Yaw_ErrAngle) - Chassis_Control_Struct.Odometer_Chassis_Vy * cosf(Chassis_Control_Struct.Yaw_ErrAngle);
        Chassis_Control_Struct.Odometer_Gimbal_Wz = Chassis_Control_Struct.Odometer_Chassis_Wz;
        Chassis_Control_Struct.Odometer_Gimbal_X += Chassis_Control_Struct.Odometer_Gimbal_Vx * Dt;
        Chassis_Control_Struct.Odometer_Gimbal_Y += Chassis_Control_Struct.Odometer_Gimbal_Vy * Dt;
        Chassis_Control_Struct.Odometer_Gimbal_Angle += Chassis_Control_Struct.Odometer_Gimbal_Wz * Dt * (180.0f / PI);
        
        
        /*===| 缓启动 |===*/
        if      (RoboControl_Struct.Robo_Target_Vx > Chassis_Control_Struct.Gimbal_Vx_Target && RoboControl_Struct.Robo_Target_Vx >= + Dt * Chassis_Acc) Chassis_Control_Struct.Gimbal_Vx_Target += Dt * Chassis_Acc;
        else if (RoboControl_Struct.Robo_Target_Vx < Chassis_Control_Struct.Gimbal_Vx_Target && RoboControl_Struct.Robo_Target_Vx <= - Dt * Chassis_Acc) Chassis_Control_Struct.Gimbal_Vx_Target -= Dt * Chassis_Acc;
        else Chassis_Control_Struct.Gimbal_Vx_Target = RoboControl_Struct.Robo_Target_Vx;                                                  
                                                                                                                                            
        if      (RoboControl_Struct.Robo_Target_Vy > Chassis_Control_Struct.Gimbal_Vy_Target && RoboControl_Struct.Robo_Target_Vy >= + Dt * Chassis_Acc) Chassis_Control_Struct.Gimbal_Vy_Target += Dt * Chassis_Acc;
        else if (RoboControl_Struct.Robo_Target_Vy < Chassis_Control_Struct.Gimbal_Vy_Target && RoboControl_Struct.Robo_Target_Vy <= - Dt * Chassis_Acc) Chassis_Control_Struct.Gimbal_Vy_Target -= Dt * Chassis_Acc;
        else Chassis_Control_Struct.Gimbal_Vy_Target = RoboControl_Struct.Robo_Target_Vy;                                                  
                                                                                                                                            
        if      (RoboControl_Struct.Robo_Target_Wz > Chassis_Control_Struct.Gimbal_Wz_Target && RoboControl_Struct.Robo_Target_Wz >= + Dt * Chassis_Acc / Chassis_R_Distance) Chassis_Control_Struct.Gimbal_Wz_Target += Dt * Chassis_Acc / Chassis_R_Distance;
        else if (RoboControl_Struct.Robo_Target_Wz < Chassis_Control_Struct.Gimbal_Wz_Target && RoboControl_Struct.Robo_Target_Wz <= - Dt * Chassis_Acc / Chassis_R_Distance) Chassis_Control_Struct.Gimbal_Wz_Target -= Dt * Chassis_Acc / Chassis_R_Distance;
        else Chassis_Control_Struct.Gimbal_Wz_Target = RoboControl_Struct.Robo_Target_Wz;  
        
        
//        /*===| 里程计闭环速度 |===*/
//        PID_Position_Calculate(&Chassis_Control_Struct.Gimbal_Vx_PID_Struct, Chassis_Control_Struct.Gimbal_Vx_Target, Chassis_Control_Struct.Odometer_Gimbal_Vx);
//        PID_Position_Calculate(&Chassis_Control_Struct.Gimbal_Vy_PID_Struct, Chassis_Control_Struct.Gimbal_Vy_Target, Chassis_Control_Struct.Odometer_Gimbal_Vy);
//        PID_Position_Calculate(&Chassis_Control_Struct.Gimbal_Wz_PID_Struct, Chassis_Control_Struct.Gimbal_Wz_Target, Chassis_Control_Struct.Odometer_Gimbal_Wz);
//        
//        Chassis_Control_Struct.Gimbal_Vx_Set = Chassis_Control_Struct.Gimbal_Vx_Target + Chassis_Control_Struct.Gimbal_Vx_PID_Struct.Output;
//        Chassis_Control_Struct.Gimbal_Vy_Set = Chassis_Control_Struct.Gimbal_Vy_Target + Chassis_Control_Struct.Gimbal_Vy_PID_Struct.Output;
//        Chassis_Control_Struct.Gimbal_Wz_Set = Chassis_Control_Struct.Gimbal_Wz_Target + Chassis_Control_Struct.Gimbal_Wz_PID_Struct.Output;
        
        Chassis_Control_Struct.Gimbal_Vx_Set = Chassis_Control_Struct.Gimbal_Vx_Target;
        Chassis_Control_Struct.Gimbal_Vy_Set = Chassis_Control_Struct.Gimbal_Vy_Target;
        Chassis_Control_Struct.Gimbal_Wz_Set = Chassis_Control_Struct.Gimbal_Wz_Target;
        
        
        /*===| 得到Yaw偏移弧度，用于相对云台全向移动 |===*/
        Chassis_Control_Struct.Yaw_ErrAngle = -RoboControl_Struct.Yaw_Err * PI / 180.0f;
        /*===| 冲刺模式关闭横向运动，前进速度增加 |===*/
        if(RoboControl_Struct.Chassis_State == Chassis_DASH)
        {
            Chassis_Control_Struct.Chassis_Vx_Set = 0;
            Chassis_Control_Struct.Chassis_Vy_Set = Chassis_Dash_Speed_Multiple * Chassis_Control_Struct.Gimbal_Vy_Set * cosf(Chassis_Control_Struct.Yaw_ErrAngle) + Chassis_Control_Struct.Gimbal_Vx_Set * sinf(Chassis_Control_Struct.Yaw_ErrAngle);
            Chassis_Control_Struct.Chassis_Wz_Set = Chassis_Control_Struct.Gimbal_Wz_Target;
        }
        else
        {
            Chassis_Control_Struct.Chassis_Vx_Set = Chassis_Control_Struct.Gimbal_Vx_Set * cosf(Chassis_Control_Struct.Yaw_ErrAngle) + Chassis_Control_Struct.Gimbal_Vy_Set * sinf(Chassis_Control_Struct.Yaw_ErrAngle);
            Chassis_Control_Struct.Chassis_Vy_Set = Chassis_Control_Struct.Gimbal_Vy_Set * cosf(Chassis_Control_Struct.Yaw_ErrAngle) - Chassis_Control_Struct.Gimbal_Vx_Set * sinf(Chassis_Control_Struct.Yaw_ErrAngle);
            Chassis_Control_Struct.Chassis_Wz_Set = Chassis_Control_Struct.Gimbal_Wz_Target;
        }
            
        /*===| 运动学解算[麦轮] |===*/
        Chassis_Control_Struct.MotorID_SpeedSet[0] = (-Chassis_Control_Struct.Chassis_Vy_Set - Chassis_Control_Struct.Chassis_Vx_Set) - Chassis_R_Distance * Chassis_Control_Struct.Chassis_Wz_Set;
        Chassis_Control_Struct.MotorID_SpeedSet[1] = (+Chassis_Control_Struct.Chassis_Vy_Set - Chassis_Control_Struct.Chassis_Vx_Set) - Chassis_R_Distance * Chassis_Control_Struct.Chassis_Wz_Set;
        Chassis_Control_Struct.MotorID_SpeedSet[2] = (-Chassis_Control_Struct.Chassis_Vy_Set + Chassis_Control_Struct.Chassis_Vx_Set) - Chassis_R_Distance * Chassis_Control_Struct.Chassis_Wz_Set;
        Chassis_Control_Struct.MotorID_SpeedSet[3] = (+Chassis_Control_Struct.Chassis_Vy_Set + Chassis_Control_Struct.Chassis_Vx_Set) - Chassis_R_Distance * Chassis_Control_Struct.Chassis_Wz_Set;
        
        /*===| PID计算得到每个电机的电流大小 |===*/
        PID_Position_Calculate(&Chassis_Control_Struct.MotorID_PID_Struct[0], Chassis_Control_Struct.MotorID_SpeedSet[0] / Chassis_RPM_to_m_s, Motor.Chassis1.Speed_RPM);
        PID_Position_Calculate(&Chassis_Control_Struct.MotorID_PID_Struct[1], Chassis_Control_Struct.MotorID_SpeedSet[1] / Chassis_RPM_to_m_s, Motor.Chassis2.Speed_RPM);
        PID_Position_Calculate(&Chassis_Control_Struct.MotorID_PID_Struct[2], Chassis_Control_Struct.MotorID_SpeedSet[2] / Chassis_RPM_to_m_s, Motor.Chassis3.Speed_RPM);
        PID_Position_Calculate(&Chassis_Control_Struct.MotorID_PID_Struct[3], Chassis_Control_Struct.MotorID_SpeedSet[3] / Chassis_RPM_to_m_s, Motor.Chassis4.Speed_RPM);
        
        /*===| 冲刺模式减少前轮输出 |===*/
        if(RoboControl_Struct.Chassis_State == Chassis_DASH) 
        {
            Chassis_Control_Struct.MotorID_PID_Struct[0].Output *= 0.3f;
            Chassis_Control_Struct.MotorID_PID_Struct[1].Output *= 0.3f;
        }
        
        /*===| 功率控制 |===*/
        if(RoboControl_Struct.SuperCap_State == 1) 
        {
            Chassis_Control_Struct.Power_MAX = Robo_State.chassis_power_limit + 50.0f + 30 * RoboControl_Struct.Chassis_Speed_Level;
            if(RoboControl_Struct.SuperCap_V >= 8 && RoboControl_Struct.SuperCap_V <= 12)Chassis_Control_Struct.Power_MAX *= ( RoboControl_Struct.SuperCap_V-8) / 4.0f; 
        }
        else
        {
            Chassis_Control_Struct.Power_MAX = Robo_State.chassis_power_limit - 5.0f * (40.0f - Robo_PowerHeatData.buffer_energy);
        }
        Power_Limit(&Chassis_Control_Struct, &Motor, Chassis_Control_Struct.Power_MAX);
        
        /*===| 发送电流参数 |===*/
        if(RoboControl_Struct.Robo_Enable) 
        {
            Motor_DJI_SendCurrent(&Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID,    
                                  Chassis_Control_Struct.MotorID_PID_Struct[0].Output,
                                  Chassis_Control_Struct.MotorID_PID_Struct[1].Output,
                                  Chassis_Control_Struct.MotorID_PID_Struct[2].Output,
                                  Chassis_Control_Struct.MotorID_PID_Struct[3].Output);
        }
        else 
        {
            Chassis_Control_Struct.Gimbal_Vx_PID_Struct.I_Output = 0;
            Chassis_Control_Struct.Gimbal_Vy_PID_Struct.I_Output = 0;
            Chassis_Control_Struct.Gimbal_Wz_PID_Struct.I_Output = 0;
            Motor_DJI_SendCurrent(&Chassis_Motor_CAN, Chassis_Motor_Send_CAN_ID, 0, 0, 0, 0);   
        }                                                                    
        
        osDelay(1);
    }
}

