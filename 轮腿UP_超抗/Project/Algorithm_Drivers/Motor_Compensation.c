#include "Motor_Compensation.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "DWT.h"
#include "Motor_Compensation_Data.h"
#include "INS.h"

//单圈采样点数，保存数组需要 2[正|反圈 & 齿槽补偿|摩擦补偿] * 4[float类型] * Point_Num字节大小
#define Point_Num 1024

Motor_Compensation_Config_StructTypedef Yaw_Compensation_Config = 
{
    .Caled_State = 0,
    
    .Compensation_Round = 1,
    .Compensation_Speed = 10,
    .Compensation_Angle_UP = 380,
    .Compensation_Angle_Down = -20,
    
    .Data = Config.Motor_Yaw_Cal_Data,
    
    .Friction_ACC = 30000,
    .Friction_DEC = 50000,
    .Friction_K = 0.8,
};

Motor_Compensation_Config_StructTypedef Pitch_Compensation_Config = 
{
    .Caled_State = 0,
    
    .Compensation_Round = 1,
    .Compensation_Speed = 5,
    .Compensation_Angle_UP = 9,
    .Compensation_Angle_Down = -43,
    
    .Data = Config.Motor_Pitch_Cal_Data,
    
    .Friction_ACC = 20000,
    .Friction_DEC = 50000,
    .Friction_K = 0,
};

void Motor_Compensation_Task(void)
{
/*===| Yaw |===*/
    //***校准参数初始化
    Yaw_Compensation_Config.Angle_Target = Yaw_Compensation_Config.Compensation_Angle_Down;
    //清空原先的校准数据
    for(uint16_t i = 0; i < 2*Point_Num; i++) Yaw_Compensation_Config.Data[i] = 0; 
    //***PID参数初始化
    PID_Init(&Yaw_Compensation_Config.Angle_PID, 3,  0, 5,  0,  0,    50);
    PID_Init(&Yaw_Compensation_Config.Speed_PID, 300, 1, 0,    50, 6000, 25000);
    //***循环校准
    while(Yaw_Compensation_Config.Caled_State <= 1)
    {
        //***采样电机角度与速度
        Yaw_Compensation_Config.Angle_Feedback = Motor.Yaw.Total_Angle;
        Yaw_Compensation_Config.SpeedRPM_Feedback = Motor.Yaw.Total_Angle_Speed_RPM;
        //***更新校准数据(设置目标角度与速度，PID计算，保存校准值)
        Motor_Compensation(&Yaw_Compensation_Config);
        //***输出电流
        Motor_DJI_SendCurrent(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID, 0, Yaw_Compensation_Config.Current_Output,0 ,0);		
        
        osDelay(1);
    }
    //***计算齿槽补偿与摩擦补偿
    for(uint16_t i = 0; i < Point_Num; i++) 
    {
        float Coggin_Torque = 0.5f * (Yaw_Compensation_Config.Data[i] + Yaw_Compensation_Config.Data[i+Point_Num]);
        float Friction_Torque = 0.5f * (Yaw_Compensation_Config.Data[i] - Yaw_Compensation_Config.Data[i+Point_Num]);
        Yaw_Compensation_Config.Data[i] = Coggin_Torque;
        Yaw_Compensation_Config.Data[i+Point_Num] = Friction_Torque;
    }
    
/*===| Pitch |===*/
    //***校准参数初始化
    Pitch_Compensation_Config.Angle_Target = Pitch_Compensation_Config.Compensation_Angle_Down;
    //清空原先的校准数据
    for(uint16_t i = 0; i < 2*Point_Num; i++) Pitch_Compensation_Config.Data[i] = 0; 
    //***PID参数初始化
    PID_Init(&Pitch_Compensation_Config.Angle_PID, 3,  0,  5,  0,  0,    20);
    PID_Init(&Pitch_Compensation_Config.Speed_PID, 200, 1, 0,    50,  16000, 25000);
    //***循环校准
    while(Pitch_Compensation_Config.Caled_State <= 1)
    {
        //***采样电机角度与速度
        Pitch_Compensation_Config.Angle_Feedback = Motor.Pitch.Angle;
        Pitch_Compensation_Config.SpeedRPM_Feedback = Motor.Pitch.Total_Angle_Speed_RPM;
        //***更新校准数据(设置目标角度与速度，PID计算，保存校准值)
        Motor_Compensation(&Pitch_Compensation_Config);
        //***输出电流
        Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN, Gimbal_Pitch_Send_CAN_ID, Pitch_Compensation_Config.Current_Output, 0, 0 ,0);		
        
        osDelay(1);
    }
    //***计算齿槽补偿与摩擦补偿
    for(uint16_t i = 0; i < Point_Num; i++) 
    {
        float Coggin_Torque = 0.5f * (Pitch_Compensation_Config.Data[i] + Pitch_Compensation_Config.Data[i+Point_Num]);
        float Friction_Torque = 0.5f * (Pitch_Compensation_Config.Data[i] - Pitch_Compensation_Config.Data[i+Point_Num]);
        Pitch_Compensation_Config.Data[i] = Coggin_Torque;
        Pitch_Compensation_Config.Data[i+Point_Num] = Friction_Torque;
    }
    
    //保存到W25Q64内
}

void Motor_Compensation(Motor_Compensation_Config_StructTypedef *Config)
{
    if(Config->Caled_State <= 1)
    {
        //串级PID定速正反转(0正传，1反转)
        if(Config->Caled_State == 0) Config->Angle_Target += 0.001f * Config->Compensation_Speed;
        else if(Config->Caled_State == 1) Config->Angle_Target -= 0.001f * Config->Compensation_Speed;
        
        if     (Config->Angle_Target >= Config->Compensation_Angle_UP   && Config->Caled_State == 0) {Config->Caled_State = 1;}
        else if(Config->Angle_Target <= Config->Compensation_Angle_Down && Config->Caled_State == 1) {Config->Caled_State = 0; Config->Compensation_Round--;}
        if(Config->Compensation_Round <= 0) {Config->Caled_State = 2; return;}
        
        PID_Position_Calculate(&Config->Angle_PID, Config->Angle_Target, Config->Angle_Feedback);
        PID_Position_Calculate(&Config->Speed_PID, Config->Angle_PID.Output, Config->SpeedRPM_Feedback);
        Config->Current_Output = Config->Speed_PID.Output;
        
        //计算角度编码器值
        int16_t Encoder = fmodf(Config->Angle_Feedback, 360.0f) * Point_Num / 360.0f;
        if(Encoder < 0) Encoder += Point_Num;
        
        //保存校准值(正转保存在数据数组前半部分，反转在后半部分)
        if(fabsf(Config->Angle_PID.Error) <= 5.0f)
        {
            if(Config->Caled_State == 0) Config->Data[Encoder]           = 0.4f*Config->Data[Encoder]             + 0.6f*Config->Current_Output;
            if(Config->Caled_State == 1) Config->Data[Encoder+Point_Num] = 0.4f*Config->Data[Encoder+Point_Num]   + 0.6f*Config->Current_Output;
        }
    }
}

float Motor_Compensation_Get_Data(float Angle, float Speed, Motor_Compensation_Config_StructTypedef *Config)
{
    float Dt = DWT_GetDeltaT(&(Config->DWT_Counter));
    if(Dt > 0.1f) Dt = 0;
    
    int16_t Encoder = fmodf(Angle, 360.0f) * Point_Num / 360.0f;
    if(Encoder < 0) Encoder += Point_Num;
    
    float Coggin_Torque = Config->Data[Encoder];
    float Friction_Torque_Target;
    if(Speed > 0)       Friction_Torque_Target = Config->Friction_K * Config->Data[Encoder+Point_Num];
    else if(Speed < 0)  Friction_Torque_Target = -Config->Friction_K * Config->Data[Encoder+Point_Num];
    
    Acc_and_Dec(Friction_Torque_Target, &Config->Friction_Out, Config->Friction_ACC, Config->Friction_DEC, Dt);
    
    return (Coggin_Torque + Config->Friction_Out);
}