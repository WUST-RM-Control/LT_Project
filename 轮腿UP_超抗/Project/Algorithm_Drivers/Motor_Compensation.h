#ifndef __Motor_Compensation__
#define __Motor_Compensation__

#include "main.h"

typedef struct
{
    uint8_t Caled_State;
    
    float Compensation_Speed;
    int8_t Compensation_Round;
    float Compensation_Angle_UP;
    float Compensation_Angle_Down;
    
    float Angle_Target;
    float Angle_Feedback; 
    float SpeedRPM_Feedback; 
    float Current_Output;
    
    PID_Struct_TypeDef Angle_PID;
    PID_Struct_TypeDef Speed_PID;
    
    float* Data;
    
    uint32_t DWT_Counter;
    float Friction_K;
    float Friction_Out;
    float Friction_ACC;
    float Friction_DEC;
} Motor_Compensation_Config_StructTypedef;

extern Motor_Compensation_Config_StructTypedef Yaw_Compensation_Config;
extern Motor_Compensation_Config_StructTypedef Pitch_Compensation_Config;

void Motor_Compensation_Task(void);
void Motor_Compensation(Motor_Compensation_Config_StructTypedef *Config);

float Motor_Compensation_Get_Data(float Angle, float Speed, Motor_Compensation_Config_StructTypedef *Config);

#endif