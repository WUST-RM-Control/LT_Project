#include "PID.h"
#include "DWT.h"

/*===| PID参数初始化 |===*/
void PID_Init(PID_Struct_TypeDef *PID_Struct, float Kp, float Ki, float Kd, float Kf, float I_Output_Max, float Output_Max)
{
    PID_Struct->Kp = Kp;
    PID_Struct->Ki = Ki;
    PID_Struct->Kd = Kd;
    PID_Struct->Kf = Kf;
    PID_Struct->I_Output_Max = I_Output_Max;
    PID_Struct->Output_Max = Output_Max;
}

/*===| PID位置式计算 |===*/
void PID_Position_Calculate(PID_Struct_TypeDef *PID_Struct, float Target, float Feedback)
{ 
    /*===| 计算时间差，并忽略间隔时间过长(>0.1s)的Dt |===*/
    float Dt = DWT_GetDeltaT(&PID_Struct->DWT_Count);
    if(Dt > 0.1f) Dt = 0.001;
    else if(Dt <= 0.0f) Dt = 0.001;

    /*===| 赋值 |===*/
    PID_Struct->Target = Target;
    PID_Struct->Feedback = Feedback;
    
    /*===| 计算误差 |===*/
    PID_Struct->Error = PID_Struct->Target - PID_Struct->Feedback;
    
    /*===| 计算各部分输出 |===*/
    PID_Struct->K_Output =  PID_Struct->Kp * PID_Struct->Error;
    PID_Struct->I_Output += PID_Struct->Ki * PID_Struct->Error * Dt;                       //I_Output每秒累积Ki*Error
    PID_Struct->D_Output =  PID_Struct->Kd * (PID_Struct->Error - PID_Struct->Error_Last);
    PID_Struct->F_Output =  PID_Struct->Kf * PID_Struct->Target;
    
    /*===| 积分输出限幅 |===*/
    if      (PID_Struct->I_Output >=  PID_Struct->I_Output_Max)        PID_Struct->I_Output =  PID_Struct->I_Output_Max;
    else if (PID_Struct->I_Output <= -PID_Struct->I_Output_Max)        PID_Struct->I_Output = -PID_Struct->I_Output_Max;

    /*===| 计算总输出 |===*/
    PID_Struct->Output = PID_Struct->K_Output + PID_Struct->I_Output + PID_Struct->D_Output + PID_Struct->F_Output;
    
    /*===| 输出限幅 |===*/
    if      (PID_Struct->Output >=  PID_Struct->Output_Max) PID_Struct->Output =  PID_Struct->Output_Max;
    else if (PID_Struct->Output <= -PID_Struct->Output_Max) PID_Struct->Output = -PID_Struct->Output_Max;
        
    /*===| 记录误差和输出 |===*/
        PID_Struct->Error_Last = PID_Struct->Error;
}

/*===| PID清空积分与输出 |===*/
void PID_Clear_Output(PID_Struct_TypeDef *PID_Struct)
{
    PID_Struct->I_Output_Max = 0;
    PID_Struct->Output_Max = 0;
}

