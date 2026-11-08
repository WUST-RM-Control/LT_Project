#include "Motor_Driver.h"
#include "DWT.h"

/*===| 电机数据结构体 |===*/
Motor_StructTypedef Motor = {0};

void Motor_Get_TotalAngle_Speed(Motor_Data_StructTypeDef *Motor_Data_Struct, float K)
{
    float Dt = DWT_GetDeltaT(&Motor_Data_Struct->Total_Angle_DWT_Count);
    Motor_Data_Struct->Total_Angle_Speed_RPM = (Motor_Data_Struct->Total_Angle - Motor_Data_Struct->Total_Angle_Last) / Dt * 60.0f / 360.0f;
    Motor_Data_Struct->Total_Angle_Speed_RPM = K*Motor_Data_Struct->Total_Angle_Speed_RPM + (1-K) * Motor_Data_Struct->Total_Angle_Speed_RPM_Last;
    Motor_Data_Struct->Total_Angle_Speed_RPM_Last = Motor_Data_Struct->Total_Angle_Speed_RPM;
    
    Motor_Data_Struct->Total_Angle_Last = Motor_Data_Struct->Total_Angle;
}

void Motor_Set_Zero_Offset(Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    Motor_Data_Struct->Total_Angle_Offset += Motor_Data_Struct->Total_Angle;
    Motor_Data_Struct->Total_Angle = 0;
}
