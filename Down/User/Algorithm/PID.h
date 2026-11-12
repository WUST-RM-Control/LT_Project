#ifndef __PID__
#define __PID__

#include "main.h"


/*===| PID结构体定义 |===*/
typedef struct
{
    uint32_t DWT_Count;
    
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    
    float Target;
    float Feedback;
    
    float Error;
    float Error_Last;      
    
    float K_Output;
    float I_Output;
    float D_Output;
    float F_Output;
    
    float Output;
    
    float I_Output_Max;
    float Output_Max;
} PID_Struct_TypeDef;

/*===| PID参数初始化 |===*/
void PID_Init(PID_Struct_TypeDef *PID_Struct, float Kp, float Ki, float Kd, float Kf, float I_Output_Max, float Output_Max);
/*===| PID位置式计算 |===*/
void PID_Position_Calculate(PID_Struct_TypeDef *PID_Struct, float Target, float Feedback);
/*===| PID清空积分与输出 |===*/
void PID_Clear_Output(PID_Struct_TypeDef *PID_Struct);



#endif
