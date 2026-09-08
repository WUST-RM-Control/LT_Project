#include "Function.h"

/*===| 限幅 |===*/
void Limit_float(float *Input, float Up, float Down)
{
    if      (*Input > Up)   *Input = Up;
    else if (*Input < Down) *Input = Down;
}
void Limit_int(int *Input, int Up, int Down)
{
    if      (*Input > Up)   *Input = Up;
    else if (*Input < Down) *Input = Down;
}

/*===| 忽略小误差 |===*/
void Ignor_float(float *Input,  float Up, float Down)
{
    if(*Input < Up && *Input > Down)
    *Input = 0;
}

/*===| 死区限制 |===*/
void Deadline_Float(float *Input,  float Up, float Down)
{
    if(*Input < Up && *Input > Down) *Input = 0;
}
void Deadline_Line_Float(float *Input,  float DeadLine)
{
    if(fabsf(*Input) < DeadLine) 
        *Input = 0;
    else
    {
        if(*Input > 0) *Input = (*Input - DeadLine) / (1.0f - DeadLine);
        else if(*Input < 0) *Input = (*Input + DeadLine) / (1.0f - DeadLine);
    }
}

/*===| 输出缓加减速 |===*/
void Acc_and_Dec(float Target, float *Output, float Acc, float Dec, float Dt)
{
    if      (Target - *Output > + Dt * Dec && *Output <= 0) *Output += Dt * Dec;
    else if (Target - *Output > + Dt * Acc && *Output > 0)  *Output += Dt * Acc;
    else if (Target - *Output < - Dt * Acc && *Output <= 0) *Output -= Dt * Acc;
    else if (Target - *Output < - Dt * Dec && *Output > 0)  *Output -= Dt * Dec; 
    else    *Output = Target;
}

/*===| 计算两点间距 |===*/
float Caculate_Distence(float X, float Y)
{
    return (sqrtf(X*X + Y*Y));
}

/*===| 计算两个任意角度的夹角，返回Target相对Now的角度差(-180~+180) |===*/
float Caculate_Included_Angle(float Now, float Target)
{
    float Err = fmodf(Target- Now, 360);
    if(Err > 180) Err -= 360;
    else if(Err < -180) Err += 360;
    return Err;
}
