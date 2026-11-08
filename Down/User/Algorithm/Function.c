//=====================================================================================
//                    Function[常用函数]
//    Limit(float *Input, float Up, float Down)
//    Ignor(float *Input,  float Up, float Down)
//    Deadline(float *Input,  float Up, float Down)
//=====================================================================================

#include "Function.h"

/**
 * @brief 限幅
 *
 * @param 输入值指针
 * @param 限制范围
 */
void Limit_double(double *Input, double Up, double Down)
{
    if      (*Input > Up)   *Input = Up;
    else if (*Input < Down) *Input = Down;
}
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

/**
 * @brief 误差忽略
 *
 * @param 输入值指针
 * @param 忽略上边缘
 * @param 忽略下边缘
 */
void Ignor_float(float *Input,  float Up, float Down)
{
    if(*Input < Up && *Input > Down)
    *Input = 0;
}

/**
 * @brief 死区限制
 *
 * @param 输入值指针
 * @param 死区上边缘
 * @param 死区下边缘
 */
void Deadline_Float(float *Input,  float Up, float Down)
{
    if(Up < Down) return;
    if(*Input < Up && *Input > Down) *Input = 0;
}

/**
 * @brief 死区外从零开始线性映射
 *
 * @param 输入值指针
 * @param 死区边缘
 */
void Deadline_Zero_Float(float *Input,  float DeadLine)
{
    if(fabs(*Input) < DeadLine) *Input = 0;
    else
    {
        if(*Input > 0) *Input = (*Input - DeadLine) / (1.0f - DeadLine);
        else if(*Input < 0) *Input = (*Input + DeadLine) / (1.0f - DeadLine);
    }
}

void Limit180(float *data)
{
	if(*data>180.0f)  *data -= 360.0f;
	if(*data<-180.0f) *data += 360.0f;
}

void Zero_Crossing(float *Target_Angle,float *Now_Angle)
{
	/*===| 180°~-180° |===*/
	if(*Now_Angle > 180.0f) *Now_Angle -= 360.0f;
	else if(*Now_Angle < -180.0f) *Now_Angle += 360.0f;
	
	/*===| 优劣弧 |===*/
	if(*Target_Angle - *Now_Angle > 180.0f) *Now_Angle += 360.0f;
	else if(*Target_Angle - *Now_Angle < -180.0f) *Now_Angle -= 360.0f;
}

float Avarage_Fliter(float *Array,uint8_t Length)
{
	float sum = 0;
	for(uint8_t i = 0;i < Length;i++)
	{
		sum += Array[i];
	}
	float avarage;
	avarage = sum / (float)Length;
	return avarage;
}

void Update_Data(float *Array,uint8_t Length,float New_Data)
{
	for(uint8_t i = 0;i<Length-1;i++)
	{
		Array[Length-i-1] = Array[Length-i-2];
	}
	Array[0] = New_Data;
}
/**
 * @brief 输出缓加速，减速
 *
 * @param Target 目标
 * @param Output 输出地址
 * @param Acc 加速度
 * @param Dec 减速度
 * @param Dt 时间差
 */
void Acc_and_Dec_double(double Target, double *Output, double Acc, double Dec, double Dt)
{
	double err = Target - *Output;
    if      (err > + Dt * Dec && *Output <= 0) *Output += Dt * Dec;
    else if (err > + Dt * Acc && *Output > 0)  *Output += Dt * Acc;
    else if (err < - Dt * Acc && *Output <= 0) *Output -= Dt * Acc;
    else if (err < - Dt * Dec && *Output > 0)  *Output -= Dt * Dec; 
    else    *Output = Target;
}

/**
 * @brief 输出缓加速，减速
 *
 * @param Target 目标
 * @param Output 输出地址
 * @param Acc 加速度
 * @param Dec 减速度
 * @param Dt 时间差
 */
void Acc_and_Dec(float Target, float *Output, float Acc, float Dec, float Dt)
{
	float err = Target - *Output;
    if      (err > + Dt * Dec && *Output <= 0) *Output += Dt * Dec;
    else if (err > + Dt * Acc && *Output > 0)  *Output += Dt * Acc;
    else if (err < - Dt * Acc && *Output <= 0) *Output -= Dt * Acc;
    else if (err < - Dt * Dec && *Output > 0)  *Output -= Dt * Dec; 
    else    *Output = Target;
}


float Caculate_Distence(float X, float Y)
{
    return (sqrtf(X*X + Y*Y));
}

//返回1有方向旋转到2的角度(-180~+180)，逆时针为正
double Caculate_Included_Angle_double(double Angle1, double Angle2)
{
	double Err = fmodf(Angle2 - Angle1, 360);
	if(Err > 180) Err -= 360;
	if(Err < -180) Err += 360;
	return Err;
}

//返回1有方向旋转到2的角度(-180~+180)，逆时针为正
float Caculate_Included_Angle(float Angle1, float Angle2)
{
	float Err = fmodf(Angle2 - Angle1, 360);
	if(Err > 180) Err -= 360;
	if(Err < -180) Err += 360;
	return Err;
}

