#ifndef __Function__
#define __Function__

#include "main.h"

/*===| 限幅 |===*/
void Limit_float(float *Input, float Up, float Down);
void Limit_int(int *Input, int Up, int Down);
/*===| 忽略小误差 |===*/
void Ignor_float(float *Input,  float Up, float Down);
/*===| 死区限制 |===*/
void Deadline_Float(float *Input,  float Up, float Down);
/*===| 输出缓加减速 |===*/
void Acc_and_Dec(float Target, float *Output, float Acc, float Dec, float Dt);
/*===| 计算两点间距 |===*/
float Caculate_Distence(float X, float Y);
/*===| 计算两个任意角度的夹角，返回Target相对Now的角度差(-180~+180) |===*/
float Caculate_Included_Angle(float Now, float Target);

#endif
