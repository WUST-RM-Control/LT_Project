#ifndef __WHEELLEG_VMC_H__
#define __WHEELLEG_VMC_H__

#include "main.h"

//VMC状态量
typedef struct
{
    // 虚拟腿输入
    float force_n;
    float torque_nm;

    //数学关节输出 
    float tau_phi1_nm;   // Back
    float tau_phi4_nm;   // Front

    // 实际电机对应输出 
    float back_motor_torque_nm;
    float front_motor_torque_nm;

    uint8_t valid;

} WheelLeg_VMC_Leg;

//双腿VMC
typedef struct
{
    WheelLeg_VMC_Leg left;
    WheelLeg_VMC_Leg right;

} WheelLeg_VMC;

extern WheelLeg_VMC wheelLeg_vmc;

void WheelLeg_VMC_SetLeft(float force_n,float torque_nm);

void WheelLeg_VMC_SetRight(float force_n,float torque_nm);

void WheelLeg_VMC_Update(void);



#endif

