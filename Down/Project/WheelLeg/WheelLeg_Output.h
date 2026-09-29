#ifndef __WHEELLEG_OUTPUT_H__
#define __WHEELLEG_OUTPUT_H__

#include "main.h"


// 单条腿最终关节输出 
typedef struct
{
    // VMC数学坐标下的力矩 
    float tau_phi1_nm;          // Back
    float tau_phi4_nm;          // Front

    // 映射到真实电机坐标，并经过最终限幅 的目标力矩
    float back_target_nm;
    float front_target_nm;

    //准备发送的力矩
    float back_command_nm;
    float front_command_nm;

    //执行器状态
    uint8_t enable_request;     // 是否希望这条腿的关节电机 Enable
    uint8_t enable_sent;        // 是否执行过 Enable 命令
    uint8_t ready;              // 两个关节是否已经具备输出条件
    uint8_t fault;              // 关节异常

    uint8_t valid;

} WheelLeg_Output_Leg;


// 整个关节输出
typedef struct
{
    WheelLeg_Output_Leg left;
    WheelLeg_Output_Leg right;

    // 关节最终力矩限制 
    float torque_limit_nm;

} WheelLeg_Output;


extern WheelLeg_Output wheelLeg_output;


void WheelLeg_Output_Init(void);

/*
 * 目前只计算最终目标力矩。
 * 下一步再真正发送MIT。
 */
void WheelLeg_Output_UpdateTarget(void);

void WheelLeg_Output_SetLegEnable(uint8_t left_enable,uint8_t right_enable);

//更新使能输出
void WheelLeg_Output_UpdateEnable(void);

// target经过slew生成真正准备发送的command
void WheelLeg_Output_UpdateCommand(float dt);

// 将command真正发送给达妙
void WheelLeg_Output_Send(void);


#endif