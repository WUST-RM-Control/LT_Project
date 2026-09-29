#ifndef __WHEELLEG_LQR_H__
#define __WHEELLEG_LQR_H__

#include "main.h"


// 上交 WBR 模型：10维状态，4维控制输入
#define WHEELLEG_LQR_STATE_NUM    10U
#define WHEELLEG_LQR_CONTROL_NUM   4U

// K(ll, lr)使用6项二元二次多项式
#define WHEELLEG_LQR_GAIN_BASIS_NUM    6U

// 真实K拟合系数尚未生成  后续填入本机模型得到的系数后再改为1
#define WHEELLEG_LQR_GAIN_COEFF_READY  1U

typedef enum
{
    WHEELLEG_LQR_STATE_S = 0,          // x0：前后位移 s 
    WHEELLEG_LQR_STATE_S_DOT,          // x1：前后速度 s_dot 

    WHEELLEG_LQR_STATE_YAW,            // x2：偏航角 phi 
    WHEELLEG_LQR_STATE_YAW_DOT,        // x3：偏航角速度 phi_dot 

    WHEELLEG_LQR_STATE_LEFT_LEG,       // x4：左腿绝对倾角 
    WHEELLEG_LQR_STATE_LEFT_LEG_DOT,   // x5：左腿绝对角速度 

    WHEELLEG_LQR_STATE_RIGHT_LEG,      // x6：右腿绝对倾角 
    WHEELLEG_LQR_STATE_RIGHT_LEG_DOT,  // x7：右腿绝对角速度 

    WHEELLEG_LQR_STATE_BODY_PITCH,     // x8：机体倾角 theta_b 
    WHEELLEG_LQR_STATE_BODY_PITCH_DOT  // x9：机体角速度 
} WheelLeg_LQR_StateIndex;

typedef enum
{
    WHEELLEG_LQR_OUTPUT_LEFT_WHEEL = 0,  // u0：左轮转矩 T_lw,l
    WHEELLEG_LQR_OUTPUT_RIGHT_WHEEL,     // u1：右轮转矩 T_lw,r

    WHEELLEG_LQR_OUTPUT_LEFT_LEG,        // u2：左腿转矩 T_bl,l
    WHEELLEG_LQR_OUTPUT_RIGHT_LEG        // u3：右腿转矩 T_bl,r
} WheelLeg_LQR_OutputIndex;



typedef struct
{
    // 实际状态 x
    float state[WHEELLEG_LQR_STATE_NUM];

    // 目标状态 xd
    float target[WHEELLEG_LQR_STATE_NUM];

    // 状态误差 xd - x
    float error[WHEELLEG_LQR_STATE_NUM];

    // 状态反馈矩阵 K
    float gain[WHEELLEG_LQR_CONTROL_NUM]
              [WHEELLEG_LQR_STATE_NUM];

    // 控制输出 u
    float output[WHEELLEG_LQR_CONTROL_NUM];

    // 当前左右腿长
    // 后续用于计算 K(ll, lr)
    float left_leg_length_m;
    float right_leg_length_m;

    // 10维状态是否有效
    uint8_t state_valid;

    // 目标状态是否已经初始化
    uint8_t target_valid;

    // 当前状态反馈矩阵K是否有效
    uint8_t gain_valid;

    // LQR状态是否有效
    uint8_t valid;

} WheelLeg_LQR;

extern WheelLeg_LQR wheelLeg_lqr;

void WheelLeg_LQR_Init(void);

void WheelLeg_LQR_Update(float dt);

void WheelLeg_LQR_CaptureTarget(void);

void WheelLeg_LQR_ClearTarget(void);

#endif