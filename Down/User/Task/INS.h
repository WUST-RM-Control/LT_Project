/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS__
#define __INS__

#include "main.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

/* ====| IMU 加热总开关 |====
 * DM_MC02 板上 IMU 加热由 TIM3_CH4 驱动, 抬的是 24V 电源, 误开有烧板风险,
 * 因此默认关闭(0): INS_Init() 不再启动 TIM3 的 PWM, IMU_Temperature_Ctrl()
 * 即使被调用也不会输出占空比。确认硬件无误、确实需要恒温时再改成 1。 */
#define INS_HEAT_ENABLE 1
//不想冒烟就别开超过1000
#define IMU_Heat_TIM_CRR_Lmit 600
// uint16_t IMU_Heat_TIM_CRR_Lmit=800;
/* ====| 四元数 EKF 总开关 |====
 * 1 = INS_Task 里用校准后的 acc/gyro 更新 EKF, 输出 Yaw/Pitch/Roll;
 * 0 = 只输出已校准的 acc/gyro 与 Z 轴角速度, 不做姿态解算。 */
#define INS_USE_EKF 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float Last_Yaw;
    float YawTotalAngle;
} INS_t;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

typedef struct
{
    uint8_t If_INS_Init;
    uint16_t INS_Data_Ticker;
   
    float Yaw;
    float Pitch;
    float Roll;
    float Yaw_Speed;
    float Pitch_Speed;
    int32_t Yaw_Round;
    float YawTotalAngle;
    float Last_Yaw;
    
    float Acc[3];
    float Gyro[3];
}INS_Data_StructTypedef;

extern INS_t INS;
extern INS_Data_StructTypedef INS_Data_External;
extern INS_Data_StructTypedef INS_Data_Internal;

void INS_Init(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

void INS_Get_Data_Call_Back(uint8_t *Data);

#endif
