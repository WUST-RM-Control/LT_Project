#ifndef __WHEELLEG_LQR_H__
#define __WHEELLEG_LQR_H__

#include "main.h"
#include "Chassis.h"

#define WHEELLEG_LQR_STATE_NUM       10U
#define WHEELLEG_LQR_CONTROL_NUM      4U
#define WHEELLEG_LQR_GAIN_BASIS_NUM   6U
#define WHEELLEG_LQR_LENGTH_MIN_M 0.12f
#define WHEELLEG_LQR_LENGTH_MAX_M 0.36f

// 顺序与原工程十状态LQR一致，角度rad，角速度rad/s。
typedef enum
{
    WHEELLEG_LQR_STATE_S = 0,
    WHEELLEG_LQR_STATE_S_DOT,
    WHEELLEG_LQR_STATE_YAW,
    WHEELLEG_LQR_STATE_YAW_DOT,
    WHEELLEG_LQR_STATE_LEFT_LEG,
    WHEELLEG_LQR_STATE_LEFT_LEG_DOT,
    WHEELLEG_LQR_STATE_RIGHT_LEG,
    WHEELLEG_LQR_STATE_RIGHT_LEG_DOT,
    WHEELLEG_LQR_STATE_BODY_PITCH,
    WHEELLEG_LQR_STATE_BODY_PITCH_DOT
} WheelLeg_LQR_StateIndex;

typedef enum
{
    WHEELLEG_LQR_OUTPUT_LEFT_WHEEL = 0,
    WHEELLEG_LQR_OUTPUT_RIGHT_WHEEL,
    WHEELLEG_LQR_OUTPUT_LEFT_LEG,
    WHEELLEG_LQR_OUTPUT_RIGHT_LEG
} WheelLeg_LQR_OutputIndex;

typedef struct
{
    float Wheel_Speed_Feedback; // m/s，前进为正
    float Displacement_Now;     // m，原始电机方向
    float Displacement_Init;    // m，保留当前位移基准变量
    float Displacement;         // m，前进为正
} WheelLeg_LQR_Displacement_Date;

typedef struct
{
    float State[WHEELLEG_LQR_STATE_NUM];
    float Target[WHEELLEG_LQR_STATE_NUM];
    float Error[WHEELLEG_LQR_STATE_NUM];
    float LQR_K[WHEELLEG_LQR_CONTROL_NUM][WHEELLEG_LQR_STATE_NUM];
    float LQR_Output[WHEELLEG_LQR_CONTROL_NUM]; // 原模型四路力矩，未转换电机方向

    float Wheel_Displacement_Feedback;
    float Wheel_Displacement_Target;
    float Wheel_Speed_Feedback;
    float Wheel_Speed_Target;
    float Wheel_displacement_add;
    float Wheel_displacement_init;
    float Wheel_Speed_Target_Last;

    uint8_t State_Valid;
    uint8_t Target_Valid;
    uint8_t Gain_Valid;
    uint8_t Valid;
    Chassis_State_EnumTypedef Chassis_State_Last;

    // 转向/防劈叉PID停用，保留变量名，不计算也不叠加。
    PID_Struct_TypeDef Wheel_Wz_Speed_PID;
    PID_Struct_TypeDef Leg_Splits_PID;
} WheelLeg_LQR_StructTypeDef;

extern WheelLeg_LQR_StructTypeDef WheelLeg_LQR_Struct;
extern WheelLeg_LQR_Displacement_Date Wheel_Left;
extern WheelLeg_LQR_Displacement_Date Wheel_Right;

void WheelLeg_LQR_Init(void);
void WheelLeg_LQR_Output(Chassis_Control_StructTypedef *Chassis_Control);
void State_Filter(Chassis_Control_StructTypedef *Chassis_Control_Struct, float INS_MotionAccel_n_Z);

#endif
