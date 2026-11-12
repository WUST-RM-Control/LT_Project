/**
 ******************************************************************************
 * @file    INS_Chassis_Data.h
 * @brief   底盘专用姿态数据(INS 与轮腿底盘之间的中间层)
 ******************************************************************************
 * @attention
 * 1. 更换下位机后 IMU 的安装方向与 Down(G474) 不同:
 *    Down 的 INS.c 对原始 acc/gyro 做了 180° 轴变换(INS.c:121-131, xb'=-yb,
 *    yb'=-xb, zb'=-zb), DM_MC02 的 INS.c 不做变换(原始机体轴直接进 EKF)。
 *    因此两块板子上 INS.Pitch / INS.Roll / INS.Gyro[] 的符号、甚至轴的含义
 *    可能不一致。
 *
 * 2. 按要求不改动 INS 模块, 所以这里把"底盘真正需要的量"单独抽出来:
 *    轮腿底盘只读 INS_Chassis_Data, 而极性/轴映射全部集中在
 *    INS_Chassis_Data.c 的 INS_CHASSIS_*_SIGN 宏里, 改一个地方全部生效。
 *
 * 3. 下面每个成员后面的注释就是"底盘期望的物理含义", 默认符号按 Down 的
 *    用法填(见 INS_Chassis_Data.c)。上车实测发现哪个量反了, 就把对应的
 *    *_SIGN 取反; 若发现是轴交换(例如 Pitch 其实读到了 Roll 的量), 改
 *    INS_Chassis_Data.c 里那一行的源变量, 并在该行注释里写清实测结论。
 ******************************************************************************
 */
#ifndef __INS_CHASSIS_DATA_H__
#define __INS_CHASSIS_DATA_H__

#include "main.h"

typedef struct
{
    float Pitch;            // (°)     车身俯仰角, 车头(前)抬起为正
    float Roll;             // (°)     车身横滚角, 右侧抬起为正
    float Yaw;              // (°)     航向角, 俯视逆时针为正, -180~180
    float Yaw_Speed;        // (rad/s) 航向角速度, 俯视逆时针为正
    float Acceleration_Y;   // (m/s²)  机体系横向加速度, 向前加速为正
    float Acceleration_Z;   // (m/s²)  机体系竖直加速度, 向上为正
    uint32_t Count;         // 更新次数, 仅调试用
    uint8_t If_Ready;       // 1:已经收到过有效姿态数据(替代 Down 的 INS_Data_Gimbal.If_INS_Init)
} INS_Chassis_Data_StructTypedef;

extern INS_Chassis_Data_StructTypedef INS_Chassis_Data;

void INS_Chassis_Data_Update(void);

#endif
