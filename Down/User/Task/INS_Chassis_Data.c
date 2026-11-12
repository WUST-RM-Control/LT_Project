/**
 ******************************************************************************
 * @file    INS_Chassis_Data.c
 * @brief   底盘专用姿态数据(INS 与轮腿底盘之间的中间层)
 ******************************************************************************
 * @attention
 * 极性/轴映射的"单一改动点"就是这个文件, 为什么需要它见 INS_Chassis_Data.h 的说明。
 *
 * 默认值取自 Down 的 Robot_Application\Chassis.c:204-213(换板前轮腿底盘的用法):
 *   Pitch        = -INS.Pitch                       -> PITCH_SIGN   = -1
 *   Roll         =  INS.Roll                        -> ROLL_SIGN    = +1
 *   Yaw_Speed    =  INS.Gyro[Z]                     -> YAW_SPEED_SIGN = +1
 *   Acceleration_Y = -INS.MotionAccel_n[1]          -> ACC_Y_SIGN   = -1
 *   Acceleration_Z =  INS.MotionAccel_n[2]          -> ACC_Z_SIGN   = +1
 * DM_MC02 没有做 Down 那套 180° 轴变换, 所以上面这些符号很可能需要上车重新标定;
 * 标定方法: 把车头抬起 -> Pitch 应为正; 向右翻 -> Roll 应为正(若反了改 ROLL_SIGN);
 * 松手自由落体 -> Acceleration_Z 应约为 -9.8(若为 +9.8 改 ACC_Z_SIGN)。
 * 只改本文件的宏, 其它模块( Chassis.c / WheelLeg_*.c )不用动。
 ******************************************************************************
 */
#include "INS_Chassis_Data.h"
#include "INS.h"

INS_Chassis_Data_StructTypedef INS_Chassis_Data;

/**
 * @brief  极性/轴映射
 * @note   INS_Chassis_Data极性说明如下
 * @note     车头抬起            -> Pitch           应为正
 * @note     右腿侧抬起          -> Roll            应为正
 * @note     俯视逆时针转        -> Yaw/Yaw_Speed   应为正
 * @note     向前加速 / 向上加速 -> Acceleration_Y / Acceleration_Z 应为正
 * @note   哪个方向反了就把对应宏取负, 其它模块不用动。
 */
#define INS_CHASSIS_PITCH_SIGN        (-1.0f)   // 车头抬起为正
#define INS_CHASSIS_ROLL_SIGN         (+1.0f)   // 右侧抬起为正
#define INS_CHASSIS_YAW_SIGN          (+1.0f)   // 俯视逆时针为正
#define INS_CHASSIS_YAW_SPEED_SIGN    (+1.0f)   // 俯视逆时针为正
#define INS_CHASSIS_ACC_Y_SIGN        (-1.0f)   // 向前加速为正
#define INS_CHASSIS_ACC_Z_SIGN        (+1.0f)   // 向上加速为正

/**
 * @brief  装配误差角(IMU 安装面相对底盘坐标系的固定倾斜, 单位: 度)
 * @note   为什么要它:
 * @note     结构件加工/装配/焊盘偏差会让 IMU 的敏感轴和底盘的几何轴差零点几度到几度。
 * @note     表现就是"车明明放平了, Pitch/Roll 却读 +0.8°" —— 这个偏置会被 LQR 当成
 * @note     真实的倾角, 平衡点整体歪一点, 而且左右两腿的支点力也会差一点。
 * @note     这种固定倾斜在小角度下等效为常值偏置, 所以直接减掉即可(<5° 时二阶误差可忽略)。
 * 
 * @note   怎么标定(车静止, 底盘放在水平台上, 车头方向摆正):
 * @note     1. 读显示值 Pitch / Roll, 把读到的数原样填进下面两个宏(带正负号);
 * @note     2. 重烧后应读到 ≈0.0°;若反而变成两倍, 说明符号填反了, 取负即可;
 * @note     3. Yaw 没有绝对基准, 不对准也无所谓 —— 轮腿底盘不用绝对航向,
 * @note        只有 Yaw_Speed 参与运算, 所以 Yaw 偏置填 0 也可以。
 * 
 * @note   注意:
 * @note   这里只补偿"角度", 不动 Acceleration_Y/Z。1° 倾斜对水平加速度的影响
 * @note   只有 9.8*sin(1°)≈0.17 m/s², 远低于离地检测阈值(-5 m/s²), 可以不管;
 * @note   若倾斜超过 5°(装歪了/换了安装板), 应当先修结构件, 而不是靠这里硬补(很很压力机械)
 * @note   这三个宏是常值偏置, 不能用来补偿随姿态变化的有害量(如轴间串扰),
 * @note   那属于标定矩阵(3x3 安装矩阵)的范畴, 当前底盘用不到。
 */
#define INS_CHASSIS_MOUNT_ENABLE        (1)        // 1: 启用装配误差补偿, 0: 关闭
#define INS_CHASSIS_MOUNT_PITCH_DEG     (+3.3660f)    // 车放平时 Pitch 的固定读数(度)
#define INS_CHASSIS_MOUNT_ROLL_DEG      (+0.0f)    // 车放平时 Roll  的固定读数(度)
// #define INS_CHASSIS_MOUNT_YAW_DEG       (+0.0f)    // 车头对准基准时 Yaw 的固定读数(度)

/**
 * @brief  把角度折到 (-180, 180], 避免减掉偏置后越界
 * @param  Angle 待处理的角度(度)
 * @retval 折叠后的角度(度)
 */
static float INS_Chassis_Wrap_Angle(float Angle)
{
    while (Angle > 180.0f)
    {
        Angle -= 360.0f;
    }
    while (Angle <= -180.0f)
    {
        Angle += 360.0f;
    }
    return Angle;
}

/**
 * @brief  用最新一帧 INS 数据刷新底盘专用姿态数据
 * @note   由 Chassis_Task 每周期调用一次(在 Get_Control_Data 之前/之内),
 *         与 INS_Task 之间是"单写者-单读者"的 float 拷贝, 不需要加锁。
 * @note   写入 INS_Chassis_Data 的字段顺序与极性由上面的宏决定, 装配误差在最后统一扣除。
 * @param  none 无(数据源是全局的 INS 结构体)
 * @retval 无
 */
void INS_Chassis_Data_Update(void)
{
    INS_Chassis_Data.Pitch          = INS_CHASSIS_PITCH_SIGN     * INS.Pitch;
    INS_Chassis_Data.Roll           = INS_CHASSIS_ROLL_SIGN      * INS.Roll;
    INS_Chassis_Data.Yaw            = INS_CHASSIS_YAW_SIGN       * INS.Yaw;
    INS_Chassis_Data.Yaw_Speed      = INS_CHASSIS_YAW_SPEED_SIGN * INS.Gyro[Z];
    INS_Chassis_Data.Acceleration_Y = INS_CHASSIS_ACC_Y_SIGN     * INS.MotionAccel_n[1];
    INS_Chassis_Data.Acceleration_Z = INS_CHASSIS_ACC_Z_SIGN     * INS.MotionAccel_n[2];

#if (INS_CHASSIS_MOUNT_ENABLE)
    // 装配误差补偿: 减掉"放平时本来就有的读数", 让水平姿态真的输出 0
    INS_Chassis_Data.Pitch = INS_Chassis_Data.Pitch - INS_CHASSIS_MOUNT_PITCH_DEG;
    INS_Chassis_Data.Roll  = INS_Chassis_Data.Roll  - INS_CHASSIS_MOUNT_ROLL_DEG;
    INS_Chassis_Data.Yaw   = INS_Chassis_Wrap_Angle(INS_Chassis_Data.Yaw);
    // INS_Chassis_Data.Yaw   = INS_Chassis_Wrap_Angle(INS_Chassis_Data.Yaw - INS_CHASSIS_MOUNT_YAW_DEG);
#endif

    INS_Chassis_Data.Count++;
    // 收到过数据即认为姿态可用:DM_MC02 的 INS 是板载 BMI088+EKF,不存在 Down 那种"外部 IMU 掉线"状态,
    // 这里用"是否已经跑过更新"代替 Down 的 INS_Data_Gimbal.If_INS_Init
    INS_Chassis_Data.If_Ready = 1;
}
