//======================================================================
//                      INS[姿态解算]
//      依靠云台C板的BMI088得到云台的欧拉角和三轴角速度
//
//      姿态数据: IMU_Data
//======================================================================

/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "INS.h"
#include "QuaternionEKF.h"
#include "PID.h"
#include "tim.h"
#include "BMI088driver.h"
#include "IMU_Cal.h"

/*
 * 2026/11 改动说明
 *  1) 传感器数据统一走 IMU_Cal 校准链路：
 *     IMU_Cal_Get_Calibrated_Data() = (raw-零偏).*比例 后，再乘 IMU_Cal 拟合出的
 *     芯片->模块旋转矩阵，正面朝上静止时加速度恒为 (0,0,+g)，与下面 gravity[]
 *     的方向约定一致。校准动作本身(IMU_Cal())已在 main.c 初始化段一次性完成
 *     （阻塞，需要人工翻转模块），这里只负责取数，不再调用。
 *  2) IMU 加热全部禁用：DM_MC02 的 IMU 加热由 TIM3_CH4 驱动、抬的是 24V，
 *     误开有烧板风险。INS_Init() 里启动 PWM 的三行已被 INS_HEAT_ENABLE 包住
 *     （默认 0），INS_Task 里也不再周期调用 IMU_Temperature_Ctrl()。
 *  3) INS_Task 内恢复四元数 EKF（算法部分原本被整段注释掉），
 *     可用 INS.h 里的 INS_USE_EKF 宏关闭。
 */


INS_Data_StructTypedef INS_Data_External = {0};
INS_Data_StructTypedef INS_Data_Internal = {0};

INS_t INS;
IMU_Param_t IMU_Param;

PID_Struct_TypeDef TempCtrl_PID_Struct;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 35;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);


void INS_Init(void)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;
	
    //不开(非正置姿态会因为实机状态与初始状态不符，数据飘好久)
    // IMU_QuaternionEKF_Init(QEKF_PROCESS_NOISE_Q1, QEKF_PROCESS_NOISE_Q2, QEKF_MEASURE_NOISE_R,
//                        QEKF_FADING_LAMBDA, QEKF_ACCEL_LPF_TAU);
// 说明: 现在改由 IMU_QuaternionEKF_Update() 首次调用时自动初始化(见 QuaternionEKF.c),
//       参数统一在 QuaternionEKF.c 顶部的 QEKF_* 宏里改, 这里保留仅作说明。

    /* imu heat init：默认关闭(INS.h 里的 INS_HEAT_ENABLE = 0)。
     * 这里启动的是 TIM3_CH4 的 PWM 输出，直接控制 24V 加热回路，
     * 没有确认硬件前不要打开，否则可能烧板。 */
#if (INS_HEAT_ENABLE)
    //不想冒烟，参数就别给大
    PID_Init(&TempCtrl_PID_Struct, 80, 40, 10, 0, 150, IMU_Heat_TIM_CRR_Lmit);
    HAL_TIM_Base_Start(IMU_Heat_TIM);
    HAL_TIM_PWM_Start(IMU_Heat_TIM, IMU_Heat_TIM_CHANNAL);
#endif

    INS.AccelLPF = 0.0085;
}


void INS_Get_Data_Call_Back(uint8_t *Data)
{
    INS_Data_External.If_INS_Init = 1;
    INS_Data_External.INS_Data_Ticker = 0;
    
    INS_Data_External.Pitch = ((int16_t)(Data[0] << 8) | (int16_t)Data[1]) * 180.0f / 32768.0f;
    INS_Data_External.Roll = ((int16_t)(Data[2] << 8) | (int16_t)Data[3]) * 180.0f / 32768.0f;
    INS_Data_External.Yaw = ((int16_t)(Data[4] << 8) | (int16_t)Data[5]) * 180.0f / 32768.0f;
    INS_Data_External.Yaw_Speed = ((int16_t)(Data[6] << 8) | (int16_t)Data[7]) * 720.0f / 32768.0f;
    
    if(INS_Data_External.Yaw - INS_Data_External.Last_Yaw > 180.0f) INS_Data_External.Yaw_Round--;
    if(INS_Data_External.Yaw - INS_Data_External.Last_Yaw < -180.0f) INS_Data_External.Yaw_Round++;
    
    INS_Data_External.YawTotalAngle = INS_Data_External.Yaw_Round * 360.0f + INS_Data_External.Yaw;
    INS_Data_External.Last_Yaw = INS_Data_External.Yaw;
}

void C_IMU_Reset(void)
{
    CAN_Send_Data_STD(&IMU_CAN, 0x44, NULL);
    MX_FDCAN2_Init();
    CAN_Init();
}


void INS_Task(void)
{
    
    INS_Init();

    for(;;)
    {
        INS_Data_External.INS_Data_Ticker++;
       
        if(INS_Data_External.INS_Data_Ticker >= 100) INS_Data_External.If_INS_Init = 0;
        if(INS_Data_External.INS_Data_Ticker >= 200) INS_Data_External.INS_Data_Ticker = 100;
       
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        t += dt;

        /* imu update：
         * 取一次校准后的 acc(m/s^2)/gyro(rad/s)。该函数内部已经调用了
         * BMI088_Read()，所以这里不要再单独 Read 一遍（会多占一次 SPI 时间，
         * 而且两份数据不同步）。未完成校准时它直接透传原始值，不会崩。 */
        IMU_Cal_Get_Calibrated_Data(INS.Accel, INS.Gyro);
        
        // // 拉
        // BMI088_Read(&BMI088);
        // INS.Accel[0] = BMI088.Accel[X];
        // INS.Accel[1] = BMI088.Accel[Y];
        // INS.Accel[2] = BMI088.Accel[Z];
        // INS.Gyro[0] = BMI088.Gyro[X];
        // INS.Gyro[1] = BMI088.Gyro[Y];
        // INS.Gyro[2] = BMI088.Gyro[Z];

#if (INS_USE_EKF)
        {
            const float gravity[3] = {0, 0, 9.81f};

            /* 核心函数, EKF 更新四元数 */
            IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z],
                                     INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

            memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            IMU_Temperature_Ctrl();
            /* 机体系基向量转换到导航坐标系，本例选取惯性系为导航系 */
            BodyFrameToEarthFrame(xb, INS.xn, INS.q);
            BodyFrameToEarthFrame(yb, INS.yn, INS.q);
            BodyFrameToEarthFrame(zb, INS.zn, INS.q);

            /* 将重力从导航坐标系n转换到机体系b, 随后根据加速度计数据计算运动加速度 */
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
            for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
            {
                INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt)
                                     + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
            }
            BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

            /* 获取最终数据 */
            INS.Yaw = QEKF_INS.Yaw;
            INS.Pitch = QEKF_INS.Pitch;
            INS.Roll = QEKF_INS.Roll;
            INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

            if(INS.Yaw - INS.Last_Yaw > 180.0f) INS_Data_Internal.Yaw_Round--;
            if(INS.Yaw - INS.Last_Yaw < -180.0f) INS_Data_Internal.Yaw_Round++;

            memcpy(INS_Data_Internal.Acc, INS.Accel, 12);
            memcpy(INS_Data_Internal.Gyro, INS.Gyro, 12);

            /* 注意：这里沿用原 demo 的写法（Roll/Pitch 交叉赋值），
             * 上层若有按此约定取值的地方请保持不变 */
            INS_Data_Internal.Yaw = INS.Yaw;
            INS_Data_Internal.Roll = INS.Pitch;
            INS_Data_Internal.Pitch = INS.Roll;
            INS_Data_Internal.Yaw_Speed = INS.Gyro[Z] * Rad2Angle;
            INS_Data_Internal.YawTotalAngle = INS_Data_Internal.Yaw_Round * 360.0f + INS.Yaw;

            INS.Last_Yaw = INS.Yaw;
        }
        // VOFA_JustFloat_AddValue(0, INS.Pitch);
        // VOFA_JustFloat_AddValue(1, INS.Roll);
        // VOFA_JustFloat_AddValue(2, INS.Yaw);
        // VOFA_JustFloat_AddValue(3, INS.Gyro[0]);
        // VOFA_JustFloat_AddValue(4, INS.Gyro[1]);
        // VOFA_JustFloat_AddValue(5, INS.Gyro[2]);
        // VOFA_JustFloat_AddValue(6, INS.Accel[0]);
        // VOFA_JustFloat_AddValue(7, INS.Accel[1]);
        // VOFA_JustFloat_AddValue(8, INS.Accel[2]);
        // VOFA_JustFloat_AddValue(9, BMI088.Temperature);
        // VOFA_JustFloat_AddValue(10, TempCtrl_PID_Struct.K_Output);
        // VOFA_JustFloat_AddValue(11, TempCtrl_PID_Struct.D_Output);
        // VOFA_JustFloat_AddValue(12, TempCtrl_PID_Struct.I_Output);
        // VOFA_JustFloat_AddValue(13, TempCtrl_PID_Struct.Output);
        // VOFA_JustFloat_AddValue(0, INS.Roll);
        // VOFA_JustFloat_AddValue(1, INS.Pitch);
        // VOFA_JustFloat_AddValue(2, INS.Yaw);
        // VOFA_JustFloat_AddValue(3, INS.MotionAccel_n[0]);
        // VOFA_JustFloat_AddValue(4, INS.MotionAccel_n[1]);
        // VOFA_JustFloat_AddValue(5, INS.MotionAccel_n[2]);
#else
        /* 不做姿态解算：只把校准后的数据与 Z 轴角速度递出去 */
        memcpy(INS_Data_Internal.Acc, INS.Accel, 12);
        memcpy(INS_Data_Internal.Gyro, INS.Gyro, 12);
        INS_Data_Internal.Yaw_Speed = INS.Gyro[Z] * Rad2Angle;
#endif
        
        osDelay(INS_TASK_PERIOD);
    }
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 * @attention !!! 危险 !!!
 *        本函数通过 TIM3_CH4 直接驱动 24V 加热回路。DM_MC02 上该回路没有
 *        做过验证，占空比失控/传感器异常都可能把板子烧坏，因此默认整段
 *        编译掉（INS.h 的 INS_HEAT_ENABLE = 0）。要启用必须先确认硬件和PID参数
 * @note  即使有人误调用了它，INS_HEAT_ENABLE = 0 时它也是一个空函数。
 */
void IMU_Temperature_Ctrl(void)
{
#if (INS_HEAT_ENABLE)
    PID_Position_Calculate(&TempCtrl_PID_Struct, RefTemp, BMI088.Temperature);
    Limit_float(&TempCtrl_PID_Struct.Output, IMU_Heat_TIM_CRR_Lmit, 0);
    __HAL_TIM_SetCompare(IMU_Heat_TIM, IMU_Heat_TIM_CHANNAL, (uint16_t)TempCtrl_PID_Struct.Output);
#else
    /* 加热已禁用：什么都不做，绝对不去碰 TIM3_CH4 的比较值 */
    (void)TempCtrl_PID_Struct;
    (void)RefTemp;
#endif
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
