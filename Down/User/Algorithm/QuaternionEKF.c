/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 */
#include "QuaternionEKF.h"

QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/* ===| EKF 参数(调参统一改这里) |===
 * @note 快速绕 Roll 转动时 Roll 出偏置的两个根因, 对应下面两个 P0/P2 改动:
 * @note  P0 加速度计低通: 快速转动时 IMU 会吃到离心力(w^2*r)与切向加速度(a*r),
 * @note     真实重力方向被污染。原来 LPF = 0(完全不过滤), 错误方向被直接当成观测值。
 * @note     取 0.01s(约 16Hz): 保留重力直流分量, 压掉转动/振动引起的交流分量。
 * @note  P2 观测噪声 R 与陀螺零偏过程噪声 Q2: R 从 1e7 降到 1e6(加速度计权重 x10),
 * @note     静止/慢速时偏置回正快一个量级; Q2 从 0.001 抬到 0.005 让零偏估计跟得上温漂。
 * @note 注意: ChiSquareTestThreshold 仍是固定 1e-8, 在 R=1e6 下等价的重力方向残差门限
 * @note     由约 18° 收紧到约 5.7°(rk = |r|^2 / R), 转动时的整帧拒测会更频繁,
 * @note     这正是 P1(取消整帧拒测, 改为自适应增益下限)要解决的问题。
 */
#define QEKF_PROCESS_NOISE_Q1   10.0f       /* 四元数过程噪声                        */
#define QEKF_PROCESS_NOISE_Q2   0.005f      /* 陀螺零偏过程噪声(原 0.001)            */
#define QEKF_MEASURE_NOISE_R    1000000.0f  /* 加速度计观测噪声(原 10000000)         */
#define QEKF_FADING_LAMBDA      1.0f        /* 渐消因子                              */
#define QEKF_ACCEL_LPF_TAU      0.010f      /* EKF 内部加速度一阶低通时间常数(s),原 0 */

/* ===| 卡方检验 / 自适应增益(P1) |===
 * @note 原版把门限写死成 1e-8, 且一旦残差超门限就"整帧丢弃量测"(只做陀螺积分)。
 * @note 实测(vofa+.csv): 快速翻转时陀螺打到 +-2000dps 满量程、加速度计打到 8g(量程 +-6g)
 * @note 双饱和, 姿态一次性丢几十度; 丢完之后残差始终 > 门限, 于是量测被永久丢弃,
 * @note 姿态卡在错误值上几秒才慢慢回来(靠 P 阵的微弱修正), 表现为"快速翻转后 Roll 出偏置"。
 * @note 对策: 门限仍用物理量表达(残差 rad), 但不再整帧丢弃 —— 只把增益压到下限,
 * @note 保证哪怕差 40° 也能在几百 ms 内被加速度计拉回来。
 */
#define QEKF_CHISQ_GATE_RAD     0.10f  /* 卡方门限允许的重力方向残差(rad, 0.1rad≈5.7°) */
#define QEKF_CHISQ_REJECT_EN    0      /* 0: 不整帧丢弃量测; 1: 原版行为                 */
#define QEKF_ADAPT_GAIN_EN      1      /* 1: 启用自适应增益(含下限)                      */
#define QEKF_ADAPT_GAIN_MIN     0.20f  /* 自适应增益下限(防止原式算出负增益=反向修正)    */

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = (QEKF_CHISQ_GATE_RAD * QEKF_CHISQ_GATE_RAD) / measure_noise; // rk ≈ |r|²/R, 由允许残差反推门限
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    // 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief 用一帧加速度把初始四元数直接对齐到真实重力方向
 * @param[in] ax ay az 机体坐标系下的加速度(单位任意,内部归一化)
 *
 * 为什么要这个函数:
 *   这个 EKF 的观测量只有重力方向 h(q)=R^T*[0,0,1]，陀螺积分负责猜、
 *   加速度负责纠。如果上电时开发板是**倒扣**的，而四元数仍从单位阵
 *   [1,0,0,0](=“正放”)起步，模型与真实姿态正好差 180°：
 *     - 残差 |z-h| ≈ 2，卡方检验几乎永远不通过，量测更新被跳过；
 *     - 剩下的修正量还被 H 在工作点处的线性化(此时在工作点附近退化)
 *       和 R=1e7 压得极小；
 *   于是姿态只能以约 1°/s 的速度沿某个任意轴“爬”过去，几十秒才收敛，
 *   期间 yaw 还会因为不可观测而乱翻 —— 表现就是“反向放置上电后，
 *   板子明明没动，姿态/角速度却一直在飘”。
 *   这里直接解析求出满足 h(q)=a/|a| 的四元数(yaw 取 0，本来不可观测)，
 *   上电即对准，残差≈0，不会再经历那段长距离爬行。
 */
void IMU_QuaternionEKF_Set_Initial_Attitude(float ax, float ay, float az)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    float half, sh, s, c, vx, vy;

    if (n < 1e-3f) // 数据无效(比如还没读成功)，保持单位四元数
    {
        return;
    }
    ax /= n;
    ay /= n;
    az /= n;

    // 目标: R^T*[0,0,1] = a，等价于 R 把 a 转到 [0,0,1]
    // 旋转轴 = a × [0,0,1] = (ay, -ax, 0)，旋转角 = acos(a·[0,0,1]) = acos(az)
    vx = ay;
    vy = -ax;
    s = sqrtf(vx * vx + vy * vy);
    c = az;

    if (s < 1e-6f)
    {
        if (c > 0.0f) // 正放
        {
            QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0.0f;
        }
        else // 完全倒扣: 绕 x 轴转 180°
        {
            QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 0.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 1.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0.0f;
            QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0.0f;
        }
    }
    else
    {
        half = 0.5f * atan2f(s, c);
        sh = sinf(half);
        QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = cosf(half);
        QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = vx / s * sh;
        QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = vy / s * sh;
        QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0.0f;
    }

    // 同步对外输出的四元数，并认为已经收敛(残差≈0)
    for (uint8_t i = 0; i < 4; i++)
    {
        QEKF_INS.q[i] = QEKF_INS.IMU_QuaternionEKF.xhat_data[i];
    }
    QEKF_INS.ConvergeFlag = 1;
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!QEKF_INS.Initialized)
    {
        IMU_QuaternionEKF_Init(QEKF_PROCESS_NOISE_Q1, QEKF_PROCESS_NOISE_Q2, QEKF_MEASURE_NOISE_R,
                               QEKF_FADING_LAMBDA, QEKF_ACCEL_LPF_TAU);
        /* 用第一帧加速度把初始姿态对准，避免反向放置上电时
           从“正放”出发、180° 误差长时间爬不回去（表现为静止也一直飘） */
        IMU_QuaternionEKF_Set_Initial_Attitude(ax, ay, az);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                        QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f)
    {
        QEKF_INS.StableFlag = 1;
    }
    else
    {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数和xy零飘值
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) * 57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++; // 初始化低通滤波用,计数测试用
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S1.numRows = kf->R.numRows;
    kf->S1.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S1); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S1, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
#if (QEKF_CHISQ_REJECT_EN)
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
#endif
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    /* ===| 自适应增益 |===
     * @note rk 越小增益越大, 否则更相信预测值。
     * @note 原式 (阈-rk)/(0.9阈) 在 rk>阈 时会算出负数(反向修正=直接发散),
     * @note 这里统一夹到 [QEKF_ADAPT_GAIN_MIN, 1]: 残差再大也保留 20% 的修正量,
     * @note 保证传感器饱和/被扰后能靠加速度计在几百 ms 内重建姿态。
     */
#if (QEKF_ADAPT_GAIN_EN)
    if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        if (QEKF_INS.AdaptiveGainScale < QEKF_ADAPT_GAIN_MIN)
        {
            QEKF_INS.AdaptiveGainScale = QEKF_ADAPT_GAIN_MIN;
        }
        else if (QEKF_INS.AdaptiveGainScale > 1.0f)
        {
            QEKF_INS.AdaptiveGainScale = 1.0f;
        }
    }
    else
    {
        QEKF_INS.AdaptiveGainScale = 1;
    }
#else
    QEKF_INS.AdaptiveGainScale = 1;
#endif

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
