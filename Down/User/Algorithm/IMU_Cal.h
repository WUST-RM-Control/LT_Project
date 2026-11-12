#ifndef __IMU_CAL_H__
#define __IMU_CAL_H__

#include "main.h"

/* ============================ 数学/数值配置 ============================
 * IMU_CAL_USE_DOUBLE=1: 拟合/旋转用 double(双精度, 精度优先);
 * =0: 用 float(单精度FPU, 更快)。实时/逐帧路径始终为 float。
 * ====================================================================== */
#define IMU_CAL_USE_DOUBLE   1u   /* 0=float(快, 够用); 1=double(更精更慢) */

#if (IMU_CAL_USE_DOUBLE == 1u)
    #define CAL_REAL    double
    #define CAL_ABS(x)  fabs(x)
    #define CAL_SQRT(x) sqrt(x)
#else
    #define CAL_REAL    float
    #define CAL_ABS(x)  fabsf(x)
    #define CAL_SQRT(x) IMU_Cal_Sqrt_F32(x)
#endif

/* 1) 日志/调试输出通道
 * IMU_CAL_PRINT_EN 是总闸门: =0 时连校准结果(%.6f 那些)都不打印, 编译后
 * 完全不调用 printf —— 不占串口、不撑任务栈。
 * 真正要现场校准的那一次, 把 PRINT_EN / DEBUG_EN 临时改成 1, 烧进去按提示
 * 操作, 校完再改回 0。
 */
#define IMU_CAL_PRINT_EN    0u   /* 1=允许打印; 0=全部静默(默认, 防干扰其它任务) */
#define IMU_CAL_DEBUG_EN    0u   /* 1=打印过程/操作引导(校准需人机交互时才开)      */

#if (IMU_CAL_PRINT_EN)
    #define IMU_CAL_PRINTF(...)     printf(__VA_ARGS__)  /* printf -> fputc -> UART1 */
#else
    #define IMU_CAL_PRINTF(...)     do {} while (0)      /* 完全静默 */
#endif

#if (IMU_CAL_PRINT_EN) && (IMU_CAL_DEBUG_EN)
    #define IMU_CAL_LOG(...)    IMU_CAL_PRINTF(__VA_ARGS__)  /* 过程信息(可关) */
#else
    #define IMU_CAL_LOG(...)    do {} while (0)
#endif

/* 2) 校准主循环里的 1ms 节拍
 * IMU_Cal() 现在在 main.c 的初始化段(osKernelStart 之前)执行, 调度器还没起来,
 * 不能用 osDelay/vTaskDelay(会访问空的 pxCurrentTCB 直接挂死), 所以统一用
 * HAL_Delay: 启动前后都能用, 代价是忙等, 只在这一次校准里发生。
 */
#define IMU_CAL_DELAY()         HAL_Delay(1u)

/*
 * IMU 全校准模块
 *  - 加速度计: 全自动采集若干个互不相同的静止姿态, 用最小二乘拟合
 *    逐轴零偏与比例(对角模型);
 *  - 陀螺仪  : 用所有静止样本的均值作为零漂;
 *  - 输出坐标对齐: 校准结束后额外采集一次“模块正面朝上、绝对水平静止”
 *    的姿态, 用它把输出坐标系的 +z 锁定为竖直朝上 —— 因此当模块正面
 *    朝上静止时, 输出加速度严格为 (0, 0, +g)。
 *
 * 使用方式(一次性, 阻塞执行):
 *   1) 在任务开头(循环之外)调用一次 IMU_Cal();  它内部自动读传感器、
 *      采集姿态、拟合、锁定正面参考面, 校准完成后把参数写入
 *      IMU_Cal_Data.Config 并返回;
 *   2) 之后在自己的循环里反复调用
 *      IMU_Cal_Get_Calibrated_Data(&acc,&gyr) 获取校准后的数据
 *      (该函数每次都会内部读一次原始传感器)。
 * 调试信息开关/打印通道等见 IMU_Cal.c 顶部“迁移接口”。
 * 结果保存在 IMU_Cal_Data.Config, 校准过程中的中间量也统一放在
 * IMU_Cal_Data 内(便于查看与管理)。
 * 单位: 加速度 = m/s^2, 角速度 = rad/s(与 BMI088_Read 输出一致)。
 */

/* ------------------------- 可调参数 ------------------------- */
#define IMU_CAL_POSE_MIN           6u      /* 最少姿态数(一般六个面即可)   */
#define IMU_CAL_POSE_MAX           12u     /* 姿态数上限, 达到后强制完成       */
#define IMU_CAL_POSE_SEP_COS       0.9511f /* cos(18°): 两个姿态之间的最小夹角 */
#define IMU_CAL_STATIC_SETTLE_MS   500u    /* 判定静止后的稳定等待时长         */
#define IMU_CAL_ACCUM_MS           1000u    /* 单个姿态的采样平均时长           */
#define IMU_CAL_MOVE_RAD_S         0.4f   /* 陀螺角速度小于该值视为静止       */
#define IMU_CAL_ACC_NORM_MIN       8.0f    /* 静止时 |加速度| 的范围下限(m/s^2)*/
#define IMU_CAL_ACC_NORM_MAX       11.5f   /* 静止时 |加速度| 的范围上限(m/s^2)*/
#define IMU_CAL_G_REF              9.8f    /* 参考重力加速度, 用于归一化尺度   */
#define IMU_CAL_RESID_TOL          0.5f    /* 各姿态校准模长的最大离散容差     */

/* 总体运行状态 */
typedef enum
{
    IMU_CAL_STATE_ACQUIRE = 0,  /* 阶段1: 采集姿态 */
    IMU_CAL_STATE_REF_UP,       /* 阶段2: 等待“正面朝上”, 锁定输出 +z */
    IMU_CAL_STATE_REF_FWD,      /* 阶段3: 等待 Pitch ~90°, 锁定输出前向 x/y */
    IMU_CAL_STATE_DONE          /* 完成: 校准结束, IMU_Cal() 返回 */
} IMU_Cal_State_e;

/* 采样状态机相位 */
typedef enum
{
    IMU_CAL_PHASE_WAIT_STATIC = 0,  /* 等待模块静止                         */
    IMU_CAL_PHASE_SETTLE,           /* 已静止, 等待 SETTLE_MS 稳定          */
    IMU_CAL_PHASE_ACCUM,            /* 对当前静止姿态取平均                 */
    IMU_CAL_PHASE_WAIT_MOVE         /* 本次已记录, 等待翻转后再采下一个     */
} IMU_Cal_Phase_e;

/* 校准参数, 保存在 IMU_Cal_Data.Config
 * @note 这里**不要**用 __PACKED_STRUCT: IMU_Cal.c 里有把成员地址交给 float* 的写法
 * @note (如 const float *off = IMU_Cal_Data.Config.Acc_Offset), 一旦结构体被 packed,
 * @note 编译器会把变量放到奇数地址(实测 0x2400479D), 而 ARMCLANG 对 float* 会生成对齐的
 * @note VFP 访问, Cortex-M7 遇到非对齐的浮点/双字访问直接 UsageFault(UNALIGNED)->HardFault。
 * @note 本结构体从来没有被当成字节块存进 Flash(保存是逐字段拷到 Config.IMU_Cal_*), 不需要 packed。 */
typedef struct
{
    float   Acc_Offset[3];  /* 加速度计零偏, 单位 m/s^2, 芯片坐标系         */
    float   Acc_Scale[3];   /* 加速度计比例, 芯片坐标系                     */
    float   Gyro_Offset[3]; /* 陀螺仪零漂,   单位 rad/s, 芯片坐标系         */
    float   Frame_R[3][3];  /* 芯片->模块(输出)旋转矩阵 R; 模块正面朝上时
                               输出加速度 = R * (校准后芯片加速度) = (0,0,+g) */
    uint8_t Is_Calibrated;  /* 0 = 未校准/校准中, 1 = 已校准                 */
} IMU_Cal_Config_StructTypedef;

typedef struct
{
    IMU_Cal_Config_StructTypedef Config;    /* 保存的数据(校准参数) */

    /* ---------------- 校准过程中间量(便于查看/调试) ---------------- */
    uint8_t  CalState;      /* 状态机: 0采集 1朝上 2Pitch90 3完成         */
    uint8_t  CalPhase;      /* 采样状态机相位(见 IMU_Cal.c 枚举)            */
    uint32_t PhaseStartMs;  /* 进入当前相位的时刻                          */
    uint8_t  PoseCount;     /* 已采集的互异姿态数量                        */
    float    PoseAcc[IMU_CAL_POSE_MAX][3];  /* 每个姿态的平均原始加速度    */
    float    PoseUnit[IMU_CAL_POSE_MAX][3]; /* 每个姿态的单位向量(判重用)  */
    float    AccSum[3];     /* 当前姿态的加速度累加和                      */
    uint32_t AccCnt;        /* 当前姿态的样本数                            */
    float    GyrSum[3];     /* 陀螺零漂的累加和                            */
    uint32_t GyrCnt;        /* 陀螺零漂累计样本数                          */
    float    RefUpRaw[3];   /* “正面朝上”参考姿态的原始加速度均值          */
    float    RefFwdRaw[3];  /* Pitch ~90° 参考姿态的原始加速度均值(定前向) */
} IMU_Cal_Data_StructTypedef;

extern IMU_Cal_Data_StructTypedef IMU_Cal_Data;

/* ============================ 函数声明 ============================ */
/* ---- 对外主接口 ---- */
void    IMU_Cal(void);                                         /* 一次性阻塞校准(任务循环外调用一次) */
void    IMU_Cal_Get_Calibrated_Data(float *Acc, float *Gyro);  /* 读一次原始数据并输出校准值         */
uint8_t IMU_Cal_Is_Calibrated(void);                           /* 是否已完成校准                      */

/* ---- 校准参数 保存/加载 ---- */
uint8_t IMU_Cal_Load_Config(void);                            /* 加载校准参数: 仅从 NVS 读取(1=有记录并已应用)    */
void    IMU_Cal_Save_Config(void);                            /* 保存当前校准参数到 NVS(校准完成后调用)              */
void    IMU_Cal_Reset_Config(void);                           /* 清除当前参数并抹除 NVS 记录, 立即回到未校准         */

/* ---- 平台接口(可在其它模块直接使用/迁移时替换) ---- */
void    IMU_Cal_Read_Raw(float acc[3], float gyr[3]);          /* 读最新原始 acc(m/s2)/gyr(rad/s) */
float   IMU_Cal_Sqrt_F32(float v);                             /* 单精度平方根(arm_math/FPU)      */

/* ---- 采样/采集相关 ---- */
uint8_t IMU_Cal_Is_Static(const float *acc, const float *gyr); /* 判断是否静止                      */
void    IMU_Cal_Accum_Gyro(const float *gyr);                  /* 静止样本累计陀螺(零漂用)          */
uint8_t IMU_Cal_Sample_One(const float *acc, const float *gyr,
                           uint8_t isStatic, uint32_t now, float mean[3]); /* 采样一个静止姿态平均 */
uint8_t IMU_Cal_Pose_Is_New(const float mean[3]);              /* 新姿态去重判断                     */
void    IMU_Cal_Pose_Add(const float mean[3]);                 /* 记录一个姿态                       */

/* ---- 拟合/输出坐标 ---- */
uint8_t IMU_Cal_GaussSolve(const CAL_REAL *N, const CAL_REAL *rhs, CAL_REAL *x, int n); /* 通用线性求解 */
uint8_t IMU_Cal_Fit_Accel(void);                               /* 两步法拟合 acc 零偏/比例(写Config) */
uint8_t IMU_Cal_Compute_Frame(void);                           /* 由正面参考构造旋转矩阵 Frame_R     */
uint8_t IMU_Cal_Try_Fit(void);                                 /* 姿态够则尝试拟合, 返回是否成功     */
void    IMU_Cal_Finish(void);                                  /* 收尾: 零漂/标志/打印校准值         */

#endif