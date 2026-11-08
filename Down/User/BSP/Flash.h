#ifndef __Flash__
#define __Flash__
#include "main.h"

/* ====| IMU 全校准参数的有效标志 |====
 * W25Q64 擦除后是 0xFF, 读出来的 float 是 NaN, 不能可靠地"看着像 0 就当没校准",
 * 所以单独存一个 magic。等于这个值才认为下面那组 IMU_Cal_* 参数是写过的。
 * 改这个常量等于让所有已经保存在 flash 里的旧参数失效(会重新校准一次)。 */
#define IMU_CAL_MAGIC   0x494D5543u     /* 'I''M''U''C' */

/* ====| 氮气弹簧标定数据的头帧 |====
 * 和 IMU 标定一样, 用"头帧 + 校验"判断 L0_Leg_L/R_Cal_Data 里是不是一份真正写过的标定数据:
 *   Magic   == SPRINGCOMP_CAL_MAGIC     头帧存在
 *   Version == SPRINGCOMP_CAL_VERSION   数据格式版本(改结构或算法时 +1, 旧数据自动失效)
 *   Size    == sizeof(L0_Leg_L_Cal_Data) 长度对得上(防止结构体被改过)
 *   Check_L/R                           每条腿数据的 CRC32(防止擦写中断/写坏导致读回乱码)
 *   Valid                                第0位=左腿有效, 第1位=右腿有效(单腿标定时不会让另一条腿被误认有效)
 * 任何一项对不上, WheelLeg_SpringComp() 就输出 GasSpring_Output = 0 并返回 -1。
 * 改这个 magic 等于让所有已保存在 W25Q64 里的旧弹簧标定数据失效。 */
#define SPRINGCOMP_CAL_MAGIC    0x53505243u     /* 'S''P''R''C' */
#define SPRINGCOMP_CAL_VERSION  1u
#define SPRINGCOMP_CAL_VALID_L  0x01u
#define SPRINGCOMP_CAL_VALID_R  0x02u

typedef struct
{
    float IMU_OFFSET_Data[5];
    float L0_Leg_L_Cal_Data[1024*2];
    float L0_Leg_R_Cal_Data[1024*2];

    /* ====| 氮气弹簧标定数据的头帧(由 WheelLeg_SpringComp_Flash_Save() 写入) |==== */
    uint32_t SpringComp_Cal_Magic;      /* == SPRINGCOMP_CAL_MAGIC 时头帧才成立 */
    uint32_t SpringComp_Cal_Version;    /* 数据格式版本                          */
    uint32_t SpringComp_Cal_Size;       /* 单腿数据字节数                        */
    uint32_t SpringComp_Cal_Check_L;    /* 左腿数据 CRC32                        */
    uint32_t SpringComp_Cal_Check_R;    /* 右腿数据 CRC32                        */
    uint32_t SpringComp_Cal_Valid;      /* bit0:左腿有效 bit1:右腿有效           */

    /* ====| IMU 全校准参数(IMU_Cal 模块用) |====
     * 由 IMU_Cal_Save_Config() 写入, IMU_Cal_Load_Config() 读回。
     * 追加在结构体末尾, 前面已有字段的偏移不变, 老参数继续可用。 */
    uint32_t IMU_Cal_Magic;          /* == IMU_CAL_MAGIC 时下面参数才有效        */
    float IMU_Cal_Acc_Offset[3];     /* 加速度计三轴零偏, m/s^2                 */
    float IMU_Cal_Acc_Scale[3];      /* 加速度计逐轴比例(对角模型)              */
    float IMU_Cal_Gyro_Offset[3];    /* 陀螺零漂, rad/s                         */
    float IMU_Cal_Frame_R[9];        /* 芯片->输出坐标 旋转矩阵, 行优先 R[3][3] */
} Config_StructTypedef;

extern Config_StructTypedef Config;

void Config_Save(void);
void Config_Read(void);
void Flash_Init(void);

#endif
