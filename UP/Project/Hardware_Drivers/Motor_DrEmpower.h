#ifndef __Motor_DrEmpower_Driver__
#define __Motor_DrEmpower_Driver__

#include "main.h"

/*===| 参数设置指令(CMD_ID=0x1F) |===*/
typedef __PACKED_STRUCT
{
    uint16_t Mask_ID;
    uint16_t Type_ID;
    uint32_t Data;
} DrEmpower_SetConfigCMD_StructTypedef;

/*===| 系统控制指令(CMD_ID=0x08) |===*/
typedef __PACKED_STRUCT
{
    uint32_t order_num;
    uint16_t order_flag;
    uint16_t none;
} DrEmpower_SyetemControlCMD_StructTypedef;

/*===| 电机力矩控制(CMD_ID=0x1D) |===*/
typedef __PACKED_STRUCT
{
    float Torque;
    uint16_t Ramp;
    uint16_t Mode;
} DrEmpower_SetTorqueCMD_StructTypedef;

/*===| 实时反馈数据结构体 |===*/
typedef __PACKED_STRUCT
{
    float Angel;
    int16_t SpeedRPM_x100;
    int16_t Torque_x100;
} DrEmpower_RxData_StructTypedef;


/*===| 设置电机ID(1~63) ID=0则为广播 |===*/
void Motor_DrEmpower_Set_ID(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint16_t Target_ID);
/*===| 开关数据反馈(不可保存) |===*/
void Motor_DrEmpower_DataFeedBack(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint8_t IF_Enable);
/*===| 设置数据反馈时间(可保存) |===*/
void Motor_DrEmpower_DataFeedBackPeriod(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint16_t Period);
/*===| 保存参数 |===*/
void MotorMotor_DrEmpower_SaveConfig(FDCAN_HandleTypeDef *hfdcan, uint8_t ID);
/*===| 重启 |===*/
void Motor_DrEmpower_Reboot(FDCAN_HandleTypeDef *hfdcan, uint8_t ID);
/*===| 设置零点 |===*/
void Motor_DrEmpower_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t ID);
/*===| 直接力矩控制 |===*/
void Motor_DrEmpower_SetTorque(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, float Torque);
/*===| 保存电机反馈参数 |===*/
void Motor_DrEmpower_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct);

#endif