#ifndef __Motor_Driver__
#define __Motor_Driver__

#include "main.h"

/*===| 电机数据结构体定义 |===*/
typedef struct
{
    int16_t Encoder;        //编码器值
    int16_t Encoder_Last;   //上一个编码器值
    int32_t Round;          //圈数
    float Angle;            //绝对角度   
    float Angle_Last;       //上一个绝对角度     
    float Total_Angle;      //总角度值
    float Total_Angle_Last; //总角度值
    float Total_Angle_Speed_RPM; //总角度值速度
    float Total_Angle_Speed_RPM_Last; //总角度值速度
    float Total_Angle_Offset;      //总角度值零点
    uint32_t Total_Angle_DWT_Count;
    float Speed_RPM;        //转速[RPM]
    float Torque;           //力矩[电流]
    int8_t Temperature;     //电机温度
    uint16_t Ticker;        //收到数据计时
    uint8_t If_Online;      //是否在线     
    uint8_t Error_ID;       //电机错误码
    uint8_t Date_New;       //收到新数据标志(Down 的通信层用)
} Motor_Data_StructTypeDef;

/**
 * @brief 电机结构体定义
 * @note 裁判主控一侧为车头，灯条一侧为车尾，以车为参考，左手边为左腿
 * @note 关节电机对应关系:Joint1(左腿车头)、Joint2(左腿车尾)、Joint3(右腿车头)、Joint4(右腿车尾)
 * @note 轮腿底盘(自 Down 移植)使用 Wheel_Motor1/2 + Joint1~4,云台/麦轮成员保留供 MC02 原有代码使用
 */
typedef struct
{
    /*===| 轮腿底盘 |===*/
    Motor_Data_StructTypeDef Wheel_Motor1;  //左腿轮电机
    Motor_Data_StructTypeDef Wheel_Motor2;  //右腿轮电机

    Motor_Data_StructTypeDef Joint1;    //左腿车头
    Motor_Data_StructTypeDef Joint2;    //左腿车尾
    Motor_Data_StructTypeDef Joint3;    //右腿车头
    Motor_Data_StructTypeDef Joint4;    //右腿车尾

    //用于UI显示
    Motor_Data_StructTypeDef Shoot_Fric_First_Left;
    Motor_Data_StructTypeDef Shoot_Fric_First_Right;

} Motor_StructTypedef;

/*===| 电机数据结构体 |===*/
extern Motor_StructTypedef Motor;

void Motor_Get_TotalAngle_Speed(Motor_Data_StructTypeDef *Motor_Data_Struct, float K);
void Motor_Set_Zero_Offset(Motor_Data_StructTypeDef *Motor_Data_Struct);
#endif
