#ifndef __Motor_Driver__
#define __Motor_Driver__

#include "main.h"

#define Motor_Num 13

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
    uint32_t Total_Angle_DWT_Count;
	float Speed_RPM;        //转速[RPM]
	float Torque;           //力矩[电流]
    int8_t Temperature;     //电机温度
	uint16_t Ticker;        //收到数据计时
    uint8_t If_Online;      //是否在线  
	uint8_t Error_ID;
} Motor_Data_StructTypeDef;


/**
 * @brief 电机结构体定义
 * @note 裁判主控一侧为车头，灯条一侧为车尾，以车为参考，左手边为左腿
 * @note 关节电机对应关系:Joint1(左腿车头)、Joint2(左腿车尾)、Joint3(右腿车头)、Joint4(右腿车尾)
 * 
 */
typedef __PACKED_STRUCT
{
    Motor_Data_StructTypeDef Wheel_Motor1;  //左腿轮电机
    Motor_Data_StructTypeDef Wheel_Motor2;  //右腿轮电机
    
    Motor_Data_StructTypeDef Joint1;    //左腿车头
    Motor_Data_StructTypeDef Joint2;    //左腿车尾
    Motor_Data_StructTypeDef Joint3;    //右腿车头
    Motor_Data_StructTypeDef Joint4;    //右腿车尾
    
    // Motor_Data_StructTypeDef Gimbal_Pitch;
    // Motor_Data_StructTypeDef Gimbal_Yaw;

    //用于UI显示
    Motor_Data_StructTypeDef Shoot_Fric_First_Left;
    Motor_Data_StructTypeDef Shoot_Fric_First_Right;
    // Motor_Data_StructTypeDef Shoot_Fric_First_Middle;
    // Motor_Data_StructTypeDef Shoot_Trigger;
    // Motor_Data_StructTypeDef Shoot_Fric_Second_Left;
    // Motor_Data_StructTypeDef Shoot_Fric_Second_Right;
    // Motor_Data_StructTypeDef Shoot_Fric_Second_Middle;

} Motor_StructTypedef;

/*===| 电机数据结构体 |===*/
extern Motor_StructTypedef Motor;

void Get_TotalAngle_Speed(Motor_Data_StructTypeDef *Motor_Data_Struct);

#endif