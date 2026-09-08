#ifndef __Gimbal__
#define __Gimbal__

#include "main.h"

/*===| 云台电机PID参数 |===*/
//#define Gimbal_Yaw_Angle_PID_Kp                 30.0f
//#define Gimbal_Yaw_Angle_PID_Ki                 0.0f
//#define Gimbal_Yaw_Angle_PID_Kd                 10.0f
//#define Gimbal_Yaw_Angle_PID_Kf					0.0f
//#define Gimbal_Yaw_Angle_PID_I_Output_Max       0.0f
//#define Gimbal_Yaw_Angle_PID_Output_Max         300.0f

//#define Gimbal_Yaw_Speed_PID_Kp                 500.0f
//#define Gimbal_Yaw_Speed_PID_Ki                 1.0f
//#define Gimbal_Yaw_Speed_PID_Kd                 0.0f
//#define Gimbal_Yaw_Speed_PID_Kf					150.0f
//#define Gimbal_Yaw_Speed_PID_I_Output_Max       1500.0f
//#define Gimbal_Yaw_Speed_PID_Output_Max         25000.0f

//#define Gimbal_Pitch_Angle_PID_Kp                 10.0f
//#define Gimbal_Pitch_Angle_PID_Ki                 0.0f
//#define Gimbal_Pitch_Angle_PID_Kd                 30.0f
//#define Gimbal_Pitch_Angle_PID_Kf				  0.0f
//#define Gimbal_Pitch_Angle_PID_I_Output_Max       0.0f
//#define Gimbal_Pitch_Angle_PID_Output_Max         50.0f

//#define Gimbal_Pitch_Speed_PID_Kp                 500.0f
//#define Gimbal_Pitch_Speed_PID_Ki                 1.0f
//#define Gimbal_Pitch_Speed_PID_Kd                 0.0f
//#define Gimbal_Pitch_Speed_PID_Kf				  50.0f
//#define Gimbal_Pitch_Speed_PID_I_Output_Max       500.0f
//#define Gimbal_Pitch_Speed_PID_Output_Max         25000.0f

//#define Gimbal_DM_PID_Kp                 120.0f
//#define Gimbal_DM_PID_Kd                 0.5f

/*===| 云台状态参数枚举 |===*/
typedef enum
{
    Gimbal_State_Normal = 0,
    Gimbal_State_Slow = 1,
    Gimbal_State_Aim = 3,
} Gimbal_State_EnumTypedef;


/*===| 云台数据结构体定义 |===*/
typedef struct
{
    Gimbal_State_EnumTypedef Gimbal_State;      //云台状态
    uint8_t If_Setting_Zero;
    
    /*===| Yaw单位为度 |===*/
    float Yaw_Target;                           //目标Yaw值
    float Yaw_Feedback;                         //当前Yaw值
    
    int16_t Yaw_Encoder_Err;                      //云台和底盘的编码器偏移角
    float Yaw_Err;                              //云台和底盘的偏移角
    float Vision_Yaw_Err;
        
    /*===| pitch单位为度 |===*/   
    float Pitch_Target;                         //目标Pitch值
    float Pitch_Feedback;                       //当前Pitch值
    
    /*===| PID结构体 |===*/
	
	/*===| 正常模式下 |===*/
    PID_Struct_TypeDef Yaw_Angle_PID_Struct;    //Yaw角度环PID结构体
    PID_Struct_TypeDef Yaw_Speed_PID_Struct;    //Yaw速度环PID结构体
    PID_Struct_TypeDef Pitch_Angle_PID_Struct;  //Pitch角度环PID结构体
    PID_Struct_TypeDef Pitch_Speed_PID_Struct;  //Pitch速度环PID结构体
	
	/*===| 倒地自起下云台yaw轴锁住 |===*/
	PID_Struct_TypeDef Yaw_Angle_Lock_PID_Struct;    //Yaw角度环PID结构体
    PID_Struct_TypeDef Yaw_Speed_Lock_PID_Struct;    //Yaw速度环PID结构体
	
	/*===| 自瞄模式下 |===*/
	PID_Struct_TypeDef Yaw_Angle_Aim_PID_Struct;    //Yaw角度环PID结构体
    PID_Struct_TypeDef Yaw_Speed_Aim_PID_Struct;    //Yaw速度环PID结构体
    PID_Struct_TypeDef Pitch_Angle_Aim_PID_Struct;  //Pitch角度环PID结构体
    PID_Struct_TypeDef Pitch_Speed_Aim_PID_Struct;  //Pitch速度环PID结构体
    
} Gimbal_Control_StructTypeDef;

/*===| 云台控制数据结构体 |===*/
extern Gimbal_Control_StructTypeDef Gimbal_Control_Struct;

#endif
