#ifndef __Chassis__
#define __Chassis__

#include "main.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*===| 底盘运动状态枚举定义 |===*/
typedef enum 
{
	//失能
		//失能
		Chassis_OFF = 0,
		//缓失能
		Chassis_OFF_Slow=1,

	//翻倒自起
		//翻倒回正
		Chassis_Recover=2,
	//机体已回正
		//倒地自起(LQR自起)
		Chassis_RESET=3,
		//倒地缓自起(PID自起)
		Chassis_RESET_Slow=4,
	
	//使能
		//底盘跟随
		Chassis_FOLLOW=5,
		//小陀螺
		Chassis_SPIN=6,
		//跳跃
		Chassis_Jump=7,
		//磕台阶
		Chassis_UpStep=8,

	//离地&错误姿态
		Chassis_Off_ground=9,

	//测试
		Chassis_Text=-1
} Chassis_State_EnumTypedef;

/*===| Leg身份枚举定义 |===*/
typedef enum 
{
	//左腿
	Leg_L = 0,
	//右腿
	Leg_R = 1
} Chassis_Leg_RL_State_EnumTypedef;

/*===| 底盘数据和一些底盘控制数据结构体定义 |===*/
typedef struct
{
	/*===| 底盘运动状态 |===*/
    Chassis_State_EnumTypedef Chassis_State;

	/*===| 外部数据 |===*/
	//INS
		float Acceleration_Z;	//Z轴加速度(滤波后)
		float Acceleration_Y;	//Y轴加速度:INS.MotionAccel_n[1]
		float Pitch;			//弧度,LQR使用的pitch,车头抬高pitch增大
		float Pitch_dot;		//弧度
		float Pitch_Last;		//弧度
		float Yaw_Speed;		//弧度/s，俯视逆时针为正方向，用于转向和小陀螺
	//Chassis
	float Vy_Target;	//前进为正方向
	float Vy_Target_Last;	//前进为正方向
	float Vx_Target;
	float Wz_Speed_Target;

	uint8_t Jump_Level;	//跳跃等级(RoboControl_Struct赋值)

	/*===| 底盘数据 |===*/
	//Leg
		float Leg_Length_Target;	//目标腿长(RoboControl_Struct赋值,用于记录)
		float Leg_Length_Length;	//当前腿长(赋值,用于LQR计算)
		float Leg_Angle_Target;		//目标腿摆角(PID控腿摆角时使用)
	//Wheek
		//Wz
		float Chassis_Wz_Speed_Target; //当前值：INS_Data_Self.Yaw_Speed
		float Yaw_Angle_Target;
	
} Chassis_Date_StructTypeDef;

/**
 * @brief ===| 底盘腿部参数定义 |===
 * @note 裁判主控一侧为车头，灯条一侧为车尾，以车为参考，左手边为左腿
 * @note 腿在相对机体垂直、竖直向下时腿总摆角为0°+N*360°
 * @note 左腿从左看逆时针为正方向(力和摆角都是)
 * @note 右腿从右看逆时针为正方向(力和摆角都是)
 * @note 沿杆的力伸腿方向为正方向
 * @note 关节电机对应关系:关节1(左腿车头)、关节2(左腿车尾)、关节3(右腿车头)、关节4(右腿车尾)
 * 
 */
typedef struct
{
//Leg
	Chassis_Leg_RL_State_EnumTypedef Leg_LR;

	//Leg数据(卡尔曼滤波后数据)
		//腿长
		float Length_Target;//一阶倒立摆腿长m
		float Length_Feedback;//m
		float Length_Last;//m
		float Length_Speed;//m/s
		float Length_Speed_Last;//m/s
		//摆角
		float Angle_Target;//一阶倒立摆腿角度°
		float Angle_Feedback;//°
		float Angle_Last;//°
		float Angle_Round;//圈数
		//总摆角
		float Total_Angle_Target;//一阶倒立摆腿总角度°
		float Total_Angle_Feedback;//°
		float Total_Angle_Last;//°
		float Total_Angle_Speed;//°/s

	//Leg力有关
		/*===| 雅可比力矩阵 |===*/
		float Jt[2][2];
		
		//氮气弹簧补偿
		float GasSpring_Output;

		//虚拟力输出
		//0：L沿杆方向的，1：T关节为绕轴力矩
		//沿杆的力：伸腿方向为正方向
		//绕轴力矩：左(右)侧看逆时针为正方向
			//LQR输出
			float LQR_Link_Torque_Target[2];
			//关节电机目标力矩(未限幅,WheelLeg_Output函数中限幅)
			//0:电机14，1:电机23
			float LQR_Wheel_Torque_Target[2];
			// //PID输出
			// float PID_Link_Torque_Target[2];

//Wheel
	//轮电机力矩(PID输出和LQR)
	float Wheel_Torque;

} Chassis_Leg_StructTypedef;

/*===| 底盘控制结构体定义 |===*/
typedef struct
{
	Chassis_Date_StructTypeDef Chassis_Control;

	Chassis_Leg_StructTypedef Leg_Left;
	Chassis_Leg_StructTypedef Leg_Right;

	//DWT
	float Chassis_Dt;
	uint32_t Chassis_Count;

} Chassis_Control_StructTypedef;
#endif