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
		//仅测试使用，比赛禁用
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

	// //离地&错误姿态
	// 	Chassis_Off_ground=9,

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

/*===| 对底盘状态的详细描述Flag |===*/
typedef struct
{

    //倒地缓自起(PID自起)
    //0：非倒地自起状态或标志位赋值完成
	//1：准备状态(标志位挂起)
	//2：摆角90°
	//3：收腿
    uint8_t Chassis_RESET_Slow_FLag;
    
    //跳跃
    //0：非跳跃状态或标志位赋值完成
	//1：准备状态(标志位挂起与蓄力，保证腿长低于Chassis_Jump_Ready_Link_Arrive蓄力腿长阈值)
	//2：蹬腿跳跃
	//3：空中收腿(LQR仅保留控制摆角的控制量,轮电机输出0)
	//4：落地缓冲(空中伸腿，落地缓冲)
    uint8_t Chassis_Jump_FLag;

    //底盘使能Chassis_FOLLOW或Chassis_SPIN时可置1
	//底盘离地标志位
	//0：正常
	//1：两腿同时离地且Z轴加速度达标
    uint8_t Off_ground_Flag;

	//底盘使能Chassis_FOLLOW或Chassis_UpStep可修改
	//0：非上台阶状态或已完成
	//1：准备上台阶，进入Chassis_UpStep状态，并和Chassis_FOLLOW运控一致
	//2：磕到台阶后向后摆腿同时收腿，Chassis_UpStep状态下PID控制摆腿收腿
	uint8_t Up_Step_Flag;

} WheelLeg_Control_Flag_StructTypeDef;


/*===| 底盘数据和一些底盘控制数据结构体定义 |===*/
typedef struct
{
	/*===| 底盘运动状态 |===*/
    Chassis_State_EnumTypedef Chassis_State;

	/*===| 外部数据 |===*/
	//INS
		float Acceleration_Z;	//Z轴加速度(滤波后)
		float Acceleration_Y;	//Y轴加速度:-INS.MotionAccel_n[1]
		float Roll;				//角度.左腿抬高增大
		float Roll_dot;			//角度
		float Roll_Last;		//角度
		float Pitch;			//角度,LQR使用的pitch*Angle2Radain,车头抬高pitch增大
		float Pitch_dot;		//角度
		float Pitch_Last;		//角度
		float Yaw_Speed;		//弧度/s，俯视逆时针为正方向，用于转向和小陀螺
								//INS.Gyro[Z]角速度，rad/s
	//Chassis
	float Vy_Target;	//前进为正方向
	float Vy_Target_Last;	//前进为正方向
	float Vx_Target;
	float Wz_Speed_Target;	//当前值：INS.Gyro[Z]角速度，rad/s

	uint8_t Jump_Level;	//跳跃等级(RoboControl_Struct赋值)

	/*===| 底盘数据 |===*/
	//Leg
		float Leg_Length_Target;	//目标腿长(RoboControl_Struct赋值,用于记录)
		float Leg_Length_Feebback;	//当前腿长(赋值,用于LQR计算和腿长PID位置环
		float Chassis_Speed_Feedback;	//滤波后底盘速度
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
		// float Length_Target;//一阶倒立摆腿长m
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
			//LQR运控下的沿杆的额外力(跳跃)[0]
			//LQR输出(Torque)[1]
			float LQR_Link_Torque_Target[2];
			//轮电机目标力矩(未限幅,WheelLeg_Output函数中限幅)
			float LQR_Wheel_Torque_Target;
			// //PID输出
			// float PID_Link_Torque_Target[2];
			//本周期实际输出的用于支撑机体的虚拟沿杆力(N,伸腿方向为正,WheelLeg_Output赋值,供离地检测使用)
			float Link_Output;
			//本周期实际输出的绕轴力矩(N,伸腿方向为正,WheelLeg_Output赋值,供离地检测使用)
			float Torque_Output;

// //Wheel
// 	//轮电机力矩(PID输出和LQR)
// 	float Wheel_Torque;

} Chassis_Leg_StructTypedef;

/*===| 底盘控制结构体定义 |===*/
typedef struct
{
	
	WheelLeg_Control_Flag_StructTypeDef WheelLeg_Control_Flag;

	Chassis_Date_StructTypeDef Chassis_Control;

	Chassis_Leg_StructTypedef Leg_Left;
	Chassis_Leg_StructTypedef Leg_Right;

	//DWT
	float Chassis_Dt;
	uint32_t Chassis_Count;

} Chassis_Control_StructTypedef;

extern Chassis_Control_StructTypedef Chassis_Control_Struct;

void Get_Control_Data(Chassis_Control_StructTypedef *Chassis_Control_Struct);

#endif