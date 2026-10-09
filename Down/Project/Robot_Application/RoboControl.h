#ifndef __RoboControl__
#define __RoboControl__

#include "main.h"
#include "Chassis.h"
#include "Shoot.h"
#include "ErrorHandle.h"

#define RoboWz_PID_Kp					0.15f
#define RoboWz_PID_Ki					0.0f
#define RoboWz_PID_Kd					0.0f
#define RoboWz_PID_I_Output_MAX			0.0f
#define RoboWz_PID_Output_MAX			5.0f

typedef enum
{
    none = 0,
	Customer,
	Joystick,
	KeyboardMouse,
} Robo_Controler_EnumTypedef;

/*===| 机器人整体状态数据结构体定义 |===*/
typedef struct
{
    uint8_t Robo_Enable;                    //是否启动机器人
	float Smooth_Start_K;                   //平滑启动比例

	Robo_Controler_EnumTypedef Controler;   //控制方式

	Gimbal_State_EnumTypedef Gimbal_State;  //云台状态
    Shoot_State_EnumTypedef Shoot_State;    //发射机构状态	
/*===| 底盘控制 |===*/

	Chassis_State_EnumTypedef Chassis_State;//底盘运动状态

	uint8_t If_SettingZero;		//设零点
	
    float Robo_Target_Vy;		//相对云台的前后速度(云台前方为正方向),m/s
    float Robo_Target_Vx;		//相对云台的左右速度(云台右为正方向)，m/s
    float Robo_Target_Wz;		//相对云台的旋转速度(逆时针为正方向)，弧度/s

    float Robo_Target_Vy_Last;		//相对云台的前后速度(云台前方为正方向),m/s
    float Robo_Target_Vx_Last;		//相对云台的左右速度(云台右为正方向)，m/s
    float Robo_Target_Wz_Last;		//相对云台的旋转速度(逆时针为正方向)，弧度/s

    float Yaw_Err;				//云台相对底盘的角度差值[-180 ~ +180]°
	float Target_Direction;		//云台相对底盘的角度差值的目标方向，°
	float Target_Direction_Err;	//当前目标方向与云台的角度差值°
    
	uint8_t Chassis_Speed_Level;//底盘速度等级1~5
	
    uint8_t SPIN_Direction_Flag;//小陀螺旋转方向标志位
	
	uint8_t SuperCap_State;	//超电放电开关
    float SuperCap_V;		//超电电压
    float Referee_P;		//裁判系统Chassis功率
    float Chassis_P;		//底盘功率
	
	float Leg_Length;
	float Leg_Angle;
		
    uint8_t Power_Limit_Flag;				//功率限制标志位
		
    uint8_t Refresh_UI_Flag;                //刷新UI标志位 
		
} RoboControl_StructTypeDef;

/*===| 机器人整体状态数据结构体 |===*/
extern RoboControl_StructTypeDef RoboControl_Struct;

/*===| 输入参数滤波 |===*/
void Control_Filter(void);

/*===| 根据底盘运动状态得到Wz |===*/
void Get_Chassis_Wz(void);

/*===| 遥控器控制 |===*/
void RemoteControl_Float(void);
void RemoteControl_Bool(void);

/*===| 键盘控制 |===*/
void KeyControl_Float(void);
void KeyControl_Bool(void);

/*===| 停止机器人 |===*/
void Robo_Stop(void);

/*===| 重启机器人 |===*/
void Robo_Restart(void);

#endif