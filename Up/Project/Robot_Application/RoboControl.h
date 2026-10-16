#ifndef __RoboControl__
#define __RoboControl__

#include "main.h"
#include "Chassis.h"
#include "Shoot.h"
#include "Gimbal.h"

typedef enum
{
    none = 0,
		Customer,
		Joystick,
		KeyboardMouse,
		Chassis_Board,
} Robo_Controler_EnumTypedef;

/*===| 机器人整体状态数据结构体定义 |===*/
typedef struct
{
    uint8_t Robo_Enable;                    //是否启动机器人
	uint8_t Robo_Last_Enable;
	
    float Smooth_Start_K;                   //平滑启动比例
    Robo_Controler_EnumTypedef Controler;   //控制方式
    
    float Robo_Target_Vx;                   //相对云台的前后速度
    float Robo_Target_Vy;                   //相对云台的左右速度
    float Robo_Target_Wz;                   //相对云台的旋转速度
    float Robo_Target_Gimbal_Yaw;           //云台目标Yaw值
    float Robo_Target_Gimbal_Pitch;         //云台目标Pitch值
	
	float Robo_Target_Down_to_UP_Yaw;
	float Robo_Target_Down_to_UP_Pitch;
    
    float Robo_Target_Vx_Last;              //上一次相对云台的前后速度
    float Robo_Target_Vy_Last;              //上一次相对云台的左右速度
    float Robo_Target_Wz_Last;              //上一次相对云台的旋转速度
    float Robo_Target_Gimbal_Yaw_Last;      //上一次云台目标Yaw值
    float Robo_Target_Gimbal_Pitch_Last;    //上一次云台目标Pitch值

    float Yaw_Err;                          //云台相对底盘的角度差值[-180 ~ +180]
    
    Gimbal_State_EnumTypedef Gimbal_State;  //云台状态
    
    Shoot_State_EnumTypedef Shoot_State;    //发射机构状态
    
    uint8_t Chassis_Speed_Level;            //底盘速度等级1~5
    Chassis_State_EnumTypedef Chassis_State;//底盘运动状态
    uint8_t SPIN_Direction_Flag;            //小陀螺旋转方向标志位
    uint8_t Chassis_Follow_45_Direction_Flag;//底盘跟随云台偏移方向标志位
    
    uint8_t SuperCap_State;                 //超电放电开关
    float SuperCap_V;                       //超电电压
    float Referee_P;                        //裁判系统Chassis功率
    float Chassis_P;                        //底盘功率
    
    uint8_t Refresh_UI_Flag;                //刷新UI标志位 
			
	uint8_t Leg_Length;						//腿长
	uint8_t	Volecity_Fdb;					//底盘速度
	
		float Yaw;
		float Pitch;
		
	float operator_yaw_offset;
	float operator_pitch_offset;
		
	uint8_t Lock_yaw_flag;					//锁住yaw轴标志位，	0未锁住，1锁住
	uint8_t Recover_from_ground_Flag;		//倒地自起标志位，	0未自起，1为自起
	uint8_t Chassis_Static_Flag;			//底盘静止标志位，	0为其他状态，1为静止
	uint8_t Gimbal_State_Flag;				//云台使能标志位，	0为失能，1为使能
		
//	  float Robo_Target_Yaw;
//    float Robo_Target_Pitch1; 
//    float Robo_Target_Pitch2;
//    float Robo_Target_J4;
//    float Robo_Target_J5;
//    float Robo_Target_J6;
//    float Robo_Target_J7;
//		
//		float Robo_Target_Clap;
		
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
