//========================================================
//                      Commnuicate[双板通信]
//      双板通信的发送和解包
//      当前有；云台Vx,Vy,Wz,YawErr，以及一些状态标志位
//========================================================

#include "Communicate.h"
#include "RoboControl.h"
#include "INS.h"
#include "stdio.h"
#include "vofa.h"
#include "Aim.h"
#include "Remote_Control.h"

extern uint16_t Board_Commnuicate_Error_Ticker;
extern uint8_t IF_Chassis_Online;

Remote_Pack1_TypedefStruct Remote_Pack1;
Remote_Pack2_TypedefStruct Remote_Pack2;

Double_Board_Up_to_Down_TypedefStruct Double_Board_Up_to_Down;
Double_Board_Down_to_UP_Referee_TypedefStruct Double_Board_Down_to_UP_Referee;
Double_Board_Down_to_UP_TypedefStruct Double_Board_Down_to_UP;

/**
* @brief 通信任务
 */
void Communicate_Task(void *argument)
{
    for(;;)
    {
		Gimbal_Send_Remote_Pack1();
		osDelay(5);
		// Gimbal_Send_Remote_Pack2();
		// osDelay(5);
		Gimbal_Send_attitude_Pack();
		osDelay(5);
	}
}

void Gimbal_Send_Remote_Pack1(void)
{
// 	Remote_Pack1.Right_X			= 127*Remote.Right_X			;
// 	Remote_Pack1.Right_Y			= 127*Remote.Right_Y			;
// 	Remote_Pack1.Left_X				= 127*Remote.Left_X				;
// 	Remote_Pack1.Left_Y				= 127*Remote.Left_Y				;
// 	Remote_Pack1.Wheel				= 127*Remote.Wheel				;
// 	Remote_Pack1.If_Remote_Connect	= Remote.If_Remote_Connect	;
// 	Remote_Pack1.Mode				= Remote.Mode				;
// //	Remote_Pack1.Pause				= Remote.Pause				;
// 	Remote_Pack1.Pause				= 0							;
// 	Remote_Pack1.Custom_L			= Remote.Custom_L			;
// 	Remote_Pack1.Custom_R			= Remote.Custom_R			;
// 	Remote_Pack1.Trigger			= Remote.Trigger			;

	Remote_Pack1.RC_Right_X			= 127*Remote_Control_Struct.RC_Right_X	;
	Remote_Pack1.RC_Right_Y			= 127*Remote_Control_Struct.RC_Right_Y;
	Remote_Pack1.RC_Left_X			= 127*Remote_Control_Struct.RC_Left_X;
	Remote_Pack1.RC_Left_Y			= 127*Remote_Control_Struct.RC_Left_Y;
	Remote_Pack1.RC_Side			= 127*Remote_Control_Struct.RC_Side;
	Remote_Pack1.If_Remote_Connect	= Remote_Control_Struct.If_Remote_Connect;
	Remote_Pack1.S1					=Remote_Control_Struct.S1;
	Remote_Pack1.S2					=Remote_Control_Struct.S2;
	
	CAN_Send_Data(&hfdcan3, 0x50, (uint8_t *)&Remote_Pack1);
}

void Gimbal_Send_Remote_Pack2(void)
{
//	Remote_Pack2.Mouse_Vx		= 127*Remote.Mouse_Vx		;
//	Remote_Pack2.Mouse_Vy		= 127*Remote.Mouse_Vy		;
//	Remote_Pack2.Mouse_Vz		= 127*Remote.Mouse_Vz		;
//	Remote_Pack2.Mouse_L		= Remote.Mouse_L		;
//	Remote_Pack2.Mouse_M		= Remote.Mouse_M		;
//	Remote_Pack2.Mouse_R		= Remote.Mouse_R		;
//	Remote_Pack2.Keyboard_W		= Remote.Keyboard_W		;
//	Remote_Pack2.Keyboard_A		= Remote.Keyboard_A		;
//	Remote_Pack2.Keyboard_S		= Remote.Keyboard_S		;
//	Remote_Pack2.Keyboard_D		= Remote.Keyboard_D		;
//	Remote_Pack2.Keyboard_Q		= Remote.Keyboard_Q		;
//	Remote_Pack2.Keyboard_E		= Remote.Keyboard_E		;
//	Remote_Pack2.Keyboard_Shift	= Remote.Keyboard_Shift	;
//	Remote_Pack2.Keyboard_Ctrl	= Remote.Keyboard_Ctrl	;
//	Remote_Pack2.Keyboard_R		= Remote.Keyboard_R		;
//	Remote_Pack2.Keyboard_F		= Remote.Keyboard_F		;
//	Remote_Pack2.Keyboard_G		= Remote.Keyboard_G		;
//	Remote_Pack2.Keyboard_Z		= Remote.Keyboard_Z		;
//	Remote_Pack2.Keyboard_X		= Remote.Keyboard_X		;
//	Remote_Pack2.Keyboard_C		= Remote.Keyboard_C		;
//	Remote_Pack2.Keyboard_V		= Remote.Keyboard_V		;
//	Remote_Pack2.Keyboard_B		= Remote.Keyboard_B		;
//	
//	CAN_Send_Data(&hfdcan3, 0x51, (uint8_t *)&Remote_Pack2);
}

void Gimbal_Send_attitude_Pack(void)
{
	memset(&Double_Board_Up_to_Down,0 ,sizeof(Double_Board_Up_to_Down));
	
    Double_Board_Up_to_Down.Yaw_Errx100                     = 100.0f*RoboControl_Struct.Yaw_Err;
    Double_Board_Up_to_Down.Gimbal_Yaw_TotalAnglex100     	= 100.0f*INS_Data_Internal.YawTotalAngle;
      
    Double_Board_Up_to_Down.Aim_If_Allow_Shoot        		= Aim_If_Allow_Shoot();
    Double_Board_Up_to_Down.Shoot_State             		= RoboControl_Struct.Shoot_State;
    Double_Board_Up_to_Down.Gimbal_State            		= RoboControl_Struct.Gimbal_State;
    
	Double_Board_Up_to_Down.Fric1_Speed						= Motor.Fric1.Speed_RPM * 7.0f / 31.0f ;
	Double_Board_Up_to_Down.Fric2_Speed						= Motor.Fric2.Speed_RPM * 7.0f / 31.0f ;
	
    Double_Board_Up_to_Down.Gimbal_Pitch       				= INS_Data_Internal.Pitch;
	Double_Board_Up_to_Down.operator_yaw_offsetx10      	= Aim_Send_Struct.operator_yaw_offset * 10.0f;
    Double_Board_Up_to_Down.operator_pitch_offsetx10 		= Aim_Send_Struct.operator_pitch_offset * 10.0f;
	
    CAN_Send_Data(&hfdcan3, 0x52, (uint8_t *)&Double_Board_Up_to_Down);
}

void Shoot_Update_VirHeat(void);
void Gimbal_Receive_Referee_Unpack(uint8_t *Data)
{
	Board_Commnuicate_Error_Ticker = 0;
	IF_Chassis_Online = 1;
	
	memcpy(&Double_Board_Down_to_UP_Referee, Data, sizeof(Double_Board_Down_to_UP_Referee_TypedefStruct));
	
	if(Robo_PowerHeatData.shooter_17mm_barrel_heat != Double_Board_Down_to_UP_Referee.Shoot_Qnow)
	{
		Robo_PowerHeatData.shooter_17mm_barrel_heat		=  Double_Board_Down_to_UP_Referee.Shoot_Qnow;
	}
	
	Robo_State.shooter_barrel_heat_limit				=  Double_Board_Down_to_UP_Referee.Shoot_Qmax;
	
	Robo_State.robot_id = Double_Board_Down_to_UP_Referee.robot_id;
	
    Robo_ShootData.initial_speed = Double_Board_Down_to_UP_Referee.initial_speedx100 /100.0f;     

	Shoot_Update_VirHeat();
}

void Gimbal_Receive_attitude_Unpack(uint8_t *Data)
{
	Board_Commnuicate_Error_Ticker = 0;
	IF_Chassis_Online = 1;

	memcpy(&Double_Board_Down_to_UP, Data, sizeof(Double_Board_Down_to_UP_TypedefStruct));
	
	RoboControl_Struct.Recover_from_ground_Flag = Double_Board_Down_to_UP.Recover_from_ground_Flag;
	
	Chassis_Control_Struct.Odometer_Gimbal_Vx = Double_Board_Down_to_UP.Chassis_Odometer_Vxx100 / 100.0f;
	Chassis_Control_Struct.Odometer_Gimbal_Vy = Double_Board_Down_to_UP.Chassis_Odometer_Vyx100 / 100.0f;

	Robo_State.shooter_barrel_cooling_value = Double_Board_Down_to_UP.Cooling_value;
}