    //====================================================================================
//                           RoboControl[机器状态控制]
//      根据操控意愿把遥控器数据处理成机器人
//      整体[RoboEnable,Vx,Vy,Wz,Yaw,Pitch等],各个模块[Gimbal,Shoot,Chassis等]的状态,
//      各个模块再根据状态执行相应的功能
//      其中包括状态设置，冲突处理，错误检测与处理等    
//
//      机器人状态结构体：RoboControl_Struct
//====================================================================================

#include "RoboControl.h"
#include "INS.h"
#include "Buzzer.h"
#include "Remote_Control.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DrEmpower.h"
#include "Aim.h"
#include "Function.h"
#include "Chassis.h"

/*===| 机器人整体状态数据结构体 |===*/
RoboControl_StructTypeDef RoboControl_Struct;

extern uint8_t IF_Chassis_Online;
extern float Shoot_Fric_First_Left_Speed;
extern float Shoot_Fric_First_Right_Speed;

uint32_t Robo_Task_DWT_Count;
float Robo_Task_Dt;

int16_t Mouse_Vx_Limit;

/*===| 计算Wz的PID结构体 |===*/
PID_Struct_TypeDef RoboGimbal_Wz_PID_Struct;

void Robo_Task(void *argument)
{
    /*===| Yaw参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Gimbal_Yaw = 0;
    
    /*===| Pitch参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Gimbal_Pitch = 0;
    
    /*===| Wz_PID初始化 |===*/
    PID_Init(&RoboGimbal_Wz_PID_Struct, 0.15, 0, 10, 0, 0, Robo_Wz_MaxSpeed);
    
    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Chassis_Speed_Level = 3;
    RoboControl_Struct.SuperCap_State = 0;
    Referee_Data_Init();
    
    for(;;)
    {
		Robo_Task_Dt = DWT_GetDeltaT(&Robo_Task_DWT_Count);
        
		/*===| 如果遥控器连接 |===*/
        if(Remote.If_Remote_Connect)
        {
            /*===| 选择控制模式：摇杆|键鼠|自定义控制器 |===*/
            if(Remote.Mode != Remote_Mode_S && RoboControl_Struct.Controler != Customer) RoboControl_Struct.Controler = Joystick;
            else if(Remote.Mode == Remote_Mode_S && RoboControl_Struct.Controler != Customer) RoboControl_Struct.Controler = KeyboardMouse;   
            
            /*===| 判断是否停机 |===*/     
            if(Remote.Mode == Remote_Mode_C && RoboControl_Struct.Robo_Enable == 1) Robo_Stop();   
            else if(Remote.Mode != Remote_Mode_C && RoboControl_Struct.Robo_Enable == 0) Robo_Restart(); 
        }
        else
        {
            Robo_Stop(); 
        }
        
        /*===| 遥控方式 |===*/
        /*===| 遥控器摇杆控制 |===*/
        if(RoboControl_Struct.Controler == Joystick)
        {
            RemoteControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
//                if(Remote_ReleaseSingle_Pause) RoboControl_Struct.Controler = Customer;
                RemoteControl_Bool();
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0;
            }
        }
        /*===| 遥控器键鼠控制 |===*/
        else if(RoboControl_Struct.Controler == KeyboardMouse)
        {
            KeyControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
//                if(Remote_ReleaseSingle_B) RoboControl_Struct.Controler = Customer; 
                KeyControl_Bool();
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0; 
            }
        }
        /*===| 自定义控制器控制 |===*/
        else if(RoboControl_Struct.Controler == Customer)
        {
            RemoteControl_Float();
            KeyControl_Float();
            if(Remote.If_Remote_Data_New == 1 && Remote_Last.If_Remote_Data_New == 1)
            {
                if(Remote_ReleaseSingle_Pause) RoboControl_Struct.Controler = Joystick;
                if(Remote_ReleaseSingle_B) RoboControl_Struct.Controler = KeyboardMouse;
                Remote.If_Remote_Data_New = 0; 
                Remote_Last.If_Remote_Data_New =0;
            }
        }
	
		/*===| Pitch控制参数限幅 |===*/        
		float Pitch_Limit_Up = INS_Data_Internal.Pitch + (Motor.Pitch.Angle-(Pitch_Angle_Limit_Max));
		float Pitch_Limit_Down = INS_Data_Internal.Pitch + (Motor.Pitch.Angle-(Pitch_Angle_Limit_Min));
		static float Pitch_Limit_Up_Set = 0,Pitch_Limit_Down_Set = 0;
		Acc_and_Dec(Pitch_Limit_Up, &Pitch_Limit_Up_Set, 5, 5, Robo_Task_Dt);
		Acc_and_Dec(Pitch_Limit_Down, &Pitch_Limit_Down_Set, 5, 5, Robo_Task_Dt);
        Limit_float(&RoboControl_Struct.Robo_Target_Gimbal_Pitch, Pitch_Limit_Up_Set, Pitch_Limit_Down_Set);

		
        /*===| 根据底盘运动状态得到Wz |===*/   
//        Get_Chassis_Wz();
        
        /*===| 控制参数低通滤波 |===*/         
        Control_Filter();
        
        /*===| 平滑启停 |===*/    
        if(RoboControl_Struct.Recover_from_ground_Flag == 1) RoboControl_Struct.Smooth_Start_K = 1;
        if(RoboControl_Struct.Robo_Enable)
            RoboControl_Struct.Smooth_Start_K += 0.001f;
        else 
            RoboControl_Struct.Smooth_Start_K -= 0.001f;
        Limit_float(&RoboControl_Struct.Smooth_Start_K, 1, 0);
			 
        osDelay(1);
    }
}

/**
 * @brief 根据底盘运动状态得到Wz
 */
void Get_Chassis_Wz(void)
{
    /*===| 底盘关闭 |===*/
    if(RoboControl_Struct.Chassis_State == Chassis_OFF)
    {
        RoboControl_Struct.Robo_Target_Wz = 0;
    }
    /*===| 底盘静止 |===*/
    else if(RoboControl_Struct.Chassis_State == Chassis_STATIC)
    {
        RoboControl_Struct.Robo_Target_Wz = 0;
    }
    /*===| 底盘跟随模式 |===*/
    else if(RoboControl_Struct.Chassis_State == Chassis_FOLLOW)
    {
        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
        RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
    }
    /*===| 底盘侧向跟随模式 |===*/
    else if(RoboControl_Struct.Chassis_State == Chassis_F_SIDE)
    {
        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 2.0f * (RoboControl_Struct.Chassis_Follow_45_Direction_Flag - 0.5f) * 45, RoboControl_Struct.Yaw_Err);
        RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
    }
    /*===| 底盘小陀螺 |===*/
    else if(RoboControl_Struct.Chassis_State == Chassis_SPIN)
    {
        /*===| 小陀螺得到Wz,每次切换正反转 |===*/
        if(RoboControl_Struct.SPIN_Direction_Flag)
        {
            RoboControl_Struct.Robo_Target_Wz = Robo_Wz_MaxSpeed;
        }                                                         
        else 
        {
            RoboControl_Struct.Robo_Target_Wz = -Robo_Wz_MaxSpeed;
        }
        /*===| 根据档位限制旋转速度，遥控器控制状态不限制 |===*/
        RoboControl_Struct.Robo_Target_Wz = RoboControl_Struct.Robo_Target_Wz * (1.0f + RoboControl_Struct.Chassis_Speed_Level) / 6.0f;
            
        Motor.Yaw.Round = 0;
    }
    /*===| 冲刺模式 |===*/
    else if(RoboControl_Struct.Chassis_State == Chassis_DASH)
    {
        /*===| 如果底盘没有回正，则等待底盘回到正方向 |===*/
        if(RoboControl_Struct.Yaw_Err > 5.0f || RoboControl_Struct.Yaw_Err < -5.0f) 
        {
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
            PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
            RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
        }
        else
        {
            RoboControl_Struct.Chassis_State = Chassis_DASH;
            RoboControl_Struct.Robo_Target_Wz = 0;
        }
    }  
}

/**
 * @brief 输入参数低通滤波
 */
void Control_Filter(void)
{
    RoboControl_Struct.Robo_Target_Vx    = 0.4f * RoboControl_Struct.Robo_Target_Vx    + (1 - 0.4f)  * RoboControl_Struct.Robo_Target_Vx_Last   ;
    RoboControl_Struct.Robo_Target_Vy    = 0.4f * RoboControl_Struct.Robo_Target_Vy    + (1 - 0.4f)  * RoboControl_Struct.Robo_Target_Vy_Last   ;
    RoboControl_Struct.Robo_Target_Wz    = 0.9f * RoboControl_Struct.Robo_Target_Wz    + (1 - 0.9f)  * RoboControl_Struct.Robo_Target_Wz_Last   ;
	
    RoboControl_Struct.Robo_Target_Gimbal_Yaw   = 0.6f * RoboControl_Struct.Robo_Target_Gimbal_Yaw  + (1 - 0.6f)  * RoboControl_Struct.Robo_Target_Gimbal_Yaw_Last  ;
    RoboControl_Struct.Robo_Target_Gimbal_Pitch = 0.6f * RoboControl_Struct.Robo_Target_Gimbal_Pitch + (1 - 0.6f)  * RoboControl_Struct.Robo_Target_Gimbal_Pitch_Last;

    RoboControl_Struct.Robo_Target_Vx_Last    =  RoboControl_Struct.Robo_Target_Vx   ;
    RoboControl_Struct.Robo_Target_Vy_Last    =  RoboControl_Struct.Robo_Target_Vy   ;              
    RoboControl_Struct.Robo_Target_Wz_Last    =  RoboControl_Struct.Robo_Target_Wz   ;              
    RoboControl_Struct.Robo_Target_Gimbal_Yaw_Last   =   RoboControl_Struct.Robo_Target_Gimbal_Yaw;              
    RoboControl_Struct.Robo_Target_Gimbal_Pitch_Last =   RoboControl_Struct.Robo_Target_Gimbal_Pitch; 
}

/**
 * @brief 遥控器控制
 */
void RemoteControl_Float(void)
{
    /*===| 底盘和云台运动控制 |===*/
    static uint32_t Remote_DWT_Count;
    float Dt = DWT_GetDeltaT(&Remote_DWT_Count);
		
    if(Remote_Mode_N)
    {		
//		if(Remote.Wheel > -0.7f && Remote.Wheel < 0.7f)
//		{
//			/*===| 云台运动控制 |===*/
//			RoboControl_Struct.Robo_Target_Gimbal_Yaw   -= Dt * 300.0f * Remote.Right_X;
//			RoboControl_Struct.Robo_Target_Gimbal_Pitch += Dt * 150.0f * Remote.Right_Y;
//		}
					
		if(Remote.Wheel > -0.7f && Remote.Wheel < 0.7f)
		{
			/*===| 云台运动控制 |===*/
			RoboControl_Struct.Robo_Target_Gimbal_Pitch += Dt * 150.0f * Remote.Right_Y;
            RoboControl_Struct.Robo_Target_Gimbal_Yaw  -= Dt * 300.0f * Remote.Right_X;
		}	
    }
}
	
/**
 * @brief 遥控器控制
 *	1. 上左左：底盘静止；上左右：底盘跟随，静止标志位清零
 *	2. 上左下：底盘静止且云台调头->云台不动且底盘跟随
 *  3. 下右上：开自瞄  ；下右下：关自瞄
 *  4. Pause单击：开关摩擦轮，开摩擦再按扳机，单/多发弹
 */
void RemoteControl_Bool(void)
{
		/*===| 滑键滑至C模式关闭所有设备 |===*/     
        if(Remote.Mode == Remote_Mode_C && RoboControl_Struct.Robo_Enable == 1) Robo_Stop();
        
        /*===| 重启设备 |===*/     
        else if(Remote.Mode == Remote_Mode_N && RoboControl_Struct.Robo_Enable == 0) Robo_Restart();
    
		/*===| 侧滑轮往上进入自定义功能选项1，往下是选项2，取消摇杆控制移动，通过摇杆选择触发一次定义的八个功能 |===*/ 
//		if(Remote_Press_Pause)
//      {
//            /*===| 自定义功能-左摇杆向右 |===*/
//            if      (Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f)
//            {
//                
//            }
//            /*===| 自定义功能-左摇杆向左 |===*/
//            else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f)
//            {
//                
//            }
//            /*===| 自定义功能-左摇杆向上 |===*/
//            if      (Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f)
//            {

//            }
//            /*===| 自定义功能-左摇杆向下 |===*/
//            else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f)
//            {

//            }
//            /*===| 自定义功能-右摇杆向右 |===*/
//            if      (Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f)
//            {
//                
//            }
//            /*===| 自定义功能-右摇杆向左 |===*/
//            else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f)
//            {
//                
//            }
//            /*===| 自定义功能-右摇杆向上 |===*/
//            if      (Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f)
//            {
//                
//            }
//            /*===| 自定义功能-右摇杆向下 |===*/
//            else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f)
//            {
//                
//            }
//		}
//       
				
		if((RoboControl_Struct.Robo_Enable == 1) && Remote.Wheel > 0.7f)
        {
            /*===| 自定义功能-左摇杆向右 |===*/
            if (Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f)
            {
				/*===| 底盘跟随模式 |===*/
//                RoboControl_Struct.Chassis_State = Chassis_FOLLOW;	
				RoboControl_Struct.Chassis_Static_Flag = 0;				//仅跟随或静止时标志位清零
            }
            /*===| 自定义功能-左摇杆向左 |===*/
            else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f)
            {
				/*===| 底盘静止模式 |===*/
//                RoboControl_Struct.Chassis_State = Chassis_STATIC;
				RoboControl_Struct.Chassis_Static_Flag = 1;				//仅跟随或静止时标志位清零
            }
            /*===| 自定义功能-左摇杆向上 |===*/
            if      (Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f)
            {

            }
            /*===| 自定义功能-左摇杆向下 |===*/
            else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f)
            {	
				if(RoboControl_Struct.Chassis_Static_Flag == 0)
				{
					RoboControl_Struct.Chassis_Static_Flag = 1;
				}
				else if(RoboControl_Struct.Chassis_Static_Flag != 0)
				{
					RoboControl_Struct.Chassis_Static_Flag = 0;
				}
				
				//	底盘静止标志位，1为静止，0为跟随
				if(RoboControl_Struct.Chassis_Static_Flag == 1)
				{
					/*===| 标志位置1：底盘静止，云台调头180度 |===*/
					RoboControl_Struct.Chassis_Static_Flag = 1;
					RoboControl_Struct.Robo_Target_Gimbal_Yaw += 300;	
				}
				else if(RoboControl_Struct.Chassis_Static_Flag == 0)
				{
					/*===| 标志位置2：云台保持不动，底盘跟随 |===*/
					RoboControl_Struct.Robo_Target_Gimbal_Yaw = INS_Data_Internal.YawTotalAngle;				
					RoboControl_Struct.Chassis_Static_Flag = 0;
				}
            }
            /*===| 自定义功能-右摇杆向右 |===*/
            if      (Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f)
            {
                
            }
            /*===| 自定义功能-右摇杆向左 |===*/
            else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f)
            {
                
            }
            /*===| 自定义功能-右摇杆向上 |===*/
            if      (Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f)
            {
                 
            }
            /*===| 自定义功能-右摇杆向下 |===*/
            else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f)
            {
                
            }
        }

		else if(Remote.Wheel < -0.7f)
		{
			/*===| 自定义功能-左摇杆向右 |===*/
            if(Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f)
            {
				
            }
            /*===| 自定义功能-左摇杆向左 |===*/
            else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f)
            {
				
            }
            /*===| 自定义功能-左摇杆向上 |===*/
            if(Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f)
            {

            }
            /*===| 自定义功能-左摇杆向下 |===*/
            else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f)
            {

            }
            /*===| 自定义功能-滑轮下滑 右摇杆向右 |===*/
            if(Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f)
            {

            }
            /*===| 自定义功能-滑轮下滑 右摇杆向左 |===*/
            else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f)
            {
			
            }
            /*===| 自定义功能-右摇杆向上 |===*/
            if(Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f)
            {
                /*===| 开自瞄 |===*/
                RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);

            }
            /*===| 自定义功能-右摇杆向下 |===*/
            else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f)
            {
                /*===| 关自瞄 |===*/
                RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
                RoboControl_Struct.Robo_Target_Gimbal_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
                RoboControl_Struct.Robo_Target_Gimbal_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
            }
		}
		
		/*===| 发射状态控制 |===*/
        /*===| 右键控制发射状态 默认状态关闭 单击准备 再单击关闭*/
		if (RoboControl_Struct.Robo_Enable == 1 && Remote_ReleaseSingle_Pause)
        {
            if(RoboControl_Struct.Shoot_State == Shoot_State_Off)
            {
                RoboControl_Struct.Shoot_State = Shoot_State_Ready;
            }
			 else if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
            {
                RoboControl_Struct.Shoot_State = Shoot_State_Off;
            }
        }    

		//正常发弹
        if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
        { 
            /*===| 扳机键长按 连发 |===*/
            if(Remote_Press_Trigger)
            {
                if(RoboControl_Struct.Gimbal_State == Gimbal_State_Aim)
                {
                    if(Aim_If_Allow_Shoot()) RoboControl_Struct.Shoot_State = Shoot_State_Continue;  
                    else RoboControl_Struct.Shoot_State = Shoot_State_Ready;
                }
                else
                {
                    RoboControl_Struct.Shoot_State = Shoot_State_Continue;
                }
            } 
            /*===| 扳机键松开回到准备状态 |===*/
            else if(Remote_Release_Trigger)
            {
                RoboControl_Struct.Shoot_State = Shoot_State_Ready;
            }
        }
}


/**
 * @brief 键盘控制
 */
void KeyControl_Float(void)
{
    static uint32_t Key_DWT_Count;
    float Dt = DWT_GetDeltaT(&Key_DWT_Count);
    

    /*===| 云台运动控制 |===*/
    if(RoboControl_Struct.Gimbal_State == Gimbal_State_Slow)
    {
        RoboControl_Struct.Robo_Target_Gimbal_Yaw    -= Dt * 30.0f * Remote.Mouse_Vx / 32.0f;
        RoboControl_Struct.Robo_Target_Gimbal_Pitch  += Dt * 30.0f * Remote.Mouse_Vy / 32.0f;
    }
    else
    {
        /*===| 云台运动控制 |===*/
        RoboControl_Struct.Robo_Target_Gimbal_Pitch += Dt * 80.0f * Remote.Mouse_Vy / 32.0f;
        RoboControl_Struct.Robo_Target_Gimbal_Yaw    -= Dt * 100.0f * Remote.Mouse_Vx / 32.0f;	
    }
}

/**
 * @brief 键盘控制
 *	1. Ctrl组合键：W（+）、S（-）云台pitch偏置 A（+）、D（-）云台yaw偏置 Z云台调头
 *	2. 鼠标右键：按住开自瞄，松开即关
 *  3. R键：开关摩擦轮
 *  4. 单击Z键：底盘静止标志位清0
 *  5. X键：失能  G键：使能
 */
void KeyControl_Bool(void) 
{	
	static uint8_t tick = 0;
			
		/*===| Ctrl组合键 |===*/
		if(Remote_Press_Ctrl)
		{
			/*===| Ctrl+W/A/S/D 按下一次增减0.2 |===*/
			/*===| Ctrl+W：pitch + 0.2 || Ctrl+S：pitch - 0.2 |===*/
			// if(Remote_PressSingle_W) 	  Aim_Send_Struct.operator_pitch_offset += 0.2f;
			// else if(Remote_PressSingle_S) Aim_Send_Struct.operator_pitch_offset -= 0.2f;
            if(Remote_PressSingle_W) 	  {Shoot_Fric_First_Left_Speed += 0.5f;Shoot_Fric_First_Right_Speed += 0.5f;}
			else if(Remote_PressSingle_S) {Shoot_Fric_First_Left_Speed -= 0.5f;Shoot_Fric_First_Right_Speed -= 0.5f;}
			
			/*===| Ctrl+A：yaw + 0.2   || Ctrl+D：yaw - 0.2 |===*/
			else if(Remote_PressSingle_A) Aim_Send_Struct.operator_yaw_offset += 0.2f;		
			else if(Remote_PressSingle_D) Aim_Send_Struct.operator_yaw_offset -= 0.2f;	
			
			

		// 	/*===| Ctrl+Z 底盘静止标志位0-1-2-1-2- |===*/
		// 	else if(Remote_PressSingle_Z)
		// 	{
		// 		//	原本为0 赋值为1（初始），原本为1 赋值为2，原本为2 赋值为1
		// 		if(RoboControl_Struct.Chassis_Static_Flag == 1)	RoboControl_Struct.Chassis_Static_Flag = 2;
		// 		else if(RoboControl_Struct.Chassis_Static_Flag != 1) RoboControl_Struct.Chassis_Static_Flag = 1;
				
		// 		//	底盘静止标志位，1为静止，2为跟随
		// 		if(RoboControl_Struct.Chassis_Static_Flag == 1)
		// 		{
		// 			/*===| 标志位置1：底盘静止，云台调头180度 |===*/
		// 			RoboControl_Struct.Chassis_Static_Flag = 1;
		// 			RoboControl_Struct.Robo_Target_Gimbal_Yaw += 300;
		// 		}
		// 		else if(RoboControl_Struct.Chassis_Static_Flag == 2)
		// 		{
		// 			/*===| 标志位置2：云台保持不动，底盘跟随 |===*/
		// 			RoboControl_Struct.Robo_Target_Gimbal_Yaw = INS_Data_Internal.YawTotalAngle;				
		// 			RoboControl_Struct.Chassis_Static_Flag = 2;
		// 		}
		// 	}
		}
		
		/*===| 按住右键自瞄 |===*/
		if(Remote_PressSingle_Mouse_R)   	//0->1
		{
			RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
			Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
		}
		if(Remote_Release_Mouse_R && RoboControl_Struct.Gimbal_State == Gimbal_State_Aim) 	//1->0
		{
			RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
			RoboControl_Struct.Robo_Target_Gimbal_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
			RoboControl_Struct.Robo_Target_Gimbal_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
			Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
		}
		
		/*===| 单按R开关摩擦轮 |===
        ！Remote_Press_Ctrl防止与组合键冲突
        */
		if(Remote_PressSingle_R && !Remote_Press_Ctrl)
		{
			if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
			{
				RoboControl_Struct.Shoot_State = Shoot_State_Off;
				RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
			}
			else
			{
				RoboControl_Struct.Shoot_State = Shoot_State_Ready;
			}
		}	
		
		if(RoboControl_Struct.Shoot_State != Shoot_State_Off)
		{
			/*===| 左键单发 |===*/
			if (Remote_Press_Mouse_L && Remote_Release_Ctrl)		//长按连发，松手则进入ready状态=>单击则单发
			{
				if ((RoboControl_Struct.Gimbal_State != Gimbal_State_Aim) || (RoboControl_Struct.Gimbal_State == Gimbal_State_Aim && Aim_If_Allow_Shoot()))
				{
					RoboControl_Struct.Shoot_State = Shoot_State_Continue;
				}
				else
					RoboControl_Struct.Shoot_State = Shoot_State_Ready;
			}
            else if(Remote_PressSingle_Mouse_L && Remote_Press_Ctrl)
            {
                Shoot_Single(1);
            }
			else
			RoboControl_Struct.Shoot_State = Shoot_State_Ready;
		}
		        
		// /*===| 下板的单击Z 底盘静止、跟随状态切换 |===*/
		// if(!Remote_Press_Ctrl && Remote_PressSingle_Z)
		// {
		// 	RoboControl_Struct.Chassis_Static_Flag = 0;				//仅跟随或静止时标志位清零
		// } 
		
		/*===| 按X失能 |===*/
		if(Remote_PressSingle_X) 
		{	
			//	云台失能标志位，0为失能，1为使能
			RoboControl_Struct.Gimbal_State_Flag = 0;
            RoboControl_Struct.Robo_Target_Gimbal_Yaw = INS_Data_Internal.YawTotalAngle;
		}
		
		/*===| 按G重启 |===*/
		if(Remote_PressSingle_G)
		{
			//	云台失能标志位，0为失能，1为使能
			RoboControl_Struct.Gimbal_State_Flag = 1;
		} 
}
		

/**
 * @brief 关闭机器人
 */
void Robo_Stop(void)
{	
    RoboControl_Struct.Robo_Enable = 0;

    RoboControl_Struct.Robo_Target_Vx = 0;
    RoboControl_Struct.Robo_Target_Vy = 0;
    RoboControl_Struct.Robo_Target_Wz = 0;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.SuperCap_State = 0;
    RoboControl_Struct.Chassis_State = Chassis_OFF;
}

/**
 * @brief 重启机器人
 */
void Robo_Restart(void)
{
    /*===| 重启后Pitch回到0，Yaw更新到当前位置 |===*/
    RoboControl_Struct.Robo_Target_Gimbal_Yaw = INS_Data_Internal.YawTotalAngle;
    RoboControl_Struct.Robo_Target_Gimbal_Pitch = 0;
	
	/*===| 无关变量赋值 |===*/
	RoboControl_Struct.Gimbal_State_Flag = 2;
	RoboControl_Struct.Chassis_Static_Flag = 0;
	
	//圈数清零
	Motor.Yaw.Round = 0;
	
	//偏置清零
//	RoboControl_Struct.operator_pitch_offset = 0;
//	RoboControl_Struct.operator_yaw_offset	 = 0;
    
    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Smooth_Start_K = 0;
	
    RoboControl_Struct.Robo_Enable = 1;
}
