//========================================================
//                      Aim[自瞄]
//      自瞄的通信与处理
//========================================================

#include "Aim.h"
#include "INS.h"
#include "RoboControl.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Referee_Unpack.h"

uint8_t If_PC_Online;
uint16_t If_PC_Online_Ticker;

#define Bullet_Speed_Average_Count_Number 5
float Bullet_Speed_Average = 20.0f;
float Bullet_Speed[Bullet_Speed_Average_Count_Number];
float Bullet_Speed_Last;

uint32_t Aim_DWT_Count;
INS_t INS;

Aim_Receive_StructTypedef Aim_Receive_Struct = {0};
Aim_Send_StructTypedef Aim_Send_Struct = {.operator_yaw_offset = 0,.operator_pitch_offset = -1.8};

void Aim_Task(void const * argument)
{
    for(uint8_t i = 0; i < Bullet_Speed_Average_Count_Number; i++) {Bullet_Speed[i] = Bullet_Speed_Average; }
    
    for(;;)
    { 
        /*===| 统计平均弹速 |===*/
        Get_Average_Bullet_Speed();
        
        Aim_Send_Struct.cmd_ID = 0x02;
        //自瞄坐标系定义： +x:前，+y:左，+z：上，+yaw 左，+pitch 下。+roll 右倾，旋转顺序ZYX,自瞄发来的还是 +yaw:左 ，+pitch 上
        Aim_Send_Struct.bullet_speed = Bullet_Speed_Average;
        if(Robo_State.robot_id < 100) Aim_Send_Struct.detect_color = 1;
        else Aim_Send_Struct.detect_color = 0;
        Aim_Send_Struct.yaw = INS_Data_Internal.Yaw;
        Aim_Send_Struct.roll = -INS_Data_Internal.Roll;
        Aim_Send_Struct.pitch = -INS_Data_Internal.Pitch; //pitch向下为正
        Aim_Send_Struct.yaw_vel = INS_Data_Internal.Yaw_Speed;
        
        //将云台坐标系速度转换到IMU静态坐标系
//        Aim_Send_Struct.v_x = sinf(-INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vx - cosf(-INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vy;
//        Aim_Send_Struct.v_y = sinf(-INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vy + cosf(-INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vx;
		Aim_Send_Struct.v_x = sinf(INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vx + cosf(INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vy;
		Aim_Send_Struct.v_y = sinf(INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vy - cosf(INS_Data_Internal.Yaw * Degree_to_PI) * Chassis_Control_Struct.Odometer_Gimbal_Vx;
		Aim_Send_Struct.v_z = INS.MotionAccel_n[2] * DWT_GetDeltaT(&Aim_DWT_Count);
		
        Aim_Send_Struct.time_stamp_send_micro = DWT->CYCCNT/170.0f;//将发送时间戳转换为微秒
        
        USB_Send((uint8_t *)&Aim_Send_Struct, sizeof(Aim_Send_StructTypedef));
        
        osDelay(1);
    }
}


void Get_Average_Bullet_Speed(void)
{
    if(Robo_ShootData.initial_speed != Bullet_Speed_Last && Robo_ShootData.initial_speed > 8.0f && Robo_ShootData.initial_speed < 25.0)
    {
        for(uint8_t i = Bullet_Speed_Average_Count_Number - 1; i > 0; i--)
        { 
            Bullet_Speed[i] = Bullet_Speed[i-1];
        }
        Bullet_Speed[0] = Robo_ShootData.initial_speed;
        
        float Speed_Sum = 0;
        for(uint8_t i = 0; i < Bullet_Speed_Average_Count_Number; i++)
        { 
            Speed_Sum += Bullet_Speed[i];
        }
        
        Bullet_Speed_Average = Speed_Sum / (float)Bullet_Speed_Average_Count_Number;
    }
    Bullet_Speed_Last = Robo_ShootData.initial_speed;
}

void Aim_Get_Data_CallBack(uint8_t *Data)
{
    Aim_Send_Struct.time_stamp_receive_micro = DWT->CYCCNT/170.0f;					//将接收时间戳转换为微秒
    If_PC_Online_Ticker = 0;
    If_PC_Online = 1;
    memcpy((uint8_t *)&Aim_Receive_Struct, Data, sizeof(Aim_Receive_StructTypedef));
    Aim_Send_Struct.time_stamp_pc = Aim_Receive_Struct.time_stamp;
}

uint8_t Aim_If_Allow_Shoot(void)
{
    if(   (fabs(Aim_Receive_Struct.target_yaw - INS_Data_Internal.Yaw) < Aim_Receive_Struct.enable_yaw_diff)
        &&(fabs(Aim_Receive_Struct.target_pitch - INS_Data_Internal.Pitch) < Aim_Receive_Struct.enable_pitch_diff)
        ) return 1;
    else return 0;
}

void Aim_Add_Operator_Offset(float Yaw, float Pitch)
{
    Aim_Send_Struct.operator_yaw_offset += Yaw;
    Aim_Send_Struct.operator_pitch_offset += Pitch;
}

