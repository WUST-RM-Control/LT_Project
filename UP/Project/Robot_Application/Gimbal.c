//======================================================================
//                      Gimbal[云台控制]
//      驱动相应电机使云台陀螺仪达到RoboControl的Pitch，Yaw目标角度
//======================================================================

#include "Gimbal.h"
#include "RoboControl.h"
#include "INS.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Aim.h"
#include "vofa.h"
#include "USB_Driver.h"
#include "Motor_Compensation.h"
#include "Motor_Compensation_Data.h"
/*===| 云台控制数据结构体 |===*/
Gimbal_Control_StructTypeDef Gimbal_Control_Struct;

void Gimbal_Task(void *argument)
{
    /*===| 云台PID参数初始化 |===*/
	
	/*===| 正常模式下 |===*/
	PID_Init(&Gimbal_Control_Struct.Pitch_Angle_PID_Struct, 	10,  0, 50,  0,  0,    100);
    PID_Init(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct, 	800, 3, 0,    150,  500, 25000);	
	
	//没锁yaw轴
	PID_Init(&Gimbal_Control_Struct.Yaw_Angle_PID_Struct,   	20,  0, 40,  0,  0,    600);
	PID_Init(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct,   	500, 1, 0,    150, 2000, 25000);

	//锁yaw轴
	PID_Init(&Gimbal_Control_Struct.Yaw_Angle_Lock_PID_Struct,   5,  0, 50,  0,  0,    50);
    PID_Init(&Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct,   500,  1, 0,   100, 1500, 25000);

	
	/*===| 电机补偿校准 |===*/
//    Motor_Compensation_Task();	
    
//    FLASH_ErasePage(100);
//    FLASH_ErasePage(101);
//    FLASH_ErasePage(102);
//    FLASH_ErasePage(103);
//    for(uint16_t i = 0; i<1024; i++)
//    {
//        FLASH_programword(0x08000000 + 2048*(100+128) + 8*i, ((uint64_t *)(Config.Motor_Yaw_Cal_Data))[i]);
//    }

    for(uint16_t i = 0; i<1024; i++)
    {
        memcpy(&((uint64_t *)(Config.Motor_Yaw_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048*(100+128) + 8*i), 8);
    }
 
//    FLASH_ErasePage(104);
//    FLASH_ErasePage(105);
//    FLASH_ErasePage(106);
//    FLASH_ErasePage(107);
//    for(uint16_t i = 0; i<1024; i++)
//    {
//        FLASH_programword(0x08000000 + 2048*(104+128) + 8*i, ((uint64_t *)(Config.Motor_Pitch_Cal_Data))[i]);
//    }
	
    for(uint16_t i = 0; i<1024; i++)
    {
        memcpy(&((uint64_t *)(Config.Motor_Pitch_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048*(104+128) + 8*i), 8);
    }
    
    for(;;)
    {
        Gimbal_Control_Struct.Gimbal_State = RoboControl_Struct.Gimbal_State;
        
        /*===| 得到Yaw与底盘正方向的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Yaw_Err = Motor.Yaw.Angle - Gimbal_Median_Yaw;
        if(Gimbal_Control_Struct.Yaw_Err >= 180) Gimbal_Control_Struct.Yaw_Err = 360 - Gimbal_Control_Struct.Yaw_Err;
        if(Gimbal_Control_Struct.Yaw_Err <= -180) Gimbal_Control_Struct.Yaw_Err = 360 + Gimbal_Control_Struct.Yaw_Err;
        RoboControl_Struct.Yaw_Err = Gimbal_Control_Struct.Yaw_Err;	
        
        /*===| 得到自瞄位置与当前位置的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Vision_Yaw_Err = INS_Data_Internal.Yaw - Aim_Receive_Struct.yaw;
        if(Gimbal_Control_Struct.Vision_Yaw_Err > 180.0f) Gimbal_Control_Struct.Vision_Yaw_Err = Gimbal_Control_Struct.Vision_Yaw_Err - 360;
        else if(Gimbal_Control_Struct.Vision_Yaw_Err < -180.0f) Gimbal_Control_Struct.Vision_Yaw_Err = 360 + Gimbal_Control_Struct.Vision_Yaw_Err;
        
		if(RoboControl_Struct.Gimbal_State == Gimbal_State_Aim && Aim_Receive_Struct.appear)
        {
            RoboControl_Struct.Robo_Target_Gimbal_Yaw   = INS_Data_Internal.YawTotalAngle - Gimbal_Control_Struct.Vision_Yaw_Err;
            RoboControl_Struct.Robo_Target_Gimbal_Pitch = Aim_Receive_Struct.pitch;   
        }
                           
        /*===| 得到当前Pitch和Yaw数据 |===*/
        Gimbal_Control_Struct.Pitch_Feedback  =  INS_Data_Internal.Pitch;		
		Gimbal_Control_Struct.Yaw_Feedback = INS_Data_Internal.YawTotalAngle;
        
        /*===| 得到云台控制参数 ===*/
        Gimbal_Control_Struct.Pitch_Target = RoboControl_Struct.Robo_Target_Gimbal_Pitch;  
        Gimbal_Control_Struct.Yaw_Target = RoboControl_Struct.Robo_Target_Gimbal_Yaw; 
                
        /*===| 串级PID串级计算得到Yaw电机的电流大小 |===*/
		Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output    = 0;
		Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output      = 0;
		Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output = 0;
		/*===| 正常模式下 |===*/
		if(RoboControl_Struct.Recover_from_ground_Flag == 0) 
		{
			PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Angle_PID_Struct, Gimbal_Control_Struct.Yaw_Target,                  Gimbal_Control_Struct.Yaw_Feedback);
			PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct, Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output, INS_Data_Internal.Yaw_Speed);
		}
		/*===| 倒地自起模式下 |===*/
		else if(RoboControl_Struct.Recover_from_ground_Flag == 1)
		{
            float BestErr = Caculate_Included_Angle(Gimbal_Median_Yaw, Motor.Yaw.Total_Angle);
            if(fabsf(BestErr) > 90) BestErr = Caculate_Included_Angle(Gimbal_Median_Yaw+180, Motor.Yaw.Total_Angle);
			PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Angle_Lock_PID_Struct, 0,  -BestErr);
			PID_Position_Calculate(&Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct, Gimbal_Control_Struct.Yaw_Angle_Lock_PID_Struct.Output, -Motor.Yaw.Total_Angle_Speed_RPM);
            RoboControl_Struct.Robo_Target_Gimbal_Yaw = INS_Data_Internal.YawTotalAngle;
        }
                                                                                                                		
        /*===| 串级PID串级计算得到Pitch电机的电流大小 |===*/
        PID_Position_Calculate(&Gimbal_Control_Struct.Pitch_Angle_PID_Struct, Gimbal_Control_Struct.Pitch_Target,                  		Gimbal_Control_Struct.Pitch_Feedback);
        PID_Position_Calculate(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct, Gimbal_Control_Struct.Pitch_Angle_PID_Struct.Output, 		INS_Data_Internal.Gyro[1]*PI_to_Degree/6.0f);
        
        /*===| 限幅 |===*/		//-25000 ~ 25000是最大范围，若还出现问题则缩小范围
        Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output    = Motor_Compensation_Get_Data(Motor.Pitch.Angle, Motor.Pitch.Total_Angle_Speed_RPM, &Pitch_Compensation_Config)+ RoboControl_Struct.Smooth_Start_K*-Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output;
        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output      = Motor_Compensation_Get_Data(Motor.Yaw.Total_Angle, Motor.Yaw.Total_Angle_Speed_RPM, &Yaw_Compensation_Config) - RoboControl_Struct.Smooth_Start_K*Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output;
        Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output = Motor_Compensation_Get_Data(Motor.Yaw.Total_Angle, Motor.Yaw.Total_Angle_Speed_RPM, &Yaw_Compensation_Config) - RoboControl_Struct.Smooth_Start_K*Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output;
//        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output      =  - RoboControl_Struct.Smooth_Start_K*Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output;
//        Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output =  - RoboControl_Struct.Smooth_Start_K*Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output;
        Limit_float(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output    ,25000,-25000);
        Limit_float(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output      ,25000,-25000);
        Limit_float(&Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output ,25000,-25000);
        
        /*===| 发送电压参数 |===*/
        if(RoboControl_Struct.Robo_Enable && INS_Data_Internal.If_INS_Init) 
        {
			Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN,Gimbal_Pitch_Send_CAN_ID,Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output,0,0,0);		
			
			/*===| Yaw正常模式下 |===*/
			if(RoboControl_Struct.Recover_from_ground_Flag == 0)
			{
				Motor_DJI_SendCurrent(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID,0,Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output,0,0);		
			}
			/*===| 倒地自起下云台yaw轴锁住 |===*/
			else if(RoboControl_Struct.Recover_from_ground_Flag == 1)
			{
                Motor_DJI_SendCurrent(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID, 0,Gimbal_Control_Struct.Yaw_Speed_Lock_PID_Struct.Output,0,0);									
			}
        }
        else 
        {
            Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN, Gimbal_Pitch_Send_CAN_ID, 0, 0 ,0 ,0);
            Motor_DJI_SendCurrent(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID, 0, 0 ,0 ,0);
        }
        osDelay(1);
    }
}
