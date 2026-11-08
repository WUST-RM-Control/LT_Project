#include "ErrorHandle.h"
#include "RoboControl.h"
#include "INS.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Chassis.h"
#include "INS_Chassis_Data.h"
#include "Aim.h"
#include "Remote_Control.h"

/*===| 云台控制数据结构体 |===*/
Gimbal_Control_StructTypeDef Gimbal_Control_Struct;


//CAN离线看门狗
extern uint16_t CAN1_Err_Ticker;
extern uint16_t CAN2_Err_Ticker;
extern uint16_t CAN3_Err_Ticker;

void ErrorHandle_Task(void *argument)
{
    for(;;)
    {
		/*===| 遥控器断连处理 |===*/
        Remote_Control_Struct.Remote_GetData_Ticker++;

        if(Remote_Control_Struct.Remote_GetData_Ticker >= 500)
        {
            Remote_Control_Struct.If_Remote_Connect = 0;
            Remote_Control_Struct.Remote_GetData_Ticker = 500;
        }

        if(Remote_Control_Struct.If_Remote_Connect == 0)
        {
            Robo_Stop();
        }
		
		
		/*===| CAN错误处理 |===*/
        CAN1_Err_Ticker++;
        CAN2_Err_Ticker++;
        CAN3_Err_Ticker++;
        if(CAN1_Err_Ticker > 100)
        {
            MX_FDCAN1_Init();
            HAL_FDCAN_Start(&hfdcan1);
            CAN_Filter_Init(&hfdcan1);
            CAN1_Err_Ticker = 0;
        }
        if(CAN2_Err_Ticker > 100)
        {
            MX_FDCAN2_Init();
            HAL_FDCAN_Start(&hfdcan2);
            CAN_Filter_Init(&hfdcan2);
            CAN2_Err_Ticker = 0;
        }
        if(CAN3_Err_Ticker > 100)
        {
            MX_FDCAN3_Init();
            HAL_FDCAN_Start(&hfdcan3);
            CAN_Filter_Init(&hfdcan3);
            CAN3_Err_Ticker = 0;
        }

		/*===| 轮电机离线检测 |===*/
		Motor.Wheel_Motor1.Ticker++; 
		if(Motor.Wheel_Motor1.Ticker >= 100) Motor.Wheel_Motor1.If_Online = 0; 
		if(Motor.Wheel_Motor1.Ticker >  110) Motor.Wheel_Motor1.Ticker = 100;
		
        Motor.Wheel_Motor2.Ticker++; 
		if(Motor.Wheel_Motor2.Ticker >= 100) Motor.Wheel_Motor2.If_Online = 0; 
		if(Motor.Wheel_Motor2.Ticker >  110) Motor.Wheel_Motor2.Ticker = 100;
		
        if(Motor.Wheel_Motor1.If_Online == 0 || Motor.Wheel_Motor2.If_Online == 0)
        {
            Robo_Stop();
        }

		/*===| 关节电机离线检测 |===*/
		Motor.Joint1.Ticker++; 
		if(Motor.Joint1.Ticker >= 100) Motor.Joint1.If_Online = 0; 
		if(Motor.Joint1.Ticker >  110) Motor.Joint1.Ticker = 100;
		
        Motor.Joint2.Ticker++; 
		if(Motor.Joint2.Ticker >= 100) Motor.Joint2.If_Online = 0; 
		if(Motor.Joint2.Ticker >  110) Motor.Joint2.Ticker = 100;
		
        Motor.Joint3.Ticker++; 
		if(Motor.Joint3.Ticker >= 100) Motor.Joint3.If_Online = 0; 
		if(Motor.Joint3.Ticker >  110) Motor.Joint3.Ticker = 100;
		
        Motor.Joint4.Ticker++; 
		if(Motor.Joint4.Ticker >= 100) Motor.Joint4.If_Online = 0; 
		if(Motor.Joint4.Ticker >  110) Motor.Joint4.Ticker = 100;
        
		/*===| 关节电机离线断连处理 |===*/
        if(Motor.Joint1.Ticker == 110)      
        {
            Motor_DM_CMD_ClearErr(&Chassis_JointMotor1_CAN, Chassis_JointMotor1_Send_CAN_ID);
            osDelay(1);
            Motor_DM_CMD_Enable(&Chassis_JointMotor1_CAN,Chassis_JointMotor1_Send_CAN_ID);
			osDelay(1);
        }
		if(Motor.Joint2.Ticker == 110)      
        {
            Motor_DM_CMD_ClearErr(&Chassis_JointMotor2_CAN, Chassis_JointMotor2_Send_CAN_ID);
            osDelay(1);
            Motor_DM_CMD_Enable(&Chassis_JointMotor2_CAN,Chassis_JointMotor2_Send_CAN_ID);
			osDelay(1);
        }
		if(Motor.Joint3.Ticker == 110)      
        {
            Motor_DM_CMD_ClearErr(&Chassis_JointMotor3_CAN, Chassis_JointMotor3_Send_CAN_ID);
            osDelay(1);
            Motor_DM_CMD_Enable(&Chassis_JointMotor3_CAN,Chassis_JointMotor3_Send_CAN_ID);
			osDelay(1);
        }
		if(Motor.Joint4.Ticker == 110)      
        {
            Motor_DM_CMD_ClearErr(&Chassis_JointMotor4_CAN, Chassis_JointMotor4_Send_CAN_ID);
            osDelay(1);
            Motor_DM_CMD_Enable(&Chassis_JointMotor4_CAN,Chassis_JointMotor4_Send_CAN_ID);
			osDelay(1);
        }
			
        osDelay(1);
    }
}
