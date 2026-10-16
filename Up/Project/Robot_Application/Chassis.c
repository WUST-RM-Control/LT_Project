//===========================================================================
//                      Chassis[底盘控制]
//      根据云台发下来的Vx,Vy,Wz,YawErr进行运动解算，实现底盘的各种运动状态
//      以及根据裁判系统反馈数据和[超电状态]实现功率控制
//===========================================================================

#include "Chassis.h"
#include "RoboControl.h"
#include "Power_Limit.h"
#include "Referee_Unpack.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Aim.h"

/*===| 底盘控制数据结构体 |===*/
Chassis_Control_StructTypeDef Chassis_Control_Struct;
extern uint32_t Tim_fire_cmd_apply_mcu_us;

/**
 * @brief 底盘任务
 */
void Chassis_Task(void *argument)
{
    osDelay(100);
    while(1)
    {
        if(RoboControl_Struct.Gimbal_State == Gimbal_State_Aim&&Aim_If_Allow_Shoot())
        {
            Aim_Tim_Send_Struct.t_gimbal_ready_mcu_us = DWT->CYCCNT/170.0f;
        }
        if(Shoot_Control_Struct.Shoot_State == Shoot_State_Continue||Shoot_Control_Struct.Shoot_State == Shoot_State_Ready)
        Aim_Tim_Send_Struct.t_fire_cmd_apply_mcu_us = Tim_fire_cmd_apply_mcu_us;

        USB_Send((uint8_t *)&Aim_Tim_Send_Struct, sizeof(Aim_Tim_Send_StructTypedef));
        osDelay(1);
    }
}

