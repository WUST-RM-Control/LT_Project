#include "WheelLeg_LegController.h"
#include "WheelLeg_Kinematics.h"
#include "PID.h"
#include "INS.h"
#include "WheelLeg_SpringComp.h"

WheelLeg_LegController wheelLeg_legController={0};



void WheelLeg_LegController_Init(void)
{

    WheelLeg_LegController_SetTarget( 0.35f,0.35f);
    //腿长位置外环
    PID_Init(&wheelLeg_legController.left.length_position_pid,20.0f,0.0f,0.0f,0.0f,0.0f,0.40f);
    PID_Init(&wheelLeg_legController.right.length_position_pid,20.0f,0.0f,0.0f,0.0f,0.0f,0.40f);
    
    //腿长速度外环
    PID_Init(&wheelLeg_legController.left.length_velocity_pid,200.0f,0.0f,0.0f,200.0f,20.0f,50.0f);
    PID_Init(&wheelLeg_legController.right.length_velocity_pid,200.0f,0.0f,0.0f,200.0f,20.0f,50.0f);

    PID_Init(&wheelLeg_legController.roll_pid,60.0f,0.0f,250.0f,0.0f,0.0f,80.0f);

    wheelLeg_legController.left.gravity_comp_n =90.0f;

    wheelLeg_legController.right.gravity_comp_n =90.0f;

    wheelLeg_legController.target_roll_deg = 0.0f;
    wheelLeg_legController.current_roll_deg = 0.0f;
    wheelLeg_legController.roll_comp_n = 0.0f;
}

void WheelLeg_LegController_SetTarget(float left_length_m,float right_length_m)
{
    wheelLeg_legController.left.target_length_m =left_length_m;
    wheelLeg_legController.right.target_length_m = right_length_m;
}


static void WheelLeg_LegController_UpdateLeg(WheelLeg_LegController_Leg *controller,WheelLeg_LegKinematics *kinematics)
{

    controller->current_length_m =kinematics->length_m;
    controller->current_length_velocity_m_s =kinematics->length_velocity_m_s;
    if(kinematics->valid == 0||kinematics->jacobian_valid == 0)
    {
        controller->length_error_m =0.0f;
        controller->target_length_velocity_m_s =0.0f;
        controller->length_velocity_error_m_s =0.0f;
        controller->force_pid_n =0.0f;
        controller->force_total_n =0.0f;
        controller->valid =0;
        return;
    }

    //腿长位置外环 Target= 目标腿长 Feedback =当前腿长   Output = 目标腿长速度
    PID_Position_Calculate(&controller->length_position_pid,controller->target_length_m,controller->current_length_m);
    controller->length_error_m =controller->length_position_pid.Error;
    controller->target_length_velocity_m_s =controller->length_position_pid.Output;

    //腿长速度内环 Target = 位置环输出的目标腿长速度  Feedback = 通过Jacobian得到的实际腿长速度 Output   = 虚拟腿力
    PID_Position_Calculate(&controller->length_velocity_pid,controller->target_length_velocity_m_s,controller->current_length_velocity_m_s);
    controller->length_velocity_error_m_s =controller->length_velocity_pid.Error;
    controller->force_pid_n =controller->length_velocity_pid.Output;

    

    controller->valid =1;
}

void WheelLeg_LegController_Update(void)
{
    //ROLL补偿
    wheelLeg_legController.current_roll_deg =INS_Data_Self.Roll;

     if(isfinite(wheelLeg_legController.current_roll_deg))
    {
        PID_Position_Calculate(&wheelLeg_legController.roll_pid,wheelLeg_legController.target_roll_deg,wheelLeg_legController.current_roll_deg);
        wheelLeg_legController.roll_comp_n =wheelLeg_legController.roll_pid.Output;
    }
    else
    {
        wheelLeg_legController.roll_comp_n =0.0f;
    }

    //计算双腿PID
    WheelLeg_LegController_UpdateLeg(&wheelLeg_legController.left,&wheelLeg_kinematics.left);
    WheelLeg_LegController_UpdateLeg(&wheelLeg_legController.right,&wheelLeg_kinematics.right);

    //氮气弹簧 补偿
    if(wheelLeg_springComp.left.valid)
    {
        wheelLeg_legController.left.spring_comp_n =wheelLeg_springComp.left.force_n;
    }
    else
    {
        wheelLeg_legController.left.spring_comp_n =0.0f;
    }

    if(wheelLeg_springComp.right.valid)
    {
        wheelLeg_legController.right.spring_comp_n =wheelLeg_springComp.right.force_n;
    }
    else
    {
        wheelLeg_legController.right.spring_comp_n =0.0f;
    }
    
    //左腿最终虚拟力
    if(wheelLeg_legController.left.valid)
    {
        wheelLeg_legController.left.force_total_n =
            wheelLeg_legController.left.force_pid_n
            +
            wheelLeg_legController.left.gravity_comp_n
            +
            wheelLeg_legController.left.spring_comp_n
            -
            wheelLeg_legController.roll_comp_n;
    }
    else
    {
        wheelLeg_legController.left.force_total_n =0.0f;
    }

    //右腿最终虚拟力
    if(wheelLeg_legController.right.valid)
    {
        wheelLeg_legController.right.force_total_n =
            wheelLeg_legController.right.force_pid_n
            +
            wheelLeg_legController.right.gravity_comp_n
            +
            wheelLeg_legController.right.spring_comp_n
            +
            wheelLeg_legController.roll_comp_n;
    }
    else
    {
        wheelLeg_legController.right.force_total_n =
            0.0f;
    }
}