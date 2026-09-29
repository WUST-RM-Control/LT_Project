#include "WheelLeg_VMC.h"
#include "WheelLeg_Kinematics.h"


WheelLeg_VMC wheelLeg_vmc = {0};

static void WheelLeg_VMC_CalculateLeg(WheelLeg_VMC_Leg *vmc,WheelLeg_LegKinematics *kinematics)
{
    //解算出现故障时不允许vmc
    if(kinematics->valid == 0 ||kinematics->jacobian_valid == 0)
    {
        vmc->tau_phi1_nm  = 0.0f;
        vmc->tau_phi4_nm  = 0.0f;

        vmc->front_motor_torque_nm = 0.0f;
        vmc->back_motor_torque_nm = 0.0f;

        vmc->valid = 0;

        return;
    }

	//vmc计算
	vmc->tau_phi1_nm  =kinematics->jacobian_virtual[0][0]* vmc->force_n+kinematics->jacobian_virtual[1][0]* vmc->torque_nm;
    vmc->tau_phi4_nm  =kinematics->jacobian_virtual[0][1]* vmc->force_n+kinematics->jacobian_virtual[1][1]* vmc->torque_nm;

	vmc->back_motor_torque_nm =vmc->tau_phi1_nm;

    vmc->front_motor_torque_nm =vmc->tau_phi4_nm;
	
    vmc->valid = 1;
}

//调试输入
void WheelLeg_VMC_SetLeft(float force_n,float torque_nm)
{
    wheelLeg_vmc.left.force_n =force_n;
    wheelLeg_vmc.left.torque_nm =torque_nm;
}


void WheelLeg_VMC_SetRight(float force_n,float torque_nm)
{
    wheelLeg_vmc.right.force_n =force_n;
    wheelLeg_vmc.right.torque_nm =torque_nm;
}

//左右腿 VMC 更新
void WheelLeg_VMC_Update(void)
{
    WheelLeg_VMC_CalculateLeg(&wheelLeg_vmc.left,&wheelLeg_kinematics.left);
    WheelLeg_VMC_CalculateLeg(&wheelLeg_vmc.right,&wheelLeg_kinematics.right);
}