/**
 * @file    WheelLeg_Kinematics.c[底盘运动学解算]
 * @brief   轮腿底盘五连杆运动学解算和卡尔曼滤波后腿参数
 * @details 通过电机角度解算得到观察腿摆角和腿长，然后通过卡尔曼滤波得到滤波数据
 */
#include "WheelLeg_Kinematics.h"
#include "WheelLeg_Motor.h"
#include <math.h>

//*******5连杆机械参数定义********* 

#define WHEELLEG_PI                     3.14159265358979323846f

//上下连杆的长度
#define WheelLeg_Up_Lengh      0.21f
#define WheelLeg_Down_Lengh      0.25f

//中间连杆的长度 等于0 约等于4连杆
#define WheelLeg_hip_Distance           0.0f

WheelLeg_Kinematics wheelLeg_kinematics={0};


/**
 * @brief 将角度限制到 [-PI, PI]
 */
static float WheelLeg_WrapPi(float angle)
{
    while(angle > WHEELLEG_PI)
    {
        angle -= 2.0f * WHEELLEG_PI;
    }

    while(angle < -WHEELLEG_PI)
    {
        angle += 2.0f * WHEELLEG_PI;
    }

    return angle;
}

//***************虚拟退正运动学解算，参考玺佬的解算过程

static void WheelLeg_ForwardKinematics(WheelLeg_LegKinematics *leg,float phi1,float phi4)
{
    float Xb;
    float Yb;

    float Xd;
    float Yd;

    float Xc;
    float Yc;

    float A0;
    float B0;
    float C0;

    float lBD;

    float discriminant;

    float phi2;
    float phi3;
    float phi0;

    const float L1 =WHEELLEG_UPPER_LINK_LENGTH;

    const float L2 =WHEELLEG_LOWER_LINK_LENGTH;

    const float L5 =WHEELLEG_HIP_DISTANCE;


	//保存输入角
    leg->phi1_rad = phi1;
    leg->phi4_rad = phi4;

	//*********1. 计算两个主动杆端点 B、D

    Xb = L1 * cosf(phi1);
    Yb = L1 * sinf(phi1);

    Xd = L5 + L1 * cosf(phi4);
    Yd =      L1 * sinf(phi4);

	//********2. B-D之间距离
    lBD = sqrtf((Xd - Xb) * (Xd - Xb)+(Yd - Yb) * (Yd - Yb));

	//*******3. 求解从动杆角度 phi2
    A0 =2.0f * L2 * (Xd - Xb);
    B0 =2.0f * L2 * (Yd - Yb);
    C0 =L2 * L2+ lBD * lBD- L2 * L2;

    //根号内部：A2 + B2 - C2
    discriminant =A0 * A0+ B0 * B0- C0 * C0;
	//错误判断
    if(discriminant < 0.0f)
    {
        leg->valid = 0;
        return;
    }
	
    phi2 =2.0f* atan2f(B0 + sqrtf(discriminant),A0 + C0);
	
	//********4. 求另一根从动杆角度 phi3
    phi3 =atan2f(Yb - Yd + L2 * sinf(phi2),Xb - Xd + L2 * cosf(phi2));

    leg->phi2_rad = phi2;
    leg->phi3_rad = phi3;

	//********5. 求虚拟腿末端 C 点
    Xc =Xb + L2 * cosf(phi2);

    Yc =Yb+ L2 * sinf(phi2);

    leg->endpoint_x_m = Xc;
    leg->endpoint_y_m = Yc;

	//**********6. 求虚拟腿长度 L0
    leg->length_m =sqrtf((Xc - L5 / 2.0f)* (Xc - L5 / 2.0f)+Yc * Yc);

	//***********7. 求虚拟腿绝对方向 phi0
    phi0 =atan2f(Yc,Xc - L5 / 2.0f);
    leg->angle_rad =WheelLeg_WrapPi(WHEELLEG_PI / 2.0f - phi0);

	//*******计算成功标识
    leg->valid = 1;
}

//连续角度计算,最短角度差
static float WheelLeg_AngleDelta(float now,float last)
{
	float delta=now-last;
	
	if(delta>WHEELLEG_PI)
	{
		delta-=2.0f * WHEELLEG_PI;
	}
	else if(delta < -WHEELLEG_PI)
    {
        delta += 2.0f * WHEELLEG_PI;
    }

    return delta;
}

//虚拟腿连续角度更新
static void WheelLeg_UpdateContinuousAngle(WheelLeg_LegKinematics *leg)
{
	float delta_angle;

    if(leg->valid == 0)
    {
        return;
    }

	//第一次计算
    if(leg->angle_initialized  == 0)
    {
  
        leg->last_angle_rad = leg->angle_rad;
        leg->angle_total_rad = leg->angle_rad;
        leg->angle_initialized  = 1;
        return;
    }	
	
	//连续变换的角度
	delta_angle =WheelLeg_AngleDelta(leg->angle_rad,leg->last_angle_rad);
	
	//累计的虚拟腿角度 
	leg->angle_total_rad += delta_angle;

	//更新last
    leg->last_angle_rad =leg->angle_rad;
	
}
//雅可比矩阵推导
static void WheelLeg_UpdateJacobian(WheelLeg_LegKinematics *leg)
{
	float denominator;
    float phi0; //虚拟腿角
    float cos_phi0;
    float sin_phi0;
    float inv_length;
    const float L1 =WHEELLEG_UPPER_LINK_LENGTH;
	
	//单独计算分母
	denominator =sinf(leg->phi2_rad-leg->phi3_rad);
	
	//分母为0 奇异保护
	if(fabsf(denominator) < 0.001f)
    {
        leg->jacobian_valid = 0;

        leg->jacobian_xy[0][0] = 0.0f;
        leg->jacobian_xy[0][1] = 0.0f;
        leg->jacobian_xy[1][0] = 0.0f;
        leg->jacobian_xy[1][1] = 0.0f;

        leg->jacobian_virtual[0][0] = 0.0f;
        leg->jacobian_virtual[0][1] = 0.0f;
        leg->jacobian_virtual[1][0] = 0.0f;
        leg->jacobian_virtual[1][1] = 0.0f;

        return;
    }


    if(leg->length_m < 0.001f)
    {
        leg->jacobian_valid = 0;

        return;
    }
	
	//根据刚体恒长原则推出c点雅可比矩阵
	leg->jacobian_xy[0][0] =(L1* sinf(leg->phi1_rad-leg->phi2_rad)* sinf(leg->phi3_rad))/ denominator;

    leg->jacobian_xy[0][1] =(L1* sinf(leg->phi3_rad-leg->phi4_rad)* sinf(leg->phi2_rad))/ denominator;

	leg->jacobian_xy[1][0] =-(L1* sinf(leg->phi1_rad-leg->phi2_rad)* cosf(leg->phi3_rad))/ denominator;

    leg->jacobian_xy[1][1] =-(L1* sinf(leg->phi3_rad-leg->phi4_rad)* cosf(leg->phi2_rad))/ denominator;

	//根据c和虚拟腿的关系 推导出第二个雅可比矩阵
	phi0 =WHEELLEG_PI / 2.0f-leg->angle_rad;

    cos_phi0 =cosf(phi0);

    sin_phi0 =sinf(phi0);

    inv_length =1.0f / leg->length_m;

	leg->jacobian_virtual[0][0] =cos_phi0 * leg->jacobian_xy[0][0]+sin_phi0 * leg->jacobian_xy[1][0];

    leg->jacobian_virtual[0][1] =cos_phi0 * leg->jacobian_xy[0][1]+sin_phi0 * leg->jacobian_xy[1][1];

    leg->jacobian_virtual[1][0] =sin_phi0 * inv_length* leg->jacobian_xy[0][0]-cos_phi0 * inv_length* leg->jacobian_xy[1][0];

    leg->jacobian_virtual[1][1] =sin_phi0 * inv_length* leg->jacobian_xy[0][1]-cos_phi0 * inv_length* leg->jacobian_xy[1][1];
	
    leg->jacobian_valid = 1;
}

//虚拟腿速度函数
static void WheelLeg_UpdateVirtualVelocity(WheelLeg_LegKinematics *leg,float phi1_velocity,float phi4_velocity)
{
	//保存关节速度
	leg->phi1_velocity_rad_s =phi1_velocity;
    leg->phi4_velocity_rad_s =phi4_velocity;
	
	//有效性保护
	 if(leg->valid == 0 ||leg->jacobian_valid == 0 ||!isfinite(phi1_velocity) ||!isfinite(phi4_velocity))
    {
        leg->length_velocity_m_s =0.0f;
        leg->angle_velocity_rad_s =0.0f;
        return;
    }
	
	//利用雅可比矩阵推导虚拟腿的速度
	
	leg->length_velocity_m_s =leg->jacobian_virtual[0][0]* phi1_velocity+leg->jacobian_virtual[0][1]* phi4_velocity;
    leg->angle_velocity_rad_s =leg->jacobian_virtual[1][0]*phi1_velocity +leg->jacobian_virtual[1][1]* phi4_velocity;
	
	
}

//运动学数据更新
void WheelLeg_Kinematics_Update(float dt)
{
    float q_lf;
    float q_lb;

    float q_rf;
    float q_rb;

	float left_phi1_velocity;
	float left_phi4_velocity;

	float right_phi1_velocity;
	float right_phi4_velocity;
	
	//实车编码器方向：LF = -rawLB = +raw
    q_lf =-wheelLeg_motor.left_front.total_position_rad;
    q_lb = wheelLeg_motor.left_back.total_position_rad;

	//实车编码器方向：：RF = +rawRB = -raw
    q_rf =wheelLeg_motor.right_front.total_position_rad;
    q_rb =-wheelLeg_motor.right_back.total_position_rad;

    //保存便于Keil Watch观察
    wheelLeg_kinematics.left.q_front_rad = q_lf;
    wheelLeg_kinematics.left.q_back_rad  = q_lb;
    wheelLeg_kinematics.right.q_front_rad = q_rf;
    wheelLeg_kinematics.right.q_back_rad  = q_rb;
	
	//左腿 关节速度
	left_phi1_velocity =-wheelLeg_motor.left_back.velocity_rad_s;
	left_phi4_velocity =-wheelLeg_motor.left_front.velocity_rad_s;
	
	//右腿关节速度
	right_phi1_velocity =wheelLeg_motor.right_back.velocity_rad_s;
	right_phi4_velocity =wheelLeg_motor.right_front.velocity_rad_s;

	//调用运动学解算
    WheelLeg_ForwardKinematics(&wheelLeg_kinematics.left, WHEELLEG_PI - q_lb,q_lf);
    WheelLeg_ForwardKinematics(&wheelLeg_kinematics.right,WHEELLEG_PI - q_rb,q_rf);
	
	//更新虚拟腿角度
    WheelLeg_UpdateContinuousAngle(&wheelLeg_kinematics.right);
	WheelLeg_UpdateContinuousAngle(&wheelLeg_kinematics.left);
	
	// Jacobian
	WheelLeg_UpdateJacobian(&wheelLeg_kinematics.left);
	WheelLeg_UpdateJacobian(&wheelLeg_kinematics.right);
	
	//更新虚拟腿速度
	WheelLeg_UpdateVirtualVelocity(&wheelLeg_kinematics.left,left_phi1_velocity,left_phi4_velocity);
	WheelLeg_UpdateVirtualVelocity(&wheelLeg_kinematics.right,right_phi1_velocity,right_phi4_velocity);
	
	
}