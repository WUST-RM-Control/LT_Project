/**
 * @file    WheelLeg_LQR.c[LQR运控]
 * @brief   轮腿底盘的LQR运控
 */
#include "WheelLeg_LQR.h"
#include "Chassis.h"
#include "kalman_filter.h"
#include <math.h>

WheelLeg_LQR_Displacement_Date Wheel_Left;
WheelLeg_LQR_Displacement_Date Wheel_Right;

WheelLeg_LQR_StructTypeDef WheelLeg_LQR_Struct;

//三次多项式拟合系数
static const float Poly_Coefficient[12][4]={
    {-204.71377921f,	289.63818168f,  -157.59015500f, -4.96782823f},
	{-7.82854075f,	    19.15680744f,	-17.41989049f,	-0.23003324f},
	{-7.53928256f,	    12.07032519f,	-6.74694158f,	-0.05542880f},
	{-13.69695924f,     22.38839045f,	-12.69903703f,	-0.34124181f},
	{202.15935394f,	    -119.52253547f,	-28.73024512f,	30.05658725f},
	{8.53035961f,	    0.97505447f,	-8.73026755f,	4.45533442f},
	{1451.56173722f,	-1146.30857312f,122.25511452f,	96.79758089f},
	{170.43065142f,	    -127.63122938f,	1.84610137f,	19.79612454f},
	{55.36361918f,	    -32.73259440f,	-7.86810167f,	8.23133542f},
	{36.92907744f,	    7.73400354f,	-41.51630122f,	19.92978630f},
	{825.88702496f,	    -1322.23787701f,739.09041989f,	6.07192070f},
	{120.04567253f,	    -175.60627295f,	92.90386168f,	-0.44963915f}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
KalmanFilter_t L_vaEstimateKF;	   // 卡尔曼滤波器结构体-左腿 
KalmanFilter_t R_vaEstimateKF;	   // 卡尔曼滤波器结构体-右腿
float L_vaEstimateKF_F[4] = {1.0f, 0.002f, 0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.002s
float L_vaEstimateKF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};      // 后验估计协方差初始值
const float L_vaEstimateKF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};// 设置矩阵H为常量

float R_vaEstimateKF_F[4] = {1.0f, 0.002f, 0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.002s
float R_vaEstimateKF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};      // 后验估计协方差初始值
const float R_vaEstimateKF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};// 设置矩阵H为常量
float L_vaEstimateKF_K[4];
float R_vaEstimateKF_K[4];

float L_vaEstimateKF_Q[4] = {0.05f, 0.0f, 0.0f, 0.01f};     // Q矩阵初始值
float L_vaEstimateKF_R[4] = {1000.0f,  0.0f,   0.0f, 500000.0f}; 	  
								 
float R_vaEstimateKF_Q[4] = {0.05f, 0.0f, 0.0f, 0.01f};     // Q矩阵初始值
float R_vaEstimateKF_R[4] = {1000.0f,  0.0f, 0.0f, 500000.0f};  	
						 
float vel_acc[2]; 
	
float V_L;							 
float V_R;	
float V_ave;

static void xvEstimateKF_Init(KalmanFilter_t *L_EstimateKF,KalmanFilter_t *R_EstimateKF)//卡尔曼滤波速度观测器初始化
{
    Kalman_Filter_Init(L_EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	Kalman_Filter_Init(R_EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
	memcpy(L_EstimateKF->F_data, L_vaEstimateKF_F, sizeof(L_vaEstimateKF_F));
    memcpy(L_EstimateKF->P_data, L_vaEstimateKF_P, sizeof(L_vaEstimateKF_P));
    memcpy(L_EstimateKF->Q_data, L_vaEstimateKF_Q, sizeof(L_vaEstimateKF_Q));
    memcpy(L_EstimateKF->R_data, L_vaEstimateKF_R, sizeof(L_vaEstimateKF_R));
    memcpy(L_EstimateKF->H_data, L_vaEstimateKF_H, sizeof(L_vaEstimateKF_H));
	
	memcpy(R_EstimateKF->F_data, R_vaEstimateKF_F, sizeof(R_vaEstimateKF_F));
    memcpy(R_EstimateKF->P_data, R_vaEstimateKF_P, sizeof(R_vaEstimateKF_P));
    memcpy(R_EstimateKF->Q_data, R_vaEstimateKF_Q, sizeof(R_vaEstimateKF_Q));
    memcpy(R_EstimateKF->R_data, R_vaEstimateKF_R, sizeof(R_vaEstimateKF_R));
    memcpy(R_EstimateKF->H_data, R_vaEstimateKF_H, sizeof(R_vaEstimateKF_H));

}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)//卡尔曼滤波速度观测器数据更新
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

/**
 * @brief 计算位移和速度
 * 
 */
static void WheelLeg_LQR_Calculated_Displacement(Chassis_Control_StructTypedef *Chassis_Control_Struct)
{

/*===| 得到底盘目标位移和速度 |===*/
    static float is_moving;
    static float was_moving;
	//位移
	// 判断当前和上一次的速度是否“有效”（非零）
	// 判断当前和上一次的速度是否“有效”（非零）
	is_moving  = fabsf(Chassis_Control_Struct->Chassis_Control.Vy_Target)      > 0.05f;
	was_moving = fabsf(Chassis_Control_Struct->Chassis_Control.Vy_Target_Last) > 0.05f;

	// 状态一：刹车瞬间 (从运动变为静止)
	if (was_moving && !is_moving) 
	{
		// 【关键修复】清零积分器，为下次起步做准备
		WheelLeg_LQR_Struct.Wheel_displacement_add = 0.0f; 
	}
	// 状态二：起步瞬间 (从静止变为运动)
	else if (!was_moving && is_moving) 
	{
		// 记录起点的实际位移
		WheelLeg_LQR_Struct.Wheel_displacement_init = WheelLeg_LQR_Struct.Target_displacement;
		// 【关键修复】清零积分器，防止历史脏数据导致瞬间跳跃
		WheelLeg_LQR_Struct.Wheel_displacement_add = 0.0f;
	}
	// 状态三：持续运动中 (不管之前是什么状态，只要现在有速度就执行积分)
	if (is_moving) 
	{
		// 正常积分
		WheelLeg_LQR_Struct.Wheel_displacement_add += Chassis_Control_Struct->Chassis_Control.Vy_Target * Chassis_Control_Struct->Chassis_Dt;
		// 计算目标位移 = 起点位移 + 积分出来的位移增量
		WheelLeg_LQR_Struct.Target_displacement = WheelLeg_LQR_Struct.Wheel_displacement_init + WheelLeg_LQR_Struct.Wheel_displacement_add;
	}

    WheelLeg_LQR_Struct.Wheel_Speed_Target = Chassis_Control_Struct->Chassis_Control.Vy_Target;

//得到轮子的实际位移 Wheel_Displacement_Feedback
    Wheel_Left.Displacement_Now  = Motor.Wheel_Motor1.Total_Angle / Motor_3508_Reduction * Chassis_Wheel_Radius * Angle_to_Radain;
    Wheel_Right.Displacement_Now = Motor.Wheel_Motor1.Total_Angle / Motor_3508_Reduction * Chassis_Wheel_Radius * Angle_to_Radain;
    Wheel_Left.Displacement  = -(Wheel_Left.Displacement_Now  - Wheel_Left.Displacement_Init);
    Wheel_Right.Displacement =   Wheel_Right.Displacement_Now - Wheel_Right.Displacement_Init;

	Wheel_Left.Wheel_Speed_Feedback  = -Motor.Chassis_DriverMotor1.Speed_RPM * Chassis_RPM_to_m_s;
	Wheel_Right.Wheel_Speed_Feedback =  Motor.Chassis_DriverMotor2.Speed_RPM * Chassis_RPM_to_m_s;

/*===| 卡尔曼滤波得到底盘实际位移和速度 |===*/
		xvEstimateKF_Update(&L_vaEstimateKF,-Chassis_Control_Struct.Acceleration_Y,Wheel_Left.Wheel_Speed_Feedback);//得到卡尔曼滤波后左轮的速度
		xvEstimateKF_Update(&R_vaEstimateKF,-Chassis_Control_Struct.Acceleration_Y,Wheel_Right.Wheel_Speed_Feedback);//得到卡尔曼滤波后右轮的速度
		WheelLeg_LQR_Struct.Wheel_Speed_Feedback = (L_vaEstimateKF.FilteredValue[0]+R_vaEstimateKF.FilteredValue[0])*0.5;

/*===|根据底盘状态更新底盘Init位移|===*/
    //非使能状态下更新
    if(Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_FOLLOW
    && Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_SPIN
    && Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_Jump
    && Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_UpStep)
    {
        //轮位移
        Wheel_Left.Displacement_Init  = Wheel_Left.Displacement_Now;
        Wheel_Right.Displacement_Init = Wheel_Right.Displacement_Now;
        //底盘位移
        WheelLeg_LQR_Struct.Wheel_Displacement_Target = WheelLeg_LQR_Struct.Wheel_Displacement_Feedback;
        WheelLeg_LQR_Struct.Wheel_displacement_init   = WheelLeg_LQR_Struct.Wheel_Displacement_Feedback;
    }

}
/**
 * @brief 添加六个状态量
 */
static void WheelLeg_LQR_add_six_state(Chassis_Control_StructTypedef *Chassis_Control)
{
	WheelLeg_LQR_Struct.State[1] =  (Chassis_Control->Leg_Left.Angle_Feedback)*Angle2Radain    - (Chassis_Control->Chassis_Control.Pitch-2.5*Angle2Radain);//θ  左腿后摆 θ  为正 增大
	WheelLeg_LQR_Struct.State[2] =  (Chassis_Control->Leg_Left.Total_Angle_Speed)*Angle2Radain - Chassis_Control->Chassis_Control.Pitch_dot;               //θ’ 左腿后摆 θ’ 为正 增大
	
	WheelLeg_LQR_Struct.State[7] = -(Chassis_Control->Leg_Right.Angle_Feedback)*Angle2Radain    - (Chassis_Control->Chassis_Control.Pitch-2.5*Angle2Radain);//θ  右腿后摆 θ  为正 增大
    WheelLeg_LQR_Struct.State[8] = -(Chassis_Control->Leg_Right.Total_Angle_Speed)*Angle2Radain - Chassis_Control->Chassis_Control.Pitch_dot;               //θ’ 右腿后摆 θ’ 为正 增大

    WheelLeg_LQR_Struct.State[3] = (WheelLeg_LQR_Struct.Wheel_Displacement_Feedback-WheelLeg_LQR_Struct.Chassis_Control.Wheel_Displacement_Target);		//位移							
    WheelLeg_LQR_Struct.State[4] = (WheelLeg_LQR_Struct.Wheel_Speed_Feedback-WheelLeg_LQR_Struct.Wheel_Speed_Target);	//位移一阶导	 
    WheelLeg_LQR_Struct.State[5] = Chassis_Control->Chassis_Control.Pitch-2.5*Angle2Radain;	//机体pitch,翘头pitch增大
    WheelLeg_LQR_Struct.State[6] = Chassis_Control->Chassis_Control.Pitch_dot;				//机体pitch弧度一阶导
}

static float LQR_K_calc(float *coe,float len)
{
  float K = coe[0]*len*len*len + coe[1]*len*len + coe[2]*len + coe[3];
	
  return K;
}

/**
 * @brief 计算LQR系数K
 * 
 * @param Length_Left_Feedback 
 * @param Length_Right_Feedback 
 */
static void calucateK(const float Length_Left_Feedback, const float Length_Right_Feedback)  
{
    for(int i = 0; i < 12; i++)
    {
		WheelLeg_LQR_Struct.LQR_K_Left[i]  = LQR_K_calc(Poly_Coefficient[i], Length_Left_Feedback);
        WheelLeg_LQR_Struct.LQR_K_Right[i] = LQR_K_calc(Poly_Coefficient[i], Length_Right_Feedback);
    }
}

/**
 * @brief LQR运控PID补丁
 * 
 */
static void WheelLeg_LQR_PID_Patch(Chassis_Control_StructTypedef *Chassis_Control)
{
	PID_Position_Calculate(&WheelLeg_LQR_Struct.Wheel_Wz_Speed_PID ,Chassis_Control->Chassis_Control.Wz_Speed_Target, Chassis_Control->Chassis_Control.Yaw_Speed);

	PID_Position_Calculate(&WheelLeg_LQR_Struct.Leg_Splits_PID ,0.0f, Chassis_Control->Leg_Left.Angle_Feedback+Chassis_Control->Leg_Right.Angle_Feedback);

}

void WheelLeg_LQR_Init(void)
{
    /*===| 卡尔曼滤波初始化  |===*/
	xvEstimateKF_Init(&L_vaEstimateKF,&R_vaEstimateKF);
	//转向&小陀螺速度环
	PID_Init(&WheelLeg_LQR_Struct.Wheel_Wz_Speed_PID, 3.0f,  0.0f, 0.1f, 0.0f,0.0f,3.0f);
	//防劈叉PID
	PID_Init(&WheelLeg_LQR_Struct.Leg_Splits_PID, 4.0f, 0.0f, 0.1f, 0.0f,0.0f,10.0f);

}

static void WheelLeg_LQR_calculation(Chassis_Leg_StructTypedef *Leg_Date, const Chassis_State_EnumTypedef Chsssis_State)
{	
	float *LQR_K_LR;
	float theta,theta_d,xb,xb_d,phi,phi_d;
	if(Leg_Date->Leg_LR == Leg_L)
	{
		LQR_K_LR = WheelLeg_LQR_Struct.LQR_K_Left;
		theta   = -WheelLeg_LQR_Struct.State[1];
    	theta_d = -WheelLeg_LQR_Struct.State[2];
		xb    = -WheelLeg_LQR_Struct.State[3];
		xb_d  = -WheelLeg_LQR_Struct.State[4];
		phi   = -WheelLeg_LQR_Struct.State[5];
		phi_d = -WheelLeg_LQR_Struct.State[6];
	}
	else if(Leg_Date->Leg_RL_State == Leg_R)
	{
		LQR_K_LR = WheelLeg_LQR_Struct.LQR_K_Right;
		theta   = WheelLeg_LQR_Struct.State[7];
    	theta_d = WheelLeg_LQR_Struct.State[8];
		xb    = WheelLeg_LQR_Struct.State[3];
		xb_d  = WheelLeg_LQR_Struct.State[4];
		phi   = WheelLeg_LQR_Struct.State[5];
		phi_d = WheelLeg_LQR_Struct.State[6];
	}
    //板凳状态下输出力矩锁定关节摆角 若用LQR控制关节则注释掉此处的Tp的输出
//	Chassis_Control_Struct.Torque_Joint_left=-Chassis_Control_Struct.Left_leg_Angle_PID.Output;
	
    float Torque_wheel_left = -( LQR_K_LR[0] * theta + LQR_K_LR[1] * theta_d + LQR_K_LR[2] * xb + LQR_K_LR[3] * xb_d + LQR_K_LR[4] * phi + LQR_K_LR[5] * phi_d );
    Leg_Date->LQR_Link_Torque_Target[1] =  ( LQR_K_LR[6] * theta + LQR_K_LR[7] * theta_d + LQR_K_LR[8] * xb + LQR_K_LR[9] * xb_d + LQR_K_LR[10] * phi + LQR_K_LR[11] * phi_d ) + (Chassis_Control_Struct.Chassis_Control.Leg_Splits_PID.Output);

	if(Chsssis_State == Chassis_Jump)
	{
		Leg_Date->LQR_Wheel_Torque_Target = Torque_wheel_left;
	}
	else if(Chsssis_State == Chassis_Off_ground)
	{
		Leg_Date->LQR_Wheel_Torque_Target = 0;//左轮电机输出置0
		Leg_Date->LQR_Link_Torque_Target[1] =(LQR_K_LR[6]*(theta) + LQR_K_LR[7]*(theta_d)) + (WheelLeg_LQR_Struct.Leg_Splits_PID.Output);//左关节力矩只保留摆角控制量
	}
	else
	{
		Leg_Date->LQR_Wheel_Torque_Target = Torque_wheel_left + WheelLeg_LQR_Struct.Wheel_Wz_Speed_PID;
	}

	Limit_float(&Leg_Date->LQR_Link_Torque_Target[1],80.0f,-80.0f);
	Limit_float(&Leg_Date->LQR_Wheel_Torque_Target,5.0f,-5.0f);
}

void WheelLeg_LQR_Output(Chassis_Control_StructTypedef *Chassis_Control)
{
	WheelLeg_LQR_Calculated_Displacement(Chassis_Control);

	WheelLeg_LQR_add_six_state(Chassis_Control);

	calucateK(Chassis_Control->Leg_Left.Length_Feedback, Chassis_Control->Leg_Right.Length_Feedback);

	WheelLeg_LQR_PID_Patch(Chassis_Control);

	WheelLeg_LQR_calculation(&Chassis_Control->Leg_Left,  Chassis_Control->Chassis_Control.Chassis_State);
	WheelLeg_LQR_calculation(&Chassis_Control->Leg_Right, Chassis_Control->Chassis_Control.Chassis_State);
}
