/**
 * @file    WheelLeg_Kinematics.c[底盘运动学解算]
 * @brief   轮腿底盘五连杆运动学解算和卡尔曼滤波后腿参数
 * @details 通过电机角度解算得到观察腿摆角和腿长，然后通过卡尔曼滤波得到滤波数据
 */
#include "WheelLeg_Kinematics.h"
#include "Chassis.h"
#include "Motor_Driver.h"
#include "kalman_filter.h"
#include "arm_math.h"  // CMSIS-DSP 库


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//上下连杆的长度
#define WheelLeg_Up_Lengh       0.21f
#define WheelLeg_Down_Lengh     0.25f

//中间连杆的长度 等于0 约等于4连杆
#define WheelLeg_Mid_Distance   0.0f

WheelLeg_Kinematics_StructTypedef wheelLeg_kinematics_Struct={0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//卡尔曼滤波相关
/**
 * @brief 2状态(位置-速度)卡尔曼滤波器, 无动态内存分配
 *        状态 x = [位置; 速度], 观测量为位置, 采用离散白噪声加速度模型:
 *        x'(k) = [1 dt; 0 1] x(k-1)
 *        Q     = Q_acc * [dt^4/4  dt^3/2; dt^3/2  dt^2]
 *        相比直接差分求速度, 可由带噪声的位置量测得到平滑且低延迟的速度估计
 */
typedef struct
{
    float x;      // 位置估计
    float v;      // 速度估计
    float P[4];   // 协方差矩阵 [P00 P01 P10 P11]
    float Q_acc;  // 加速度过程噪声方差 (位置单位^2/s^4)
    float R;      // 位置量测噪声方差 (位置单位^2)
    float P0;     // 初始位置方差
    float V0;     // 初始速度方差
    uint8_t Init; // 首次更新标志, 首次用第一个量测初始化状态
} PosVelKF_t;

/*===| 腿长L0、腿摆角Total_A0 的 位置-速度 卡尔曼滤波器 |===*/
static PosVelKF_t L_L0_KF, R_L0_KF; // 左右腿腿长
static PosVelKF_t L_A0_KF, R_A0_KF; // 左右腿腿摆角(总角度)

// 整定参数: Q_acc=加速度过程噪声方差, R=位置量测噪声方差
// 位置更平滑 -> 减小Q_acc或增大R; 速度响应更快 -> 增大Q_acc或减小R
#define L0_KF_Q_ACC 50.0f  // (m/s^2)^2, 腿长加速度噪声
#define L0_KF_R     1.0e-6f // m^2,       腿长位置量测噪声
#define A0_KF_Q_ACC 3.0e6f  // (deg/s^2)^2, 腿摆角加速度噪声
#define A0_KF_R     2.5e-3f // deg^2,      腿摆角位置量测噪声
/**
 * @brief 2状态(位置-速度)卡尔曼滤波更新
 *
 * @param kf 滤波器结构体
 * @param z  位置量测值
 * @param dt 距上次更新的时间间隔, 单位s
 */
static void PosVelKF_Update(PosVelKF_t *kf, float z, float dt)
{
    float x_pred, v_pred, p00, p01, p10, p11, s, k0, k1, y;

    // 量测或周期非法时跳过, 避免滤波器发散(NaN 不满足范围比较)
    if (kf == NULL || !(dt > 0.0f) || dt > 0.1f || !(z > -1.0e30f) || !(z < 1.0e30f))
        return;

    // 首次更新: 用第一个量测初始化状态, 避免启动时状态从0收敛带来的大幅跳变
    if (kf->Init == 0)
    {
        kf->x = z;
        kf->v = 0.0f;
        kf->P[0] = kf->P0;
        kf->P[1] = 0.0f;
        kf->P[2] = 0.0f;
        kf->P[3] = kf->V0;
        kf->Init = 1;
        return;
    }

    // 1. 先验估计 x'(k) = F x(k-1),  F = [1 dt; 0 1]
    x_pred = kf->x + kf->v * dt;
    v_pred = kf->v;

    // 2. 先验协方差 P'(k) = F P(k-1) F^T + Q
    p00 = kf->P[0] + dt * (kf->P[2] + kf->P[1]) + dt * dt * kf->P[3] + kf->Q_acc * dt * dt * dt * dt * 0.25f;
    p01 = kf->P[1] + dt * kf->P[3] + kf->Q_acc * dt * dt * dt * 0.5f;
    p10 = kf->P[2] + dt * kf->P[3] + kf->Q_acc * dt * dt * dt * 0.5f;
    p11 = kf->P[3] + kf->Q_acc * dt * dt;

    // 3. 卡尔曼增益 K = P'(k) H^T / (H P'(k) H^T + R),  H = [1 0]
    s = p00 + kf->R;
    k0 = p00 / s;
    k1 = p10 / s;

    // 4. 后验估计 x(k) = x'(k) + K (z - H x'(k))
    y = z - x_pred;
    kf->x = x_pred + k0 * y;
    kf->v = v_pred + k1 * y;

    // 5. 后验协方差 P(k) = (I - K H) P'(k)
    kf->P[0] = (1.0f - k0) * p00;
    kf->P[1] = (1.0f - k0) * p01;
    kf->P[2] = p10 - k1 * p00;
    kf->P[3] = p11 - k1 * p01;
}
/**
 * @brief 用卡尔曼滤波更新单腿的位置和速度反馈
 */
static void Leg_State_KF_Update(Chassis_Leg_StructTypedef *Leg_Date, PosVelKF_t *kf_L0, PosVelKF_t *kf_A0, float dt)
{
	PosVelKF_Update(kf_L0, Leg_Date->Length_Feedback, dt);
	PosVelKF_Update(kf_A0, Leg_Date->Total_Angle_Feedback, dt);

	Leg_Date->Length_Feedback       = kf_L0->x;
	Leg_Date->Length_Speed          = kf_L0->v;
	Leg_Date->Total_Angle_Feedback  = kf_A0->x;
	Leg_Date->Total_Angle_Speed     = kf_A0->v;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief ===|虚拟腿正运动学解算|===
 * 
 * @note Joint_Angle是电机角度(-180~180°)
 * 
 * @param Leg_Date 
 * @param Joint_Angle_14 左腿：关节1(左腿车头)角度；右腿：关节4(右腿车尾)角度
 * @param Joint_Angle_23 左腿：关节2(左腿车尾)角度；右腿：关节3(右腿车头)角度
 */
static void WheelLeg_ForwardKinematics(WheelLeg_LegKinematics_StructTypedef *Leg_Date, float Jt[2][2], const float Joint_Angle_14,const float Joint_Angle_23)
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//哈工程正运动学解算
	float Xb,Yb;
	float Xd,Yd;
	float Xc,Yc;
	
	const float L5=WheelLeg_Mid_Distance;
    const float L1=WheelLeg_Up_Lengh;
    const float L2=WheelLeg_Down_Lengh;

	float A0,B0,C0,lBD;
	float PHI0,PHI1,PHI2,PHI3,PHI4; //弧度制
	float phi1,phi4;		        //角度制
    // float phi2,phi3;
	float L0;
	float phi0;
	
	// if(Leg_Date->Leg_RL_State == Leg_L)
	// {
	// 	phi1 = Motor.Chassis_Joint1.Total_Angle + 180.0f;
	// 	phi4 = Motor.Chassis_Joint2.Total_Angle;
	// }
	// else if(Leg_Date->Leg_RL_State == Leg_R)
	// {
	// 	phi1 = Motor.Chassis_Joint4.Total_Angle + 180.0f;
	// 	phi4 = Motor.Chassis_Joint3.Total_Angle;
	// }

    //范围：[0,2PI)
    phi1 = Joint_Angle_14 + 180.0f;

    if(Joint_Angle_23 < 0.0f)
    {
        phi4 = Joint_Angle_23 + 360.0f;
    }
    else
    {
        phi4 = Joint_Angle_23;
    }

	PHI1 = phi1 * Angle_to_Radain;
	PHI4 = phi4 * Angle_to_Radain;
	
	Xb = L1*arm_cos_f32(PHI1);
	Yb = L1*arm_sin_f32(PHI1);
	Xd = L1*arm_cos_f32(PHI4) + L5;
	Yd = L1*arm_sin_f32(PHI4);
	
	lBD=sqrt((Xd-Xb)*(Xd-Xb)+(Yd-Yb)*(Yd-Yb));
	A0=2*L2*(Xd-Xb);
	B0=2*L2*(Yd-Yb);
	C0=L2*L2+lBD*lBD-L2*L2;
	
	PHI2=2*atan2f((B0+sqrtf(A0*A0+B0*B0-C0*C0)),(A0+C0));
	PHI3=atan2f(Yb-Yd+L2*arm_sin_f32(PHI2),Xb-Xd+L2*arm_cos_f32(PHI2));
	
	Xc=Xb+L2*arm_cos_f32(PHI2);
	Yc=Yb+L2*arm_sin_f32(PHI2);

	L0=sqrt((Xc-L5/2)*(Xc-L5/2)+Yc*Yc);
	PHI0=atan2f(Yc,(Xc-L5/2));
	phi0=PHI0*Radain_to_Angle;
	


	//Feedback数据赋值
	Leg_Date->Angle_Feedback_No_kalman = L0;
	Leg_Date->Angle_Feedback_No_kalman = Caculate_Included_Angle(90.0f,phi0);
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//哈工程VMC
	// float d_phi1_L, d_phi4_L;
	// float dxc_L   , dyc_L;

	// float J[2][2] ={0};
	// float Jt[2][2]={0};
	// float JT[2][2]={0};
	
	// d_phi1_L=Motor.Chassis_Joint1.Speed_RPM *2.0f*PI/60.0f;//角速度
	// d_phi4_L=Motor.Chassis_Joint2.Speed_RPM *2.0f*PI/60.0f;//角速度

	// /*===| 计算雅各比矩阵J |===*/
	// J[0][0]= (L1*arm_sin_f32(PHI1-PHI2)*arm_sin_f32(PHI3)) / (arm_sin_f32(PHI2-PHI3));
	// J[0][1]= (L1*arm_sin_f32(PHI3-PHI4)*arm_sin_f32(PHI2)) / (arm_sin_f32(PHI2-PHI3));
	// J[1][0]=-(L1*arm_sin_f32(PHI1-PHI2)*arm_cos_f32(PHI3)) / (arm_sin_f32(PHI2-PHI3));
	// J[1][1]=-(L1*arm_sin_f32(PHI3-PHI4)*arm_cos_f32(PHI2)) / (arm_sin_f32(PHI2-PHI3));
	/*===| 计算矩阵Jᵀ*R*M |===*/
	Jt[0][0]=(L1*arm_sin_f32(PHI0-PHI3)*arm_sin_f32(PHI1-PHI2)) / (arm_sin_f32(PHI3-PHI2));
	Jt[0][1]=(L1*arm_cos_f32(PHI0-PHI3)*arm_sin_f32(PHI1-PHI2)) / (L0*arm_sin_f32(PHI3-PHI2));
	Jt[1][0]=(L1*arm_sin_f32(PHI0-PHI2)*arm_sin_f32(PHI3-PHI4)) / (arm_sin_f32(PHI3-PHI2));
	Jt[1][1]=(L1*arm_cos_f32(PHI0-PHI2)*arm_sin_f32(PHI3-PHI4)) / (L0*arm_sin_f32(PHI3-PHI2));
	
	// /*===| 速度映射部分 |===*/
	// dxc_L=J[0][0]*d_phi1_L + J[0][1]*d_phi4_L;
	// dyc_L=J[1][0]*d_phi1_L + J[1][1]*d_phi4_L;
}

/**
 * @brief 滤波前数据赋值
 * 
 * @param Chassis_Leg_Date 
 * @param LegKinematics_Leg_Date 
 */
static void WheelLeg_Kinematics_Assign(Chassis_Leg_StructTypedef *Chassis_Leg_Date,WheelLeg_LegKinematics_StructTypedef LegKinematics_Leg_Date)
{
    //Last数据赋值
	Chassis_Leg_Date->Length_Last = Chassis_Leg_Date->Length_Feedback;
	Chassis_Leg_Date->Angle_Last =  Chassis_Leg_Date->Angle_Feedback;
	Chassis_Leg_Date->Total_Angle_Last =  Chassis_Leg_Date->Total_Angle_Feedback;
	//Feedback数据赋值
	Chassis_Leg_Date->Length_Feedback = LegKinematics_Leg_Date.Length_Feedback_No_kalman;
	Chassis_Leg_Date->Angle_Feedback  = LegKinematics_Leg_Date.Angle_Feedback_No_kalman;
	//计算总角度值
    if      (Chassis_Leg_Date->Angle_Feedback -  Chassis_Leg_Date->Angle_Last >  180.0f)  Chassis_Leg_Date->Angle_Round--;
    else if (Chassis_Leg_Date->Angle_Feedback -  Chassis_Leg_Date->Angle_Last < -180.0f)  Chassis_Leg_Date->Angle_Round++;
    Chassis_Leg_Date->Total_Angle_Feedback = 360.0f * Chassis_Leg_Date->Angle_Round + Chassis_Leg_Date->Angle_Feedback;
}

//运动学数据更新
void WheelLeg_Kinematics_Update(Chassis_Control_StructTypedef *Chassis_Control_Struct,const float dt)
{
    //运动学结束算
    WheelLeg_ForwardKinematics(&wheelLeg_kinematics_Struct.Left,  Chassis_Control_Struct->Leg_Left.Jt,  Motor.Joint1.Angle, Motor.Joint2.Angle);
    WheelLeg_ForwardKinematics(&wheelLeg_kinematics_Struct.Right, Chassis_Control_Struct->Leg_Right.Jt, Motor.Joint4.Angle, Motor.Joint3.Angle);

    //滤波前数据赋值
    WheelLeg_Kinematics_Assign(&Chassis_Control_Struct->Leg_Left, wheelLeg_kinematics_Struct.Left);
    WheelLeg_Kinematics_Assign(&Chassis_Control_Struct->Leg_Right,wheelLeg_kinematics_Struct.Right);

    //卡尔曼滤波
    //位置-速度卡尔曼滤波
    Leg_State_KF_Update(&Chassis_Control_Struct->Leg_Left,  &L_L0_KF, &L_A0_KF, dt);
    Leg_State_KF_Update(&Chassis_Control_Struct->Leg_Right, &R_L0_KF, &R_A0_KF, dt);
	
}

