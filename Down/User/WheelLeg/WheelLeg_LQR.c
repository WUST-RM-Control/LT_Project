/**
 * @file WheelLeg_LQR.c
 * @brief 沿用当前数据接口接入原工程十状态、四输出LQR。
 */
#include "WheelLeg_LQR.h"
#include "INS_Chassis_Data.h"
#include "RoboControl.h"
#include "kalman_filter.h"
#include "user_lib.h"
#include <math.h>

WheelLeg_LQR_Displacement_Date Wheel_Left;
WheelLeg_LQR_Displacement_Date Wheel_Right;
WheelLeg_LQR_StructTypeDef WheelLeg_LQR_Struct;
// Q diagonal: 50 3000 1000 5 400 5 400 5 60000 50 
// Q off-diagonal: Q(5,7)=0, Q(7,5)=0
// R diagonal: 100 100 5 5 
// 直接使用原工程当前启用的6x4x10系数，基项顺序：Ll²,LlLr,Lr²,Ll,Lr,1。
static const float Poly_Coefficient[WHEELLEG_LQR_GAIN_BASIS_NUM][WHEELLEG_LQR_CONTROL_NUM][WHEELLEG_LQR_STATE_NUM] = {
    {
        {2.301974909e+00f, 1.849853720e+01f, -7.092222871e+00f, -1.749562090e+00f, 3.024629144e+01f, -1.375540576e+00f, 4.476683592e+00f, 2.341203354e+00f, -2.556196057e+01f, -5.775318018e-01f},
        {4.031577753e-01f, 3.220545489e+00f, -4.142880286e+00f, -1.031192221e+00f, 2.427538233e+01f, 2.409037550e-01f, -5.302864837e+00f, 1.393634622e-01f, -1.311489139e+01f, -1.419245825e+00f},
        {7.727798196e+00f, 6.210317974e+01f, 3.316774451e+01f, 1.073941509e+01f, -1.436540300e+02f, -1.272840501e+01f, 1.883222505e+02f, 1.859715608e+01f, 5.075382013e+02f, 3.962785882e+01f},
        {7.788990386e-01f, 6.468029332e+00f, -1.014470246e+01f, -3.168105187e+00f, 3.418556811e+01f, 1.448793320e+00f, -2.874283318e+01f, -1.055902361e+00f, -1.282938002e+02f, -8.328664628e+00f}
    },
    {
        {1.146711478e-01f, 1.005010523e+00f, 1.083773633e+00f, 4.131677500e-01f, 6.182825874e+00f, 5.655340033e-01f, 1.405112007e+01f, -7.182937314e-01f, -3.620024546e+01f, -3.924482955e+00f},
        {1.146711478e-01f, 1.005010523e+00f, -1.083773633e+00f, -4.131677500e-01f, 1.405112007e+01f, -7.182937314e-01f, 6.182825874e+00f, 5.655340033e-01f, -3.620024546e+01f, -3.924482955e+00f},
        {1.204859215e+00f, 8.731730249e+00f, -2.119720808e+01f, -8.341400888e+00f, 1.230264871e+02f, 1.006146075e+01f, -2.474397435e+02f, -3.870948735e+01f, 1.243306929e+02f, 1.326066692e+01f},
        {1.204859215e+00f, 8.731730249e+00f, 2.119720808e+01f, 8.341400888e+00f, -2.474397435e+02f, -3.870948735e+01f, 1.230264871e+02f, 1.006146075e+01f, 1.243306929e+02f, 1.326066692e+01f}
    },
    {
        {4.031577753e-01f, 3.220545489e+00f, 4.142880286e+00f, 1.031192221e+00f, -5.302864837e+00f, 1.393634622e-01f, 2.427538233e+01f, 2.409037550e-01f, -1.311489139e+01f, -1.419245825e+00f},
        {2.301974909e+00f, 1.849853720e+01f, 7.092222871e+00f, 1.749562090e+00f, 4.476683592e+00f, 2.341203354e+00f, 3.024629144e+01f, -1.375540576e+00f, -2.556196057e+01f, -5.775318018e-01f},
        {7.788990386e-01f, 6.468029332e+00f, 1.014470246e+01f, 3.168105187e+00f, -2.874283318e+01f, -1.055902361e+00f, 3.418556811e+01f, 1.448793320e+00f, -1.282938002e+02f, -8.328664628e+00f},
        {7.727798196e+00f, 6.210317974e+01f, -3.316774451e+01f, -1.073941509e+01f, 1.883222505e+02f, 1.859715608e+01f, -1.436540300e+02f, -1.272840501e+01f, 5.075382013e+02f, 3.962785882e+01f}
    },
    {
        {-1.710640939e+00f, -1.391717609e+01f, 6.684902366e+00f, 1.919890414e+00f, -5.097562092e+01f, -5.019639131e+00f, 2.079321984e+00f, -1.060398646e+00f, 4.112704314e+01f, 2.896693902e+00f},
        {-2.348907021e-01f, -1.999512751e+00f, 3.960947931e+00f, 1.182442090e+00f, -3.564936405e+01f, -3.755429572e+00f, 6.829634258e+00f, 1.587083422e-02f, 2.800709165e+01f, 2.898070578e+00f},
        {-8.277341068e+00f, -6.629750598e+01f, -1.730617103e+01f, -5.829875056e+00f, 7.845149094e+01f, 9.326033155e+00f, -1.042574436e+02f, -8.287247950e+00f, -4.143840713e+02f, -3.548270089e+01f},
        {-4.521860046e-01f, -3.405200336e+00f, 2.920369669e+00f, 5.545206972e-01f, 3.932157500e+01f, 1.331074245e+01f, -5.318398254e-01f, -5.868828023e-01f, 6.576686497e+01f, 3.964023859e+00f}
    },
    {
        {-2.348907021e-01f, -1.999512751e+00f, -3.960947931e+00f, -1.182442090e+00f, 6.829634258e+00f, 1.587083422e-02f, -3.564936405e+01f, -3.755429572e+00f, 2.800709165e+01f, 2.898070578e+00f},
        {-1.710640939e+00f, -1.391717609e+01f, -6.684902366e+00f, -1.919890414e+00f, 2.079321984e+00f, -1.060398646e+00f, -5.097562092e+01f, -5.019639131e+00f, 4.112704314e+01f, 2.896693902e+00f},
        {-4.521860046e-01f, -3.405200336e+00f, -2.920369669e+00f, -5.545206972e-01f, -5.318398254e-01f, -5.868828023e-01f, 3.932157500e+01f, 1.331074245e+01f, 6.576686497e+01f, 3.964023859e+00f},
        {-8.277341068e+00f, -6.629750598e+01f, 1.730617103e+01f, 5.829875056e+00f, -1.042574436e+02f, -8.287247950e+00f, 7.845149094e+01f, 9.326033155e+00f, -4.143840713e+02f, -3.548270089e+01f}
    },
    {
        {-1.300579016e-01f, -1.045577001e+00f, -2.187557526e+00f, -5.175667055e-01f, -4.077827797e-01f, 1.576252832e-01f, -1.762447400e-01f, 2.422502549e-01f, -2.256106586e+01f, -2.103316234e+00f},
        {-1.300579016e-01f, -1.045577001e+00f, 2.187557526e+00f, 5.175667055e-01f, -1.762447400e-01f, 2.422502549e-01f, -4.077827797e-01f, 1.576252832e-01f, -2.256106586e+01f, -2.103316234e+00f},
        {2.644511695e+00f, 2.130874701e+01f, -3.214735841e+00f, -6.407540580e-01f, 1.599254479e+01f, 1.223697686e+00f, 1.134674263e+01f, 7.347248338e-01f, -1.215213181e+01f, 1.420188620e+00f},
        {2.644511695e+00f, 2.130874701e+01f, 3.214735841e+00f, 6.407540580e-01f, 1.134674263e+01f, 7.347248338e-01f, 1.599254479e+01f, 1.223697686e+00f, -1.215213181e+01f, 1.420188620e+00f}
    }
};

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
float L_vaEstimateKF_R[4] = {30.0f,  0.0f,   0.0f, 500000.0f}; 	  
								 
float R_vaEstimateKF_Q[4] = {0.05f, 0.0f, 0.0f, 0.01f};     // Q矩阵初始值
float R_vaEstimateKF_R[4] = {30.0f,  0.0f, 0.0f, 500000.0f};  	
						 
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
static void WheelLeg_LQR_Calculated_Displacement(Chassis_Control_StructTypedef *Chassis_Control)
{
    Wheel_Left.Displacement_Now = Motor.Wheel_Motor1.Total_Angle / Motor_3508_Reduction * Chassis_Wheel_Radius * Angle_to_Radain;
    Wheel_Right.Displacement_Now = Motor.Wheel_Motor2.Total_Angle / Motor_3508_Reduction * Chassis_Wheel_Radius * Angle_to_Radain;
    // 与原工程wheelLeg_motor一致：前进左负右正，退出平衡不重置编码器基准。
    Wheel_Left.Displacement = -(Wheel_Left.Displacement_Now - Wheel_Left.Displacement_Init);
    Wheel_Right.Displacement = Wheel_Right.Displacement_Now - Wheel_Right.Displacement_Init;
    WheelLeg_LQR_Struct.Wheel_Displacement_Feedback = 0.5f * (Wheel_Left.Displacement + Wheel_Right.Displacement);
    Wheel_Left.Wheel_Speed_Feedback = -Motor.Wheel_Motor1.Speed_RPM * Chassis_RPM_to_m_s;
    Wheel_Right.Wheel_Speed_Feedback = Motor.Wheel_Motor2.Speed_RPM * Chassis_RPM_to_m_s;

    // 沿用当前速度KF，状态转移使用本周期实际dt。
    L_vaEstimateKF.F_data[1] = Chassis_Control->Chassis_Dt;
    R_vaEstimateKF.F_data[1] = Chassis_Control->Chassis_Dt;
    xvEstimateKF_Update(&L_vaEstimateKF, Chassis_Control->Chassis_Control.Acceleration_Y, Wheel_Left.Wheel_Speed_Feedback);
    xvEstimateKF_Update(&R_vaEstimateKF, Chassis_Control->Chassis_Control.Acceleration_Y, Wheel_Right.Wheel_Speed_Feedback);
    WheelLeg_LQR_Struct.Wheel_Speed_Feedback = 0.5f * (L_vaEstimateKF.FilteredValue[0] + R_vaEstimateKF.FilteredValue[0]);
    Chassis_Control->Chassis_Control.Chassis_Speed_Feedback = WheelLeg_LQR_Struct.Wheel_Speed_Feedback;
}

static uint8_t WheelLeg_LQR_add_ten_state(const Chassis_Control_StructTypedef *Chassis_Control)
{
    uint8_t i, j;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_S] = WheelLeg_LQR_Struct.Wheel_Displacement_Feedback;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_S_DOT] = WheelLeg_LQR_Struct.Wheel_Speed_Feedback;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW] = INS_Chassis_Data.YawTotalAngle * Angle2Radain;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW_DOT] = INS_Chassis_Data.Yaw_Speed;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH] = -Chassis_Control->Chassis_Control.Pitch * Angle2Radain;
    // 等价于原工程BodyPitchRate=+INS.Gyro[X]，不额外减安装补偿角。
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH_DOT] = -INS_Chassis_Data.Pitch_dot * Angle2Radain;
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_LEFT_LEG] = Chassis_Control->Leg_Left.Angle_Feedback * Angle2Radain
        + WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH];
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_LEFT_LEG_DOT] = Chassis_Control->Leg_Left.Total_Angle_Speed * Angle2Radain
        + WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH_DOT];
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_RIGHT_LEG] = -Chassis_Control->Leg_Right.Angle_Feedback * Angle2Radain
        + WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH];
    WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_RIGHT_LEG_DOT] = -Chassis_Control->Leg_Right.Total_Angle_Speed * Angle2Radain
        + WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_BODY_PITCH_DOT];

    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        if(!isfinite(WheelLeg_LQR_Struct.State[i])) return 0;
    }
    for(i = 0; i < 2; i++)
    for(j = 0; j < 2; j++)
    {
        if(!isfinite(Chassis_Control->Leg_Left.Jt[i][j]) || !isfinite(Chassis_Control->Leg_Right.Jt[i][j])) return 0;
    }
    return 1;
}

static uint8_t calucateK(float Length_Left_Feedback, float Length_Right_Feedback)
{
    float basis[WHEELLEG_LQR_GAIN_BASIS_NUM];
    uint8_t i, j, k;
    if(!isfinite(Length_Left_Feedback) || !isfinite(Length_Right_Feedback) ||
       Length_Left_Feedback < WHEELLEG_LQR_LENGTH_MIN_M || Length_Left_Feedback > WHEELLEG_LQR_LENGTH_MAX_M ||
       Length_Right_Feedback < WHEELLEG_LQR_LENGTH_MIN_M || Length_Right_Feedback > WHEELLEG_LQR_LENGTH_MAX_M) return 0;
    basis[0] = Length_Left_Feedback * Length_Left_Feedback;
    basis[1] = Length_Left_Feedback * Length_Right_Feedback;
    basis[2] = Length_Right_Feedback * Length_Right_Feedback;
    basis[3] = Length_Left_Feedback;
    basis[4] = Length_Right_Feedback;
    basis[5] = 1.0f;
    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    for(j = 0; j < WHEELLEG_LQR_STATE_NUM; j++)
    {
        WheelLeg_LQR_Struct.LQR_K[i][j] = 0.0f;
        for(k = 0; k < WHEELLEG_LQR_GAIN_BASIS_NUM; k++)
            WheelLeg_LQR_Struct.LQR_K[i][j] += Poly_Coefficient[k][i][j] * basis[k];
        if(!isfinite(WheelLeg_LQR_Struct.LQR_K[i][j])) return 0;
    }
    return 1;
}

static void WheelLeg_LQR_ClearTarget(void)
{
    memset(WheelLeg_LQR_Struct.Target, 0, sizeof(WheelLeg_LQR_Struct.Target));
    memset(WheelLeg_LQR_Struct.Error, 0, sizeof(WheelLeg_LQR_Struct.Error));
    WheelLeg_LQR_Struct.Target_Valid = 0;
    WheelLeg_LQR_Struct.Wheel_Speed_Target = 0.0f;
    WheelLeg_LQR_Struct.Wheel_Speed_Target_Last = 0.0f;
    WheelLeg_LQR_Struct.Wheel_displacement_add = 0.0f;
    WheelLeg_LQR_Struct.Chassis_State_Last = Chassis_OFF;
}

// 在现有低通之后限制纵向变化率，Chassis_Acc/Chassis_Dec单位m/s²，Dt单位s。
static float WheelLeg_LQR_ForwardRamp(float Target, float Output, float Dt)
{
    float max_step;
    float delta;

    // 正反切换先以减速度回到零，下一周期再向反方向加速。
    if(Target * Output < 0.0f) Target = 0.0f;
    max_step = (fabsf(Target) > fabsf(Output) ? Chassis_Acc : Chassis_Dec) * Dt;
    delta = Target - Output;
    if(delta > max_step) return Output + max_step;    if(delta < -max_step) return Output - max_step;
    return Target;
}

static void WheelLeg_LQR_Calculated_Target(Chassis_Control_StructTypedef *Chassis_Control, uint8_t Off_ground_Flag)
{
    uint8_t is_moving, was_moving;
    float yaw_error, yaw_max_step;
    float dt = Chassis_Control->Chassis_Dt;
    Chassis_State_EnumTypedef state = Chassis_Control->Chassis_Control.Chassis_State;

    if(WheelLeg_LQR_Struct.Target_Valid == 0)
    {
        memset(WheelLeg_LQR_Struct.Target, 0, sizeof(WheelLeg_LQR_Struct.Target));
        WheelLeg_LQR_Struct.Wheel_Displacement_Target = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_S];
        WheelLeg_LQR_Struct.Wheel_displacement_init = WheelLeg_LQR_Struct.Wheel_Displacement_Target;
        WheelLeg_LQR_Struct.Wheel_displacement_add = 0.0f;
        WheelLeg_LQR_Struct.Wheel_Speed_Target_Last = 0.0f;
        // 仅在重新捕获目标时复位斜坡，失能和自起后不继承上次运动速度。
        WheelLeg_LQR_Struct.Wheel_Speed_Target = 0.0f;
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW] = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW];
        WheelLeg_LQR_Struct.Target_Valid = 1;
    }

    // Vy_Target已由RoboControl低通，本处不重复滤波；速度目标和位移积分共用斜坡结果。
    WheelLeg_LQR_Struct.Wheel_Speed_Target = WheelLeg_LQR_ForwardRamp(
        Chassis_Control->Chassis_Control.Vy_Target,
        WheelLeg_LQR_Struct.Wheel_Speed_Target, dt);
    is_moving = fabsf(WheelLeg_LQR_Struct.Wheel_Speed_Target) > 0.05f;
    was_moving = fabsf(WheelLeg_LQR_Struct.Wheel_Speed_Target_Last) > 0.05f;
    if(is_moving && !was_moving)
    {
        WheelLeg_LQR_Struct.Wheel_displacement_init = WheelLeg_LQR_Struct.Wheel_Displacement_Target;
        WheelLeg_LQR_Struct.Wheel_displacement_add = 0.0f;
    }
    if(is_moving)
    {
        WheelLeg_LQR_Struct.Wheel_displacement_add += WheelLeg_LQR_Struct.Wheel_Speed_Target * dt;
        WheelLeg_LQR_Struct.Wheel_Displacement_Target = WheelLeg_LQR_Struct.Wheel_displacement_init + WheelLeg_LQR_Struct.Wheel_displacement_add;
    }
    WheelLeg_LQR_Struct.Wheel_Speed_Target_Last = WheelLeg_LQR_Struct.Wheel_Speed_Target;
    Chassis_Control->Chassis_Control.Vy_Target_Last = Chassis_Control->Chassis_Control.Vy_Target;
    WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_S] = WheelLeg_LQR_Struct.Wheel_Displacement_Target;
    WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_S_DOT] = WheelLeg_LQR_Struct.Wheel_Speed_Target;

    if(state != WheelLeg_LQR_Struct.Chassis_State_Last)
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW] = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW];
    WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW_DOT] = Chassis_Control->Chassis_Control.Wz_Speed_Target;
    if(state == Chassis_FOLLOW || state == Chassis_UpStep)
    {
        // 与原工程FOLLOW一致：云台方向给直接角度参考，Wz限定每周期逼近步长。
        yaw_error = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW]
                  + (RoboControl_Struct.Target_Direction - RoboControl_Struct.Target_Direction_Err) * Angle2Radain
                  - WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW];
        yaw_error = fmodf(yaw_error, 2.0f * PI);
        if(yaw_error > PI) yaw_error -= 2.0f * PI;
        if(yaw_error < -PI) yaw_error += 2.0f * PI;
        yaw_max_step = fabsf(Chassis_Control->Chassis_Control.Wz_Speed_Target) * dt;
        Limit_float(&yaw_error, yaw_max_step, -yaw_max_step);
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW] += yaw_error;
    }
    else
    {
        // SPIN使用连续航向目标，不折回±180°。
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW] += Chassis_Control->Chassis_Control.Wz_Speed_Target * dt;
    }
    WheelLeg_LQR_Struct.Chassis_State_Last = state;
    if(Off_ground_Flag)
    {
        // 空中不积累地面位置/航向目标，落地重新捕获。
        WheelLeg_LQR_Struct.Wheel_Displacement_Target = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_S];
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_S] = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_S];
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_S_DOT] = 0.0f;
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW] = WheelLeg_LQR_Struct.State[WHEELLEG_LQR_STATE_YAW];
        WheelLeg_LQR_Struct.Target[WHEELLEG_LQR_STATE_YAW_DOT] = 0.0f;
    }
}

static uint8_t WheelLeg_LQR_calculation(uint8_t Off_ground_Flag)
{
    uint8_t i, j;
    for(j = 0; j < WHEELLEG_LQR_STATE_NUM; j++)
    {
        // 原工程控制律：u=K*(目标-实际)，不再套旧六状态控制器的负号。
        WheelLeg_LQR_Struct.Error[j] = WheelLeg_LQR_Struct.Target[j] - WheelLeg_LQR_Struct.State[j];
        if(!isfinite(WheelLeg_LQR_Struct.Error[j])) return 0;
    }
    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    {
        WheelLeg_LQR_Struct.LQR_Output[i] = 0.0f;
        for(j = 0; j < WHEELLEG_LQR_STATE_NUM; j++)
        {
            // 保留当前离地策略：轮子置零，摆腿只保留腿角/角速度项。
            if(Off_ground_Flag && (i < WHEELLEG_LQR_OUTPUT_LEFT_LEG || j < WHEELLEG_LQR_STATE_LEFT_LEG || j > WHEELLEG_LQR_STATE_RIGHT_LEG_DOT)) continue;
            WheelLeg_LQR_Struct.LQR_Output[i] += WheelLeg_LQR_Struct.LQR_K[i][j] * WheelLeg_LQR_Struct.Error[j];
        }
        if(!isfinite(WheelLeg_LQR_Struct.LQR_Output[i])) return 0;
    }
    return 1;
}

void WheelLeg_LQR_Init(void)
{
    memset(&WheelLeg_LQR_Struct, 0, sizeof(WheelLeg_LQR_Struct));
    memset(&Wheel_Left, 0, sizeof(Wheel_Left));
    memset(&Wheel_Right, 0, sizeof(Wheel_Right));
    xvEstimateKF_Init(&L_vaEstimateKF, &R_vaEstimateKF);
    // PID_Init(&WheelLeg_LQR_Struct.Wheel_Wz_Speed_PID, 3.0f, 0.0f, 0.1f, 0.0f, 0.0f, 3.0f);
    // PID_Init(&WheelLeg_LQR_Struct.Leg_Splits_PID, 4.0f, 0.0f, 0.1f, 0.0f, 0.0f, 10.0f);
}

void WheelLeg_LQR_Output(Chassis_Control_StructTypedef *Chassis_Control)
{
    uint8_t Off_ground_Flag;
    Chassis_State_EnumTypedef state = Chassis_Control->Chassis_Control.Chassis_State;
    // 每周期清理LQR接收字段，不清除跳跃轴向力[0]。
    Chassis_Control->Leg_Left.LQR_Wheel_Torque_Target = 0.0f;
    Chassis_Control->Leg_Right.LQR_Wheel_Torque_Target = 0.0f;
    Chassis_Control->Leg_Left.LQR_Link_Torque_Target[1] = 0.0f;
    Chassis_Control->Leg_Right.LQR_Link_Torque_Target[1] = 0.0f;
    memset(WheelLeg_LQR_Struct.LQR_Output, 0, sizeof(WheelLeg_LQR_Struct.LQR_Output));
    WheelLeg_LQR_Struct.State_Valid = WheelLeg_LQR_Struct.Gain_Valid = WheelLeg_LQR_Struct.Valid = 0;

    if(!isfinite(Chassis_Control->Chassis_Dt) || Chassis_Control->Chassis_Dt <= 0.0f || Chassis_Control->Chassis_Dt > 0.03f ||
       !isfinite(Chassis_Control->Chassis_Control.Acceleration_Y) ||
       !isfinite(Motor.Wheel_Motor1.Total_Angle) || !isfinite(Motor.Wheel_Motor2.Total_Angle) ||
       !isfinite(Motor.Wheel_Motor1.Speed_RPM) || !isfinite(Motor.Wheel_Motor2.Speed_RPM))
    {
        WheelLeg_LQR_ClearTarget();
        return;
    }
    WheelLeg_LQR_Calculated_Displacement(Chassis_Control);
    if(INS_Chassis_Data.If_Ready == 0 || !WheelLeg_LQR_add_ten_state(Chassis_Control))
    {
        WheelLeg_LQR_ClearTarget();
        return;
    }
    WheelLeg_LQR_Struct.State_Valid = 1;
    if((state != Chassis_FOLLOW && state != Chassis_SPIN && state != Chassis_Jump && state != Chassis_UpStep) ||
       (state == Chassis_UpStep && Chassis_Control->WheelLeg_Control_Flag.Up_Step_Flag == 2) ||
       !Motor.Wheel_Motor1.If_Online || !Motor.Wheel_Motor2.If_Online ||
       !Motor.Joint1.If_Online || !Motor.Joint2.If_Online || !Motor.Joint3.If_Online || !Motor.Joint4.If_Online ||
       !isfinite(Chassis_Control->Chassis_Control.Vy_Target) || !isfinite(Chassis_Control->Chassis_Control.Wz_Speed_Target) ||
       !isfinite(RoboControl_Struct.Target_Direction) || !isfinite(RoboControl_Struct.Target_Direction_Err) ||
       !calucateK(Chassis_Control->Leg_Left.Length_Feedback, Chassis_Control->Leg_Right.Length_Feedback))
    {
        WheelLeg_LQR_ClearTarget();
        return;
    }
    WheelLeg_LQR_Struct.Gain_Valid = 1;
    Off_ground_Flag = Chassis_Control->WheelLeg_Control_Flag.Off_ground_Flag ||
        (state == Chassis_Jump && Chassis_Control->WheelLeg_Control_Flag.Chassis_Jump_FLag == 3);
    WheelLeg_LQR_Calculated_Target(Chassis_Control, Off_ground_Flag);
    if(!WheelLeg_LQR_calculation(Off_ground_Flag))
    {
        memset(WheelLeg_LQR_Struct.LQR_Output, 0, sizeof(WheelLeg_LQR_Struct.LQR_Output));
        WheelLeg_LQR_ClearTarget();
        return;
    }

    // 原工程完整电机链路等价映射：Tw左负右正，Tp左正右负。
    Chassis_Control->Leg_Left.LQR_Wheel_Torque_Target = -WheelLeg_LQR_Struct.LQR_Output[WHEELLEG_LQR_OUTPUT_LEFT_WHEEL];
    Chassis_Control->Leg_Right.LQR_Wheel_Torque_Target = WheelLeg_LQR_Struct.LQR_Output[WHEELLEG_LQR_OUTPUT_RIGHT_WHEEL];
    Chassis_Control->Leg_Left.LQR_Link_Torque_Target[1] = WheelLeg_LQR_Struct.LQR_Output[WHEELLEG_LQR_OUTPUT_LEFT_LEG];
    Chassis_Control->Leg_Right.LQR_Link_Torque_Target[1] = -WheelLeg_LQR_Struct.LQR_Output[WHEELLEG_LQR_OUTPUT_RIGHT_LEG];
    Limit_float(&Chassis_Control->Leg_Left.LQR_Wheel_Torque_Target, 5.0f, -5.0f);
    Limit_float(&Chassis_Control->Leg_Right.LQR_Wheel_Torque_Target, 5.0f, -5.0f);
    Limit_float(&Chassis_Control->Leg_Left.LQR_Link_Torque_Target[1], 80.0f, -80.0f);
    Limit_float(&Chassis_Control->Leg_Right.LQR_Link_Torque_Target[1], 80.0f, -80.0f);
    // 不调用旧WheelLeg_LQR_PID_Patch，不叠加Wheel_Wz_Speed_PID或Leg_Splits_PID。
    WheelLeg_LQR_Struct.Valid = 1;
    if(Off_ground_Flag) WheelLeg_LQR_Struct.Target_Valid = 0;
}
