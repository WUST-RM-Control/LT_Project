#include "WheelLeg_LQR.h"

#include "WheelLeg_Kinematics.h"
#include "WheelLeg_Motor.h"
#include "INS.h"

#include <math.h>

WheelLeg_LQR wheelLeg_lqr = {0};

#define WHEELLEG_DEG_TO_RAD    0.017453292519943295f
#define WHEELLEG_PITCH_BALANCE_DEG    2.5f
#define WHEELLEG_PITCH_BALANCE_RAD (WHEELLEG_PITCH_BALANCE_DEG * WHEELLEG_DEG_TO_RAD)

// K(ll, lr)的6组拟合系数
// coefficient[k][u][x]

// 与 MATLAB poly_coeffs 的系数顺序完全一致：
//
// k = 0：Ll^2
// k = 1：Ll*Lr
// k = 2：Lr^2
// k = 3：Ll
// k = 4：Lr
// k = 5：常数项


//matlab仿真数据
static const float WheelLeg_LQR_GainCoefficient[WHEELLEG_LQR_GAIN_BASIS_NUM][WHEELLEG_LQR_CONTROL_NUM][WHEELLEG_LQR_STATE_NUM] = {
    {
        {1.122448988e+00f, 3.222140500e+01f, 1.892555802e-02f, 7.195614614e-02f, 3.564723301e+01f, 8.099866086e-01f, 4.435905965e+01f, 6.833450749e+00f, 2.143847117e+01f, 2.753323293e+00f},
        {9.621362296e-01f, 2.764610343e+01f, -6.486189784e-02f, -1.822068372e-01f, 5.529405817e+01f, 5.211933753e+00f, 3.144549961e+01f, 5.498095997e+00f, 1.297381042e+01f, 1.693516730e+00f},
        {6.115009666e+00f, 1.751585449e+02f, 3.237594460e+00f, 9.694133083e+00f, -5.658698114e+02f, -1.070806843e+02f, 5.615808199e+02f, 6.230040716e+01f, 6.252244138e+02f, 5.597242022e+01f},
        {-6.925129284e+00f, -1.985381691e+02f, -1.522276352e+00f, -4.620774102e+00f, 1.084286312e+02f, 4.880102285e+01f, -4.084992009e+02f, -5.024964223e+01f, -4.024643342e+02f, -3.840935293e+01f}
    },
    {
        {-1.494219366e+00f, -4.290862323e+01f, -7.786609268e-02f, -2.347601987e-01f, -3.871417016e+01f, -5.845511614e+00f, -6.835278219e+01f, -1.100551027e+01f, -5.084944819e+01f, -5.512875491e+00f},
        {-1.494219366e+00f, -4.290862323e+01f, 7.786609268e-02f, 2.347601987e-01f, -6.835278219e+01f, -1.100551027e+01f, -3.871417016e+01f, -5.845511614e+00f, -5.084944819e+01f, -5.512875491e+00f},
        {5.400763000e+00f, 1.545705782e+02f, -4.690620851e+00f, -1.418485309e+01f, 9.019958484e+02f, 1.398009004e+02f, -8.035588455e+02f, -1.534452129e+02f, 3.200570966e+02f, 3.300245158e+01f},
        {5.400763000e+00f, 1.545705782e+02f, 4.690620851e+00f, 1.418485309e+01f, -8.035588455e+02f, -1.534452129e+02f, 9.019958484e+02f, 1.398009004e+02f, 3.200570966e+02f, 3.300245158e+01f}
    },
    {
        {9.621362296e-01f, 2.764610343e+01f, 6.486189784e-02f, 1.822068372e-01f, 3.144549961e+01f, 5.498095997e+00f, 5.529405817e+01f, 5.211933753e+00f, 1.297381042e+01f, 1.693516730e+00f},
        {1.122448988e+00f, 3.222140500e+01f, -1.892555802e-02f, -7.195614614e-02f, 4.435905965e+01f, 6.833450749e+00f, 3.564723301e+01f, 8.099866086e-01f, 2.143847117e+01f, 2.753323293e+00f},
        {-6.925129284e+00f, -1.985381691e+02f, 1.522276352e+00f, 4.620774102e+00f, -4.084992009e+02f, -5.024964223e+01f, 1.084286312e+02f, 4.880102285e+01f, -4.024643342e+02f, -3.840935293e+01f},
        {6.115009666e+00f, 1.751585449e+02f, -3.237594460e+00f, -9.694133083e+00f, 5.615808199e+02f, 6.230040716e+01f, -5.658698114e+02f, -1.070806843e+02f, 6.252244138e+02f, 5.597242022e+01f}
    },
    {
        {-3.667258223e-01f, -1.054291766e+01f, 7.999549389e-02f, 2.323897311e-01f, -3.265858384e+01f, -3.009815536e+00f, -5.025626385e+00f, -1.144105498e+00f, 5.409563650e+00f, 2.475451672e-01f},
        {-7.907533272e-02f, -2.299510108e+00f, 7.409950207e-02f, 2.156982095e-01f, -2.633873360e+01f, -2.594856779e+00f, 3.630587946e-01f, -7.822632208e-01f, 1.253811346e+01f, 1.170723303e+00f},
        {-9.723392555e+00f, -2.789749103e+02f, -9.189887176e-01f, -2.730059952e+00f, -4.588501710e+01f, 7.202915800e+00f, -3.462404072e+02f, -3.129444558e+01f, -5.869031314e+02f, -5.600839573e+01f},
        {5.291023273e+00f, 1.520965728e+02f, -5.251760876e-01f, -1.584835253e+00f, 3.247538669e+02f, 4.614924553e+01f, 1.040828113e+02f, 1.062224446e+01f, 2.006504536e+02f, 1.935950720e+01f}
    },
    {
        {-7.907533272e-02f, -2.299510108e+00f, -7.409950207e-02f, -2.156982095e-01f, 3.630587946e-01f, -7.822632208e-01f, -2.633873360e+01f, -2.594856779e+00f, 1.253811346e+01f, 1.170723303e+00f},
        {-3.667258223e-01f, -1.054291766e+01f, -7.999549389e-02f, -2.323897311e-01f, -5.025626385e+00f, -1.144105498e+00f, -3.265858384e+01f, -3.009815536e+00f, 5.409563650e+00f, 2.475451672e-01f},
        {5.291023273e+00f, 1.520965728e+02f, 5.251760876e-01f, 1.584835253e+00f, 1.040828113e+02f, 1.062224446e+01f, 3.247538669e+02f, 4.614924553e+01f, 2.006504536e+02f, 1.935950720e+01f},
        {-9.723392555e+00f, -2.789749103e+02f, 9.189887176e-01f, 2.730059952e+00f, -3.462404072e+02f, -3.129444558e+01f, -4.588501710e+01f, 7.202915800e+00f, -5.869031314e+02f, -5.600839573e+01f}
    },
    {
        {8.980121616e-03f, 2.586699060e-01f, -2.509502442e-02f, -7.183459859e-02f, 6.523055019e-01f, 2.326616653e-01f, 7.250470008e-01f, 2.365430935e-01f, -6.945132876e+00f, -6.785661009e-01f},
        {8.980121616e-03f, 2.586699060e-01f, 2.509502442e-02f, 7.183459859e-02f, 7.250470008e-01f, 2.365430935e-01f, 6.523055019e-01f, 2.326616653e-01f, -6.945132876e+00f, -6.785661009e-01f},
        {1.356658394e+00f, 3.894164440e+01f, -1.840184262e-02f, -4.981860250e-02f, 2.899384190e+01f, 1.699894418e+00f, 2.427620878e+01f, 1.570484581e+00f, 2.019408982e+01f, 4.318711116e+00f},
        {1.356658394e+00f, 3.894164440e+01f, 1.840184262e-02f, 4.981860250e-02f, 2.427620878e+01f, 1.570484581e+00f, 2.899384190e+01f, 1.699894418e+00f, 2.019408982e+01f, 4.318711116e+00f}
    }
};

static uint8_t WheelLeg_LQR_UpdateState(void)
{
    float pitch_error_rad;
    float body_pitch_rad;
    float body_pitch_rate_rad_s;

    uint8_t i;

    // 左右腿运动学必须有效
    if((wheelLeg_kinematics.left.valid == 0) ||(wheelLeg_kinematics.right.valid == 0))
    {
        return 0;
    }

    // 检查IMU数据
    if(!isfinite(INS_Data_Self.Pitch) ||!isfinite(INS_Data_Self.YawTotalAngle) ||!isfinite(INS_Data_Self.Yaw_Speed) ||!isfinite(INS_Data_Self.Gyro[0]))
    {
        return 0;
    }

    // 保存左右腿长
    wheelLeg_lqr.left_leg_length_m =wheelLeg_kinematics.left.length_m;
    wheelLeg_lqr.right_leg_length_m =wheelLeg_kinematics.right.length_m;

    // 检查腿长
    if(!isfinite(wheelLeg_lqr.left_leg_length_m) ||!isfinite(wheelLeg_lqr.right_leg_length_m) || wheelLeg_lqr.left_leg_length_m <= 0.0f || wheelLeg_lqr.right_leg_length_m <= 0.0f)
    {
        return 0;
    }

    // IMU的Pitch：抬头为正  平衡姿态约为2.5度
    pitch_error_rad =(INS_Data_Self.Pitch - WHEELLEG_PITCH_BALANCE_DEG)* WHEELLEG_DEG_TO_RAD;

    // 上交模型theta_b：低头为正 与我们的Pitch方向相反
    body_pitch_rad = -pitch_error_rad;

    // 已经验证Gyro[0]方向与上交theta_b_dot一致
    body_pitch_rate_rad_s = INS_Data_Self.Gyro[0];

    // x0：机器人前后位移
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_S] = 0.5f *(wheelLeg_motor.left_wheel.displacement_m + wheelLeg_motor.right_wheel.displacement_m);

    // x1：机器人前后速度
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_S_DOT] = 0.5f * (wheelLeg_motor.left_wheel.linear_velocity_m_s + wheelLeg_motor.right_wheel.linear_velocity_m_s);

    // x2：Yaw 实车已经验证左转为正，与上交phi方向一致
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_YAW] =INS_Data_Self.YawTotalAngle * WHEELLEG_DEG_TO_RAD;

    // x3：Yaw角速度
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_YAW_DOT] =INS_Data_Self.Yaw_Speed;

    // x4：左腿相对世界坐标系的绝对倾角
    // theta_l = theta + theta_b
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_LEFT_LEG] = wheelLeg_kinematics.left.angle_total_rad  + body_pitch_rad;

    // x5：左腿绝对角速度
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_LEFT_LEG_DOT] = wheelLeg_kinematics.left.angle_velocity_rad_s + body_pitch_rate_rad_s;

    // x6：右腿相对世界坐标系的绝对倾角
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_RIGHT_LEG] = wheelLeg_kinematics.right.angle_total_rad  + body_pitch_rad;

    // x7：右腿绝对角速度
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_RIGHT_LEG_DOT] = wheelLeg_kinematics.right.angle_velocity_rad_s + body_pitch_rate_rad_s;

    // x8：机体Pitch
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_BODY_PITCH] = body_pitch_rad;

    // x9：机体Pitch角速度
    wheelLeg_lqr.state[WHEELLEG_LQR_STATE_BODY_PITCH_DOT] = body_pitch_rate_rad_s;

    // 最后统一检查10个状态
    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        if(!isfinite(wheelLeg_lqr.state[i]))
        {
            return 0;
        }
    }

    return 1;
}

void WheelLeg_LQR_Init(void)
{
    uint8_t i;
    uint8_t j;

    // 清空10维状态
    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        wheelLeg_lqr.state[i] = 0.0f;
        wheelLeg_lqr.target[i] = 0.0f;
        wheelLeg_lqr.error[i] = 0.0f;
    }

    // 清空4x10状态反馈矩阵
    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    {
        for(j = 0; j < WHEELLEG_LQR_STATE_NUM; j++)
        {
            wheelLeg_lqr.gain[i][j] = 0.0f;
        }

        wheelLeg_lqr.output[i] = 0.0f;
    }

    wheelLeg_lqr.left_leg_length_m = 0.0f;
    wheelLeg_lqr.right_leg_length_m = 0.0f;

    wheelLeg_lqr.state_valid = 0;
    wheelLeg_lqr.valid = 0;

    wheelLeg_lqr.target_valid = 0;

    wheelLeg_lqr.gain_valid = 0;
}
//将当前加入LQR的变量作为初始值
void WheelLeg_LQR_CaptureTarget(void)
{
    uint8_t i;

    // 当前状态必须有效
    if(wheelLeg_lqr.state_valid == 0)
    {
        wheelLeg_lqr.target_valid = 0;
        return;
    }

    // 默认所有目标状态都为0
    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        wheelLeg_lqr.target[i] = 0.0f;
    }

    // 进入平衡时锁定当前位置
    wheelLeg_lqr.target[WHEELLEG_LQR_STATE_S] =wheelLeg_lqr.state[WHEELLEG_LQR_STATE_S];

    // 进入平衡时锁定当前Yaw
    wheelLeg_lqr.target[WHEELLEG_LQR_STATE_YAW] =wheelLeg_lqr.state[WHEELLEG_LQR_STATE_YAW];

    wheelLeg_lqr.target_valid = 1;
}

void WheelLeg_LQR_ClearTarget(void)
{
    uint8_t i;

    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        wheelLeg_lqr.target[i] = 0.0f;
        wheelLeg_lqr.error[i] = 0.0f;
    }

    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    {
        wheelLeg_lqr.output[i] = 0.0f;
    }

    wheelLeg_lqr.target_valid = 0;
    wheelLeg_lqr.valid = 0;
}

static uint8_t WheelLeg_LQR_UpdateError(void)
{
    uint8_t i;

    if((wheelLeg_lqr.state_valid == 0) ||(wheelLeg_lqr.target_valid == 0))
    {
        return 0;
    }

    for(i = 0; i < WHEELLEG_LQR_STATE_NUM; i++)
    {
        // 上交控制律：u = K(xd - x)
        wheelLeg_lqr.error[i] =wheelLeg_lqr.target[i] -wheelLeg_lqr.state[i];
    }

    return 1;
}

static uint8_t WheelLeg_LQR_UpdateGain(void)
{
    float ll;
    float lr;
    float basis[WHEELLEG_LQR_GAIN_BASIS_NUM];

    uint8_t u;
    uint8_t x;
    uint8_t k;

    ll = wheelLeg_lqr.left_leg_length_m;
    lr = wheelLeg_lqr.right_leg_length_m;

    // 检查左右腿长
    if(!isfinite(ll) ||!isfinite(lr) ||ll <= 0.0f ||lr <= 0.0f)
    {
        return 0;
    }

    // 当前还没有真实K拟合系数 禁止认为gain有效
    if(WHEELLEG_LQR_GAIN_COEFF_READY == 0U)
    {
        return 0;
    }

    // 与 MATLAB poly_coeffs 的系数顺序保持完全一致
    // Kij = c0*Ll^2 + c1*Ll*Lr + c2*Lr^2+ c3*Ll   + c4*Lr    + c5
    basis[0] = ll * ll;
    basis[1] = ll * lr;
    basis[2] = lr * lr;
    basis[3] = ll;
    basis[4] = lr;
    basis[5] = 1.0f;

    // 计算4×10状态反馈矩阵K
    for(u = 0; u < WHEELLEG_LQR_CONTROL_NUM; u++)
    {
        for(x = 0; x < WHEELLEG_LQR_STATE_NUM; x++)
        {
            wheelLeg_lqr.gain[u][x] = 0.0f;

            for(k = 0; k < WHEELLEG_LQR_GAIN_BASIS_NUM; k++)
            {
                wheelLeg_lqr.gain[u][x] +=WheelLeg_LQR_GainCoefficient[k][u][x]* basis[k];
            }
        }
    }

    return 1;
}

void WheelLeg_LQR_Update(float dt)
{
    uint8_t i;
    uint8_t j;

    // 当前阶段还没有真正输出LQR力矩
    // 先始终保持4个输出为0
    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    {
        wheelLeg_lqr.output[i] = 0.0f;
    }

    // 默认本周期LQR无效
    // 默认本周期各阶段均无效
    wheelLeg_lqr.state_valid = 0;
    wheelLeg_lqr.gain_valid = 0;
    wheelLeg_lqr.valid = 0;

    (void)dt;

    // 1. 更新10维状态
    if(WheelLeg_LQR_UpdateState() == 0)
    {
        return;
    }
    wheelLeg_lqr.state_valid = 1;

    // 2. 根据左右腿长计算当前4×10 K矩阵
    if(WheelLeg_LQR_UpdateGain() == 0)
    {
        return;
    }
    wheelLeg_lqr.gain_valid = 1;

    // 3. 目标值还没有建立时，不继续计算
    if(wheelLeg_lqr.target_valid == 0)
    {
        return;
    }

    // 4. 计算状态误差 xd - x
    if(WheelLeg_LQR_UpdateError() == 0)
    {
        return;
    }

    // 5. LQR状态反馈  u = K * (xd - x)
    for(i = 0; i < WHEELLEG_LQR_CONTROL_NUM; i++)
    {
        for(j = 0; j < WHEELLEG_LQR_STATE_NUM; j++)
        {
            wheelLeg_lqr.output[i] +=wheelLeg_lqr.gain[i][j] *wheelLeg_lqr.error[j];
        }
    }

    // 状态、K矩阵、目标值全部有效
    wheelLeg_lqr.valid = 1;
}