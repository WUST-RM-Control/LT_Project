/**
 * @file    WheelLeg_Output.c[底盘发送电机电流]
 * @brief   LQR控制下传入期望力矩和腿长数据(Feedback和Target)和VMC数据，自动完成关节电机和轮毂电机的电流发送(耗时1ms)
 * @details 关节电机到达限幅：关机两电机等比例限幅，保证虚拟力正确
 *          
 */
#include "WheelLeg_Output.h"
#include "Motor_DAMIAO_Driver.h"
#include "<math>.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//内部结构体定义

// 单条腿最终关节输出 
typedef struct
{
//Leg

    /*===| VMC控关节PID参数 |===*/
    //腿摆角位置环PID
    PID_Struct_TypeDef Leg_Angle_PID;
    //腿摆角速度环PID
    PID_Struct_TypeDef Leg_Angle_Speed_PID;
    //腿长位置环PID
    PID_Struct_TypeDef Leg_Length_PID;
    //腿长速度环PID
    PID_Struct_TypeDef Leg_Length_Speed_PID;

//Motor
    //关节电机力矩的期望力矩NM(未限幅)
    //车头
    float Motor_Target_Torque13;
    float Motor_Target_Torque24;   //车尾

    //关节电机力矩的发送力矩NM(限幅)
    //车头
    float Motor_Send_Torque13;
    float Motor_Send_Torque24;   //车尾

    //轮电机发送扭矩
    float Motor_Send_Torque;

} WheelLeg_Output_Leg_StructTypedef;

// 整个关节输出
typedef struct
{
//Leg
    WheelLeg_Output_Leg_StructTypedef Leg_Left;
    WheelLeg_Output_Leg_StructTypedef Leg_Right;

} WheelLeg_Output_StructTypedef;


//关节电机限幅
#define WheelLeg_Joint_Output_Torque_Limit_NM    40.0f
//轮电机扭矩to电流
#define WheelLeg_Wheel_Output_Current2Torque     3.138094644e+3f

static WheelLeg_Output_StructTypedef WheelLeg_Output_Struct;


//按比例限幅一对关节电机力矩
static void WheelLeg_Output_LimitPair(float *back_nm,float *front_nm,float limit_nm);
//发送电机电流，有1ms阻塞(等待修改fdcan接线后去掉)
static void WheelLeg_Output_Motor_Send_Current(const float Joint1, const float Joint2, const float Joint3, const float Joint4, const float Wheel_Left,const float Wheel_Right);

/**
 * @brief ===| PID控制的初始化 |===
 **/
void WheelLeg_Output_Init(void)
{
    //腿摆角位置环PID
	PID_Init(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID,    80.0f,  0.0f,  	150.0f, 0.0f, 	0.0f,  	150.0f);
	PID_Init(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID,   80.0f,  0.0f,  	150.0f, 0.0f, 	0.0f,  	150.0f);
	//腿摆角速度环PID
	PID_Init(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID,  0.1f, 	0.0f,  	0.0f,   0.15f, 	0.0f,  	30.0f);
	PID_Init(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID, 0.1f, 	0.0f,  	0.0f,   0.15f, 	0.0f,  	30.0f);
	//腿长位置环PID
	PID_Init(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID,   20.0f, 	0.0f, 	500.0f, 	0.0f, 	0.0f, 	3.0f);
	PID_Init(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID,  20.0f, 	0.0f, 	500.0f, 	0.0f, 	0.0f, 	3.0f);
	//腿长速度环PID
	PID_Init(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID, 100.0f, 0.4f, 	0.0f, 	30.0f, 20.0f, 50.0f);
	PID_Init(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID,100.0f, 0.4f, 	0.0f, 	30.0f, 20.0f, 50.0f);

}


/**
 * @brief ===| 电机电流输出 |===
 * 
 * @attention 本函数内部有1ms阻塞(等待修改fdcan接线后去掉)
 * @attention 未考虑轮电机的限速
 * 
 * @note 参数注意：
 * @note 裁判主控一侧为车头，灯条一侧为车尾，以车为参考，左手边为左腿
 * @note 腿在相对机体垂直、竖直向下时腿总摆角为0°+N*360°
 * @note 左腿从左看逆时针为正方向(力和摆角都是)
 * @note 右腿从右看逆时针为正方向(力和摆角都是)
 * @note 沿杆的力伸腿方向为正方向
 * 
 * @note 注意Jt_Left和Jt_Right矩阵的正确
 * @note 关节电机对应关系:关节1(左腿车头)、关节2(左腿车尾)、关节3(右腿车头)、关节4(右腿车尾)
 * @note //VMC:虚拟腿力/力矩经Jt映射成关节电机力矩(Jt行0为后关节，行1为前关节)
 * @note WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Jt_Left[0][0]  * Leg_Left_Link_F  + Jt_Left[0][1]  * Leg_Left_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24   = Jt_Left[1][0]  * Leg_Left_Link_F  + Jt_Left[1][1]  * Leg_Left_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Jt_Right[0][0] * Leg_Right_Link_F + Jt_Right[0][1] * Leg_Right_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24  = Jt_Right[1][0] * Leg_Right_Link_F + Jt_Right[1][1] * Leg_Right_Angle_Torque;
 * 
 * @note 说明：
 * @note Control_State==0:关节电机和轮电机电流全部输出0，函数直接返回0
 * @note Control_State==1:摆角串级PID生效；腿长串级PID生效
 * @note Control_State==2:摆角串级PID不生效，摆角力矩只保留Angle_Torque_NM_Add前馈；腿长串级PID仍然生效
 * @note 虚拟力/力矩经Jt映射成关节电机力矩，每条腿的两个关节按相同比例限幅(WheelLeg_Joint_Output_Torque_Limit_NM)，保证虚拟力方向不变
 * @note Jt_Left/Jt_Right为VMC转置雅可比，直接完成[沿杆的力,摆角力矩]到[后关节,前关节]电机力矩的映射，方向、极性已包含在Jt中，本函数不再乘方向系数
 * 
 * @note 使用：
 * @note 先调用WheelLeg_Output_Init完成PID初始化
 * @note 放在底盘控制周期的最后，传入本周期解算出的雅可比、状态量和期望虚拟力/力矩
 * @note 关节电机实际输出力矩已由本函数限幅,期望虚拟力/力矩 = Add前馈 + K_PID_Control_Weight × PID输出
 * 
 * @param Control_State------------------------0:失能，1:失能且需要PID控制摆角，2:失能且不需要PID控制摆角
 * @param Leg_Left_Link_F_N_Add----------------左腿沿杆的力(伸腿方向为正方向)
 * @param Leg_Left_Angle_Torque_NM_Add---------左腿摆角方向的力矩
 * @param Leg_Right_Link_F_N_Add---------------右腿沿杆的力(伸腿方向为正方向)
 * @param Leg_Right_Angle_Torque_NM_Add--------右腿摆角方向的力矩
 * @param Leg_Left_L_Target--------------------左腿目标腿长(m)
 * @param Leg_Left_L_Feedback------------------左腿当前腿长反馈(m)
 * @param Leg_Left_L_Speed_Feedback------------左腿当前腿长速度反馈(m/s)
 * @param Leg_Left_Total_Angle_Target----------左腿目标总摆角(°)
 * @param Leg_Left_Total_Angle_Feedback--------左腿当前总摆角反馈(°)
 * @param Leg_Left_Total_Angle_Speed_Feedback--左腿当前总摆角速度反馈(°/s)
 * @param Leg_Right_L_Target-------------------右腿目标腿长(m)
 * @param Leg_Right_L_Feedback-----------------右腿当前腿长反馈(m)
 * @param Leg_Right_L_Speed_Feedback-----------右腿当前腿长速度反馈(m/s)
 * @param Leg_Right_Total_Angle_Target---------右腿目标总摆角(°)
 * @param Leg_Right_Total_Angle_Feedback-------右腿当前总摆角反馈(°)
 * @param Leg_Right_Total_Angle_Speed_Feedback-右腿当前总摆角速度反馈(°/s)
 * @param K_PID_Control_Weight-----------------腿长腿摆角PID计算输出的权重系数(Output=PID输出*系数+Add)
 * @param Jt_Left------------------------------左腿VMC转置雅可比[2][2]
 * @param Jt_Right-----------------------------右腿VMC转置雅可比[2][2]
 * @param Wheel_Left_Torque--------------------左轮电机扭矩
 * @param Wheel_Right_Torque-------------------右轮电机扭矩
 * 
 * @return 1：输出电流
 * @return 0：输出0
 * @return -1：Control_State非法，已输出0
 **/
int8_t WheelLeg_Output
(const uint8_t Control_State,

const float Leg_Left_Link_F_N_Add,
const float Leg_Left_Angle_Torque_NM_Add,
const float Leg_Right_Link_F_N_Add,
const float Leg_Right_Angle_Torque_NM_Add,

const float Leg_Left_L_Target,
const float Leg_Left_L_Feedback,
const float Leg_Left_L_Speed_Feedback,
const float Leg_Left_Total_Angle_Target,
const float Leg_Left_Total_Angle_Feedback,
const float Leg_Left_Total_Angle_Speed_Feedback,
const float Leg_Right_L_Target,
const float Leg_Right_L_Feedback,
const float Leg_Right_L_Speed_Feedback,
const float Leg_Right_Total_Angle_Target,
const float Leg_Right_Total_Angle_Feedback,
const float Leg_Right_Total_Angle_Speed_Feedback,

const float K_PID_Control_Weight,

const float Jt_Left[2][2],
const float Jt_Right[2][2],

const float Wheel_Left_Torque,
const float Wheel_Right_Torque
)
{

    //赋个值
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13 = Wheel_Left_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Wheel_Right_Torque;

    //虚拟腿期望力/力矩(沿杆的力、摆角力矩)
    float Leg_Left_Link_F;
    float Leg_Left_Angle_Torque;
    float Leg_Right_Link_F;
    float Leg_Right_Angle_Torque;

    //PID输出(摆角PID不生效时保持为0)
    float Leg_Left_PID_Link_F = 0.0f;
    float Leg_Left_PID_Angle_Torque = 0.0f;
    float Leg_Right_PID_Link_F = 0.0f;
    float Leg_Right_PID_Angle_Torque = 0.0f;

    //失能:关节电机和轮电机电流全部输出0
    if(Control_State == 0)
    {
        WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID.I_Output = 0.0f;
        
        WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID.I_Output = 0.0f;

        WheelLeg_Output_Motor_Send_Current(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return 0;
    }

    //状态非法:不输出，防止误用
    if(Control_State > 2)
    {
        WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID.I_Output = 0.0f;
        
        WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID.I_Output = 0.0f;

        WheelLeg_Output_Motor_Send_Current(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return -1;
    }

    //腿长串级PID:位置外环输出目标腿长速度，速度内环输出虚拟腿力
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID,        Leg_Left_L_Target,                                      Leg_Left_L_Feedback);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID,  WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID.Output,  Leg_Left_L_Speed_Feedback);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID,       Leg_Right_L_Target,                                     Leg_Right_L_Feedback);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID, WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID.Output, Leg_Right_L_Speed_Feedback);

    Leg_Left_PID_Link_F  = WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID.Output;
    Leg_Right_PID_Link_F = WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID.Output;

    //摆角串级PID:位置外环输出目标摆角速度，速度内环输出虚拟摆角力矩(不需要PID控制摆角时不生效)
    if(Control_State == 1)
    {
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID,        Leg_Left_Total_Angle_Target,                           Leg_Left_Total_Angle_Feedback);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID,  WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID.Output,  Leg_Left_Total_Angle_Speed_Feedback);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID,       Leg_Right_Total_Angle_Target,                          Leg_Right_Total_Angle_Feedback);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID, WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID.Output, Leg_Right_Total_Angle_Speed_Feedback);

        Leg_Left_PID_Angle_Torque  = WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID.Output;
        Leg_Right_PID_Angle_Torque = WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID.Output;
    }
    else
    {
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID.I_Output = 0.0f;
        WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID.I_Output = 0.0f;
    }

    //期望虚拟力/力矩 = Add前馈 + K*PID输出(K_PID_Control_Weight用于PID权重和平滑切入)
    Leg_Left_Link_F        = Leg_Left_Link_F_N_Add         + K_PID_Control_Weight * Leg_Left_PID_Link_F;
    Leg_Right_Link_F       = Leg_Right_Link_F_N_Add        + K_PID_Control_Weight * Leg_Right_PID_Link_F;
    Leg_Left_Angle_Torque  = Leg_Left_Angle_Torque_NM_Add  + K_PID_Control_Weight * Leg_Left_PID_Angle_Torque;
    Leg_Right_Angle_Torque = Leg_Right_Angle_Torque_NM_Add + K_PID_Control_Weight * Leg_Right_PID_Angle_Torque;

    //VMC:虚拟腿力/力矩经Jt映射成关节电机力矩(Jt行0为后关节，行1为前关节)
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Jt_Left[0][0]  * Leg_Left_Link_F  + Jt_Left[0][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24  = Jt_Left[1][0]  * Leg_Left_Link_F  + Jt_Left[1][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Jt_Right[0][0] * Leg_Right_Link_F + Jt_Right[0][1] * Leg_Right_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24 = Jt_Right[1][0] * Leg_Right_Link_F + Jt_Right[1][1] * Leg_Right_Angle_Torque;

    
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Jt_Left[0][0]  * Leg_Left_Link_F  + Jt_Left[0][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24  = Jt_Left[1][0]  * Leg_Left_Link_F  + Jt_Left[1][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Jt_Right[0][0] * Leg_Right_Link_F + Jt_Right[0][1] * Leg_Right_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24 = Jt_Right[1][0] * Leg_Right_Link_F + Jt_Right[1][1] * Leg_Right_Angle_Torque;

    //每条腿的两个关节按相同比例限幅，保证虚拟力方向不变
    WheelLeg_Output_LimitPair(&WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24, &WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13, WheelLeg_Joint_Output_Torque_Limit_NM);
    WheelLeg_Output_LimitPair(&WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24, &WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13, WheelLeg_Joint_Output_Torque_Limit_NM);

    //发送关节电机力矩(关节1:左前 关节2:左后 关节3:右前 关节4:右后)，轮电机力矩由其他模块发送故传0
    WheelLeg_Output_Motor_Send_Current(WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13,
                                       WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24,
                                       WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13*WheelLeg_Wheel_Output_Current2Torque,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13*WheelLeg_Wheel_Output_Current2Torque);

    return 1;
}

//按比例限幅一对关节电机力矩
static void WheelLeg_Output_LimitPair(float *back_nm,float *front_nm,float limit_nm)
{
//比较最大值
    //关节电机绝对值
    float back_abs;
    float front_abs;
    float max_abs;
    

    back_abs = fabsf(*back_nm);
    front_abs = fabsf(*front_nm);

    max_abs = back_abs;

    if(front_abs > max_abs)
    {
        max_abs = front_abs;
    }

    //没超过限制，不处理
    if(max_abs <= limit_nm)
    {
        return;
    }

//超过限幅，两个关节按相同比例缩小
    float scale;
    scale = limit_nm / max_abs;

    *back_nm *= scale;
    *front_nm *= scale;
}

/**
 * @brief 发送电机电流，有1ms阻塞(等待修改fdcan接线后去掉)
 * 
 * @note 灯条一侧为车尾
 * @param Joint1 左前
 * @param Joint2 左后
 * @param Joint3 右前
 * @param Joint4 右后
 * @param Wheel_Left 
 * @param Wheel_Right 
 */
static void WheelLeg_Output_Motor_Send_Current
(const float Joint1, 
const float Joint2, 
const float Joint3, 
const float Joint4, 
const float Wheel_Left,
const float Wheel_Right)
{
    Motor_DM_CMD_MIT(&Chassis_JointMotor_CAN, Chassis_JointMotor1_Send_CAN_ID, 0, 0, 0, 0, Joint1);
    Motor_DM_CMD_MIT(&Chassis_JointMotor_CAN, Chassis_JointMotor2_Send_CAN_ID, 0, 0, 0, 0, Joint2);	
    osDelay(1);
    Motor_DM_CMD_MIT(&Chassis_JointMotor_CAN, Chassis_JointMotor3_Send_CAN_ID, 0, 0, 0, 0, Joint3);
    Motor_DM_CMD_MIT(&Chassis_JointMotor_CAN, Chassis_JointMotor4_Send_CAN_ID, 0, 0, 0, 0, Joint4);
    Motor_DJI_SendCurrent(&Chassis_DriverMotor_CAN,Chassis_DriverMotor_Send_CAN_ID,0,0,Wheel_Left,Wheel_Right);
}

