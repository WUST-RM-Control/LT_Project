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
//内部定义

static WheelLeg_Output_StructTypedef WheelLeg_Output_Struct;

//按比例限幅一对关节电机力矩
static void WheelLeg_Output_LimitPair(float *back_nm,float *front_nm,float limit_nm);
//发送电机电流，有1ms阻塞(等待修改fdcan接线后去掉)
static void WheelLeg_Output_Motor_Send_Current(const float Joint1, const float Joint2, const float Joint3, const float Joint4, const float Wheel_Left,const float Wheel_Right);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// //输入数据
// WheelLeg_Output_InputDate_StructTypedef WheelLeg_Output_InputDate_Struct;

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
 * @note WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Chassis_Control->Leg_Left.Jt[0][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[0][1]  * Leg_Left_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24   = Chassis_Control->Leg_Left.Jt[1][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[1][1]  * Leg_Left_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Chassis_Control->Leg_Right.Jt[0][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[0][1] * Leg_Right_Angle_Torque;
 * @note WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24  = Chassis_Control->Leg_Right.Jt[1][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[1][1] * Leg_Right_Angle_Torque;
 * 
 * @note 说明：
 * @note Chassis_Control->Chassis_Control.Chassis_State==0:关节电机和轮电机电流全部输出0，函数直接返回0
 * @note Chassis_Control->Chassis_Control.Chassis_State==1:摆角串级PID生效；腿长串级PID生效
 * @note Chassis_Control->Chassis_Control.Chassis_State==2:摆角串级PID不生效，摆角力矩只保留Angle_Torque_NM_Add前馈；腿长串级PID仍然生效
 * @note 虚拟力/力矩经Jt映射成关节电机力矩，每条腿的两个关节按相同比例限幅(WheelLeg_Joint_Output_Torque_Limit_NM)，保证虚拟力方向不变
 * @note Chassis_Control->Leg_Left.Jt/Jt_Right为VMC转置雅可比，直接完成[沿杆的力,摆角力矩]到[后关节,前关节]电机力矩的映射，方向、极性已包含在Jt中，本函数不再乘方向系数
 * 
 * @note 使用：
 * @note 先调用WheelLeg_Output_Init完成PID初始化
 * @note 放在底盘控制周期的最后，传入本周期解算出的雅可比、状态量和期望虚拟力/力矩
 * @note 关节电机实际输出力矩已由本函数限幅,期望虚拟力/力矩 = Add前馈 + K_PID_Control_Weight × PID输出
 * 
 * @param Chassis_Control->Chassis_Control.Chassis_State------------------------0:失能，1:失能且需要PID控制摆角，2:失能且不需要PID控制摆角
 * @param Chassis_Control->Leg_Left.LQR_Link_Torque_Target[0]----------------左腿沿杆的力(伸腿方向为正方向)
 * @param Chassis_Control->Leg_Left.LQR_Link_Torque_Target[1]---------左腿摆角方向的力矩
 * @param Chassis_Control->Leg_Right.LQR_Link_Torque_Target[0]---------------右腿沿杆的力(伸腿方向为正方向)
 * @param Chassis_Control->Leg_Right.LQR_Link_Torque_Target[1]--------右腿摆角方向的力矩
 * @param Chassis_Control->Leg_Left.Length_Target--------------------左腿目标腿长(m)
 * @param Chassis_Control->Leg_Left.Length_Feedback------------------左腿当前腿长反馈(m)
 * @param Chassis_Control->Leg_Left.Length_Speed------------左腿当前腿长速度反馈(m/s)
 * @param Chassis_Control->Leg_Left.Total_Angle_Target----------左腿目标总摆角(°)
 * @param Chassis_Control->Leg_Left.Total_Angle_Feedback--------左腿当前总摆角反馈(°)
 * @param Chassis_Control->Leg_Left.Total_Angle_Speed--左腿当前总摆角速度反馈(°/s)
 * @param Chassis_Control->Leg_Right.Length_Target-------------------右腿目标腿长(m)
 * @param Chassis_Control->Leg_Right.Length_Feedback-----------------右腿当前腿长反馈(m)
 * @param Chassis_Control->Leg_Right.Length_Speed-----------右腿当前腿长速度反馈(m/s)
 * @param Chassis_Control->Leg_Right.Total_Angle_Target---------右腿目标总摆角(°)
 * @param Chassis_Control->Leg_Right.Total_Angle_Feedback-------右腿当前总摆角反馈(°)
 * @param Chassis_Control->Leg_Right.Total_Angle_Speed-右腿当前总摆角速度反馈(°/s)
 * @param K_PID_Control_Weight-----------------腿长腿摆角PID计算输出的权重系数(Output=PID输出*系数+Add)
 * @param Chassis_Control->Leg_Left.Jt------------------------------左腿VMC转置雅可比[2][2]
 * @param Chassis_Control->Leg_Right.Jt-----------------------------右腿VMC转置雅可比[2][2]
 * @param Chassis_Control->Leg_Left.Wheel_Torque--------------------左轮电机扭矩
 * @param Chassis_Control->Leg_Right.Wheel_Torque-------------------右轮电机扭矩
 * 
 * @return 1：输出电流
 * @return 0：输出0
 **/
WheelLeg_Output(Chassis_Control_StructTypedef *Chassis_Control)
{
    float K_PID_Control_Weight;

    //赋个值
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque = Chassis_Control->Leg_Left.Wheel_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque = Chassis_Control->Leg_Right.Wheel_Torque;

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
    if(Chassis_Control->Chassis_Control.Chassis_State == Chassis_OFF)
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

    //腿长串级PID:位置外环输出目标腿长速度，速度内环输出虚拟腿力
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID,        Chassis_Control->Leg_Left.Length_Target,                                      Chassis_Control->Leg_Left.Length_Feedback);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID,  WheelLeg_Output_Struct.Leg_Left.Leg_Length_PID.Output,  Chassis_Control->Leg_Left.Length_Speed);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID,       Chassis_Control->Leg_Right.Length_Target,                                     Chassis_Control->Leg_Right.Length_Feedback);
    PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID, WheelLeg_Output_Struct.Leg_Right.Leg_Length_PID.Output, Chassis_Control->Leg_Right.Length_Speed);

    Leg_Left_PID_Link_F  = WheelLeg_Output_Struct.Leg_Left.Leg_Length_Speed_PID.Output;
    Leg_Right_PID_Link_F = WheelLeg_Output_Struct.Leg_Right.Leg_Length_Speed_PID.Output;

    //摆角串级PID:位置外环输出目标摆角速度，速度内环输出虚拟摆角力矩(不需要PID控制摆角时不生效)
    if(Chassis_Control->Chassis_Control.Chassis_State == Chassis_Recover
    || Chassis_Control->Chassis_Control.Chassis_State == Chassis_RESET_Slow)
    {
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID,        Chassis_Control->Leg_Left.Total_Angle_Target,                           Chassis_Control->Leg_Left.Total_Angle_Feedback);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Left.Leg_Angle_Speed_PID,  WheelLeg_Output_Struct.Leg_Left.Leg_Angle_PID.Output,  Chassis_Control->Leg_Left.Total_Angle_Speed);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID,       Chassis_Control->Leg_Right.Total_Angle_Target,                          Chassis_Control->Leg_Right.Total_Angle_Feedback);
        PID_Position_Calculate(&WheelLeg_Output_Struct.Leg_Right.Leg_Angle_Speed_PID, WheelLeg_Output_Struct.Leg_Right.Leg_Angle_PID.Output, Chassis_Control->Leg_Right.Total_Angle_Speed);

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
    Leg_Left_Link_F        = Chassis_Control->Leg_Left.LQR_Link_Torque_Target[0]  + K_PID_Control_Weight * Leg_Left_PID_Link_F;
    Leg_Right_Link_F       = Chassis_Control->Leg_Right.LQR_Link_Torque_Target[0] + K_PID_Control_Weight * Leg_Right_PID_Link_F;
    Leg_Left_Angle_Torque  = Chassis_Control->Leg_Left.LQR_Link_Torque_Target[1]  + K_PID_Control_Weight * Leg_Left_PID_Angle_Torque;
    Leg_Right_Angle_Torque = Chassis_Control->Leg_Right.LQR_Link_Torque_Target[1] + K_PID_Control_Weight * Leg_Right_PID_Angle_Torque;

    //VMC:虚拟腿力/力矩经Jt映射成关节电机力矩(Jt行0为后关节，行1为前关节)
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Chassis_Control->Leg_Left.Jt[0][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[0][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24  = Chassis_Control->Leg_Left.Jt[1][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[1][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Chassis_Control->Leg_Right.Jt[0][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[0][1] * Leg_Right_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24 = Chassis_Control->Leg_Right.Jt[1][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[1][1] * Leg_Right_Angle_Torque;

    
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13  = Chassis_Control->Leg_Left.Jt[0][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[0][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24  = Chassis_Control->Leg_Left.Jt[1][0]  * Leg_Left_Link_F  + Chassis_Control->Leg_Left.Jt[1][1]  * Leg_Left_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13 = Chassis_Control->Leg_Right.Jt[0][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[0][1] * Leg_Right_Angle_Torque;
    WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24 = Chassis_Control->Leg_Right.Jt[1][0] * Leg_Right_Link_F + Chassis_Control->Leg_Right.Jt[1][1] * Leg_Right_Angle_Torque;

    //每条腿的两个关节按相同比例限幅，保证虚拟力方向不变
    WheelLeg_Output_LimitPair(&WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24, &WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13, WheelLeg_Joint_Output_Torque_Limit_NM);
    WheelLeg_Output_LimitPair(&WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24, &WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13, WheelLeg_Joint_Output_Torque_Limit_NM);

    //发送关节电机力矩(关节1:左前 关节2:左后 关节3:右前 关节4:右后)
    WheelLeg_Output_Motor_Send_Current(WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque13,
                                       WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque24,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque13,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque24,
                                       WheelLeg_Output_Struct.Leg_Left.Motor_Send_Torque*WheelLeg_Wheel_Output_Current2Torque,
                                       WheelLeg_Output_Struct.Leg_Right.Motor_Send_Torque*WheelLeg_Wheel_Output_Current2Torque);

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

