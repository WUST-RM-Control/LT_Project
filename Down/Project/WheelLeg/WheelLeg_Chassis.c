/**
 * @file    WheelLeg_Chassis.c[底盘任务]
 * @brief   轮腿底盘任务
 * @details 底盘有关的一些任务函数
 */
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"

#include "WheelLeg_Chassis.h"
#include "WheelLeg_Motor.h"
#include "WheelLeg_Kinematics.h"
#include "WheelLeg_VMC.h"
#include "WheelLeg_LegController.h"
#include "WheelLeg_SpringComp.h"
#include "WheelLeg_LQR.h"
#include "WheelLeg_Output.h"

#include <math.h>

//调试变量
volatile float WheelLeg_Control_dt = 0.0f;

//安全锁
volatile uint8_t WheelLeg_Safety_Lock = 1;

//轮电机零电流、关节电机失能及底盘控制函数，状态机选择函数声明
static void WheelLeg_Wheel_ZeroCurrent(void);
static void WheelLeg_Joint_Disable(void);
static void WheelLeg_Safety_Control(void);
static void WheelLeg_Update_ControlCommand(void);
static void WheelLeg_Update_ControlMode(void);
static void WheelLeg_Clear_ControlCommand(void);
static void WheelLeg_Update_ControlTarget(void);
static void WheelLeg_Update_VMCCommand(void);

//定义力矩限制
#define WHEELLEG_TEST_TP_LIMIT_NM       1.0f
#define WHEELLEG_TEST_WHEEL_LIMIT_NM    0.3f

//定义目标腿长
#define WHEELLEG_PREPARE_LENGTH_M    0.15f

#define WHEELLEG_PREPARE_LENGTH_TOL_M    0.02f

WheelLeg_ControlCommand wheelLeg_controlCommand = {0};

//状态机初始化
WheelLeg_ControlMode wheelLeg_controlMode =WHEELLEG_MODE_DISABLE;
volatile uint8_t WheelLeg_Enable_Request  = 0;

// 进入平衡模式的请求
// 当前默认关闭，防止PREPARE完成后自动进入LQR
volatile uint8_t WheelLeg_Balance_Request = 0;

// 正常平衡阶段目标腿长
#define WHEELLEG_BALANCE_LENGTH_M    0.18f
void WheelLeg_Chassis_Task(void const *argument)
{
	uint32_t wheelleg_dwt_count = 0;
	osDelay(100);
	
    //获取周期 
	DWT_GetDeltaT(&wheelleg_dwt_count);

	WheelLeg_Wheel_ZeroCurrent();
	WheelLeg_Joint_Disable();

	WheelLeg_LegController_Init();
    WheelLeg_SpringComp_Init();
	WheelLeg_LQR_Init();
    WheelLeg_Output_Init();

	for(;;)
	{
		
		

    /* =========================================================
     * 1. 更新反馈状态
     * ========================================================= */

    // 获取实际控制周期
    WheelLeg_Control_dt = DWT_GetDeltaT(&wheelleg_dwt_count);

    // 更新轮电机、关节电机反馈
    WheelLeg_Motor_Update();

    // 五连杆正运动学：
    // 计算左右腿长、腿角、腿长速度、腿角速度等
    WheelLeg_Kinematics_Update(WheelLeg_Control_dt);

    // 根据当前左右腿长更新氮气弹簧补偿
    WheelLeg_SpringComp_Update(wheelLeg_kinematics.left.length_m,wheelLeg_kinematics.right.length_m);

    /* =========================================================
     * 2. 状态机
     * ========================================================= */

    // 根据 Safety / Enable / Balance Request
    // 决定当前处于 DISABLE / PREPARE / BALANCE
    WheelLeg_Update_ControlMode();

    /* =========================================================
     * 3. 执行器使能管理
     * ========================================================= */

    // 当前仍处于单右腿调试阶段：
    // DISABLE：左右腿都关闭
    // 其他模式：左腿关闭，右腿开启
    if(wheelLeg_controlMode == WHEELLEG_MODE_DISABLE)
    {
        WheelLeg_Output_SetLegEnable(0, 0);
    }
    else
    {
        WheelLeg_Output_SetLegEnable(0, 1);
    }


    /* =========================================================
     * 4. 更新控制目标
     * ========================================================= */

    WheelLeg_Update_ControlTarget();


    /* =========================================================
     * 5. 控制器计算
     * ========================================================= */

    // 腿部控制器：腿长控制 + Roll + 重力/弹簧等
    WheelLeg_LegController_Update();

    // 10状态变腿长LQR：state -> error -> K(Ll,Lr) -> TwL/TwR/TpL/TpR
    WheelLeg_LQR_Update(WheelLeg_Control_dt);


    /* =========================================================
     * 6. 汇总最终控制命令
     * ========================================================= */
    WheelLeg_Update_ControlCommand();

    /* =========================================================
     * 7. 虚拟力 / 虚拟力矩 -> 关节力矩
     * ========================================================= */
    // 把 ControlCommand 中的 F、Tp 送入左右腿VMC
    WheelLeg_Update_VMCCommand();

    // VMC：
    WheelLeg_VMC_Update();

    /* =========================================================
     * 8. 关节电机输出
     * ========================================================= */

    // 将VMC关节力矩写入Output目标
    WheelLeg_Output_UpdateTarget();

    // 更新达妙电机 Enable / Ready / Fault
    WheelLeg_Output_UpdateEnable();

    // 目标力矩经过slew限制生成实际发送力矩
    WheelLeg_Output_UpdateCommand(WheelLeg_Control_dt);

    // 真正发送关节电机MIT命令
    WheelLeg_Output_Send();


    /* =========================================================
     * 9. 轮电机输出
     * ========================================================= */

    if((wheelLeg_controlMode == WHEELLEG_MODE_BALANCE) &&(wheelLeg_controlCommand.valid != 0))
    {
        // BALANCE阶段：LQR轮端力矩 -> 3508电流
        WheelLeg_Motor_SendWheelTorque(wheelLeg_controlCommand.left_wheel_torque_nm, wheelLeg_controlCommand.right_wheel_torque_nm);}
    else
    {
        // 轮电机始终0电流
        WheelLeg_Wheel_ZeroCurrent();
    }


    /* =========================================================
     * 10. 最终安全保护
     * ========================================================= */

    // Safety Lock：轮电机清零、关节失能等
    WheelLeg_Safety_Control();

    /* =========================================================
     * 11. 控制周期
     * ========================================================= */

    osDelay(1);
}
		

	
}

static float WheelLeg_LimitFloat(float value,float limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

static void WheelLeg_Clear_ControlCommand(void)
{
    wheelLeg_controlCommand.left_force_n = 0.0f;
    wheelLeg_controlCommand.right_force_n = 0.0f;

    wheelLeg_controlCommand.left_tp_nm = 0.0f;
    wheelLeg_controlCommand.right_tp_nm = 0.0f;

    wheelLeg_controlCommand.left_wheel_torque_nm = 0.0f;
    wheelLeg_controlCommand.right_wheel_torque_nm = 0.0f;

    wheelLeg_controlCommand.valid = 0;
}


//发送0力矩
static void WheelLeg_Wheel_ZeroCurrent(void)
{
    Motor_DJI_SendCurrent(&Chassis_DriverMotor_CAN,Chassis_DriverMotor_Send_CAN_ID,0,0,0,0);
}

static void WheelLeg_Joint_Disable(void)
{

    Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor1_Send_CAN_ID);

    osDelay(2);

    Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor2_Send_CAN_ID);

    osDelay(2);

    Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor3_Send_CAN_ID);

    osDelay(2);

    Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor4_Send_CAN_ID);

    osDelay(2);

}

static void WheelLeg_Safety_Control(void)
{
    static uint16_t safety_counter = 0;
    static uint8_t last_safety_lock = 0;

    if(WheelLeg_Safety_Lock)
    {

        WheelLeg_Wheel_ZeroCurrent();

        if(last_safety_lock == 0)
        {
            WheelLeg_Joint_Disable();
            safety_counter = 0;
        }

        safety_counter++;

        if(safety_counter >= 100)
        {
            safety_counter = 0;
            WheelLeg_Joint_Disable();
        }
    }
    else
    {
        safety_counter = 0;
    }

    last_safety_lock = WheelLeg_Safety_Lock;
}

//VMC命令
static void WheelLeg_Update_VMCCommand(void)
{
    //主动给VMC输入清零。
    if(wheelLeg_controlCommand.valid == 0)
    {
        WheelLeg_VMC_SetLeft(0.0f, 0.0f);
        WheelLeg_VMC_SetRight(0.0f, 0.0f);

        return;
    }
    
    //左腿：force_n = 虚拟腿轴向力F tp_nm   = 虚拟腿摆动力矩Tp
    WheelLeg_VMC_SetLeft(wheelLeg_controlCommand.left_force_n,wheelLeg_controlCommand.left_tp_nm );

    //右腿
    WheelLeg_VMC_SetRight(wheelLeg_controlCommand.right_force_n,wheelLeg_controlCommand.right_tp_nm
    );
}

static void WheelLeg_Update_ControlCommand(void)
{
    //每个控制周期先默认所有输出为0
    WheelLeg_Clear_ControlCommand();

    switch(wheelLeg_controlMode)
    {
        case WHEELLEG_MODE_DISABLE:

            break;

        case WHEELLEG_MODE_PREPARE:

            //PREPARE阶段只依赖腿长控制器
            if((wheelLeg_legController.left.valid == 0) ||(wheelLeg_legController.right.valid == 0))
            {
                break;
            }

            //起立阶段
            wheelLeg_controlCommand.left_force_n = wheelLeg_legController.left.spring_comp_n + 2.0f * wheelLeg_legController.left.force_pid_n;

            wheelLeg_controlCommand.right_force_n = wheelLeg_legController.right.spring_comp_n  + 2.0f * wheelLeg_legController.right.force_pid_n;

            //不使用LQR
            wheelLeg_controlCommand.left_tp_nm = 0.0f;
            wheelLeg_controlCommand.right_tp_nm = 0.0f;

            wheelLeg_controlCommand.left_wheel_torque_nm = 0.0f;
            wheelLeg_controlCommand.right_wheel_torque_nm = 0.0f;

            wheelLeg_controlCommand.valid = 1;

            break;

        case WHEELLEG_MODE_BALANCE:

            //平衡阶段再有效lqr
            if((wheelLeg_lqr.valid == 0) ||(wheelLeg_legController.left.valid == 0) ||(wheelLeg_legController.right.valid == 0))
            {
                break;
            }

            //正常平衡阶段的完整腿力
            wheelLeg_controlCommand.left_force_n = wheelLeg_legController.left.force_total_n;
            wheelLeg_controlCommand.right_force_n = wheelLeg_legController.right.force_total_n;

            //LQR虚拟腿摆动力矩
            wheelLeg_controlCommand.left_tp_nm = WheelLeg_LimitFloat(wheelLeg_lqr.output[WHEELLEG_LQR_OUTPUT_LEFT_LEG], WHEELLEG_TEST_TP_LIMIT_NM);
            wheelLeg_controlCommand.right_tp_nm =WheelLeg_LimitFloat(wheelLeg_lqr.output[WHEELLEG_LQR_OUTPUT_RIGHT_LEG],WHEELLEG_TEST_TP_LIMIT_NM);
            
            //LQR轮子力矩
            wheelLeg_controlCommand.left_wheel_torque_nm =WheelLeg_LimitFloat(wheelLeg_lqr.output[WHEELLEG_LQR_OUTPUT_LEFT_WHEEL],WHEELLEG_TEST_WHEEL_LIMIT_NM);
            wheelLeg_controlCommand.right_wheel_torque_nm =WheelLeg_LimitFloat(wheelLeg_lqr.output[WHEELLEG_LQR_OUTPUT_RIGHT_WHEEL],WHEELLEG_TEST_WHEEL_LIMIT_NM);

            wheelLeg_controlCommand.valid = 1;

            break;


        default:

            //未知状态：保持全部输出为0。
            break;
    }
}


static void WheelLeg_Update_ControlMode(void)
{
    /*
     * 安全锁优先级最高
     */
    if(WheelLeg_Safety_Lock)
    {
        WheelLeg_Balance_Request = 0;
        wheelLeg_controlMode =WHEELLEG_MODE_DISABLE;
         // 清除上一轮LQR目标
        WheelLeg_LQR_ClearTarget();

        return;
    }


    switch(wheelLeg_controlMode)
    {
        case WHEELLEG_MODE_DISABLE:

           
            if(WheelLeg_Enable_Request)
            {
                wheelLeg_controlMode = WHEELLEG_MODE_PREPARE;
            }

            break;


        case WHEELLEG_MODE_PREPARE:
     
             //取消启动请求，
            if(WheelLeg_Enable_Request == 0)
            {
                WheelLeg_Balance_Request = 0;
                wheelLeg_controlMode =WHEELLEG_MODE_DISABLE;
                 // 清除上一轮LQR目标
                 WheelLeg_LQR_ClearTarget();
                break;
            }

            // 没有明确请求进入BALANCE，就一直保持PREPARE
            if(WheelLeg_Balance_Request == 0)
            {
                break;
            }

			// 左右腿运动学必须有效
			if((wheelLeg_kinematics.left.valid == 0) ||
				(wheelLeg_kinematics.right.valid == 0))
			{
				break;
			}
			
			// 左右腿必须都已经到达PREPARE目标腿长附近
			if((fabsf(wheelLeg_kinematics.left.length_m -WHEELLEG_PREPARE_LENGTH_M) >=WHEELLEG_PREPARE_LENGTH_TOL_M) ||
				(fabsf(wheelLeg_kinematics.right.length_m -WHEELLEG_PREPARE_LENGTH_M) >=WHEELLEG_PREPARE_LENGTH_TOL_M))
			{
				break;
			}
			
			// LQR当前状态必须已经成功构造
			if(wheelLeg_lqr.state_valid == 0)
			{
				break;
			}
			
			// 捕获进入平衡瞬间的位置和Yaw
			WheelLeg_LQR_CaptureTarget();
			
			// 捕获失败，不允许进入BALANCE
			if(wheelLeg_lqr.target_valid == 0)
			{
				break;
			}
			
			// 请求归位
			WheelLeg_Balance_Request = 0;
			
			// 正式进入平衡模式
			wheelLeg_controlMode =WHEELLEG_MODE_BALANCE;
			
			break;
			

        case WHEELLEG_MODE_BALANCE:

           

            if(WheelLeg_Enable_Request == 0)
            {
                wheelLeg_controlMode = WHEELLEG_MODE_DISABLE;

                // 清除上一轮LQR目标
                WheelLeg_LQR_ClearTarget();
                break;
            }

                // 后续这里再加入：
                // 1. 运动学失效
                // 2. 电机离线
                // 3. Pitch过大
                // 4. 腿长超范围
                // 5. LQR持续无效
                // 等异常退出条件

            break;


        default:

            wheelLeg_controlMode =WHEELLEG_MODE_DISABLE;
            // 清除上一轮LQR目标
            WheelLeg_LQR_ClearTarget();

            break;
    }
}

static void WheelLeg_Update_ControlTarget(void)
{
    switch(wheelLeg_controlMode)
    {
        case WHEELLEG_MODE_PREPARE:
            //设置腿长目标 
            WheelLeg_LegController_SetTarget( WHEELLEG_PREPARE_LENGTH_M, WHEELLEG_PREPARE_LENGTH_M);

            break;


        case WHEELLEG_MODE_BALANCE:

            // 正常平衡阶段目标腿长
            WheelLeg_LegController_SetTarget(WHEELLEG_BALANCE_LENGTH_M, WHEELLEG_BALANCE_LENGTH_M);

            // 第一版保持机体Roll为0
            wheelLeg_legController.target_roll_deg = 0.0f;

            break;


        case WHEELLEG_MODE_DISABLE:
        default:

            break;
    }
}


/**
 * ===| 翻倒自起 & 检测意外翻倒 |===
 * 
 * @note 参数注意：
 * @note 车头抬高,车尾放低时pitch增大(灯条一侧为车尾)
 * @note Pitch范围:-180~180度,车水平正置时Pitch为0度
 * @note 腿在相对机体垂直、竖直向下时腿总摆角为0°+N*360°
 * @note 左腿从左看逆时针为正方向
 * @note 右腿从右看逆时针为正方向
 * 
 * @note 使用说明：
 * @note 只有当Chassis_State等于任意翻倒自起枚举时，才会有输出(改变指针)
 * @note 检测到意外翻倒：Chassis_State不等于任意翻倒自起枚举,而Pitch却不对劲时会返回-1(Error)
 * @note 指针未定义返回0，正常返回1
 * 
 * @note 使用：
 * @note 在需要自起时,让Chassis_State等于Chassis_Recover
 * @note 直接将Total_Angle_Target、Leg_Length_Target带入串级PID计算即可(摆角PID参数给硬一点)
 * @note 最终Chassis_State等于Chassis_FOLLOW时完成翻倒自起
 * 
 *  @param dt            控制周期(s)
 *  @param Chassis_State [out]当前底盘状态
 *  @param INS_Pitch     当前底盘Pitch(极性见使用注意)
 *  @param L_Total_Angle_Feedback 左腿总角度反馈(°)
 *  @param R_Total_Angle_Feedback 右腿总角度反馈(°)
 *  @param L_Angle_Err       左腿摆角 PID 误差(°)
 *  @param R_Angle_Err       右腿摆角 PID 误差(°)
 *  @param Length_Feedback 	 腿长反馈(m)
 *  @param L_Length_Err      左腿腿长 PID 误差(m)
 *  @param R_Length_Err      右腿腿长 PID 误差(m)
 *
 *  @param L_Total_Angle_Target [out] 左腿总角度目标(°)
 *  @param R_Total_Angle_Target [out] 右腿总角度目标(°)
 *  @param Leg_Length_Target    [out] 目标腿长(m)
 * 
 * 待完善：
 * 摆角堵转保护
 * 改摆腿逻辑:收腿时腿部从自然位置起来(L:62 70 76 R:64 72 -77   低头，正置，抬头)
 * 改收腿逻辑:机体不会因为摆腿角动量守恒和腿长蹬地导致摇摇晃晃起来
 *=========================================================================================*/
int8_t Chassis_Smooth_Restand
(const float dt,
volatile Chassis_State_EnumTypedef * const Chassis_State,
const float INS_Pitch,
const float L_Total_Angle_Feedback,
const float R_Total_Angle_Feedback,
const float L_Angle_Err,
const float R_Angle_Err,
const float Length_Feedback,
const float L_Length_Err,
const float R_Length_Err,
float * const L_Total_Angle_Target,
float * const R_Total_Angle_Target,
float * const Leg_Length_Target
)
{
	/*===| 输出指针保护 |===*/
	if(!Chassis_State || !L_Total_Angle_Target || !R_Total_Angle_Target || !Leg_Length_Target)
	{
		return 0;
	}

	//储存Chassis_State值，用于比较底盘状态与赋值
	Chassis_State_EnumTypedef Chassis_Return = *Chassis_State;

	//用于翻倒回正，左腿摆向，逆时针:1，顺时针:-1
	//翻倒回正:腿摆角方向相同，前翻(Pitch<0车头触地)，左腿逆时针转
	static int8_t Angle_Speed = 0;

	//记录目标最终摆角值用
	static Chassis_State_EnumTypedef Last_State = Chassis_OFF;
	static float L_Total_End = 0.0f, R_Total_End = 0.0f;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒自起腿摆向和速度
	//记录最终目标腿摆角Total_End
	if(Chassis_Return != Last_State)
	{
		if(Chassis_Return == Chassis_Recover)
		{
			//起点 = 当前总角度反馈
			*L_Total_Angle_Target = L_Total_Angle_Feedback;
			*R_Total_Angle_Target = R_Total_Angle_Feedback;
			//车头触地,左腿逆时针转
			if(INS_Pitch<0.0f)
			{
				Angle_Speed = 1;
			}
			//车尾触地,左腿顺时针转
			else
			{
				Angle_Speed = -1;
			}
		}
		if(Chassis_Return == Chassis_RESET_Slow_1)
		{
			//起点 = 当前总角度反馈
			*L_Total_Angle_Target = L_Total_Angle_Feedback;
			*R_Total_Angle_Target = R_Total_Angle_Feedback;

			//把总角包到 (-180,180],只用于判断"是否落在允许区间"
			float L_A0 = Caculate_Included_Angle(0.0f, L_Total_Angle_Feedback);
			float R_A0 = Caculate_Included_Angle(0.0f, R_Total_Angle_Feedback);

			
			//特殊区段，顺时针方向绕远
			if(L_A0 <= 0.0f && L_A0 >= -90.0f)
			{
				L_Total_End = L_Total_Angle_Feedback -(360.0f-Caculate_Included_Angle(L_Total_Angle_Feedback, 90.0f));
			}
			else//左腿目标 +90°:走最短弧;
			{
				L_Total_End = L_Total_Angle_Feedback + Caculate_Included_Angle(L_Total_Angle_Feedback, 90.0f);
			}
			
			if(R_A0 >= 0.0f && R_A0 <= 90.0f)
			{
				R_Total_End = R_Total_Angle_Feedback + (360.0f+Caculate_Included_Angle(R_Total_Angle_Feedback, -90.0f));
			}
			else
			{
				R_Total_End = R_Total_Angle_Feedback + Caculate_Included_Angle(R_Total_Angle_Feedback, -90.0f);
			}

		}
		else if(Chassis_Return == Chassis_RESET_Slow_2)
		{
			//收腿阶段:端点 = 从当前总角度走"最短弧"回到 0°
			L_Total_End = L_Total_Angle_Feedback + Caculate_Included_Angle(L_Total_Angle_Feedback, 0.0f);
			R_Total_End = R_Total_Angle_Feedback + Caculate_Included_Angle(R_Total_Angle_Feedback, 0.0f);
		}
		Last_State = Chassis_Return;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒回正
	if(Chassis_Return == Chassis_Recover)
	{
		//自由腿长
		*Leg_Length_Target = Length_Feedback;

		//根据摆角夹角设置固定速度归中
		float CenterSpeed_Target = Caculate_Included_Angle(L_Total_Angle_Feedback, -R_Total_Angle_Feedback);
		Limit_float(&CenterSpeed_Target, 50.0f, -50.0f);

		if(fabsf(L_Angle_Err)< 50.0f)
		*L_Total_Angle_Target += (125 * Angle_Speed + CenterSpeed_Target)*dt;
		if(fabsf(R_Angle_Err)< 50.0f)
		*R_Total_Angle_Target +=(-125 * Angle_Speed + CenterSpeed_Target)*dt;
		//翻倒回正检测
		//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
		if(fabsf(INS_Pitch)<25.0f)
		{
			Chassis_Return = Chassis_RESET_Slow_1;
		}
	}
	
	//倒地缓自起(摆腿至90度)
	else if(Chassis_Return == Chassis_RESET_Slow_1)
	{
		//自由腿长
		*Leg_Length_Target = Length_Feedback;
		
		/*===| 摆腿阶段:双腿总角度分别缓摆至 90° / -90°(速率与原来一致:45°/s) |===*/
		Acc_and_Dec(L_Total_End, L_Total_Angle_Target, 45.0f, 45.0f, dt);
		Acc_and_Dec(R_Total_End, R_Total_Angle_Target, 45.0f, 45.0f, dt);

		/*===| 摆腿到位:锁存当前腿长(自由腿长)并进入收腿阶段 |===*/
		if(fabsf(L_Total_End - *L_Total_Angle_Target) < 0.05f
		&& fabsf(R_Total_End - *R_Total_Angle_Target) < 0.05f
		&& fabsf(L_Angle_Err) < 5.0f
		&& fabsf(R_Angle_Err) < 5.0f)
		{
			Chassis_Return     = Chassis_RESET_Slow_2;
			*Leg_Length_Target = Length_Feedback;
		}
	}
	//倒地缓自起(收腿)
	else if(Chassis_Return == Chassis_RESET_Slow_2)
	{
		/*===| 收腿阶段:总角度回零、腿长缓收至最短(速率与原来一致:90°/s、0.5m/s) |===*/
		Acc_and_Dec(L_Total_End, L_Total_Angle_Target, 90.0f, 90.0f, dt);
		Acc_and_Dec(R_Total_End, R_Total_Angle_Target, 90.0f, 90.0f, dt);
		Acc_and_Dec(Leg_Length_MIN, Leg_Length_Target, 0.5f, 0.5f, dt);

		if(fabsf(L_Total_End - *L_Total_Angle_Target)  < 0.05f
		&& fabsf(R_Total_End - *R_Total_Angle_Target)  < 0.05f
		&& fabsf(L_Angle_Err)  < 5.0f
		&& fabsf(R_Angle_Err)  < 5.0f
		&& fabsf(L_Length_Err) < 0.1f
		&& fabsf(R_Length_Err) < 0.1f)
		{
			/*===| 收腿完成:允许站立 |===*/
			Chassis_Return = Chassis_FOLLOW;
		}
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//检测意外翻倒计数器，单位:s
	static float Chassis_Pitch_Error_Tim = 0.0f;
	//翻倒检测
	//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
	if(Chassis_Return!=Chassis_Recover&&fabsf(INS_Pitch)>100.0f)
	{	
		Chassis_Pitch_Error_Tim += dt;

		//仍在倒地自起
		if(Chassis_Return==Chassis_RESET_Slow_1
		|| Chassis_Return==Chassis_RESET_Slow_2)
		{
			//持续0.1s后重新翻倒回正
			Chassis_Return = Chassis_Recover;
			Chassis_Pitch_Error_Tim = 0.0f;
		}
		//意外翻倒
		else if(Chassis_Pitch_Error_Tim>0.05f)
		{
			Chassis_Pitch_Error_Tim = 0.06f;
			return -1;
		}
	}
	else
	{
		Chassis_Pitch_Error_Tim = 0.0f;
	}

	//检测底盘状态改变时间计数器，单位:s
	static float Chassis_State_Change_Tim = 0.0f;
	if(*Chassis_State != Chassis_Return)
	{
		Chassis_State_Change_Tim += dt;
		if(Chassis_State_Change_Tim>0.1f)
		{
			*Chassis_State = Chassis_Return;
			Chassis_State_Change_Tim = 0.0f;
		}
	}
	else
	{
		Chassis_State_Change_Tim = 0.0f;
	}
	return 1;
}