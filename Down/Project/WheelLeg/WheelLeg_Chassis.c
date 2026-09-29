#include "WheelLeg_Chassis.h"
#include "WheelLeg_Motor.h"
#include "WheelLeg_Kinematics.h"
#include "Define.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "DWT.h"
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

    osDelay(10);
}
		

	
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

