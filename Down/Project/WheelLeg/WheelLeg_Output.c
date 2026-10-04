#include "WheelLeg_Output.h"

#include "WheelLeg_VMC.h"

#include <math.h>

#include "WheelLeg_Motor.h"
#include "Motor_DAMIAO_Driver.h"
#include "Define.h"

WheelLeg_Output wheelLeg_output = {0};

#define LEFT_BACK_TORQUE_DIRECTION      (-1.0f)
#define LEFT_FRONT_TORQUE_DIRECTION     (-1.0f)

#define RIGHT_BACK_TORQUE_DIRECTION     ( 1.0f)
#define RIGHT_FRONT_TORQUE_DIRECTION    ( 1.0f)

#define WHEELLEG_OUTPUT_TORQUE_LIMIT_NM    20.0f
// 关节力矩变化速度限制
#define WHEELLEG_OUTPUT_TORQUE_SLEW_NM_S   20.0f

//按比例限幅
static void WheelLeg_Output_LimitPair(float *back_nm,float *front_nm,float limit_nm)
{
    float back_abs;
    float front_abs;
    float max_abs;
    float scale;

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

    //两个关节按相同比例缩小
    scale = limit_nm / max_abs;

    *back_nm *= scale;
    *front_nm *= scale;
}

static void WheelLeg_Output_SlewPair(
    float target_back,
    float target_front,
    float *command_back,
    float *command_front,
    float slew_nm_s,
    float dt)
{
    float delta_back;
    float delta_front;
    float max_delta;
    float max_step;
    float scale;

    // dt异常时不更新
    if((dt <= 0.0f) || (!isfinite(dt)))
    {
        return;
    }

    // 防止偶发长周期造成一次力矩跳太大
    if(dt > 0.05f)
    {
        dt = 0.05f;
    }

    max_step = slew_nm_s * dt;

    delta_back =
        target_back - *command_back;

    delta_front =
        target_front - *command_front;

    max_delta = fabsf(delta_back);

    if(fabsf(delta_front) > max_delta)
    {
        max_delta = fabsf(delta_front);
    }

    // 已经足够接近目标
    if(max_delta <= max_step)
    {
        *command_back = target_back;
        *command_front = target_front;
        return;
    }

    // 两个关节按照同样比例接近目标
    scale = max_step / max_delta;

    *command_back += delta_back * scale;
    *command_front += delta_front * scale;
}

//初始化
void WheelLeg_Output_Init(void)
{
    wheelLeg_output.left.tau_phi1_nm = 0.0f;
    wheelLeg_output.left.tau_phi4_nm = 0.0f;
    wheelLeg_output.left.back_target_nm = 0.0f;
    wheelLeg_output.left.front_target_nm = 0.0f;
    wheelLeg_output.left.valid = 0;

    wheelLeg_output.right.tau_phi1_nm = 0.0f;
    wheelLeg_output.right.tau_phi4_nm = 0.0f;
    wheelLeg_output.right.back_target_nm = 0.0f;
    wheelLeg_output.right.front_target_nm = 0.0f;
    wheelLeg_output.right.valid = 0;

    wheelLeg_output.left.back_command_nm = 0.0f;
    wheelLeg_output.left.front_command_nm = 0.0f;

    wheelLeg_output.left.enable_request = 0;
    wheelLeg_output.left.enable_sent = 0;
    wheelLeg_output.left.ready = 0;
    wheelLeg_output.left.fault = 0;

    wheelLeg_output.right.back_command_nm = 0.0f;
    wheelLeg_output.right.front_command_nm = 0.0f;

    wheelLeg_output.right.enable_request = 0;
    wheelLeg_output.right.enable_sent = 0;
    wheelLeg_output.right.ready = 0;
    wheelLeg_output.right.fault = 0;

    wheelLeg_output.torque_limit_nm =WHEELLEG_OUTPUT_TORQUE_LIMIT_NM;
}

void WheelLeg_Output_UpdateTarget(void)
{
    float left_back;
    float left_front;

    float right_back;
    float right_front;


    // 默认无效 
    wheelLeg_output.left.valid = 0;
    wheelLeg_output.right.valid = 0;

    // 每周期先清零目标，防止失效时残留上一周期力矩
    wheelLeg_output.left.tau_phi1_nm = 0.0f;
    wheelLeg_output.left.tau_phi4_nm = 0.0f;
    wheelLeg_output.left.back_target_nm = 0.0f;
    wheelLeg_output.left.front_target_nm = 0.0f;

    wheelLeg_output.right.tau_phi1_nm = 0.0f;
    wheelLeg_output.right.tau_phi4_nm = 0.0f;
    wheelLeg_output.right.back_target_nm = 0.0f;
    wheelLeg_output.right.front_target_nm = 0.0f;


    //  左腿 

    if(wheelLeg_vmc.left.valid)
    {
        wheelLeg_output.left.tau_phi1_nm =wheelLeg_vmc.left.tau_phi1_nm;
        wheelLeg_output.left.tau_phi4_nm =wheelLeg_vmc.left.tau_phi4_nm;

        //VMC数学坐标 到 左侧真实电机坐标
        left_back =LEFT_BACK_TORQUE_DIRECTION *wheelLeg_vmc.left.tau_phi1_nm;
        left_front = LEFT_FRONT_TORQUE_DIRECTION * wheelLeg_vmc.left.tau_phi4_nm;

        WheelLeg_Output_LimitPair( &left_back, &left_front, wheelLeg_output.torque_limit_nm );

        wheelLeg_output.left.back_target_nm =left_back;

        wheelLeg_output.left.front_target_nm = left_front;

        wheelLeg_output.left.valid = 1;
    }

        //  右腿 

    if(wheelLeg_vmc.right.valid)
    {
        wheelLeg_output.right.tau_phi1_nm = wheelLeg_vmc.right.tau_phi1_nm;
        wheelLeg_output.right.tau_phi4_nm =wheelLeg_vmc.right.tau_phi4_nm;

        ////VMC数学坐标 到 右侧真实电机坐标
        right_back =RIGHT_BACK_TORQUE_DIRECTION *wheelLeg_vmc.right.tau_phi1_nm;
        right_front = RIGHT_FRONT_TORQUE_DIRECTION * wheelLeg_vmc.right.tau_phi4_nm;

        WheelLeg_Output_LimitPair( &right_back, &right_front, wheelLeg_output.torque_limit_nm);

        wheelLeg_output.right.back_target_nm =right_back;

        wheelLeg_output.right.front_target_nm = right_front;

        wheelLeg_output.right.valid = 1;
    }
}

void WheelLeg_Output_SetLegEnable(uint8_t left_enable,uint8_t right_enable)
{
    wheelLeg_output.left.enable_request =left_enable ? 1U : 0U;

    wheelLeg_output.right.enable_request = right_enable ? 1U : 0U;
}

void WheelLeg_Output_UpdateEnable(void)
{

// 没有使能请求
if(wheelLeg_output.left.enable_request == 0)
{
    // 如果之前已经使能，则主动退出
    if(wheelLeg_output.left.enable_sent)
    {
        Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor1_Send_CAN_ID);

        osDelay(1);

        Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor2_Send_CAN_ID );

        osDelay(1);
    }

    wheelLeg_output.left.enable_sent = 0;
    wheelLeg_output.left.ready = 0;
    wheelLeg_output.left.fault = 0;

    wheelLeg_output.left.back_command_nm = 0.0f;
    wheelLeg_output.left.front_command_nm = 0.0f;
}
else
{
    // fault出现以后，不自动重新使能
    if(wheelLeg_output.left.fault)
    {
        wheelLeg_output.left.ready = 0;

        wheelLeg_output.left.back_command_nm = 0.0f;
        wheelLeg_output.left.front_command_nm = 0.0f;
    }

    // 第一次收到Enable请求
    else if(wheelLeg_output.left.enable_sent == 0)
    {
    
        // 左后 LB = JointMotor2
      
        Motor_DM_CMD_ClearErr( &Chassis_JointMotor_CAN,Chassis_JointMotor2_Send_CAN_ID);
        osDelay(1);
        Motor_DM_CMD_Enable(&Chassis_JointMotor_CAN, Chassis_JointMotor2_Send_CAN_ID );
        osDelay(1);

        //左前 LF = JointMotor1
        
        Motor_DM_CMD_ClearErr(&Chassis_JointMotor_CAN,Chassis_JointMotor1_Send_CAN_ID);
        osDelay(1);
        Motor_DM_CMD_Enable(&Chassis_JointMotor_CAN,Chassis_JointMotor1_Send_CAN_ID );
        osDelay(1);

        wheelLeg_output.left.enable_sent = 1;
        wheelLeg_output.left.ready = 0;
    }

    else
    {
        // 检查是否出现异常状态
        if(
            ((wheelLeg_motor.left_front.error_id != 0) &&
             (wheelLeg_motor.left_front.error_id != 1))
            ||
            ((wheelLeg_motor.left_back.error_id != 0) &&
             (wheelLeg_motor.left_back.error_id != 1))
          )
        {
            wheelLeg_output.left.fault = 1;
            wheelLeg_output.left.ready = 0;

            wheelLeg_output.left.back_command_nm = 0.0f;
            wheelLeg_output.left.front_command_nm = 0.0f;

            Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN, Chassis_JointMotor1_Send_CAN_ID );

            osDelay(1);

            Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor2_Send_CAN_ID);

            wheelLeg_output.left.enable_sent = 0;
        }

        // 两个左腿电机都在线且进入正常Enable状态
        else if(
            (wheelLeg_motor.left_front.online == 1) &&
            (wheelLeg_motor.left_back.online == 1) &&
            (wheelLeg_motor.left_front.error_id == 1) &&
            (wheelLeg_motor.left_back.error_id == 1)
          )
        {
            wheelLeg_output.left.ready = 1;
        }
        else
        {
            wheelLeg_output.left.ready = 0;
        }
    }
}

    /*
     * ========== 右腿 ==========
     */

    // 没有使能请求
    if(wheelLeg_output.right.enable_request == 0)
    {
        // 如果此前确实使能过，则退出时主动Disable
        if(wheelLeg_output.right.enable_sent)
        {
            Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN, Chassis_JointMotor3_Send_CAN_ID );
            osDelay(1);
            Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN,Chassis_JointMotor4_Send_CAN_ID );
            osDelay(1);
        }

        wheelLeg_output.right.enable_sent = 0;
        wheelLeg_output.right.ready = 0;
        wheelLeg_output.right.fault = 0;

        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;

        return;
    }

        // fault一旦出现，不允许自动重新使能
    // 必须先把enable_request撤掉，再重新启动
    if(wheelLeg_output.right.fault)
    {
        wheelLeg_output.right.ready = 0;

        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;

        return;
    }

        if(wheelLeg_output.right.enable_sent == 0)
    {
        
        //右后 RB = JointMotor4
        
        Motor_DM_CMD_ClearErr( &Chassis_JointMotor_CAN,Chassis_JointMotor4_Send_CAN_ID);
        osDelay(1);
        Motor_DM_CMD_Enable(&Chassis_JointMotor_CAN,Chassis_JointMotor4_Send_CAN_ID);
        osDelay(1);


        
        //右前 RF = JointMotor3
         
        Motor_DM_CMD_ClearErr(&Chassis_JointMotor_CAN,Chassis_JointMotor3_Send_CAN_ID);
        osDelay(1);
        Motor_DM_CMD_Enable( &Chassis_JointMotor_CAN, Chassis_JointMotor3_Send_CAN_ID );
        osDelay(1);

        wheelLeg_output.right.enable_sent = 1;
        wheelLeg_output.right.ready = 0;

        return;
    }

        /*
     * 达妙正常Enable状态为 error_id == 1。
     * 0表示尚未进入正常工作状态；
     * 其他状态当前按异常处理。
     */
    if(
        ((wheelLeg_motor.right_front.error_id != 0) &&
         (wheelLeg_motor.right_front.error_id != 1))
        ||
        ((wheelLeg_motor.right_back.error_id != 0) &&
         (wheelLeg_motor.right_back.error_id != 1))
      )
    {
        wheelLeg_output.right.fault = 1;
        wheelLeg_output.right.ready = 0;

        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;

        // 异常立即失能
        Motor_DM_CMD_Disable( &Chassis_JointMotor_CAN, Chassis_JointMotor3_Send_CAN_ID);
        osDelay(1);
        Motor_DM_CMD_Disable(&Chassis_JointMotor_CAN, Chassis_JointMotor4_Send_CAN_ID);
        wheelLeg_output.right.enable_sent = 0;

        return;
    }

        if(
        (wheelLeg_motor.right_front.online == 1) &&
        (wheelLeg_motor.right_back.online == 1) &&
        (wheelLeg_motor.right_front.error_id == 1) &&
        (wheelLeg_motor.right_back.error_id == 1)
      )
    {
        wheelLeg_output.right.ready = 1;
    }
    else
    {
        wheelLeg_output.right.ready = 0;
    }
}

void WheelLeg_Output_UpdateCommand(float dt)
{
    /*
     * dt异常时，两条腿都立即清零。
     * 不允许继续保留上一周期力矩。
     */
    if((dt <= 0.0f) || (!isfinite(dt)))
    {
        wheelLeg_output.left.back_command_nm = 0.0f;
        wheelLeg_output.left.front_command_nm = 0.0f;

        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;

        return;
    }


    /* =========================================================
     * 左腿
     * ========================================================= */

    if(
        (wheelLeg_output.left.enable_request == 0) ||
        (wheelLeg_output.left.enable_sent == 0) ||
        (wheelLeg_output.left.ready == 0) ||
        (wheelLeg_output.left.fault != 0) ||
        (wheelLeg_output.left.valid == 0)
      )
    {
        wheelLeg_output.left.back_command_nm = 0.0f;
        wheelLeg_output.left.front_command_nm = 0.0f;
    }
    else if(
        (!isfinite(wheelLeg_output.left.back_target_nm)) ||
        (!isfinite(wheelLeg_output.left.front_target_nm))
      )
    {
        wheelLeg_output.left.back_command_nm = 0.0f;
        wheelLeg_output.left.front_command_nm = 0.0f;
    }
    else
    {
        WheelLeg_Output_SlewPair(
            wheelLeg_output.left.back_target_nm,
            wheelLeg_output.left.front_target_nm,

            &wheelLeg_output.left.back_command_nm,
            &wheelLeg_output.left.front_command_nm,

            WHEELLEG_OUTPUT_TORQUE_SLEW_NM_S,
            dt
        );
    }


    /* =========================================================
     * 右腿
     * ========================================================= */

    if(
        (wheelLeg_output.right.enable_request == 0) ||
        (wheelLeg_output.right.enable_sent == 0) ||
        (wheelLeg_output.right.ready == 0) ||
        (wheelLeg_output.right.fault != 0) ||
        (wheelLeg_output.right.valid == 0)
      )
    {
        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;
    }
    else if(
        (!isfinite(wheelLeg_output.right.back_target_nm)) ||
        (!isfinite(wheelLeg_output.right.front_target_nm))
      )
    {
        wheelLeg_output.right.back_command_nm = 0.0f;
        wheelLeg_output.right.front_command_nm = 0.0f;
    }
    else
    {
        WheelLeg_Output_SlewPair(
            wheelLeg_output.right.back_target_nm,
            wheelLeg_output.right.front_target_nm,

            &wheelLeg_output.right.back_command_nm,
            &wheelLeg_output.right.front_command_nm,

            WHEELLEG_OUTPUT_TORQUE_SLEW_NM_S,
            dt
        );
    }
}

void WheelLeg_Output_Send(void)
{

    /*
     * 左前 LF = JointMotor1
     * Front command
     */
    Motor_DM_CMD_MIT(
        &Chassis_JointMotor_CAN,
        Chassis_JointMotor1_Send_CAN_ID,

        0.0f,
        0.0f,
        0.0f,
        0.0f,

        wheelLeg_output.left.front_command_nm
    );


    /*
     * 左后 LB = JointMotor2
     * Back command
     */
    Motor_DM_CMD_MIT(
        &Chassis_JointMotor_CAN,
        Chassis_JointMotor2_Send_CAN_ID,

        0.0f,
        0.0f,
        0.0f,
        0.0f,

        wheelLeg_output.left.back_command_nm
    );

    /*
     * 右前 RF = JointMotor3
     * Front command
     */
    Motor_DM_CMD_MIT(
        &Chassis_JointMotor_CAN,
        Chassis_JointMotor3_Send_CAN_ID,

        0.0f,
        0.0f,
        0.0f,
        0.0f,

        wheelLeg_output.right.front_command_nm
    );

    /*
     * 右后 RB = JointMotor4
     * Back command
     */
    Motor_DM_CMD_MIT(
        &Chassis_JointMotor_CAN,
        Chassis_JointMotor4_Send_CAN_ID,

        0.0f,
        0.0f,
        0.0f,
        0.0f,

        wheelLeg_output.right.back_command_nm
    );
}