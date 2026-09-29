#include "WheelLeg_Motor.h"
#include "Motor_Driver.h"
#include "Define.h"
#include "Motor_DJI_Driver.h"

//常量定义
#define WHEELLEG_DEG_TO_RAD        0.017453292519943295f
#define WHEELLEG_RPM_TO_RAD_S      0.10471975511965977f

#define LEFT_WHEEL_DIRECTION       (-1.0f)
#define RIGHT_WHEEL_DIRECTION      ( 1.0f)//右轮顺时针运动方向为正 左轮取反

#define WHEEL_MOTOR_REDUCTION       16.71f //轮电机减速比

// 沿用原底盘工程的Torque2Current
#define WHEEL_TORQUE_TO_CURRENT    3138.094644f

// DJI C620电流指令保护限幅
#define WHEEL_CURRENT_CMD_LIMIT    16000.0f

WheelLeg_Motor wheelLeg_motor = {0};

//关节电机数据更新
static void WheelLeg_Update_JointMotor(WheelLeg_JointMotor *joint,uint8_t motor_id)
{
	Motor_Data_StructTypeDef *motor =&Motor_Data_Struct[motor_id];

	joint->raw_angle_deg = motor->Angle;

    joint->raw_total_angle_deg =motor->Total_Angle;

    joint->raw_velocity_deg_s =motor->Speed_RPM;

    joint->raw_torque =motor->Torque;

    joint->temperature =motor->Temperature;

    joint->online =motor->If_Online;

    joint->error_id =motor->Error_ID;
	
	//换成国际单位
	joint->position_rad =joint->raw_angle_deg* WHEELLEG_DEG_TO_RAD;

    joint->total_position_rad =joint->raw_total_angle_deg* WHEELLEG_DEG_TO_RAD;

    joint->velocity_rad_s =joint->raw_velocity_deg_s* WHEELLEG_DEG_TO_RAD;
	
}
//
static void WheelLeg_Update_WheelMotor(WheelLeg_WheelMotor *wheel,uint8_t motor_id,float direction)
{
    Motor_Data_StructTypeDef *motor =&Motor_Data_Struct[motor_id];

	//电机转子侧数据
    wheel->rotor_angle_deg =motor->Angle;

    wheel->rotor_total_angle_deg =motor->Total_Angle;

    wheel->rotor_speed_rpm =motor->Speed_RPM;

    wheel->current_feedback_raw =(int16_t)motor->Torque;

    wheel->temperature =motor->Temperature;

    wheel->online =motor->If_Online;
	
	//减速箱后的数据
	wheel->position_rad =direction* motor->Total_Angle* WHEELLEG_DEG_TO_RAD/ WHEEL_MOTOR_REDUCTION;

    wheel->velocity_rad_s =direction* motor->Speed_RPM* WHEELLEG_RPM_TO_RAD_S/ WHEEL_MOTOR_REDUCTION;
	
	//转化为线速度
	wheel->displacement_m =wheel->position_rad* Chassis_Wheel_Radius;

    wheel->linear_velocity_m_s =wheel->velocity_rad_s* Chassis_Wheel_Radius;
}

//更新整个轮腿所有电机
void WheelLeg_Motor_Update(void)
{
  //
  //四个关节电机
  //
  //Motor1 → 左前
  //Motor2 → 左后
  //Motor3 → 右前
  //Motor4 → 右后

    WheelLeg_Update_JointMotor(&wheelLeg_motor.left_front,Chassis_Joint1_ID);

    WheelLeg_Update_JointMotor(&wheelLeg_motor.left_back,Chassis_Joint2_ID);

    WheelLeg_Update_JointMotor(&wheelLeg_motor.right_front,Chassis_Joint3_ID);

    WheelLeg_Update_JointMotor(&wheelLeg_motor.right_back,Chassis_Joint4_ID);

    //两个轮电机 Motor5 → 左轮   Motor6 → 右轮

    WheelLeg_Update_WheelMotor(&wheelLeg_motor.left_wheel,Chassis_DriverMotor1_ID,LEFT_WHEEL_DIRECTION);
    WheelLeg_Update_WheelMotor(&wheelLeg_motor.right_wheel,Chassis_DriverMotor2_ID,RIGHT_WHEEL_DIRECTION);
	
}

void WheelLeg_Motor_SendWheelTorque(float left_torque_nm,float right_torque_nm)
{
    float left_current;
    float right_current;

    // 统一物理力矩 -> 电机原始方向
    left_current =LEFT_WHEEL_DIRECTION *left_torque_nm *WHEEL_TORQUE_TO_CURRENT;

    right_current =RIGHT_WHEEL_DIRECTION *right_torque_nm *WHEEL_TORQUE_TO_CURRENT;

    // DJI电流指令保护
    if(left_current > WHEEL_CURRENT_CMD_LIMIT)
        left_current = WHEEL_CURRENT_CMD_LIMIT;
    else if(left_current < -WHEEL_CURRENT_CMD_LIMIT)
        left_current = -WHEEL_CURRENT_CMD_LIMIT;

    if(right_current > WHEEL_CURRENT_CMD_LIMIT)
        right_current = WHEEL_CURRENT_CMD_LIMIT;
    else if(right_current < -WHEEL_CURRENT_CMD_LIMIT)
        right_current = -WHEEL_CURRENT_CMD_LIMIT;

    // 3508分别对应0x200控制帧中的第3、4个电机
    Motor_DJI_SendCurrent(&Chassis_DriverMotor_CAN, Chassis_DriverMotor_Send_CAN_ID, 0, 0,(int16_t)left_current,(int16_t)right_current);
}