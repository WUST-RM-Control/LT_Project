//======================================================================
//                    Define[全局定义]
//      这里定义了一些可能需要经常修改或者引用的宏定义
//======================================================================

#ifndef __Define__
#define __Define__

////////////////////////////////////////////////////////////////////
//                                                               //
// 修改了CANID后接收数据不用动，需要在发送函数处根据ID重新排序 //
//                                                               //
////////////////////////////////////////////////////////////////////
/*===| 底盘电机通信相关定义 |===*/
#define Chassis_DriverMotor_CAN          	    hfdcan2     //轮电机CAN        
#define Chassis_DriverMotor_Send_CAN_ID         0x200       //轮电机CAN发送ID
#define Chassis_Motor1_Feedback_CAN_ID          0x203       //左腿轮电机1CAN反馈ID 	
#define Chassis_Motor2_Feedback_CAN_ID      	0x204       //右腿轮电机2CAN反馈ID

#define Chassis_JointMotor1_CAN					hfdcan1     //左腿车头电机
#define Chassis_JointMotor2_CAN					hfdcan1     //左腿车尾电机
#define Chassis_JointMotor3_CAN					hfdcan1     //右腿车头电机
#define Chassis_JointMotor4_CAN					hfdcan2     //右腿车尾电机
#define Chassis_JointMotor1_Send_CAN_ID			0x02    //左腿车头电机
#define Chassis_JointMotor1_Feedback_ID			0x12    //左腿车头电机
#define Chassis_JointMotor2_Send_CAN_ID			0x01    //左腿车尾电机
#define Chassis_JointMotor2_Feedback_ID			0x11    //左腿车尾电机
#define Chassis_JointMotor3_Send_CAN_ID			0x03    //右腿车头电机
#define Chassis_JointMotor3_Feedback_ID			0x13    //右腿车头电机
#define Chassis_JointMotor4_Send_CAN_ID			0x04    //右腿车尾电机
#define Chassis_JointMotor4_Feedback_ID			0x14    //右腿车尾电机

/*===| 超电通信相关定义 |==*/
    #define SuperCap_CAN                        hfdcan1     //hcan1
    #define SuperCap_Control_CAN_ID             0x030     //超电控制CANID
    #define SuperCap_FeedBack_CAN_ID            0x031     //超电反馈CANID
    
/*===| 陀螺仪通信相关定义 |==*/
    #define IMU_CAN                             hfdcan3   //hcan3
    #define IMU_Control_CAN_ID                  0x001     //IMU控制CANID
    #define IMU_FeedBack_CAN_ID                 0x002     //IMU反馈CANID

/*===| 电机定义 |===*/
    #define Motor_3508_Reduction                16.71f
    #define Motor_2006_Reduction                36.0f

/*===| 底盘相关参数定义 |===*/
    #define Chassis_Wheel_Radius                0.06f   //轮半径
    #define Chassis_RPM_to_m_s                  2.0f*PI*Chassis_Wheel_Radius/(60.0f*Motor_3508_Reduction)  //底盘RPM转换成m/s的系数
    #define Robo_Vy_MaxSpeed                    4.0f     //机器最大Y速度
	#define Robo_Vx_MaxSpeed					4.0f	 //机器最大X速度
    #define Robo_Vx_MaxSpeed_SPIN               1.0f     //机器最大X速度
    #define Robo_Vy_MaxSpeed_SPIN               1.0f     //机器最大Y速度	
    #define Robo_Wz_MaxSpeed                    12.0f    //机器最大Z速度				
	
	#define Chassis_Acc                         15.0f     //底盘加速度
	#define Chassis_Dec                         15.0f     //底盘加速度
	#define Chassis_Acc_SPIN                    5.0f     //底盘加速度
    #define Chassis_Slow                        15.0f    //底盘减速度

	/*===| 实际力矩(Nm)=发送的力矩电流整数 * (20/16384) * 3508屁股力矩常数(0.01562Nm/A) * 减速比 |===*/  
	#define Current2Torque 					   3.186647037e-4f	//    (20/16384)              *    (0.3)    / (3591/187)  *  (16.71)
	#define Torque2Current 					   3.138094644e+3f  //电调最大持续电流/输出限幅      转矩系数   19.2减速比    16.71减速比
	#define Radain2Angle					   57.29578f
	#define Angle2Radain					   0.0174533f
	
	#define K_gear2chain                       1.0f
        
/*===| 云台相关参数定义 |===*/  
    #define Gimbal_Median_Yaw                 	0     //云台朝前时Yaw电机的编码值
    #define Gimbal_Median_Pitch                 0     //云台水平时Pitch电机的角度
 
    #define Gimbal_Pitch_Limit_Up               90.0f
    #define Gimbal_Pitch_Limit_Down             -90.0f
/*===| 发射相关参数定义 |===*/
    #define Fric_Acc                 0.1f
    #define Shoot_Trigger_Speed_Max  -20.0f
    #define Fric_Radius 			 0.03f
    #define Fric_RPM_to_m_s     	 (2*PI*Fric_Radius/60.0f)
    #define Triger_Reduction 		 (Motor_2006_Reduction * 55/23)


/*===| 底盘计算参数定义 |===*/
    #define Leg_Length_Min  0.15f
    #define Leg_Length_Max  0.366f

//	#define 	PI   3.14159265359f
	#define 	Angle_to_Radain	 0.0174533f //角度to弧度(PI/180)
    #define	    Radain_to_Angle	 57.29578f  //弧度to角度(180/PI)
    /* @note MC02 原有代码(INS.c 等)用 Rad2Angle,Down 的 Define.h 里没有,补一条兼容定义 */
    #define     Rad2Angle        (180.0f/PI)
    
	#define 	MB    13
	#define 	RL_W  0.523
	#define 	Q_L_F 0 
	#define 	Q_L_B 1 
	#define 	Q_R_F 2 
	#define 	Q_R_B 3 
	

/*===| 移植兼容层(Down 的名字 -> MC02 现有的名字) |===*/
/*
 * @note 1. INS:
 * @note    Down 的 INS.h 定义 INS_Data_Gimbal / INS_Data_Self,
 * @note    MC02 的 INS.h 同名结构体叫 INS_Data_External / INS_Data_Internal。
 * @note    INS 模块按规定不动,所以在这里做别名映射:
 * @note      INS_Data_Gimbal -> INS_Data_External (由双板通信 0x52 姿态帧 / 外部 IMU 帧填充)
 * @note      INS_Data_Self   -> INS_Data_Internal (板载 BMI088 + EKF 的姿态)
 * @note    底盘自己用的姿态请走 INS_Chassis_Data(极性隔离在那里面做)。
 */
#define INS_Data_Gimbal                 INS_Data_External
#define INS_Data_Self                   INS_Data_Internal

/*
 * @note 2. 运控结构体实例:
 * @note    Down 里叫 RoboControl_Struct,MC02 旧代码(Gimbal.c / UI_Task.c / SuperCap_Driver.c)
 * @note    里还叫 Robo。这里做别名,避免逐处修改 MC02 的旧文件。
 */
#define Robo                            RoboControl_Struct

#endif
