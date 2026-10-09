/**
 * @file    Chassis.c[底盘控制]
 * @brief   轮腿底盘控制
 * @details 统一接口,外部控制底盘数据和传感器数据统一从 Chassis_Task 传入；
 *          Chassis_Task 调用 WheelLeg 文件的上层函数实现底盘的控制；
 *          Chassis_Control_Struct中存储需要输入到WheelLeg文件任务函数的参数
 * 			底层相关的底盘数据定义在各个 WheelLeg 前缀文件中
 */

#include "Chassis.h"
#include "RoboControl.h"
#include "Power_Limit.h"
#include "CAN_Driver.h"
#include "Motor_DJI_Driver.h"
#include "PID.h"
#include "USART_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Referee_Unpack.h"
#include "Vofa.h"
#include "INS.h"
#include "Function.h"
#include "Buzzer.h"
#include "arm_math.h"  // CMSIS-DSP 库
#include "user_lib.h"
#include "Remote_Control.h"
#include "kalman_filter.h"
#include <math.h>   
#include <stdbool.h>