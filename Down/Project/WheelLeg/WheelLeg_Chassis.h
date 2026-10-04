#ifndef _WheelLeg_Chassis_
#define _WheelLeg_Chassis_

#include "main.h"

typedef struct
{
    float left_force_n;
    float right_force_n;

    float left_tp_nm;
    float right_tp_nm;

    float left_wheel_torque_nm;
    float right_wheel_torque_nm;

    uint8_t valid;

} WheelLeg_ControlCommand;

typedef enum
{
    WHEELLEG_MODE_DISABLE = 0,
    WHEELLEG_MODE_PREPARE,
    WHEELLEG_MODE_BALANCE

} WheelLeg_ControlMode;

extern WheelLeg_ControlMode wheelLeg_controlMode;
extern volatile uint8_t WheelLeg_Enable_Request;

extern WheelLeg_ControlCommand wheelLeg_controlCommand;

extern volatile uint8_t WheelLeg_Balance_Request;



void WheelLeg_Chassis_Task(void const *argument);


#endif
