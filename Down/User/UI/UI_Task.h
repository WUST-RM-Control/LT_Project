#ifndef UI_TASK_H
#define UI_TASK_H

#include "main.h"
#include "ui_interface.h"
#include "ui.h"
#include "shoot.h"
#include "Chassis.h"

typedef struct 
{
    float YawErr;
    float Pitch1;
    float Pitch2;
    
    float Trigger_Angle;
    float Fric1_Speed;
    float Fric2_Speed;
    
    float Chassis_Motor1_Speed;
    float Chassis_Motor2_Speed;
    float Chassis_Motor3_Speed;
    float Chassis_Motor4_Speed;
    
    uint8_t SuperCap_IF_Enable;
    float SuperCap_K;
    
    uint8_t Chassis_Speed_Level;
} UI_State_StructTypedef;

extern UI_State_StructTypedef UI_State;

void UI_Task(void const * argument);
void String_Printf(char *String_Arr, const char *fmt,...);


#endif
