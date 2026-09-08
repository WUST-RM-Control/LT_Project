#ifndef __Motor_Compensation_Data__
#define __Motor_Compensation_Data__

#include "main.h"

typedef struct
{
    float Motor_Yaw_Cal_Data[2048];
    float Motor_Pitch_Cal_Data[2048];
} Config_StructTypedef;

extern Config_StructTypedef Config;

#endif
