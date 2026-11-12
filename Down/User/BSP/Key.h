#ifndef __Key__
#define __Key__

#include "main.h"

typedef enum
{
    SW_Up = 1,
    SW_Mid = 3,
    SW_Down = 2,
} SW_State;

typedef enum
{
    Release = 0,
    ShortPush = 1,
    LongPush = 2,
} Key_State;

typedef __PACKED_STRUCT
{
    uint8_t Pin_Now;
    uint8_t Pin_Last;
    
    int Push_Time;
    int Release_Time;
    
    Key_State State;
} Key_Data_StructTypeDef;

typedef __PACKED_STRUCT
{
    Key_Data_StructTypeDef User;
} Key_StructTypeDef;

extern Key_StructTypeDef Key_Data;

void Key_Init(void);

void Key_Scan_Loop(void);

Key_State Key_Get_State(Key_Data_StructTypeDef *Key);

#endif
