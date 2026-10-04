#ifndef __Motor_Unitree_Driver__
#define __Motor_Unitree_Driver__

#include "main.h"
  
typedef __PACKED_STRUCT
{
    int16_t HEAD;
    int8_t ID:4;
    int8_t STATUS:3;
    int8_t None:1;
    int16_t Touque;
    int16_t Speed;
    int32_t Position;
    int16_t Kp;
    int16_t Kd;
    uint16_t CRC16;
} Unitree_6010_Control_StructTypedef;

typedef struct
{ 
    int16_t HEAD;
    int8_t ID:4;
    int8_t STATUS:3;
    int8_t None1:1;			
    int16_t Touque;
    int16_t Speed;
    int32_t Position;
    int8_t Tempture;
    uint8_t Err_Flag:3;
    int16_t Force:12;
    int8_t None2:1;
    uint16_t CRC16;
} Unitree_6010_Receive_StructTypedef;

void Unitree_6010_Receive_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct);

void Unitree_6010_Send_Control_Data(UART_HandleTypeDef *huart, uint8_t ID, float Position, float Speed, float Torque, float Kp, float Kd);

#endif
