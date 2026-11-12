#include "Motor_Unitree_Driver.h"


#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

void Unitree_6010_Send_Control_Data(UART_HandleTypeDef *huart, uint8_t ID, float Position, float Speed, float Torque, float Kp, float Kd)
{
    SATURATE(Kp,  0.0f,   25.599f);
    SATURATE(Kd,  0.0f,   25.599f);
    SATURATE(Torque, -127.99f,  127.99f);
    SATURATE(Speed, -804.00f,  804.00f);
    SATURATE(Position, -411774.0f,  411774.0f);

    
    Unitree_6010_Control_StructTypedef Data = {0};
    Data.HEAD = 0xEEFE;
    Data.STATUS = 1;
    Data.None = 0;
    Data.Touque = Torque*256;
    Data.Speed = Speed / 6.2832f * 256;
    Data.Position = Position / 6.2832f * 32768;
    Data.ID = ID;
    Data.Kp = Kp / 25.6f * 32768;
    Data.Kd = Kd / 25.6f * 32768;
    
    Data.CRC16 = crc_ccitt(0, (uint8_t *)&Data, sizeof(Unitree_6010_Control_StructTypedef) - 2);

    HAL_UART_Transmit(huart, (uint8_t *)&Data, sizeof(Unitree_6010_Control_StructTypedef), 1000);
}

void Unitree_6010_Receive_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    /*===| 协议解包 |===*/
    Unitree_6010_Receive_StructTypedef Unitree_6010_Receive_Struct;
    memcpy(&Unitree_6010_Receive_Struct, Data, sizeof(Unitree_6010_Receive_StructTypedef));
    
    /*===| 转移数据 |===*/
    Motor_Data_Struct->Total_Angle = Unitree_6010_Receive_Struct.Position / 32768.0f * 360.0f;
    Motor_Data_Struct->Speed_RPM    = Unitree_6010_Receive_Struct.Speed / 256.0f * 60;
    Motor_Data_Struct->Torque      = Unitree_6010_Receive_Struct.Touque / 256.0f;
    Motor_Data_Struct->Temperature = Unitree_6010_Receive_Struct.Tempture;
    
    Motor_Data_Struct->Ticker = 0;
    Motor_Data_Struct->If_Online = 1; 
}

