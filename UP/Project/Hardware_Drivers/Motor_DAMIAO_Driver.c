#include "Motor_DAMIAO_Driver.h"

//电机-达妙-指令-使能（FDCAN，电机MasterID）
void Motor_DM_CMD_Enable(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID)
{
    uint8_t DM_Enable_Data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
     CAN_Send_Data(hfdcan, CAN_ID, DM_Enable_Data);
}

//电机-达妙-指令-失能（FDCAN，电机MasterID）
void Motor_DM_CMD_Disable(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID)
{
    uint8_t DM_Disable_Data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
     CAN_Send_Data(hfdcan, CAN_ID, DM_Disable_Data);
}

//电机-达妙-指令-设置零点（FDCAN，电机Master_ID）
void Motor_DM_CMD_SetZero(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID)
{
    uint8_t DM_SetZero_Data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
     CAN_Send_Data(hfdcan, CAN_ID, DM_SetZero_Data);
}

//电机-达妙-指令-清除错误（FDCAN，电机Master_ID）
void Motor_DM_CMD_ClearErr(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID)
{
    uint8_t DM_ClearErr_Data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
     CAN_Send_Data(hfdcan, CAN_ID, DM_ClearErr_Data);
}

//电机-达妙-指令-MID控制（FDCAN，电机Master_ID，位置，速度，Kp，Kd，力矩）
uint8_t DM_Send_Data[8];
void Motor_DM_CMD_MIT(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    
    pos_tmp = DM_float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = DM_float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = DM_float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = DM_float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = DM_float_to_uint(_torq, T_MIN, T_MAX, 12);

    DM_Send_Data[0] = (pos_tmp >> 8);
    DM_Send_Data[1] = pos_tmp;
    DM_Send_Data[2] = (vel_tmp >> 4);
    DM_Send_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM_Send_Data[4] = kp_tmp;
    DM_Send_Data[5] = (kd_tmp >> 4);
    DM_Send_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM_Send_Data[7] = tor_tmp;
    
     CAN_Send_Data(hfdcan, CAN_ID, DM_Send_Data);
}

//电机-达妙-指令-位置控制（FDCAN，电机Master_ID，位置，速度）
void Motor_DM_CMD_Position(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID, float Position, float Speed)
{
    float Position_Speed_Buf[2] = {Position, Speed};
    
    CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)Position_Speed_Buf);
}

//电机-达妙-指令-速度控制（FDCAN，电机Master_ID，速度）
void Motor_DM_CMD_Speed(FDCAN_HandleTypeDef* hfdcan, uint16_t CAN_ID, float Speed)
{
    float Speed_Buf[1] = {Speed};
    
     CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)Speed_Buf);
}

//电机-uint转float（供其他函数调用，根据.h文件内的范围进行数据映射）
float DM_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//电机-float转uint（供其他函数调用，根据.h文件内的范围进行数据映射）
int DM_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

//电机-达妙-存储反馈数据（CAN收到的反馈数组地址，电机数据结构体）
void Motor_DM_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    /*===| 协议解包 |===*/
    uint8_t Error_ID =  Data[0] >> 4;
    int p_int=(Data[1]<<8)|Data[2];
    int v_int=(Data[3]<<4)|(Data[4]>>4);
    int t_int=((Data[4]&0xF)<<8)|Data[5];
    int tempture=Data[7];
    
    /*===| 转移数据 |===*/
    Motor_Data_Struct->Angle = DM_uint_to_float(p_int, P_MIN, P_MAX, 16) / 2.0f/PI * 360.0f;// (-12.5,12.5)
    Motor_Data_Struct->Speed_RPM = DM_uint_to_float(v_int, V_MIN, V_MAX, 12) / 2.0f/PI * 360.0f;// (-45.0,45.0)
    Motor_Data_Struct->Torque = DM_uint_to_float(t_int, T_MIN, T_MAX, 12); //(-18.0,18.0)
    Motor_Data_Struct->Temperature = tempture;
    
    /*===| 得到总角度值 |===*/
    if      (Motor_Data_Struct->Angle - Motor_Data_Struct->Angle_Last >  180)    Motor_Data_Struct->Round--;
    else if (Motor_Data_Struct->Angle - Motor_Data_Struct->Angle_Last < -180)    Motor_Data_Struct->Round++;
    Motor_Data_Struct->Total_Angle = 360.0f * Motor_Data_Struct->Round + Motor_Data_Struct->Angle  ;
    
    /*===| 记录编码器值 |===*/
    Motor_Data_Struct->Angle_Last = Motor_Data_Struct->Angle;
    
    if(Error_ID == 1)
    {
        Motor_Data_Struct->Ticker = 0;
        Motor_Data_Struct->If_Online = 1; 
    }
}

//电机-达妙1拖4-存储反馈数据（CAN收到的反馈数组地址，电机数据结构体）
void Motor_DM1to4_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    /*===| 协议解包 |===*/
    Motor_Data_Struct->Encoder = (int16_t)(Data[0] << 8 | Data[1]);
    Motor_Data_Struct->Speed_RPM = (int16_t)(Data[2] << 8 | Data[3]) / 100.0f;
    Motor_Data_Struct->Torque = (int16_t)(Data[4] << 8 |Data[5]);
    Motor_Data_Struct->Temperature = (int16_t)(Data[6]);
    
    Motor_Data_Struct->Angle = (Motor_Data_Struct->Encoder - 4096.0f) * 180.0f / 4096.0f;
    
    /*===| 得到总角度值 |===*/
    if      (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last >  4096)    Motor_Data_Struct->Round--;
    else if (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last < -4096)    Motor_Data_Struct->Round++;
    Motor_Data_Struct->Total_Angle = 360.0f * (Motor_Data_Struct->Round + Motor_Data_Struct->Encoder / 8192.0f)  ;
    
    /*===| 记录编码器值 |===*/
    Motor_Data_Struct->Encoder_Last = Motor_Data_Struct->Encoder;
    Motor_Data_Struct->Angle_Last = Motor_Data_Struct->Angle;
    
    Motor_Data_Struct->Ticker = 0;
    Motor_Data_Struct->If_Online = 1; 
}

//电机-达妙1拖4-发送电流控制帧（FDCAN，电机MasterID，1号电机电流，2号电机电流，3号电机电流，4号电机电流）
void Motor_DM1to4_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, int16_t ID1_Currnet, int16_t ID2_Currnet, int16_t ID3_Currnet, int16_t ID4_Currnet)
{
    uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[1] = ID1_Currnet >> 8;
        Motor_Tx_Data[0] = ID1_Currnet & 0xFF;
    Motor_Tx_Data[3] = ID2_Currnet >> 8;
        Motor_Tx_Data[2] = ID2_Currnet & 0xFF;
    Motor_Tx_Data[5] = ID3_Currnet >> 8;
        Motor_Tx_Data[4] = ID3_Currnet & 0xFF;
    Motor_Tx_Data[7] = ID4_Currnet >> 8;
        Motor_Tx_Data[6] = ID4_Currnet & 0xFF;
    
         CAN_Send_Data(hfdcan, CAN_ID, Motor_Tx_Data);
}

//电机-达妙1拖4-清除错误
void Motor_DM1to4_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID)
{
    uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[0] = CAN_ID & 0xFF;
        Motor_Tx_Data[1] = CAN_ID >> 8;
    Motor_Tx_Data[2] = 0x55;
        Motor_Tx_Data[3] = 0x3C;
    Motor_Tx_Data[4] = 0;
        Motor_Tx_Data[5] = 0;
    Motor_Tx_Data[6] = 0;
        Motor_Tx_Data[7] = 0;
    
         CAN_Send_Data(hfdcan, 0x7FF, Motor_Tx_Data);
}