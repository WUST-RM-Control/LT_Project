#include "Motor_DrEmpower.h"

/*===| 设置电机ID(1~63) ID=0则为广播，默认ID7 |===*/
void Motor_DrEmpower_Set_ID(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint16_t Target_ID)
{
    DrEmpower_SetConfigCMD_StructTypedef ConfigCMD = {0};;
    
    ConfigCMD.Mask_ID = 31001;  //键码31001(0x7919) - CAN 总线 ID 号 
    ConfigCMD.Type_ID = 3;      //数据类型3 - uint32_t 
    ConfigCMD.Data = Target_ID; //数据 - 设置的CANID号

    CAN_Send_Data(hfdcan, 0x01F | (ID << 5), (uint8_t *)&ConfigCMD); //参数设置指令(CMD_ID=0x1F)
}

/*===| 开关数据反馈(不可保存) |===*/
void Motor_DrEmpower_DataFeedBack(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint8_t IF_Enable)
{
    DrEmpower_SetConfigCMD_StructTypedef ConfigCMD = {0};;
    
    ConfigCMD.Mask_ID = 22001;  //键码22001(0x55F1) - 开关数据实时反馈
    ConfigCMD.Type_ID = 3;      //数据类型3 - uint32_t 
    ConfigCMD.Data = IF_Enable; //数据 - 1开0关

    CAN_Send_Data(hfdcan, 0x01F | (ID << 5), (uint8_t *)&ConfigCMD); //参数设置指令(CMD_ID=0x1F)
}

/*===| 设置数据反馈时间(可保存) |===*/
void Motor_DrEmpower_DataFeedBackPeriod(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, uint16_t Period)
{
    DrEmpower_SetConfigCMD_StructTypedef ConfigCMD = {0};;
    
    ConfigCMD.Mask_ID = 31002;  //键码31002(0x791A) - 数据实时反馈间隔
    ConfigCMD.Type_ID = 3;      //数据类型3 - uint32_t 
    ConfigCMD.Data = Period;    //数据 - 周期(ms)

    CAN_Send_Data(hfdcan, 0x01F | (ID << 5), (uint8_t *)&ConfigCMD); //参数设置指令(CMD_ID=0x1F)
}

/*===| 保存参数 |===*/
void Motor_DrEmpower_SaveConfig(FDCAN_HandleTypeDef *hfdcan, uint8_t ID)
{
    DrEmpower_SyetemControlCMD_StructTypedef SyetemControlCMD = {0};
    
    SyetemControlCMD.order_num = 0x01;  //保存参数 - 0x01
    SyetemControlCMD.order_flag = 0;

    CAN_Send_Data(hfdcan, 0x008 | (ID << 5), (uint8_t *)&SyetemControlCMD); //系统控制指令(CMD_ID=0x08)
}

/*===| 重启 |===*/
void Motor_DrEmpower_Reboot(FDCAN_HandleTypeDef *hfdcan, uint8_t ID)
{
    DrEmpower_SyetemControlCMD_StructTypedef SyetemControlCMD = {0};
    
    SyetemControlCMD.order_num = 0x01;  //重启 - 0x03
    SyetemControlCMD.order_flag = 0;

    CAN_Send_Data(hfdcan, 0x008 | (ID << 5), (uint8_t *)&SyetemControlCMD); //系统控制指令(CMD_ID=0x08)
}

/*===| 设置零点 |===*/
void Motor_DrEmpower_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t ID)
{
    DrEmpower_SyetemControlCMD_StructTypedef SyetemControlCMD = {0};
    
    SyetemControlCMD.order_num = 0x05;  //保存零点 - 0x05
    SyetemControlCMD.order_flag = 0;

    CAN_Send_Data(hfdcan, 0x008 | (ID << 5), (uint8_t *)&SyetemControlCMD); //系统控制指令(CMD_ID=0x08)
}


/*===| 直接力矩控制 |===*/
void Motor_DrEmpower_SetTorque(FDCAN_HandleTypeDef *hfdcan, uint8_t ID, float Torque)
{
    DrEmpower_SetTorqueCMD_StructTypedef SetTorqueCMD = {0};;
    
    SetTorqueCMD.Torque = Torque;   //设置力矩(Nm)
    SetTorqueCMD.Ramp = 0;          //力矩斜坡    
    SetTorqueCMD.Mode = 1;          //直接力矩模式

    CAN_Send_Data(hfdcan, 0x01D | (ID << 5), (uint8_t *)&SetTorqueCMD); //电机力矩控制(CMD_ID=0x1D)
}


/*===| 保存电机反馈参数 |===*/
void Motor_DrEmpower_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *Motor_Data_Struct)
{
    /*===| 协议解包 |===*/
    DrEmpower_RxData_StructTypedef RxData;
    memcpy(&RxData, Data, 8);
    
    /*===| 转移数据 |===*/
    Motor_Data_Struct->Angle = RxData.Angel;
    Motor_Data_Struct->Speed_RPM = RxData.SpeedRPM_x100 / 100.0f;
    Motor_Data_Struct->Torque = RxData.Torque_x100 / 100.0f;
    
    /*===| 得到总角度值 |===*/
    if      (Motor_Data_Struct->Angle - Motor_Data_Struct->Angle_Last >  180)    Motor_Data_Struct->Round--;
    else if (Motor_Data_Struct->Angle - Motor_Data_Struct->Angle_Last < -180)    Motor_Data_Struct->Round++;
    Motor_Data_Struct->Total_Angle = 360.0f * Motor_Data_Struct->Round + Motor_Data_Struct->Angle;
    
    /*===| 记录编码器值 |===*/
    Motor_Data_Struct->Angle_Last = Motor_Data_Struct->Angle;
    
    Motor_Data_Struct->Ticker = 0;
    Motor_Data_Struct->If_Online = 1; 
}

