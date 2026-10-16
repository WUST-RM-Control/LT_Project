//=====================================================================================
//                    CAN_Driver[CAN通信驱动]
//      包含了一些CAN通信相关的封装好的函数
//
//      CAN初始化 CAN_Init(void)
//      CAN发送 CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data)
//      CAN接收回调函数：CAN1CAN2默认无过滤，接收到数据都会来到CAN_RxCallback内统一处理
//=====================================================================================

#include "CAN_Driver.h"
#include "SuperCap_Driver.h"
#include "INS.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DrEmpower.h"
#include "Communicate.h"
#include "Remote_Control.h"

uint16_t CAN1_Err_Ticker;
uint16_t CAN2_Err_Ticker;
uint16_t CAN3_Err_Ticker;

uint8_t CAN_RX_Data1[8];
uint16_t CAN_RX_ID;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RX_Data1);
    
    if(hfdcan == &hfdcan1) CAN1_Err_Ticker = 0;
    if(hfdcan == &hfdcan2) CAN2_Err_Ticker = 0;
    if(hfdcan == &hfdcan3) CAN3_Err_Ticker = 0;
    
    CAN_RX_ID = RxHeader.Identifier;
    
        /*===| 云台电机数据 |===*/
        if      (hfdcan == &Gimbal_Pitch_CAN    && CAN_RX_ID == Gimbal_Pitch_Feedback_CAN_ID)       Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Pitch);
        else if (hfdcan == &Gimbal_Yaw_CAN      && CAN_RX_ID == Gimbal_Yaw_Feedback_CAN_ID)         Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Yaw);
        
        /*===| 发射机构电机数据 |===*/
        else if (hfdcan == &Shoot_Trigger_CAN     && CAN_RX_ID == Shoot_Trigger_Feedback_CAN_ID)      Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Trigger);
        else if (hfdcan == &Shoot_Fric_First_CAN  && CAN_RX_ID == Shoot_Fric_First_Left_Feedback_CAN_ID)    Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Fric1);
        else if (hfdcan == &Shoot_Fric_First_CAN  && CAN_RX_ID == Shoot_Fric_First_Right_Feedback_CAN_ID)   Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Fric2);

        /*===| 底盘电机数据 |===*/
//        else if (hfdcan == &Chassis_Motor_CAN   && CAN_RX_ID == Chassis_Motor1_Feedback_CAN_ID)   Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Chassis1);
//        else if (hfdcan == &Chassis_Motor_CAN   && CAN_RX_ID == Chassis_Motor2_Feedback_CAN_ID)   Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Chassis2);
//        else if (hfdcan == &Chassis_Motor_CAN   && CAN_RX_ID == Chassis_Motor3_Feedback_CAN_ID)   Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Chassis3);
//        else if (hfdcan == &Chassis_Motor_CAN   && CAN_RX_ID == Chassis_Motor4_Feedback_CAN_ID)   Motor_DJI_Storage_Data(CAN_RX_Data1, &Motor.Chassis4);
        
//        /*===| 超电数据 |===*/  
//        else if (hfdcan == &SuperCap_CAN        && CAN_RX_ID == SuperCap_FeedBack_CAN_ID) SuperCap_Unpack_Data(CAN_RX_Data1);
		
        // /*===| IMU数据 |===*/  
        // else if (hfdcan == &IMU_CAN             && CAN_RX_ID == IMU_FeedBack_CAN_ID) INS_Get_Data_Call_Back(CAN_RX_Data1);

		/*===| 双板通信数据 |===*/  
        else if (hfdcan == &hfdcan3             && CAN_RX_ID == 0x53) Gimbal_Receive_Referee_Unpack(CAN_RX_Data1);
	    else if (hfdcan == &hfdcan3             && CAN_RX_ID == 0x54) Gimbal_Receive_attitude_Unpack(CAN_RX_Data1);
		 
		// /*===| 遥控CAN转发数据 |===*/  
        // else if (hfdcan == &Remote_CAN         && CAN_RX_ID == Remote_CAN_Joystic_CAN_ID) Remote_CAN_ReceiveData_Joystic(CAN_RX_Data1);
        // else if (hfdcan == &Remote_CAN         && CAN_RX_ID == Remote_CAN_KeyboardMouse_CAN_ID) Remote_CAN_ReceiveData_KeyboardMouse(CAN_RX_Data1);

		 
}

void CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *TX_Data)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0; 
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data) == HAL_ERROR)
        if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data) == HAL_ERROR)
            HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TX_Data);
}

void CAN_Init(void)
{
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
    CAN_Filter_Init(&hfdcan1);
    CAN_Filter_Init(&hfdcan2);
    CAN_Filter_Init(&hfdcan3);
}

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan) 
{
	FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00;
    sFilterConfig.FilterID2 = 0x7FF;
    
    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

