#ifndef __CAN_Driver__
#define __CAN_Driver__

#include "main.h"

void CAN_Init(void);

void CAN_Send_Data_STD(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *TX_Data);
void CAN_Send_Data_EXD(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *TX_Data, uint8_t Length);

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan);

extern uint16_t CAN1_Err_Ticker;
extern uint16_t CAN2_Err_Ticker;
extern uint16_t CAN3_Err_Ticker;

#endif
