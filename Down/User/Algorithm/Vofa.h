#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

void VOFA_Rx_CallBack(uint8_t *VOFA_RxDataBuff);
// 仅更新待发送数据,不进行串口传输。通道0~31。
void VOFA_JustFloat_AddValue(uint8_t NUM, float Data);
// Chassis_Task末尾统一调用:5ms节流,USART1 DMA忙则跳过,无阻塞等待。
void VOFA_JustFloat_Send(UART_HandleTypeDef *huart);

extern volatile uint32_t VOFA_JustFloat_SentCount;
extern volatile uint32_t VOFA_JustFloat_BusySkipCount;
extern volatile uint32_t VOFA_JustFloat_ErrorCount;

#endif
