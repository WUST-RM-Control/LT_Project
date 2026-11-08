#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

void VOFA_Rx_CallBack(uint8_t *VOFA_RxDataBuff);
void VOFA_JustFloat_AddValue(uint8_t NUM, float Data);
void VOFA_JustFloat_Send(UART_HandleTypeDef *huart);

#endif