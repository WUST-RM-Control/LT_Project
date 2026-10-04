#include "VOFA.h"

#define NUM_Date 32

/**
 * @brief VOFA串口接收中断回调函数
 *
 * @param 串口收到的数组
 */
void VOFA_Rx_CallBack(uint8_t *VOFA_RxDataBuff)
{
	// float VOFA_RX_Float =   (VOFA_RxDataBuff[3]-0x30)*10000+
    //                         (VOFA_RxDataBuff[4]-0x30)*1000+
    //                         (VOFA_RxDataBuff[5]-0x30)*100+
    //                         (VOFA_RxDataBuff[6]-0x30)*10+
    //                         (VOFA_RxDataBuff[7]-0x30)*1+
    //                         (VOFA_RxDataBuff[9]-0x30)*0.1f+
    //                         (VOFA_RxDataBuff[10]-0x30)*0.01f+
    //                         (VOFA_RxDataBuff[11]-0x30)*0.001f;
    // if(VOFA_RxDataBuff[2] == '-') VOFA_RX_Float = -VOFA_RX_Float;
	
    
	// if		(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='P') 
    // {
    //     Shoot_Fric_First_Left_Speed   = VOFA_RX_Float;
    //     Shoot_Fric_First_Right_Speed  = VOFA_RX_Float;
    //     Shoot_Fric_First_Middle_Speed = VOFA_RX_Float;
    // }
	// else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='I')
    // {
    //     Shoot_Fric_Second_Left_Speed   = VOFA_RX_Float;
    //     Shoot_Fric_Second_Right_Speed  = VOFA_RX_Float;
    //     Shoot_Fric_Second_Middle_Speed = VOFA_RX_Float;
    // }
    // else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='D')
    // {

    // }
    // else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='E') 
    // {

    // }
}

float VOFA_JustFloat_Buffer[NUM_Date];
uint8_t VOFA_JustFloat_Buffer_End = 0;
void VOFA_JustFloat_AddValue(uint8_t NUM, float Data)
{
    VOFA_JustFloat_Buffer[NUM] = Data;
    VOFA_JustFloat_Buffer_End = MAX(VOFA_JustFloat_Buffer_End, NUM);
}

void VOFA_JustFloat_Send(UART_HandleTypeDef *huart)
{
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    memcpy(&VOFA_JustFloat_Buffer[VOFA_JustFloat_Buffer_End + 1], tail, 4);
    
    HAL_UART_Transmit(huart, VOFA_JustFloat_Buffer, 4*(VOFA_JustFloat_Buffer_End + 2), 10);
}
