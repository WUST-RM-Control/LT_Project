#include "VOFA.h"

#define NUM_Date 32
#define VOFA_SEND_PERIOD_MS 5U
#define VOFA_DMA_BUFFER_BYTES 160U // 32个float+4字节尾,按32字节cache line向上对齐

// DMA1不能访问DTCM。与本工程LED缓冲区相同采用独立SRAM地址,
// 为USART1预留D2 SRAM1的0x30000000~0x3000009F,当前scatter不分配此区域。
static uint8_t *const VOFA_JustFloat_DMA_Buffer = (uint8_t *)0x30000000UL;
static uint32_t VOFA_JustFloat_Send_Tick = 0U;
static uint8_t VOFA_JustFloat_Has_Data = 0U;

volatile uint32_t VOFA_JustFloat_SentCount = 0U;
volatile uint32_t VOFA_JustFloat_BusySkipCount = 0U;
volatile uint32_t VOFA_JustFloat_ErrorCount = 0U;

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
    if(NUM >= NUM_Date) return;
    VOFA_JustFloat_Buffer[NUM] = Data;
    VOFA_JustFloat_Buffer_End = MAX(VOFA_JustFloat_Buffer_End, NUM);
    VOFA_JustFloat_Has_Data = 1U;
}

void VOFA_JustFloat_Send(UART_HandleTypeDef *huart)
{
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    uint32_t now = HAL_GetTick();
    uint16_t data_bytes;
    HAL_StatusTypeDef status;

    // 唯一调用点在Chassis_Task末尾,固定使用USART1。控制函数不发送串口。
    if(huart != &huart1 || huart->hdmatx == NULL || !VOFA_JustFloat_Has_Data ||
       (uint32_t)(now - VOFA_JustFloat_Send_Tick) < VOFA_SEND_PERIOD_MS) return;
    VOFA_JustFloat_Send_Tick = now;
    if(huart->gState != HAL_UART_STATE_READY)
    {
        VOFA_JustFloat_BusySkipCount++;
        return; // DMA未结束时不改写发送缓冲区,不等待也不重试
    }

    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    data_bytes = (uint16_t)(sizeof(float) * (VOFA_JustFloat_Buffer_End + 1U));
    memcpy(VOFA_JustFloat_DMA_Buffer, VOFA_JustFloat_Buffer, data_bytes);
    memcpy(VOFA_JustFloat_DMA_Buffer + data_bytes, tail, sizeof(tail));
    if((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_CleanDCache_by_Addr((uint32_t *)VOFA_JustFloat_DMA_Buffer, VOFA_DMA_BUFFER_BYTES);
    __DSB();
    status = HAL_UART_Transmit_DMA(huart, VOFA_JustFloat_DMA_Buffer, data_bytes + sizeof(tail));
    if(status == HAL_OK) VOFA_JustFloat_SentCount++;
    else if(status == HAL_BUSY) VOFA_JustFloat_BusySkipCount++;
    else VOFA_JustFloat_ErrorCount++;
}
