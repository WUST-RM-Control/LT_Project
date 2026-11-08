//=====================================================================================
//                    USART_Driver[USART通信驱动]
//      包含了一些USART通信相关的封装好的函数
//
//      USART初始化 USART_Init(void)
//      USART发送 USART1/3/6_Printf(char *format, ...)
//      USART接收事件回调函数统一处理串口收到的数据
//=====================================================================================

#include "USART_Driver.h"
#include "Vofa.h"
#include "Remote_Control.h"
#include "Referee_Unpack.h"

uint8_t UART1_RX_Data[256];
uint8_t UART7_RX_Data[256];
uint8_t UART10_RX_Data[128];
uint8_t RS485_UART2_RX_Data[128];
uint8_t RS485_UART3_RX_Data[128];
uint8_t SBUS_UART5_RX_Data[128];

uint16_t Tof_Displacement = 0;//毫米
float Tof_Displacement_M = 0.0f;//米
//Tof测距解包
void Tof_Date_Unpack(uint8_t *Date)
{
    char *p = strstr((char *)Date, "d:");
    uint16_t dat_tmp = 0;//毫米
    while (*p != 'm')
    {
        if (*p >= '0' && *p <= '9')
        {
            dat_tmp = dat_tmp * 10 + (*p - '0');
        }
        p++;
    }
    Tof_Displacement = dat_tmp;
	Tof_Displacement_M = Tof_Displacement*0.001f;
}


/**
 * @brief USART接收空闲事件回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart1)
	{
        // ImageLink_Data_Unpack(UART1_RX_Data);
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 256);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
    else if(huart == &huart7)
	{
        Referee_Data_Unpack(UART7_RX_Data);
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_RX_Data, 256);
        __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	}
    else if(huart == &huart10)
	{
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
	}
    else if(huart == &huart2)
	{
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_UART2_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
    else if(huart == &huart3)
	{
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RS485_UART3_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
    else if(huart == &huart5)
	{
        // Remote_Control_Rx_CallBack(SBUS_UART5_RX_Data);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, SBUS_UART5_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart == &huart1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 256);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
    else if(huart == &huart7)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_RX_Data, 256);
        __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	}
    else if(huart == &huart10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
	}
    else if(huart == &huart2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_UART2_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
    else if(huart == &huart3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RS485_UART3_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
    else if(huart == &huart5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, SBUS_UART5_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	}
}

/**
 * @brief 串口初始化
 *
 * @param 无
 */
void USART_Init(void)
{    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 256);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_RX_Data, 256);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_RX_Data, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_UART2_RX_Data, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RS485_UART3_RX_Data, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, SBUS_UART5_RX_Data, 128);
    
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

