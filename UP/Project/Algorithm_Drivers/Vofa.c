#include "main.h"
#include <stdint.h>
#include "usart.h"
#include "usbd_cdc_if.h"

#include "vofa.h"

typedef struct {
    uint8_t header[4];      // 帧头：0xAF 0xFA 0x00 0x00
    uint16_t data_length;   // 数据长度（浮点数个数）
    float *data;            // 浮点数数组
    uint8_t footer[4];      // 帧尾：0x00 0x00 0xFA 0xAF
}  JustFloat_Frame;

HAL_StatusTypeDef JustFloat_Send(float *data, uint16_t length)
{
    JustFloat_Frame frame;
    frame.data = data;
    frame.footer[0] = 0x00;
    frame.footer[1] = 0x00;
    frame.footer[2] = 0x80;
    frame.footer[3] = 0x7F;


	  if (CDC_Transmit_FS((uint8_t *)frame.data, length * sizeof(float)) != HAL_OK) return HAL_ERROR;
	  osDelay(1);//防止发送过快导致帧尾不发送
    if (CDC_Transmit_FS(frame.footer, 4) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}
