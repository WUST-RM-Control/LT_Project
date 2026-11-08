#include "Key_LED.h"
#include "Key.h"
#include "LED.h"
#include "Remote_Control.h"

/* 灯光任务间隔，10ms。建议不要高于 30，否则闪烁和渐变会明显变卡。 */
#define KeyTASK_CONTROL_TIME   10

void KeyTask(void const * argument)
{
    static uint8_t index = 0u;

    Key_Init();
    LED_Init();

    /* 上电默认彩色灯 */
    LED_Set_Effect(LED_Effect_Colorful);

    for(;;)
    {
        Key_Scan_Loop();

        LED_Task_Loop();            /* 按当前灯效把这一拍的颜色发出去 */

        osDelay(KeyTASK_CONTROL_TIME);
    }
}
