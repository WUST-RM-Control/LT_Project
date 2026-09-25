//=====================================================================================
//                    Remote_Control[遥控器数据解包]
//      解析遥控器的数据，得到遥控器各个通道状态和键鼠信息，供RoboControl使用
//=====================================================================================

#include "Remote_Control.h"

/*===| 遥控器数据结构体 |===*/
remote_rawdata_t remote_rawdata;
Remote_StructTypeDef Remote;
Remote_StructTypeDef Remote_Last;

/**
 * @brief 遥控串口接收中断回调函数
 *
 * @param 串口收到的数组
 */
void Remote_Rx_CallBack(uint8_t *DataBuff)
{
    //CRC16校验
    if(verify_CRC16_check_sum(DataBuff, 21) == TRUE)
    {
        //更新连接状态
        Remote.If_Remote_Connect = 1;
        Remote.Remote_GetData_Ticker = 0;
        Remote.If_Remote_Data_New = 1;

        //数据解包
        memcpy(&remote_rawdata, DataBuff, sizeof(remote_rawdata));
        //存储上一次数据(建议注释此处，在其他地方50ms触发一次该函数用于消抖)
//        Remote_Save_Last_Data();
        //存储数据
        Remote.Right_X  = (remote_rawdata.ch_0 - Remote_CH_VALUE_OFFSET) / 660.0f;
        Remote.Right_Y  = (remote_rawdata.ch_1 - Remote_CH_VALUE_OFFSET) / 660.0f;
        Remote.Left_Y   = (remote_rawdata.ch_2 - Remote_CH_VALUE_OFFSET) / 660.0f;
        Remote.Left_X   = (remote_rawdata.ch_3 - Remote_CH_VALUE_OFFSET) / 660.0f;
        Remote.Mode     = remote_rawdata.mode_sw;
        Remote.Pause    = remote_rawdata.pause;
        Remote.Custom_L = remote_rawdata.fn_1;
        Remote.Custom_R = remote_rawdata.fn_2;
        Remote.Wheel    = (remote_rawdata.wheel - Remote_CH_VALUE_OFFSET) / 660.0f;
        Remote.Trigger  = remote_rawdata.trigger;
        
        Remote.Mouse_Vx = remote_rawdata.mouse_x;
        Remote.Mouse_Vy = remote_rawdata.mouse_y;
        Remote.Mouse_Vz = (remote_rawdata.mouse_z) / 32.0f;
        
        Remote.Mouse_L = remote_rawdata.mouse_left;
        Remote.Mouse_M = remote_rawdata.mouse_middle;
        Remote.Mouse_R = remote_rawdata.mouse_right;
        
        Remote.Keyboard_W       = !!(remote_rawdata.key1 & 0x01);
        Remote.Keyboard_S       = !!(remote_rawdata.key1 & 0x02);
        Remote.Keyboard_A       = !!(remote_rawdata.key1 & 0x04);
        Remote.Keyboard_D       = !!(remote_rawdata.key1 & 0x08);
        Remote.Keyboard_Shift   = !!(remote_rawdata.key1 & 0x10);
        Remote.Keyboard_Ctrl    = !!(remote_rawdata.key1 & 0x20);
        Remote.Keyboard_Q       = !!(remote_rawdata.key1 & 0x40);
        Remote.Keyboard_E       = !!(remote_rawdata.key1 & 0x80);
        Remote.Keyboard_R       = !!(remote_rawdata.key2 & 0x01);
        Remote.Keyboard_F       = !!(remote_rawdata.key2 & 0x02);
        Remote.Keyboard_G       = !!(remote_rawdata.key2 & 0x04);
        Remote.Keyboard_Z       = !!(remote_rawdata.key2 & 0x08);
        Remote.Keyboard_X       = !!(remote_rawdata.key2 & 0x10);
        Remote.Keyboard_C       = !!(remote_rawdata.key2 & 0x20);
        Remote.Keyboard_V       = !!(remote_rawdata.key2 & 0x40);
        Remote.Keyboard_B       = !!(remote_rawdata.key2 & 0x80);
    }
}

//记录上一次值，用于消抖(建议50ms以上触发一次)
void Remote_Save_Last_Data(void) 
{
    Remote_Last.If_Remote_Data_New = 1;
    while(Remote_Last.If_Remote_Data_New == 1) osDelay(1);
    memcpy(&Remote_Last, &Remote, sizeof(Remote_StructTypeDef));
}

//通过CAN转发遥控器数据
void Remote_CAN_SendData_Joystic(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID)
{
    Remote_CAN_Data_Joystic_TypedefStruct Joystick;
    
    Joystick.If_Remote_Connect  = Remote.If_Remote_Connect ;
    
    //限幅
    Limit_float(&Remote.Right_X, 1, -1);
    Limit_float(&Remote.Right_Y, 1, -1);
    Limit_float(&Remote.Left_X, 1, -1);
    Limit_float(&Remote.Left_Y, 1, -1);
    Limit_float(&Remote.Wheel, 1, -1);
    
    Joystick.Right_X_x100       = 100*Remote.Right_X       ;
    Joystick.Right_Y_x100       = 100*Remote.Right_Y       ;
    Joystick.Left_X_x100        = 100*Remote.Left_X        ;
    Joystick.Left_Y_x100        = 100*Remote.Left_Y        ;
    Joystick.Wheel_x100         = 100*Remote.Wheel         ;
    Joystick.If_Remote_Connect  = Remote.If_Remote_Connect ;
    Joystick.Mode               = Remote.Mode              ;
    Joystick.Pause              = Remote.Pause             ;
    Joystick.Custom_L           = Remote.Custom_L          ;
    Joystick.Custom_R           = Remote.Custom_R          ;
    Joystick.Trigger            = Remote.Trigger           ;
    
    CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)&Joystick);
}

//通过CAN转发键鼠数据
void Remote_CAN_SendData_KeyboardMouse(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID)
{
    Remote_CAN_Data_KeyboardMouse_TypedefStruct KeyboardMouse;
    
    Limit_float(&Remote.Mouse_Vz, 1, -1);
    
    KeyboardMouse.Mouse_Vx       = Remote.Mouse_Vx  ;
    KeyboardMouse.Mouse_Vy       = Remote.Mouse_Vy  ;
    KeyboardMouse.Mouse_Vz_x100  = 100*Remote.Mouse_Vz  ;
    KeyboardMouse.Mouse_L        = Remote.Mouse_L       ;
    KeyboardMouse.Mouse_M        = Remote.Mouse_M       ;
    KeyboardMouse.Mouse_R        = Remote.Mouse_R       ;
    KeyboardMouse.Keyboard_W     = Remote.Keyboard_W    ;
    KeyboardMouse.Keyboard_A     = Remote.Keyboard_A    ;
    KeyboardMouse.Keyboard_S     = Remote.Keyboard_S    ;
    KeyboardMouse.Keyboard_D     = Remote.Keyboard_D    ;
    KeyboardMouse.Keyboard_Q     = Remote.Keyboard_Q    ;
    KeyboardMouse.Keyboard_E     = Remote.Keyboard_E    ;
    KeyboardMouse.Keyboard_Shift = Remote.Keyboard_Shift;
    KeyboardMouse.Keyboard_Ctrl  = Remote.Keyboard_Ctrl ;
    KeyboardMouse.Keyboard_R     = Remote.Keyboard_R    ;
    KeyboardMouse.Keyboard_F     = Remote.Keyboard_F    ;
    KeyboardMouse.Keyboard_G     = Remote.Keyboard_G    ;
    KeyboardMouse.Keyboard_Z     = Remote.Keyboard_Z    ;
    KeyboardMouse.Keyboard_X     = Remote.Keyboard_X    ;
    KeyboardMouse.Keyboard_C     = Remote.Keyboard_C    ;
    KeyboardMouse.Keyboard_V     = Remote.Keyboard_V    ;
    KeyboardMouse.Keyboard_B     = Remote.Keyboard_B    ;
    
    CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)&KeyboardMouse);
}

//通过CAN接收遥控器数据
void Remote_CAN_ReceiveData_Joystic(uint8_t *RxData)
{
    Remote_CAN_Data_Joystic_TypedefStruct Joystick;
    memcpy(&Joystick, RxData, sizeof(Remote_CAN_Data_Joystic_TypedefStruct));    
    
    Remote.If_Remote_Connect  = Joystick.If_Remote_Connect    ;
        
    Remote.Right_X            = 0.01f * Joystick.Right_X_x100 ;
    Remote.Right_Y            = 0.01f * Joystick.Right_Y_x100 ;
    Remote.Left_X             = 0.01f * Joystick.Left_X_x100  ;
    Remote.Left_Y             = 0.01f * Joystick.Left_Y_x100  ;
    Remote.Wheel              = 0.01f * Joystick.Wheel_x100   ;
    Remote.If_Remote_Connect  = Joystick.If_Remote_Connect    ;
    Remote.Mode               = Joystick.Mode                 ;
    Remote.Pause              = Joystick.Pause                ;
    Remote.Custom_L           = Joystick.Custom_L             ;
    Remote.Custom_R           = Joystick.Custom_R             ;
    Remote.Trigger            = Joystick.Trigger              ;
    
    //更新连接状态
    Remote.If_Remote_Connect = 1;
    Remote.Remote_GetData_Ticker = 0;
    Remote.If_Remote_Data_New = 1;
}

//通过CAN接收键鼠数据
void Remote_CAN_ReceiveData_KeyboardMouse(uint8_t *RxData)
{
    Remote_CAN_Data_KeyboardMouse_TypedefStruct KeyboardMouse;
    memcpy(&KeyboardMouse, RxData, sizeof(Remote_CAN_Data_KeyboardMouse_TypedefStruct)); 

    Remote.Mouse_Vx       = KeyboardMouse.Mouse_Vx ;
    Remote.Mouse_Vy       = KeyboardMouse.Mouse_Vy ;
    Remote.Mouse_Vz       = 0.01f * KeyboardMouse.Mouse_Vz_x100 ;
    Remote.Mouse_L        = KeyboardMouse.Mouse_L               ;
    Remote.Mouse_M        = KeyboardMouse.Mouse_M               ;
    Remote.Mouse_R        = KeyboardMouse.Mouse_R               ;
    Remote.Keyboard_W     = KeyboardMouse.Keyboard_W            ;
    Remote.Keyboard_A     = KeyboardMouse.Keyboard_A            ;
    Remote.Keyboard_S     = KeyboardMouse.Keyboard_S            ;
    Remote.Keyboard_D     = KeyboardMouse.Keyboard_D            ;
    Remote.Keyboard_Q     = KeyboardMouse.Keyboard_Q            ;
    Remote.Keyboard_E     = KeyboardMouse.Keyboard_E            ;
    Remote.Keyboard_Shift = KeyboardMouse.Keyboard_Shift        ;
    Remote.Keyboard_Ctrl  = KeyboardMouse.Keyboard_Ctrl         ;
    Remote.Keyboard_R     = KeyboardMouse.Keyboard_R            ;
    Remote.Keyboard_F     = KeyboardMouse.Keyboard_F            ;
    Remote.Keyboard_G     = KeyboardMouse.Keyboard_G            ;
    Remote.Keyboard_Z     = KeyboardMouse.Keyboard_Z            ;
    Remote.Keyboard_X     = KeyboardMouse.Keyboard_X            ;
    Remote.Keyboard_C     = KeyboardMouse.Keyboard_C            ;
    Remote.Keyboard_V     = KeyboardMouse.Keyboard_V            ;
    Remote.Keyboard_B     = KeyboardMouse.Keyboard_B            ;
    
    //更新连接状态
    Remote.If_Remote_Connect = 1;
    Remote.Remote_GetData_Ticker = 0;
    Remote.If_Remote_Data_New = 1;
}

