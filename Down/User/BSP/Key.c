#include "Key.h"

uint8_t KeyNum;

Key_StructTypeDef Key_Data;

void Key_Scan_Loop(void) //10ms
{
    Key_Data.User.Pin_Now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    
    for(uint8_t i = 0; i < KeyNum; i++)
    {
        if(((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now == 0 && ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last == 1)
        {
            //按下瞬间
            ((Key_Data_StructTypeDef *)&Key_Data)[i].Push_Time = 0;
            ((Key_Data_StructTypeDef *)&Key_Data)[i].Release_Time = 0;
        }
        if(((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now == 0 && ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last == 0)
        {
            //持续按下
            ((Key_Data_StructTypeDef *)&Key_Data)[i].Push_Time++;
            Limit_int(&((Key_Data_StructTypeDef *)&Key_Data)[i].Push_Time, 110, 0);
        }
        if(((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now == 1 && ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last == 0)
        {
            //松开瞬间
            if(((Key_Data_StructTypeDef *)&Key_Data)[i].Push_Time >= 100) ((Key_Data_StructTypeDef *)&Key_Data)[i].State = LongPush;
            else ((Key_Data_StructTypeDef *)&Key_Data)[i].State = ShortPush;
        }
        if(((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now == 1 && ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last == 1)
        {
            //持续松开
            ((Key_Data_StructTypeDef *)&Key_Data)[i].Release_Time++;
            Limit_int(&((Key_Data_StructTypeDef *)&Key_Data)[i].Release_Time, 100, 0);
            if(((Key_Data_StructTypeDef *)&Key_Data)[i].Release_Time >= 50)
                ((Key_Data_StructTypeDef *)&Key_Data)[i].State = Release;
        }
		
		((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last = ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now;
    }
}

void Key_Init(void)
{
    KeyNum = sizeof(Key_StructTypeDef) / sizeof(Key_Data_StructTypeDef);
    
    for(uint8_t i = 0; i < KeyNum; i++)
    {
        ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Now = 1;
        ((Key_Data_StructTypeDef *)&Key_Data)[i].Pin_Last = 1;     
        ((Key_Data_StructTypeDef *)&Key_Data)[i].State = Release;
    }  
}


Key_State Key_Get_State(Key_Data_StructTypeDef *Key)
{
    Key_State temp = Key->State;
    Key->State = Release;
    return temp;
}