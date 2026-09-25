#ifndef __Buzzer_Task__
#define __Buzzer_Task__

#include "main.h"

//蜂鸣器音效任务间隔，10ms。建议不要高于30。
#define BUZZER_TASK_CONTROL_TIME 10

/*===| 预设音效枚举定义 |===*/
typedef enum
{
	Buzzer_SoundEffect_OFF = 0,
    Buzzer_SoundEffect_SystemStart,
    Buzzer_SoundEffect_SuperCap_ON,
    Buzzer_SoundEffect_SuperCap_OFF,
    Buzzer_SoundEffect_Aim_ON,
    Buzzer_SoundEffect_Aim_OFF,
    Buzzer_SoundEffect_Stuck,
    Buzzer_SoundEffect_Stuck_Processed,
    Buzzer_SoundEffect_Speed3,
    Buzzer_SoundEffect_Speed4,
    Buzzer_SoundEffect_Speed5,
    Buzzer_SoundEffect_Shoot1,
    Buzzer_SoundEffect_Shoot2,
    Buzzer_SoundEffect_Shoot3,
    Buzzer_SoundEffect_Shoot4,
    Buzzer_SoundEffect_Shoot5,
    Buzzer_SoundEffect_Shoot6,
    Buzzer_SoundEffect_Shoot7,
    Buzzer_SoundEffect_Shoot8,
} Buzzer_SoundEffect_EnumTypedef;

/*===| 蜂鸣器音调枚举定义 |===*/
typedef enum
{
	P = 0,    
    L1, L2, L3, L4, L5, L6, L7,
    M1, M2, M3, M4, M5, M6, M7,
    H1, H2, H3, H4, H5, H6, H7,
} Buzzer_Tone_EnumTypedef;


void Buzzer_Init(void);

void Buzzer_Set_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect);

void Buzzer_Set_Tone(Buzzer_Tone_EnumTypedef Tone);

#endif


